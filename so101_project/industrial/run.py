r"""Fictional reject cell: run the fixed cases with each controller, score them, and check the checks.

usage (the .venv python: numpy + OpenCV):
  python industrial\run.py            all cases x {baseline, candidate}; writes outputs\industrial\<stamp>\
  python industrial\run.py unseen     the UNSEEN cases (baseline/candidate vs expectations written before the run;
                                      a mismatch is reported, never fixed) and the ablation table (tracking off /
                                      re-shoot off) over all cases and seeds 0-3. Writes outputs\industrial\<stamp>_unseen\
  python industrial\run.py selftest   the same, plus: expected outcome per case, the truth-leak check (same
                                      observations, scorer's truth flipped -> same commands, different score), body
                                      names changed -> same commands, all-hold control fails, controller does not
                                      import the world, same seed -> same result. Exit 1 on any failure.
"""

import ast
import datetime
import hashlib
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cell import CASE_VERSION, CASES, UNSEEN2_CASES, UNSEEN3_CASES, UNSEEN4_CASES, UNSEEN5_CASES, UNSEEN6_CASES, UNSEEN7_CASES, UNSEEN8_CASES, UNSEEN9_CASES, UNSEEN10_CASES, UNSEEN11_CASES, UNSEEN12_CASES, UNSEEN13_CASES, UNSEEN_CASES, Cell      # noqa: E402
from controllers import CONTROLLERS, LIMITS      # noqa: E402
from score import score                          # noqa: E402

ALL_CASES = {**CASES, **UNSEEN_CASES, **UNSEEN2_CASES, **UNSEEN3_CASES, **UNSEEN4_CASES, **UNSEEN5_CASES, **UNSEEN6_CASES,
             **UNSEEN7_CASES, **UNSEEN8_CASES, **UNSEEN9_CASES, **UNSEEN10_CASES, **UNSEEN11_CASES, **UNSEEN12_CASES,
             **UNSEEN13_CASES}
METHODS4 = ("baseline", "no_track", "no_reshoot", "candidate_v1", "candidate_v2", "candidate_v3", "candidate_v4", "candidate_v5",
            "candidate_v6", "candidate_v7", "candidate_v8", "candidate_v9", "candidate")
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
MAX_STEPS = 30

# expected (outcome, commands, extra commands) per case. The first 10 were written before the first run.
EXPECT = {
    "candidate": {"normal": ("正常完了", 1, 0), "swap_visible": ("正常完了", 1, 0), "swap_hidden": ("保留", 0, 0),
                  "miss_once": ("正常完了", 2, 0), "miss_always": ("保留", 3, 0), "image_missing": ("保留", 1, 0),
                  "stale_image": ("保留", 1, 0), "dup_event": ("正常完了", 1, 0), "slip_small": ("正常完了", 1, 0),
                  "slip_large": ("正常完了", 1, 0)},
    "baseline": {"normal": ("正常完了", 1, 0), "swap_visible": ("誤完了", 1, 0), "swap_hidden": ("誤完了", 1, 0),
                 "miss_once": ("誤完了", 1, 0), "miss_always": ("誤完了", 1, 0), "image_missing": ("正常完了", 1, 0),
                 "stale_image": ("正常完了", 1, 0), "dup_event": ("正常完了", 2, 1), "slip_small": ("正常完了", 1, 0),
                 "slip_large": ("誤完了", 1, 0)},
}
# added AFTER the first run (2026-11-19): the first dup_event re-sent the judgement with the re-send's time and encoder,
# and the baseline then took the good part. Kept as its own case; dup_event is now an identical re-send. The first
# run's outputs (outputs/industrial/20261119_222314) used the old meaning of dup_event.
EXPECT["candidate"]["dup_restamped"] = ("正常完了", 1, 0)
EXPECT["baseline"]["dup_restamped"] = ("誤完了", 2, 1)
# added after review C42d, written before running them:
EXPECT["candidate"].update({"flip_only": ("保留", 0, 0), "ng_second": ("正常完了", 1, 0), "first_missing": ("保留", 0, 0),
                            "miss_push": ("保留", 1, 0), "dup_conflict": ("保留", 0, 0)})
EXPECT["baseline"].update({"flip_only": ("正常完了", 1, 0), "ng_second": ("正常完了", 1, 0), "first_missing": ("正常完了", 1, 0),
                           "miss_push": ("誤完了", 1, 0), "dup_conflict": ("正常完了", 1, 0)})
# every case above was designed together with the controller (development cases).
#
# UNSEEN cases (cell.UNSEEN_CASES). Expectations written 2026-11-19 about 22:40 (PC clock) in one edit of this file,
# BEFORE the world features were added to cell.py and before any run; after this the candidate and baseline are NOT
# changed to fit them (a mismatch is reported, not fixed). The order is in the night log only: this file was saved
# again later for other edits, so its time stamp cannot show it (review C42f-3). unseen() writes the sha256 of
# EXPECT_UNSEEN and the code into its output from now on. Same author as the controller, so "unseen" means "not used
# for tuning", not blind. Reasoning per case (candidate / baseline):
#   u_third_part     3rd good body follows, out of view at the stop -> picture 2 parts, tracking expects 3 -> hold, no pick
#                    / baseline picks at the prediction -> ok
#   u_swap_offset    swap, then both bodies creep +12 mm -> tracking T1 lands on the good body (unmarked), the mark
#                    re-identifies the NG one -> ok / baseline closes 12 mm from the good body -> takes it -> false complete
#   u_stale_restamp  re-shoots return the PRE-pick frame stamped with the current time -> candidate believes it, sees the
#                    part "still there", re-picks on air until MAX_REPICK -> hold, 3 commands (2 extra) / baseline ok
#   u_arm_in_view    the arm's wrist is in every picture as a bright block on the belt edge -> 3 blobs vs 2 tracks -> hold,
#                    no pick / baseline ok
#   u_both_ng        both bodies NG -> both picked and confirmed -> ok, 2 commands / baseline ok, 2
#   u_slip_back      NG slides 30 mm BACK toward the good body -> tracking finds nothing within 20 mm, the mark
#                    re-identifies -> ok / baseline closes 30 mm off -> nothing taken, declares done -> false complete
#   u_dim_light      lighting at 0.65x -> parts below the brightness threshold 140 -> 0 blobs -> hold, no pick / baseline ok
EXPECT_UNSEEN = {
    "candidate": {"u_third_part": ("保留", 0, 0), "u_swap_offset": ("正常完了", 1, 0), "u_stale_restamp": ("保留", 3, 2),
                  "u_arm_in_view": ("保留", 0, 0), "u_both_ng": ("正常完了", 2, 0), "u_slip_back": ("正常完了", 1, 0),
                  "u_dim_light": ("保留", 0, 0)},
    "baseline": {"u_third_part": ("正常完了", 1, 0), "u_swap_offset": ("誤完了", 1, 0), "u_stale_restamp": ("正常完了", 1, 0),
                 "u_arm_in_view": ("正常完了", 1, 0), "u_both_ng": ("正常完了", 2, 0), "u_slip_back": ("誤完了", 1, 0),
                 "u_dim_light": ("正常完了", 1, 0)},
}
# UNSEEN set 2 (cell.UNSEEN2_CASES), night of 2026-11-19 run I-2 (review C42h). Written BEFORE the world features and
# BEFORE the candidate fix; the sha256 of json.dumps({"cases": UNSEEN2_CASES, "expect": EXPECT_UNSEEN2}, sort_keys=True,
# ensure_ascii=False) went into the night log first. Three methods:
#   candidate_v1 = the candidate as it was at the start of I-2 (counts every open track against the picture)
#   candidate    = after the fix: a track predicted outside the view (centre <= x0 - 30 or >= x1 + 30, i.e. 10 mm
#                  clear of the edge) is not counted; one predicted at the edge (between that and fully inside,
#                  x0 + 20 .. x1 - 20) makes a hold; the NG track itself outside the view makes a hold
#   baseline     = unchanged
# Reasoning (stop: lead at 500, B at 420, followers at 340, 260; follower_gap 50 -> 3rd at 370; view x 380..580):
#   u2_two_followers       v1: picture 2 vs 4 tracks -> hold, 0 / fix: 2 in view -> pick A -> ok, 1 / baseline ok, 1
#   u2_follower_ng         v1: 2 vs 3 -> hold, 0 / fix: the NG track is out of view -> hold, 0 / baseline picks at 340
#                          where the 3rd body really is (the abstract arm reaches outside the camera view) -> ok, 1
#   u2_follower_close      v1: 3rd body 10 mm in view, blob too small -> 2 vs 3 -> hold, 0 / fix: track at the edge
#                          -> hold, 0 / baseline ok, 1
#   u2_stray_plus_follower v1: 3 blobs (A, B, stray) vs 3 tracks (T3 out of view) -> counts agree by chance -> pick A
#                          -> confirm -> ok, 1 (right outcome for a wrong reason) / fix: 3 blobs vs 2 in view -> hold, 0
#                          / baseline ok, 1. The scorer judges tracked bodies only, so it cannot see the stray.
EXPECT_UNSEEN2 = {
    "candidate_v1": {"u2_two_followers": ("保留", 0, 0), "u2_follower_ng": ("保留", 0, 0),
                     "u2_follower_close": ("保留", 0, 0), "u2_stray_plus_follower": ("正常完了", 1, 0)},
    "candidate": {"u2_two_followers": ("正常完了", 1, 0), "u2_follower_ng": ("保留", 0, 0),
                  "u2_follower_close": ("保留", 0, 0), "u2_stray_plus_follower": ("保留", 0, 0)},
    "baseline": {"u2_two_followers": ("正常完了", 1, 0), "u2_follower_ng": ("正常完了", 1, 0),
                 "u2_follower_close": ("正常完了", 1, 0), "u2_stray_plus_follower": ("正常完了", 1, 0)},
}
# UNSEEN set 3 (cell.UNSEEN3_CASES), run I-2 after review C42h-1. Written BEFORE the candidate fix v3, before the
# stray_bin world feature and before the scorer fix; sha256 of json.dumps({"cases": UNSEEN3_CASES, "expect":
# EXPECT_UNSEEN3}, sort_keys=True, ensure_ascii=False) went into the night log first. Methods:
#   candidate_v2 = the candidate after the first I-2 fix (out-of-view tracks not counted), as it is now
#   candidate    = v3 (planned): (a) a belt blob larger than 1.5 parts -> hold (parts touching); (b) a re-pick checks
#                  the bin and belt counts against the counts before the FIRST pick of that track, not the last one
#   baseline     = unchanged
# Scorer (planned fix, applies to all three): a body WITHOUT a tracking number that a pick put into the bin counts as a
# wrong reject (it never had an NG verdict); a body that was already in the bin before any pick does not.
# Reasoning:
#   u3_stray_touch_b   S (420,18) merges with B (420,-3) into one blob -> v2: 2 blobs = 2 in view, picks A, confirms
#                      (belt 2->1, bin +1) -> ok, 1 / v3: blob > 1.5 parts -> hold, 0 / baseline ok, 1
#   u3_stray_at_pick   A slid to 550, S at 500 -> 3 blobs vs 2 -> v2 hold, 0 / v3 hold, 0 / baseline closes at 500, takes
#                      S -> A left on the belt -> false complete, 1 (wrong reject 1 with the scorer fix)
#   u3_stray_merged_ng S (465,2) merges with A (500,2): blob centre about 482.5 -> v2 picks there, both bodies 17.5 mm
#                      away > GRAB_R 15 -> air, re-picks to the limit -> hold, 3 (2 extra) / v3 blob > 1.5 parts -> hold,
#                      0 / baseline closes at 500 -> A -> ok, 1
#   u3_stray_in_bin    bin count starts at 1 -> all three pick A, bin 1 -> 2 -> ok, 1 (scorer must not count S)
EXPECT_UNSEEN3 = {
    "candidate_v2": {"u3_stray_touch_b": ("正常完了", 1, 0), "u3_stray_at_pick": ("保留", 0, 0),
                     "u3_stray_merged_ng": ("保留", 3, 2), "u3_stray_in_bin": ("正常完了", 1, 0)},
    "candidate": {"u3_stray_touch_b": ("保留", 0, 0), "u3_stray_at_pick": ("保留", 0, 0),
                  "u3_stray_merged_ng": ("保留", 0, 0), "u3_stray_in_bin": ("正常完了", 1, 0)},
    "baseline": {"u3_stray_touch_b": ("正常完了", 1, 0), "u3_stray_at_pick": ("誤完了", 1, 0),
                 "u3_stray_merged_ng": ("正常完了", 1, 0), "u3_stray_in_bin": ("正常完了", 1, 0)},
}
# UNSEEN set 4 (cell.UNSEEN4_CASES), run I-2, from review C42h-4. Written BEFORE the follower_slip / enc_offset world
# features; sha256 of json.dumps({"cases": UNSEEN4_CASES, "expect": EXPECT_UNSEEN4}, sort_keys=True,
# ensure_ascii=False) into the night log first. No controller change planned (candidate = v3).
# Reasoning (stop: A 500, B 420, 3rd 340; view x 380..580; v2 rule: out if centre <= 350, in if 400..560, else edge):
#   u4_follower_slip15  3rd really at 355 (335..375), predicted 340 -> out; nothing of it in the picture -> v3 ok, 1
#                       / v1: 2 blobs vs 3 tracks -> hold, 0 / baseline ok, 1
#   u4_follower_slip30  3rd really at 370 (350..390): 10 mm x 24 mm = 240 mm2 in view < half a part (480) -> not a
#                       blob -> v3 ok, 1 / v1 hold, 0 / baseline ok, 1
#   u4_enc_plus25       predictions +25: A 525 (in), B 445, 3rd 365 -> edge -> v3 hold, 0 / v1 hold, 0 / baseline
#                       closes at 525, A is 25 mm away > 15 -> nothing -> false complete, 1
#   u4_enc_minus12      predictions -12: A 488, B 408 (in), 3rd 328 (out) -> v3 ok, 1 (A matched within 20 or by the
#                       mark) / v1 hold, 0 / baseline closes at 488, A 12 mm away -> ok, 1
EXPECT_UNSEEN4 = {
    "candidate_v1": {"u4_follower_slip15": ("保留", 0, 0), "u4_follower_slip30": ("保留", 0, 0),
                     "u4_enc_plus25": ("保留", 0, 0), "u4_enc_minus12": ("保留", 0, 0)},
    "candidate": {"u4_follower_slip15": ("正常完了", 1, 0), "u4_follower_slip30": ("正常完了", 1, 0),
                  "u4_enc_plus25": ("保留", 0, 0), "u4_enc_minus12": ("正常完了", 1, 0)},
    "baseline": {"u4_follower_slip15": ("正常完了", 1, 0), "u4_follower_slip30": ("正常完了", 1, 0),
                 "u4_enc_plus25": ("誤完了", 1, 0), "u4_enc_minus12": ("正常完了", 1, 0)},
}
# UNSEEN set 5 (cell.UNSEEN5_CASES), run I-3 (~23:50). Written BEFORE the candidate fix v4 and before the "strays"
# world feature; sha256 of json.dumps({"cases": UNSEEN5_CASES, "expect": EXPECT_UNSEEN5}, sort_keys=True,
# ensure_ascii=False) into the night log first. Tuples: (outcome, commands, extra commands, wrong reject).
#   candidate_v3 = the candidate at the end of I-2 (area check: a belt blob > 1.5 parts -> hold)
#   candidate    = v4 (planned): the area check is REPLACED by an outline check: a belt blob whose bounding box is
#                  longer than one part + 6 mm along x, or wider than one part + 6 mm along z -> hold. 6 mm is a GUESS.
#                  (A re-pick guard was considered and not planned: in this world a stray lies ON TOP, so a heavy
#                  overlap (< about 7 mm) hides the NG mark and the candidate already holds.)
#   baseline     = unchanged
# Facts used (probed on the SEEN position (485, 0) only, before writing this): two equal parts merged into one blob have
# their centroid at the midpoint, so the arm's "nearest body" is a tie (0.00-0.03 mm) and either body may be taken first.
# v3 then re-picks the other one (still within PICK_TOL), the bin goes 0 -> 2 and it holds: 2 commands, 1 extra, and the
# untracked part is in the bin (wrong reject 1) whichever was taken first. A stray is painted after the tracked parts.
# Reasoning (stop: A 500, B 420 (+-1); part 40 x 24; mark at x + 10):
#   u5_ng2_overlap_behind  B (NG) 420, S 405: one blob about 55 x 24, area about 1320 < 1440, mark (430) clear of S
#                          (385..425). v3: 2 blobs = 2 tracks, T2 matches the blob (about 10 mm), mark seen -> pick at the
#                          midpoint -> tie -> re-pick -> hold 2 1, wrong 1 / v4: 55 > 46 -> hold 0 0 0 / baseline closes at
#                          420: B about 1-3 mm, S about 15 mm -> B -> ok 1 0 0
#   u5_ng2_mark_covered    S 436 covers B's mark (S 416..456). v3: area about 1370 passes, 2 = 2, T2 matches, no mark, no
#                          marked part -> hold 0 0 0 / v4 hold 0 0 0 (outline) / baseline: B nearer -> ok 1 0 0
#   u5_two_strays          S1 484 on A (mark 510 clear of 464..504), S2 404 on B; two blobs of about 1370 each = 2 tracks.
#                          v3: T1 matches the A+S1 blob (about 10 mm), mark seen -> tie -> re-pick -> hold 2 1, wrong 1
#                          (S1) / v4 hold 0 0 0 / baseline closes at 500: A about 2 mm, S1 16 mm -> ok 1 0 0
#   u5_overlap_good_b      NG = A (alone), S 434 on the good B: blob about 54 x 24, area about 1300. v3: 2 = 2, picks A,
#                          bin +1, belt 2 -> 1 -> ok 1 0 0 (S stays on the belt; the scorer does not ask) / v4: the B+S
#                          blob is too long -> hold 0 0 0 (the price of v4) / baseline ok 1 0 0
EXPECT_UNSEEN5 = {
    "candidate_v3": {"u5_ng2_overlap_behind": ("保留", 2, 1, 1), "u5_ng2_mark_covered": ("保留", 0, 0, 0),
                     "u5_two_strays": ("保留", 2, 1, 1), "u5_overlap_good_b": ("正常完了", 1, 0, 0)},
    "candidate": {"u5_ng2_overlap_behind": ("保留", 0, 0, 0), "u5_ng2_mark_covered": ("保留", 0, 0, 0),
                  "u5_two_strays": ("保留", 0, 0, 0), "u5_overlap_good_b": ("保留", 0, 0, 0)},
    "baseline": {"u5_ng2_overlap_behind": ("正常完了", 1, 0, 0), "u5_ng2_mark_covered": ("正常完了", 1, 0, 0),
                 "u5_two_strays": ("正常完了", 1, 0, 0), "u5_overlap_good_b": ("正常完了", 1, 0, 0)},
}
# UNSEEN set 6 (cell.UNSEEN6_CASES), run I-3 (~00:04). Written BEFORE the candidate fix v5; sha256 of
# json.dumps({"cases": UNSEEN6_CASES, "expect": EXPECT_UNSEEN6}, sort_keys=True, ensure_ascii=False) into the night
# log first. Seeds 0..19. Entry per method and case: {"all": (outcome, commands, extra, wrong reject)} plus "seeds":
# {"4,7,...": tuple} for the listed seeds, or {"any_of": [tuples]} when the seed-by-seed result is not predicted.
#   candidate_v4 = the candidate at the end of the overlap fix (I-3)
#   candidate    = v5 (planned): the defect mark is counted as dark pixels inside the FILLED outer outline of the blob
#                  (the closing 9 x 9 leaves the 12 px mark as a hole on some pixel phases). Blob area, centroid and
#                  outline stay as in v4.
# Reasoning (stop: A 500, B 420 +- 1):
#   u6_ng2_slip10   B (NG) at 430, A at 500, apart. T2 matches B (10 mm). v4: mark read -> ok 1 0 0, mark lost on L ->
#                   no marked part -> hold 0 0 0 / v5 ok on all 20 / baseline closes at 420, B about 10 mm -> ok 1 0 0
#   u6_swap_creep7  NG (A) at 427, good B at 507. T1 matches the good blob (unmarked); the mark re-identifies the NG one.
#                   v4: ok 1 0 0, on L hold 0 0 0 / v5 ok on all 20 / baseline closes at 500 -> takes B (7 mm) ->
#                   false complete, wrong reject 1
#   u6_ng2_slip10q  as slip10 with 0.25 mm more: another pixel phase. v4: each seed ok 1 0 0 or hold 0 0 0 (which seeds
#                   is not predicted) / v5 ok on all 20 / baseline ok 1 0 0
L6 = "4,7,8,9,10,12,14,17,19"
EXPECT_UNSEEN6 = {
    "candidate_v4": {"u6_ng2_slip10": {"all": ("正常完了", 1, 0, 0), "seeds": {L6: ("保留", 0, 0, 0)}},
                     "u6_swap_creep7": {"all": ("正常完了", 1, 0, 0), "seeds": {L6: ("保留", 0, 0, 0)}},
                     "u6_ng2_slip10q": {"any_of": [("正常完了", 1, 0, 0), ("保留", 0, 0, 0)]}},
    "candidate": {"u6_ng2_slip10": {"all": ("正常完了", 1, 0, 0)}, "u6_swap_creep7": {"all": ("正常完了", 1, 0, 0)},
                  "u6_ng2_slip10q": {"all": ("正常完了", 1, 0, 0)}},
    "baseline": {"u6_ng2_slip10": {"all": ("正常完了", 1, 0, 0)}, "u6_swap_creep7": {"all": ("誤完了", 1, 0, 1)},
                 "u6_ng2_slip10q": {"all": ("正常完了", 1, 0, 0)}},
}
# UNSEEN sets 7, 8, 9 (cell.UNSEEN7/8/9_CASES), run I-4 (~00:27, review C42l). Written BEFORE the candidate fix v6 and
# before the b_x / b_z world features; sha256 of json.dumps({"cases": {"7": UNSEEN7_CASES, "8": UNSEEN8_CASES,
# "9": UNSEEN9_CASES}, "expect": EXPECT_UNSEEN789}, sort_keys=True, ensure_ascii=False) into the night log first.
# Seeds 0..19; entry format as set 6. Methods:
#   candidate_v5 = the candidate as it is now (v5)
#   candidate    = v6 (planned): the 9 x 9 closing counts the outside of the picture as DARK (BORDER_CONSTANT 0), so no
#                  belt strip between a part and the picture's edge joins the blob. Nothing else changes.
#   baseline     = unchanged
# Facts used: A stops at exactly x 500 (+ creep / slip, no jitter); B at 420 +- 1.4; z: A 2 +- 1, B -3 +- 1. A strip of
# k px rows x 48 cols (or k cols x 80 rows) is >= 56.5 px = a "mark" for k >= 1 (k <= 4 px fills).
# Reasoning:
#   u7_edge59_ng2flip  A 539..579 -> strip rows 398-399 (96 px) -> A "marked". T2 predicted 420, B at 479: no match.
#                      v5: marked = [A] -> picks the GOOD A -> confirms -> false complete 1 0 1 / v6: no mark -> hold 0 0 0
#                      / baseline closes at 420: nothing within 15 -> false complete 1 0 0 (escape)
#   u7_zedge_flip      B z -59.5..-35.5 -> strip col 0 (80 px). T1 matches A (unmarked, flipped). v5: marked = [B] -> picks
#                      B -> false complete 1 0 1 / v6 hold 0 0 0 / baseline closes at 500 -> A -> ok 1 0 0
#   u7_topedge_flip    B 381.5..421.5 -> strip rows 0-2 (144 px). As above: v5 false complete 1 0 1 / v6 hold 0 0 0 /
#                      baseline ok 1 0 0
#   u7_edge59_ng2      A strip + B's real mark -> v5: 2 marked -> hold 0 0 0 / v6: marked = [B] -> ok 1 0 0 / baseline
#                      false complete 1 0 0
#   u8_flip_slip25     A at 525 (no match, cost >= 25), no mark anywhere -> v5 and v6 hold 0 0 0 / baseline: A 25 mm from
#                      500 -> nothing -> false complete 1 0 0
#   u8_flip_edge59     NG A strip-marked in v5 -> marked = [A] -> picks the NG part for a wrong reason -> ok 1 0 0 / v6:
#                      no mark -> hold 0 0 0 / baseline: B at 479 is about 21 mm from (500, 0) -> nothing -> false complete 1 0 0
#   u8_mark_at_edge    A 546..586 cut by the edge; mark rows 386-397, rows 398-399 bright below it. T1 matches B (486:
#                      cost about 17) or nothing. v5: erosion counts the outside as bright, the mark stays enclosed ->
#                      marked = [A] -> ok 1 0 0 / v6: erosion removes rows 396-399, the mark opens to the edge and is no
#                      longer inside the filled outline -> hold 0 0 0 (the price of v6) / baseline closes at 500: B about
#                      14.3 mm (taken on most seeds, wrong 1) or nothing -> false complete either way
#   u9_flip_zfar       B z 35.5..59.5; z 60 is the belt edge, not the picture's edge (dark background beyond) -> no
#                      strip -> v5 and v6 hold 0 0 0 / baseline ok 1 0 0
#   u9_flip_close_behind  B 438..478, A 480..520: 4 px gap filled by the closing -> one blob about 82 mm long -> outline
#                      check -> v5 and v6 hold 0 0 0 / baseline ok 1 0 0
#   u9_flip_cut_edge   A 544..584 (cut, no gap to the edge), mark down; B 484 -> v5 and v6 hold 0 0 0 / baseline closes
#                      at 500: B about 16.3 mm (taken on some seeds, wrong 1) or nothing -> false complete either way
_OK, _HOLD, _FC, _FCW = ("正常完了", 1, 0, 0), ("保留", 0, 0, 0), ("誤完了", 1, 0, 0), ("誤完了", 1, 0, 1)
EXPECT_UNSEEN789 = {
    "candidate_v5": {"u7_edge59_ng2flip": {"all": _FCW}, "u7_zedge_flip": {"all": _FCW}, "u7_topedge_flip": {"all": _FCW},
                     "u7_edge59_ng2": {"all": _HOLD}, "u8_flip_slip25": {"all": _HOLD}, "u8_flip_edge59": {"all": _OK},
                     "u8_mark_at_edge": {"all": _OK}, "u9_flip_zfar": {"all": _HOLD}, "u9_flip_close_behind": {"all": _HOLD},
                     "u9_flip_cut_edge": {"all": _HOLD}},
    "candidate": {"u7_edge59_ng2flip": {"all": _HOLD}, "u7_zedge_flip": {"all": _HOLD}, "u7_topedge_flip": {"all": _HOLD},
                  "u7_edge59_ng2": {"all": _OK}, "u8_flip_slip25": {"all": _HOLD}, "u8_flip_edge59": {"all": _HOLD},
                  "u8_mark_at_edge": {"all": _HOLD}, "u9_flip_zfar": {"all": _HOLD}, "u9_flip_close_behind": {"all": _HOLD},
                  "u9_flip_cut_edge": {"all": _HOLD}},
    "baseline": {"u7_edge59_ng2flip": {"all": _FC}, "u7_zedge_flip": {"all": _OK}, "u7_topedge_flip": {"all": _OK},
                 "u7_edge59_ng2": {"all": _FC}, "u8_flip_slip25": {"all": _FC}, "u8_flip_edge59": {"all": _FC},
                 "u8_mark_at_edge": {"any_of": [_FC, _FCW]}, "u9_flip_zfar": {"all": _OK},
                 "u9_flip_close_behind": {"all": _OK}, "u9_flip_cut_edge": {"any_of": [_FC, _FCW]}},
}
# UNSEEN set 10 (cell.UNSEEN10_CASES), run I-5 (copied 01:10:04; the comment said ~01:15 by mistake. Review C42n). Written BEFORE the candidate fix v7, before the
# flip_b / a_jit world features and before the scorer's false_done column; sha256 of json.dumps({"cases":
# UNSEEN10_CASES, "expect": EXPECT_UNSEEN10}, sort_keys=True, ensure_ascii=False) into the night log first. Seeds 0..19.
# Tuples: (outcome, commands, extra, wrong reject, escape, false_done). false_done = tracks declared complete whose own
# body is not in the bin at the end (planned scorer column; the outcome rule is NOT changed). Methods:
#   candidate_v6 = the candidate as it is now (v6)
#   candidate    = v7 (planned): when two or more NG tracks (top_mark) are open and in view, (a) a part is picked for a
#                  track only if EVERY open in-view track matches a blob one to one and the track's own blob is marked;
#                  (b) re-identification by the mark ("the only marked part") is not used. Otherwise hold. One NG track
#                  open: as v6.
#   baseline     = unchanged
# Facts used: A stops at x 500 + creep exactly (a_jit adds N(0, 1.5)); B at 420 +- 1.4 (b_x: exactly); z A 2 +- 1,
# B -3 +- 1. Match cost = |x - prediction| + |z|, within 20. The arm takes the nearest body within 15 mm.
#   u10_ng2_flip_a       A (T1) unmarked at 500, B (T2) marked at 420. v6: T1 matches A (unmarked); the only marked part
#                        is B -> re-identified -> B picked for T1 -> pick point empty, bin +1, belt 2 -> 1 -> T1 complete;
#                        T2: one blob (A) 80 mm off, no mark -> hold. A (T1) on the belt with T1 complete -> escape,
#                        false_done 1: hold 1 0 0 yes 1 / v7: 2 NG open, T1's blob unmarked -> hold T1; T2: 2 blobs vs
#                        1 open track -> hold: hold 0 0 0 no 0 / baseline: 500 -> A, 420 -> B: ok 2 0 0 no 0
#   u10_ng2_creep72_jit  A at 572 +- 1.5 (552..580 in view, about 26-28 mm: still a blob), mark at 582: mostly outside,
#                        seen on some seeds or not (not predicted per seed). B at 492. v6: T1 matches B (cost about 11),
#                        marked -> picks B for T1 -> complete; T2: blob A no match; A's mark seen -> re-identified -> A
#                        picked -> both in the bin, ok 2 0 0 no 0; not seen -> hold 1 0 0 yes 1 / v7: T1 -> B but T2 has
#                        no match -> hold T1; T2: 2 blobs vs 1 -> hold: hold 0 0 0 no 0 / baseline: 500 -> B (8.5 mm),
#                        420 -> nothing -> A (T1) on the belt: false complete 2 0 0 yes 1
#   u10_ng2_flip_b       A (T1) marked at 500, B (T2) unmarked at 420. v6: T1 -> A marked -> picked -> complete; T2: B
#                        matched, unmarked, no marked part -> hold (B on the belt, T2 held: no escape): hold 1 0 0 no 0 /
#                        v7: all open matched, A marked -> same: hold 1 0 0 no 0 / baseline ok 2 0 0 no 0
#   u10_ng2_b445_jit     A at 500 +- 1.5, B at 445 (cost to T2 about 28 > 20). v6: T1 -> A marked -> complete; T2: no
#                        match, the only marked part is B -> picked -> complete: ok 2 0 0 no 0 / v7: at T1, two NG open and
#                        T2 has no match -> hold T1; T2: 2 blobs vs 1 -> hold: hold 0 0 0 no 0 (the price of v7) /
#                        baseline: 500 -> A, 420 -> nothing (B 25 mm off) -> B (T2) on the belt: false complete 2 0 0 yes 1
#   u10_ng2_swap         A (T1) at 420, B (T2) at 500, both marked. v6: T1 -> B (marked) -> complete; T2 -> A -> complete;
#                        both in the bin at the end: ok 2 0 0 no 0 (the labels crossed; the end-state score cannot see it)
#                        / v7: all open matched, marked -> same / baseline: 500 -> B, 420 -> A: ok 2 0 0 no 0
_OK2, _H0, _H1, _H1E, _FC2E = (("正常完了", 2, 0, 0, False, 0), ("保留", 0, 0, 0, False, 0), ("保留", 1, 0, 0, False, 0),
                               ("保留", 1, 0, 0, True, 1), ("誤完了", 2, 0, 0, True, 1))
EXPECT_UNSEEN10 = {
    "candidate_v6": {"u10_ng2_flip_a": {"all": _H1E}, "u10_ng2_creep72_jit": {"any_of": [_OK2, _H1E]},
                     "u10_ng2_flip_b": {"all": _H1}, "u10_ng2_b445_jit": {"all": _OK2}, "u10_ng2_swap": {"all": _OK2}},
    "candidate": {"u10_ng2_flip_a": {"all": _H0}, "u10_ng2_creep72_jit": {"all": _H0}, "u10_ng2_flip_b": {"all": _H1},
                  "u10_ng2_b445_jit": {"all": _H0}, "u10_ng2_swap": {"all": _OK2}},
    "baseline": {"u10_ng2_flip_a": {"all": _OK2}, "u10_ng2_creep72_jit": {"all": _FC2E}, "u10_ng2_flip_b": {"all": _OK2},
                 "u10_ng2_b445_jit": {"all": _FC2E}, "u10_ng2_swap": {"all": _OK2}},
}
# UNSEEN set 11 (cell.UNSEEN11_CASES), run I-5 (copied 01:42:11; the comment said ~01:30 by mistake. Review C42n items 1-2). Written BEFORE the candidate fix v8; sha256
# of json.dumps({"cases": UNSEEN11_CASES, "expect": EXPECT_UNSEEN11}, sort_keys=True, ensure_ascii=False) into the night
# log first. Seeds 0..19; tuples as set 10. Methods:
#   candidate_v7 = the candidate as it is now (v7)
#   candidate    = v8 (planned): the guard counts the NG tracks NOT declared complete (held, out of view, at the edge
#                  included); with two or more, every open in-view track must match a blob one to one AND every open
#                  in-view NG track's blob must be marked; no re-identification by the only marked part. Else hold.
#   baseline     = unchanged
# Reasoning:
#   u11_ng2_b_past_end     A (T1) at 500, B (T2) at 600 (outside). v7: 1 blob vs 2 tracks -> hold T1; T2 alone: no match,
#                          the only marked part is A -> picked for T2 -> complete; B on the belt: hold 1 0 0 yes 1 / v8: T1
#                          held, 2 NG not complete -> T2 has no match -> hold: hold 0 0 0 no 0 / baseline: 500 -> A,
#                          420 -> nothing -> B (T2) on the belt: false complete 2 0 0 yes 1
#   u11_ng2_swap_flip_jit  B (T2, marked) at 500, A (T1, mark down) at ~420 +- 2.4. v7: T1 -> B marked, all matched ->
#                          B picked for T1 -> complete; T2 -> A unmarked -> hold: hold 1 0 0 yes 1 / v8: T2's blob is
#                          unmarked -> hold T1; T2: own blob unmarked -> hold: hold 0 0 0 no 0 / baseline: 500 -> B,
#                          420 -> A: ok 2 0 0 no 0
#   u11_ng2_ng_follower    T1 OK (A 500), T2 NG (B 420), T3 NG (C 340, out of view). v7: only T2 open in view -> B marked
#                          -> complete; T3 out of view -> hold: hold 1 0 0 no 0 / v8: 2 NG not complete (T2, T3) -> T1 and
#                          T2 matched, T2's blob marked -> same: hold 1 0 0 no 0 / baseline: 420 -> B, 340 -> C: ok 2 0 0
#   u11_ng2_creep90        A (T1) at 590 (10 mm in view, below half a part: no blob), B (T2) at ~510. v7: 1 blob vs 2 ->
#                          hold T1; T2 alone: no match, the only marked part is B (its own body) -> complete: hold 1 0 0 no 0
#                          / v8: T1 held, 2 NG not complete, T2 no match -> hold: hold 0 0 0 no 0 (the price of v8) /
#                          baseline: 500 -> B (about 10.4 mm) for T1, 420 -> nothing: false complete 2 0 0 yes 1
EXPECT_UNSEEN11 = {
    "candidate_v7": {"u11_ng2_b_past_end": {"all": _H1E}, "u11_ng2_swap_flip_jit": {"all": _H1E},
                     "u11_ng2_ng_follower": {"all": _H1}, "u11_ng2_creep90": {"all": _H1}},
    "candidate": {"u11_ng2_b_past_end": {"all": _H0}, "u11_ng2_swap_flip_jit": {"all": _H0},
                  "u11_ng2_ng_follower": {"all": _H1}, "u11_ng2_creep90": {"all": _H0}},
    "baseline": {"u11_ng2_b_past_end": {"all": _FC2E}, "u11_ng2_swap_flip_jit": {"all": _OK2},
                 "u11_ng2_ng_follower": {"all": _OK2}, "u11_ng2_creep90": {"all": _FC2E}},
}
# UNSEEN set 12 (cell.UNSEEN12_CASES), run I-6 (review C42-I6). Written BEFORE the candidate fix v9 and BEFORE the world
# fix (swap with a third / follower body crashed); sha256 of json.dumps({"cases": UNSEEN12_CASES, "expect":
# EXPECT_UNSEEN12}, sort_keys=True, ensure_ascii=False) into the night log first. Seeds 0..19; tuples as set 10. Methods:
#   candidate_v8 = the candidate as it is now (v8)
#   candidate    = v9 (planned): with two or more NG tracks not declared complete, pick only when EVERY one of them is
#                  still open AND predicted inside the view (none held, none out of view or at the edge), on top of the
#                  v8 rule (every open in-view track matches a blob one to one, every open in-view NG blob is marked).
#                  Else hold. One NG track not complete: as v6.
#   baseline     = unchanged
# Facts used: A stops at 500 exactly (+ slip / creep / a_jit); B at 420 + jitter, z -3 +- 1; a follower at 340 (or 320
# with follower_gap 100), z 1 +- 1; predictions 500 / 420 / 340 (320) minus A's start jitter. A blob needs half a part
# of area (5 mm of a part in view is not one). Match cost |dx| + |z| <= 20. The arm takes the nearest body within 15 mm.
#   u12_swap_b_far        A (T1, marked) at ~420, B (T2) at 625 (not in the picture). v8: 1 blob vs 2 -> hold T1; T2
#                         alone matches A (marked), 2 NG not complete but T1 is no longer open -> A picked for T2 ->
#                         complete: hold 1 0 0 escape 1 / v9: T1 held -> hold T2: hold 0 0 0 no 0 / baseline: 500 ->
#                         nothing, 420 -> A; B on the belt with T2 complete: false complete 2 0 0 escape 1
#   u12_swap_b_up_jit     as the first, B at 335 (upstream, out of the picture), A's x jitter 2.5: same three results
#   u12_follower_in_view  T1 OK (A 500), T2 NG (B at 333: out), T3 NG (C at ~415, marked, in view; T3 predicted 340 = out).
#                         v8: T1 -> A, T2 -> C (cost about 6), C marked -> C picked for T2 -> complete; T3 out -> hold:
#                         hold 1 0 0 escape 1 / v9: T3 not in view -> hold T2; T3 hold: hold 0 0 0 no 0 / baseline: 420
#                         -> C (about 5 mm), 340 -> B (about 7.6 mm): ok 2 0 0 no 0
# (review C42-I6 i6-3: "same picture" below is wrong; see the correction in cell.py above UNSEEN12_CASES)
#   u12_ng_follower_gap   T1 OK (A 500), T2 NG (B 420 marked), T3 NG at ~320 (out). v8: B picked for T2 (right) ->
#                         complete; T3 hold: hold 1 0 0 no 0 / v9: T3 not in view -> hold T2: hold 0 0 0 no 0 (the price
#                         of v9; same picture as u12_follower_in_view) / baseline: 420 -> B, 320 -> C: ok 2 0 0 no 0
#   u12_a_slid_out        A (T1) slid to 595 (no blob), B (T2, marked) at 420. v8: 1 blob vs 2 -> hold T1; T2 -> B
#                         (right) -> complete: hold 1 0 0 no 0 / v9: T1 held -> hold T2: hold 0 0 0 no 0 (the price of
#                         v9; same picture as u12_swap_b_far) / baseline: 500 -> nothing, 420 -> B; A on the belt with
#                         T1 complete: false complete 2 0 0 escape 1
#   u12_swap_third        only T1 NG; swap with a 3rd good body out of view. v8 = v9 (one NG track): T1 -> B unmarked,
#                         the only marked part is A -> re-identified -> A picked -> complete: ok 1 0 0 no 0 / baseline:
#                         500 -> B (good) -> wrong reject 1, A on the belt: false complete 1 0 1 escape 1
#   u12_ng2_swap_follower both NG swapped, a good follower out of view. v8 = v9: all open in-view tracks matched and marked
#                         -> B picked for T1, then T2 (only NG left) -> A: ok 2 0 0 no 0 / baseline: ok 2 0 0 no 0
_OK1, _FC1W = ("正常完了", 1, 0, 0, False, 0), ("誤完了", 1, 0, 1, True, 1)
EXPECT_UNSEEN12 = {
    "candidate_v8": {"u12_swap_b_far": {"all": _H1E}, "u12_swap_b_up_jit": {"all": _H1E},
                     "u12_follower_in_view": {"all": _H1E}, "u12_ng_follower_gap": {"all": _H1},
                     "u12_a_slid_out": {"all": _H1}, "u12_swap_third": {"all": _OK1}, "u12_ng2_swap_follower": {"all": _OK2}},
    "candidate": {"u12_swap_b_far": {"all": _H0}, "u12_swap_b_up_jit": {"all": _H0},
                  "u12_follower_in_view": {"all": _H0}, "u12_ng_follower_gap": {"all": _H0},
                  "u12_a_slid_out": {"all": _H0}, "u12_swap_third": {"all": _OK1}, "u12_ng2_swap_follower": {"all": _OK2}},
    "baseline": {"u12_swap_b_far": {"all": _FC2E}, "u12_swap_b_up_jit": {"all": _FC2E},
                 "u12_follower_in_view": {"all": _OK2}, "u12_ng_follower_gap": {"all": _OK2},
                 "u12_a_slid_out": {"all": _FC2E}, "u12_swap_third": {"all": _FC1W}, "u12_ng2_swap_follower": {"all": _OK2}},
}
# UNSEEN set 13 (cell.UNSEEN13_CASES), run I-7 (review C42-I6 i6-1 / i6-2 / i6-7). Written AFTER the world features
# (miss_from, stray_mark, the stray_bin + creep fix) and BEFORE the candidate fix v10; sha256 of json.dumps({"cases":
# UNSEEN13_CASES, "expect": EXPECT_UNSEEN13}, sort_keys=True, ensure_ascii=False) into the night log first. Seeds 0..19;
# tuples (outcome, commands, extra, wrong reject, escape, false_done) as set 10. Methods:
#   candidate_v9 = the candidate as it is now (v9)
#   candidate    = v10 (planned, i6-1 only): an NG track completed while two or more NG tracks were not complete at its
#                  pick is complete only provisionally (two marked NG parts that traded places look the same as two that
#                  did not, so the part taken may be the other track's body). If a later NG track is held, every
#                  provisional completion turns into a hold. Nothing else changes; i6-2 (re-identification by "the only
#                  marked part") is NOT fixed in v10, so v10 = v9 on the stray cases.
#   baseline     = unchanged
# Facts used: A stops at 500 exactly (+ creep / a_jit); B at 420 + jitter; T1 / T2 predicted at 500 / 420 minus A's start
# jitter; the 3rd body at 340 is out of view (340 + 20 + 10 <= 380). MAX_REPICK 2: a part still at the pick point is
# picked 3 times, then the track is held. extra = commands - NG tracks - misses, floored at 0. Baseline picks at z 0.
#   u13_ng2_swap_miss2       both NG, swapped: B (T2's body) at 500, A at ~420. v9: both marked, all matched -> T1 picks
#                            B at 500 -> fresh picture: empty, bin +1 -> T1 complete; T2 (now the only NG not complete)
#                            -> A -> 3 misses -> hold. A on the belt with T1 complete: hold 4 0 0 escape 1 / v10: T1
#                            turns into a hold: hold 4 0 0 no 0 / baseline: 500 -> B (done), 420 -> miss (done): A on
#                            the belt with T1 complete: false complete 2 0 0 escape 1
#   u13_ng2_miss2_creep4     no swap (control), all creep 4. v9: T1 -> A (right) complete, T2 -> B misses 3 -> hold:
#                            hold 4 0 0 no 0 / v10: T1 also held (the price: a right completion withdrawn; not in the
#                            tuple): hold 4 0 0 no 0 / baseline: T2 complete with B on the belt: false complete 2 0 0 escape 1
#   u13_ng2_swap_miss1_from3 swapped; the 1st command misses, the re-pick takes B, the 3rd command and later miss. v9:
#                            T1 complete (B), T2 -> A: cmds 3, 4, 5 miss -> hold: hold 5 0 0 escape 1 / v10: hold 5 0 0
#                            no 0 / baseline: cmd 1 misses (T1 "done"), cmd 2 at 420 takes A (T2 "done"); B on the
#                            belt with T2 complete: false complete 2 0 0 escape 1
#   u13_ng2_swap_third_miss2 as the first, + a 3rd good body (T3 OK) out of view, no a_jit: same three results
#   u13_mstray_b_up          one NG (A, mark down, at 500); B (OK) at 330, out of the picture; a MARKED stray at (425, 1)
#                            where T2 is predicted. 2 blobs, 2 tracks in view; T1 -> A unmarked; the only marked part
#                            is the stray -> re-identified -> stray picked -> complete. v9 = v10: false complete 1 0 1
#                            escape 1 / baseline: 500 -> A: ok 1 0 0 no 0
#   u13_mstray_b_far_creep3  as above with A and the stray at 503 / (416, -4), B at 645: same three results
#   u13_mstray_ng_seen       control: A (NG) mark up at 500; B at 335 (out); a marked stray at (428, -2). T1 -> A marked
#                            -> picked: ok 1 0 0 no 0 for all three
#   u13_bin_creep8           a good part already in the bin, everything creeps 8 mm (crashed before the world fix). T1
#                            -> A at 508 (cost about 10), bin 1 -> 2: ok 1 0 0 no 0 for all three (baseline: about 8 to
#                            11 mm from A, within 15)
# Also (not a table row): v10 gives the same tuple as v9 on every earlier case (dev, sets 1..12, old set 3) for seeds
# 0..19, since a provisional completion only turns into a hold when a hold already exists (outcome hold either way) and
# it only removes escape / false_done; v9 has none there. Checked in the run I-7 notes, not in this hash.
_H4E, _H4, _H5E, _H5 = ("保留", 4, 0, 0, True, 1), ("保留", 4, 0, 0, False, 0), ("保留", 5, 0, 0, True, 1), ("保留", 5, 0, 0, False, 0)
_FC2E1, _FC1WE, _OK1_13 = ("誤完了", 2, 0, 0, True, 1), ("誤完了", 1, 0, 1, True, 1), ("正常完了", 1, 0, 0, False, 0)
EXPECT_UNSEEN13 = {
    "candidate_v9": {"u13_ng2_swap_miss2": {"all": _H4E}, "u13_ng2_miss2_creep4": {"all": _H4},
                     "u13_ng2_swap_miss1_from3": {"all": _H5E}, "u13_ng2_swap_third_miss2": {"all": _H4E},
                     "u13_mstray_b_up": {"all": _FC1WE}, "u13_mstray_b_far_creep3": {"all": _FC1WE},
                     "u13_mstray_ng_seen": {"all": _OK1_13}, "u13_bin_creep8": {"all": _OK1_13}},
    "candidate": {"u13_ng2_swap_miss2": {"all": _H4}, "u13_ng2_miss2_creep4": {"all": _H4},
                  "u13_ng2_swap_miss1_from3": {"all": _H5}, "u13_ng2_swap_third_miss2": {"all": _H4},
                  "u13_mstray_b_up": {"all": _FC1WE}, "u13_mstray_b_far_creep3": {"all": _FC1WE},
                  "u13_mstray_ng_seen": {"all": _OK1_13}, "u13_bin_creep8": {"all": _OK1_13}},
    "baseline": {"u13_ng2_swap_miss2": {"all": _FC2E1}, "u13_ng2_miss2_creep4": {"all": _FC2E1},
                 "u13_ng2_swap_miss1_from3": {"all": _FC2E1}, "u13_ng2_swap_third_miss2": {"all": _FC2E1},
                 "u13_mstray_b_up": {"all": _OK1_13}, "u13_mstray_b_far_creep3": {"all": _OK1_13},
                 "u13_mstray_ng_seen": {"all": _OK1_13}, "u13_bin_creep8": {"all": _OK1_13}},
}
ALLOWED_KEYS = {"inspection": {"type", "t", "event_id", "track_id", "verdict", "x_at_inspection", "encoder_at_inspection", "defect_feature"},
                "conveyor": {"type", "t", "state", "encoder"},
                "image": {"type", "t_capture", "t_request", "image", "px_per_mm", "view_mm", "why"},
                "robot": {"type", "t", "cmd_id", "done"}}


def run_case(case, ctrl_name, seed=0, labels=("A", "B")):
    cell = Cell(ALL_CASES[case], seed, labels)
    ctrl = CONTROLLERS[ctrl_name]()
    steps = []
    batch = cell.start() + [cell.shoot("first")]
    for ob in batch:
        ctrl.feed(ob)
    act = None
    for _ in range(MAX_STEPS):
        act = ctrl.next_action()
        steps.append({"fed": batch, "action": act, "t": round(cell.t, 3)})   # world time of the decision (video, history)
        if act is None or "end" in act:
            break
        batch = [cell.pick(act["pick"])] if "pick" in act else [cell.shoot("reshoot")]
        for ob in batch:
            ctrl.feed(ob)
    decl = act["end"] if act and "end" in act else {}
    cell.finish(decl)
    return {"case": case, "ctrl": ctrl_name, "seed": seed, "steps": steps, "history": ctrl.history,
            "truth_log": cell.truth_log, "cmd_log": cell.cmd_log, "decl": decl,
            "score": score(cell.truth_log, cell.cmd_log)}


def replay(steps, ctrl_name):
    """feed the recorded observations to a fresh controller; return its actions (must equal the recorded ones)."""
    ctrl = CONTROLLERS[ctrl_name]()
    out = []
    for s in steps:
        for ob in s["fed"]:
            ctrl.feed(ob)
        out.append(ctrl.next_action())
    return out


def action_key(actions):
    return hashlib.sha256(json.dumps(actions, sort_keys=True, ensure_ascii=False).encode()).hexdigest()[:16]


def strip(ob):
    return {k: (f"<image {v.shape[1]}x{v.shape[0]}>" if isinstance(v, np.ndarray) else v) for k, v in ob.items()}


def sheet(r, path):
    """all pictures the controller got, side by side, with time and the next decision written under each."""
    tiles = []
    for s in r["steps"]:
        for ob in s["fed"]:
            if ob["type"] != "image":
                continue
            if ob.get("image") is None:
                t = np.full((400, 440), 255, np.uint8)
                cv2.putText(t, "NO FRAME", (110, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
                cap = "no frame"
            else:
                t = ob["image"].copy()
                cap = f"t={ob['t_capture']}"
            a = s["action"]
            nxt = "end" if a is None or "end" in a else ("pick" if "pick" in a else "reshoot")
            t = cv2.copyMakeBorder(t, 0, 50, 4, 4, cv2.BORDER_CONSTANT, value=255)
            cv2.putText(t, f"{cap} -> {nxt}", (8, t.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 1)
            tiles.append(t)
    if tiles:
        cv2.imwrite(path, np.hstack(tiles))


def run_all(seed=0):
    return {(c, k): run_case(c, k, seed) for c in CASES for k in ("baseline", "candidate")}


def write(results, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    lines = ["| case | method | outcome | wrong reject | escape | false_done | commands (extra) | really in the bin | time s | controller's declarations |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for (c, k), r in results.items():
        s = r["score"]
        dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
        lines.append(f"| {c} | {k} | {s['outcome']} | {s['wrong_reject']} | {'yes' if s['escape'] else ''} | {s['false_done'] or ''} | {s['commands']} ({s['extra_commands']}) | "
                     f"{', '.join(s['bin']) or '-'} | {s['cycle_s']} | {dd} |")
        hist = {"case": c, "case_setting": CASES[c], "case_version": CASE_VERSION, "limits": LIMITS if k == "candidate" else {},
                "method": k, "seed": r["seed"], "score": s, "declarations": r["decl"], "controller_history": r["history"],
                "steps": [{"t": st["t"], "fed": [strip(o) for o in st["fed"]], "action": st["action"]} for st in r["steps"]],
                "truth_log_for_scoring_only": r["truth_log"], "commands": r["cmd_log"]}
        with open(os.path.join(out_dir, f"history_{c}_{k}.json"), "w", encoding="utf-8") as f:
            json.dump(hist, f, ensure_ascii=False, indent=1, default=str)
        sheet(r, os.path.join(out_dir, f"pictures_{c}_{k}.png"))
    tot = {}
    for (c, k), r in results.items():
        tot.setdefault(k, {}).setdefault(r["score"]["outcome"], 0)
        tot[k][r["score"]["outcome"]] += 1
    lines += ["", "totals: " + "; ".join(f"{k} " + ", ".join(f"{o} {n}" for o, n in sorted(v.items())) for k, v in tot.items())]
    with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"wrote {out_dir}")


def selftest(results):
    fails = 0

    def check(ok, msg):
        nonlocal fails
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {msg}")

    print("== expected outcome per case (written before the first run) ==")
    for (c, k), r in results.items():
        o, n, x = EXPECT[k][c]
        s = r["score"]
        check(s["outcome"] == o and s["commands"] == n and s["extra_commands"] == x,
              f"{k:9s} {c:13s}: {s['outcome']} (expect {o}), commands {s['commands']} (expect {n}), extra {s['extra_commands']} (expect {x})")
    print("== truth leaks into the controller? ==")
    extra = set()
    for r in results.values():
        for st in r["steps"]:
            for ob in st["fed"]:
                extra |= {f"{ob['type']}.{k}" for k in set(ob) - ALLOWED_KEYS.get(ob["type"], set())}
    check(not extra, "observations carry only allowed keys" + (f"; unexpected {sorted(extra)}" if extra else ""))
    a, b = results[("normal", "candidate")], results[("ng_second", "candidate")]
    pa = [s["action"]["pick"] for s in a["steps"] if s["action"] and "pick" in s["action"]]
    pb = [s["action"]["pick"] for s in b["steps"] if s["action"] and "pick" in s["action"]]
    check(bool(pa and pb) and pa[0]["x"] > 480 and pb[0]["x"] < 440 and b["score"]["bin"] == ["B(NG)"],
          f"NG moved to the second body -> the pick moves with it (x {pa[0]['x'] if pa else '-'} -> {pb[0]['x'] if pb else '-'}), bin {b['score']['bin']}")
    for c in ALL_CASES:   # dev + unseen (review C42f: the unseen cases were not covered)
        r = results[(c, "candidate")] if (c, "candidate") in results else run_case(c, "candidate", 0)
        rec = [s["action"] for s in r["steps"]]
        rep = replay(r["steps"], "candidate")
        check(action_key(rec) == action_key(rep), f"{c}: replaying the recorded observations gives the same actions")
        r2 = run_case(c, "candidate", r["seed"], labels=("Q", "P"))
        check(action_key([s["action"] for s in r2["steps"]]) == action_key(rec), f"{c}: body names changed -> same commands")
    for c in ("normal", "swap_visible", "slip_large"):
        r = results[(c, "candidate")]
        s1, s2 = score(r["truth_log"], r["cmd_log"]), score(r["truth_log"], r["cmd_log"], flip_truth=True)
        check(s1["outcome"] == "正常完了" and s2["outcome"] == "誤完了" and s2["wrong_reject"] == 1,
              f"{c}: scorer's truth flipped -> score {s1['outcome']} -> {s2['outcome']} (wrong reject {s2['wrong_reject']}). "
              "Checks the SCORER only (same command log by construction)")
    back = [c for c in ALL_CASES for k in ("baseline", "candidate")
            for r in [results.get((c, k)) or run_case(c, k, 0)]
            if any(b["t"] < a["t"] for a, b in zip(r["steps"], r["steps"][1:]))]
    check(not back, "world time of the steps never goes backwards (video and history use it)" + (f"; back in {back}" if back else ""))
    same = [c for c in CASES for seed in (0, 1)
            if action_key([st["action"] for st in run_case(c, "candidate", seed)["steps"]])
            != action_key([st["action"] for st in run_case(c, "candidate_v1", seed)["steps"]])]
    check(not same, "run I-2 fix (out-of-view tracks not counted) leaves every development case's commands unchanged"
          + (f"; changed {same}" if same else ""))
    for c, k, w in (("u3_stray_at_pick", "baseline", 1), ("u3_stray_in_bin", "baseline", 0), ("u3_stray_in_bin", "candidate", 0)):
        s = run_case(c, k, 0)["score"]
        check(s["wrong_reject"] == w, f"scorer: {k} {c}: wrong reject {s['wrong_reject']} (expect {w}; an untracked body"
              " picked into the bin counts, one already in the bin does not)")
    print("== negative controls ==")
    r = run_case("normal", "all_hold")
    check(r["score"]["outcome"] != EXPECT["candidate"]["normal"][0], f"all-hold controller on the normal case: {r['score']['outcome']} -> fails the normal check")
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "controllers.py"), encoding="utf-8").read()
    imports = {n.names[0].name if isinstance(n, ast.Import) else n.module for n in ast.walk(ast.parse(src)) if isinstance(n, (ast.Import, ast.ImportFrom))}
    check(not ({"cell", "score", "run"} & imports), f"controllers.py imports only {sorted(imports)} (not the world or the scorer)")
    a = run_case("swap_visible", "candidate", 0)
    b = run_case("swap_visible", "candidate", 0)
    check(action_key([s["action"] for s in a["steps"]]) == action_key([s["action"] for s in b["steps"]]), "same seed -> same commands")
    print("== other seeds (jitter and image noise) ==")
    # seeds 1..19 since run I-3 (review C42j-8): with seeds 1..3 only, the v1..v4 mark misread (9 of 20 seeds) was missed
    for seed in range(1, 20):
        for k in ("baseline", "candidate"):
            bad = []
            for c in CASES:
                s = run_case(c, k, seed)["score"]
                if (s["outcome"], s["commands"], s["extra_commands"]) != EXPECT[k][c]:
                    bad.append(c)
            check(not bad, f"seed {seed} {k}: outcome and commands as expected" + (f" except {bad}" if bad else ""))
    print("== picture-edge sets 7-9 with the current candidate (v10 since run I-7), seeds 0..19 (run I-4, review C42l-7) ==")
    # not the pre-registered table (that is run.py unseen); a property of v6 on these 10 cases: no false complete, no
    # wrong reject, no escape. A hold that hides a "complete" on a body left on the belt (review C42l-2) shows as escape.
    bad = [(c, seed) for c in {**UNSEEN7_CASES, **UNSEEN8_CASES, **UNSEEN9_CASES} for seed in range(20)
           for s in [run_case(c, "candidate", seed)["score"]]
           if s["outcome"] == "誤完了" or s["wrong_reject"] or s["escape"] or s["false_done"]]
    check(not bad, "candidate on sets 7-9: no false complete, wrong reject, escape or false_done" + (f"; {bad}" if bad else ""))
    print("== two NG tracks (sets 10, 11, 12) with the current candidate (v10 since run I-7), seeds 0..19 (run I-5, review C42n; set 12 run I-6) ==")
    # a property of v7, not the pre-registered table (run.py unseen): no track is declared complete while its own body
    # stays on the belt, and no escape / wrong reject / false complete. v6 must fail it (else the check sees nothing).
    for k, want in (("candidate", True), ("candidate_v8", False), ("candidate_v7", False), ("candidate_v6", False)):
        bad = [(c, seed) for c in {**UNSEEN10_CASES, **UNSEEN11_CASES, **UNSEEN12_CASES} for seed in range(20)
               for s in [run_case(c, k, seed)["score"]]
               if s["outcome"] == "誤完了" or s["wrong_reject"] or s["escape"] or s["false_done"]]
        check((not bad) == want, f"{k} on sets 10-12: " + ("no false complete, wrong reject, escape or false_done" if want else
              f"the check catches the {k} error ({len(bad)} runs)") + (f"; {bad[:6]}" if bad and want else ""))
    print("== set 13 (run I-7, review C42-I6): a second pick that fails after a swap (miss_from), a marked stray, stray_bin + creep ==")
    # a property of v10, not the pre-registered table (run.py unseen). The miss_from cases: no completion is left on
    # another track's body. v9 must fail it (else the check sees nothing). The marked-stray cases are the i6-2 path,
    # NOT fixed in v10: the check only shows the world feature reaches it.
    miss13 = [c for c in UNSEEN13_CASES if "miss_from" in UNSEEN13_CASES[c]]
    for k, want in (("candidate", True), ("candidate_v9", False)):
        bad = [(c, seed) for c in miss13 for seed in range(20) for s in [run_case(c, k, seed)["score"]]
               if s["outcome"] == "誤完了" or s["wrong_reject"] or s["escape"] or s["false_done"]]
        check((not bad) == want, f"{k} on the set-13 miss_from cases: " + ("no false complete, wrong reject, escape or "
              "false_done" if want else f"the check catches the {k} error ({len(bad)} runs)") + (f"; {bad[:6]}" if bad and want else ""))
    s13 = [run_case(c, "candidate", seed)["score"]["outcome"] for c in ("u13_mstray_b_up", "u13_mstray_b_far_creep3")
           for seed in range(4)]
    check(s13.count("誤完了") == 8, f"world: a marked stray reaches the i6-2 path (candidate false complete {s13.count('誤完了')}/8;"
          " NOT fixed in v10, known)")
    s13b = run_case("u13_bin_creep8", "candidate", 0)["score"]
    check(s13b["outcome"] == "正常完了", f"world: stray_bin + creep runs (was a crash, i6-7): {s13b['outcome']}")
    fake = [(0.0, "tracks", {"T1": "A", "T2": "B"}),
            (3.0, "end", {"snapshot": {"bodies": [{"id": "A", "defect": True, "where": "belt"},
                                                  {"id": "B", "defect": True, "where": "bin"}], "bin_count": 1},
                          "declarations": {"T1": {"state": "complete"}, "T2": {"state": "hold"}}, "cycle_s": 3.0})]
    s0 = score(fake, [{"cmd_id": "C1"}])
    check(s0["outcome"] == "保留" and s0["false_done"] == 1 and s0["escape"],
          f"scorer on a hand-made record (T1 complete, its body on the belt; T2 hold, its body in the bin): false_done "
          f"{s0['false_done']}, escape {s0['escape']}")
    s1 = run_case("u10_ng2_flip_a", "candidate_v6", 0)["score"]
    check(s1["outcome"] == "保留" and s1["false_done"] == 1 and s1["escape"],
          f"scorer: a hold on T2 does not hide T1 declared complete with its body on the belt (false_done {s1['false_done']},"
          f" escape {s1['escape']}, outcome {s1['outcome']})")
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    return fails


def unseen(out_dir):
    """unseen cases (set 1 vs candidate_v1, set 2 vs all three) vs their pre-written expectations, then the ablation table."""
    os.makedirs(out_dir, exist_ok=True)
    lines = ["# 初見のケースと、片方ずつ外す比較", "",
             f"case version {CASE_VERSION}. 期待は流す前に書いた（run.py の EXPECT_UNSEEN）。食い違いは直さずに書く。",
             "sha256（頭 12 文字）：EXPECT_UNSEEN " + hashlib.sha256(json.dumps(EXPECT_UNSEEN, sort_keys=True, ensure_ascii=False).encode()).hexdigest()[:12]
             + "、" + "、".join(f + " " + hashlib.sha256(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f), "rb").read()).hexdigest()[:12]
                                for f in ("cell.py", "controllers.py", "score.py", "run.py")), "",
             "## 初見のケース（乱数種 0〜3）", "",
             "| case | method | seed | outcome | expect | commands (extra) | expect | bin | declarations (not pass) |",
             "|---|---|---|---|---|---|---|---|---|"]
    mism = 0
    # EXPECT_UNSEEN was written for the I-1 candidate, which is candidate_v1 since run I-2 (the fix of I-2 changes
    # u_third_part; that change is tuning on a case already seen, so it is shown apart below, not as evidence).
    for c in UNSEEN_CASES:
        for k in ("baseline", "candidate_v1"):
            o, n, x = EXPECT_UNSEEN["candidate" if k == "candidate_v1" else k][c]
            for seed in (0, 1, 2, 3):
                r = run_case(c, k, seed)
                s = r["score"]
                ok = (s["outcome"], s["commands"], s["extra_commands"]) == (o, n, x)
                mism += not ok
                if seed == 0 or not ok:
                    dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
                    lines.append(f"| {c} | {k} | {seed} | {s['outcome']}{'' if ok else ' **≠**'} | {o} | {s['commands']} ({s['extra_commands']}) | "
                                 f"{n} ({x}) | {', '.join(s['bin']) or '-'} | {dd} |")
    lines += ["", f"期待との食い違い: {mism} 件（7 ケース × 2 やり方 × 乱数種 4。乱数種 1〜3 は食い違いだけ行にした）", ""]
    lines += ["### 2 回目の直し（視野の外の追跡番号を数に入れない）の後の候補で、初見 1 の 7 ケース（調整。証拠にしない）", "",
              "| case | seed 0..3 outcome (commands) | declarations (seed 0, not pass) |", "|---|---|---|"]
    for c in UNSEEN_CASES:
        rs = [run_case(c, "candidate", seed) for seed in (0, 1, 2, 3)]
        dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(rs[0]["decl"].items()) if d["state"] != "pass") or "-"
        lines.append(f"| {c} | " + ", ".join(f"{r['score']['outcome']} ({r['score']['commands']})" for r in rs) + f" | {dd} |")
    lines += ["", "## 初見 2（2 回目。直す前に期待を書いた。`EXPECT_UNSEEN2`、乱数種 0〜3）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN2_CASES, "expect": EXPECT_UNSEEN2}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録に先に書いた値と同じか見る）", "",
              "| case | method | seed | outcome | expect | commands (extra) | expect | bin | declarations (not pass) |",
              "|---|---|---|---|---|---|---|---|---|"]
    mism2 = 0
    # EXPECT_UNSEEN2["candidate"] was written for the first I-2 fix, which is candidate_v2 since the v3 fix (C42h-1)
    for c in UNSEEN2_CASES:
        for k in ("baseline", "candidate_v1", "candidate_v2"):
            o, n, x = EXPECT_UNSEEN2["candidate" if k == "candidate_v2" else k][c]
            for seed in (0, 1, 2, 3):
                r = run_case(c, k, seed)
                s = r["score"]
                ok = (s["outcome"], s["commands"], s["extra_commands"]) == (o, n, x)
                mism2 += not ok
                if seed == 0 or not ok:
                    dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
                    lines.append(f"| {c} | {k} | {seed} | {s['outcome']}{'' if ok else ' **≠**'} | {o} | {s['commands']} ({s['extra_commands']}) | "
                                 f"{n} ({x}) | {', '.join(s['bin']) or '-'} | {dd} |")
    lines += ["", f"期待との食い違い: {mism2} 件（4 ケース × 3 やり方 × 乱数種 4）", ""]
    mism += mism2
    lines += ["", "### v3（点検 C42h-1 の後の直し）の候補で、初見 2 の 4 ケース（調整。証拠にしない）", "",
              "| case | seed 0..3 outcome (commands) |", "|---|---|"]
    for c in UNSEEN2_CASES:
        rs = [run_case(c, "candidate", seed) for seed in (0, 1, 2, 3)]
        lines.append(f"| {c} | " + ", ".join(f"{r['score']['outcome']} ({r['score']['commands']})" for r in rs) + " |")
    lines += ["", "## 旧 初見 3 → 開発用に移した組（初見ではない。2026-11-19 列 I の 3 回目）", "",
              "初めて流した後に世界の側（cell.py）を 2 か所直してから食い違い 0 になった。結果を見た後に世界を直したので、"
              "初見の集計から外す（下の食い違いは「開発用に移した組」として別に数える）。v3 の期待は candidate_v3 で見る。"
              "代わりの初見の組は初見 5。", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN3_CASES, "expect": EXPECT_UNSEEN3}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 23:12 の節の値と同じか見る）", "",
              "| case | method | seed | outcome | expect | commands (extra) | expect | wrong reject | bin | declarations (not pass) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    mism3 = 0
    for c in UNSEEN3_CASES:
        for k in ("baseline", "candidate_v2", "candidate_v3"):     # EXPECT_UNSEEN3["candidate"] was written for v3
            o, n, x = EXPECT_UNSEEN3["candidate" if k == "candidate_v3" else k][c]
            for seed in (0, 1, 2, 3):
                r = run_case(c, k, seed)
                s = r["score"]
                ok = (s["outcome"], s["commands"], s["extra_commands"]) == (o, n, x)
                mism3 += not ok
                if seed == 0 or not ok:
                    dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
                    lines.append(f"| {c} | {k} | {seed} | {s['outcome']}{'' if ok else ' **≠**'} | {o} | {s['commands']} ({s['extra_commands']}) | "
                                 f"{n} ({x}) | {s['wrong_reject']} | {', '.join(s['bin']) or '-'} | {dd} |")
    lines += ["", f"期待との食い違い: {mism3} 件（4 ケース × 3 やり方 × 乱数種 4。開発用に移した組なので、初見の集計に入れない）", ""]
    lines += ["## 初見 4（点検 C42h-4 から。v2 の境目の余裕。制御は直さない。`EXPECT_UNSEEN4`、乱数種 0〜3）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN4_CASES, "expect": EXPECT_UNSEEN4}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 23:19 の節の値と同じか見る）", "",
              "| case | method | seed | outcome | expect | commands (extra) | expect | wrong reject | bin | declarations (not pass) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    mism4 = 0
    for c in UNSEEN4_CASES:
        for k in ("baseline", "candidate_v1", "candidate_v3"):     # EXPECT_UNSEEN4["candidate"] was written for v3
            o, n, x = EXPECT_UNSEEN4["candidate" if k == "candidate_v3" else k][c]
            for seed in (0, 1, 2, 3):
                r = run_case(c, k, seed)
                s = r["score"]
                ok = (s["outcome"], s["commands"], s["extra_commands"]) == (o, n, x)
                mism4 += not ok
                if seed == 0 or not ok:
                    dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
                    lines.append(f"| {c} | {k} | {seed} | {s['outcome']}{'' if ok else ' **≠**'} | {o} | {s['commands']} ({s['extra_commands']}) | "
                                 f"{n} ({x}) | {s['wrong_reject']} | {', '.join(s['bin']) or '-'} | {dd} |")
    lines += ["", f"期待との食い違い: {mism4} 件（4 ケース × 3 やり方 × 乱数種 4）", ""]
    mism += mism4
    lines += ["## 初見 5（列 I の 3 回目。重なる部品。v4 の直しの前に期待を書いた。`EXPECT_UNSEEN5`、乱数種 0〜3。誤排除も期待に入れた）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN5_CASES, "expect": EXPECT_UNSEEN5}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 23:42 の節の値と同じか見る）", "",
              "| case | method | seed | outcome | expect | commands (extra) | expect | wrong reject | expect | bin | declarations (not pass) |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    mism5 = 0
    for c in UNSEEN5_CASES:
        for k in ("baseline", "candidate_v3", "candidate_v4"):     # EXPECT_UNSEEN5["candidate"] was written for v4
            o, n, x, w = EXPECT_UNSEEN5["candidate" if k == "candidate_v4" else k][c]
            for seed in (0, 1, 2, 3):
                r = run_case(c, k, seed)
                s = r["score"]
                ok = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"]) == (o, n, x, w)
                mism5 += not ok
                if seed == 0 or not ok:
                    dd = "; ".join(f"{t} {d['state']}: {d['why']}" for t, d in sorted(r["decl"].items()) if d["state"] != "pass") or "-"
                    lines.append(f"| {c} | {k} | {seed} | {s['outcome']}{'' if ok else ' **≠**'} | {o} | {s['commands']} ({s['extra_commands']}) | "
                                 f"{n} ({x}) | {s['wrong_reject']} | {w} | {', '.join(s['bin']) or '-'} | {dd} |")
    lines += ["", f"期待との食い違い: {mism5} 件（4 ケース × 3 やり方 × 乱数種 4）", ""]
    mism += mism5
    lines += ["## 初見 6（列 I の 3 回目。印の読み落とし。v5 の直しの前に期待を書いた。`EXPECT_UNSEEN6`、乱数種 0〜19）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN6_CASES, "expect": EXPECT_UNSEEN6}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 00:04 の節の値と同じか見る）", "",
              "| case | method | outcome (commands, extra, wrong reject) over seeds 0..19 | mismatching seeds |", "|---|---|---|---|"]
    mism6 = 0
    for c in UNSEEN6_CASES:     # EXPECT_UNSEEN6["candidate"] was written for v5 (candidate_v5 since run I-4)
        for k in ("baseline", "candidate_v4", "candidate_v5"):
            e = EXPECT_UNSEEN6["candidate" if k == "candidate_v5" else k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"])
                if "any_of" in e:
                    want_ok = got in [tuple(t) for t in e["any_of"]]
                else:
                    want = e["all"]
                    for ks, t in e.get("seeds", {}).items():
                        if seed in [int(v) for v in ks.split(",")]:
                            want = t
                    want_ok = got == tuple(want)
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]})", []).append(seed)
                if not want_ok:
                    bad.append(seed)
            mism6 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism6} 件（3 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism6
    v6_6 = [(c, seed) for c in UNSEEN6_CASES for seed in range(20)
            if run_case(c, "candidate_v6", seed)["score"]["outcome"] != "正常完了"]
    lines += [f"v6（列 I の 4 回目の直し）で初見 6 を流すと、v5 の期待（60 回とも正常完了）と違う回: {len(v6_6)} 件"
              f"{' ' + str(v6_6) if v6_6 else ''}（v6 は初見 6 を見た後の版。数えない）", ""]
    lines += ["## 初見 7・8・9（列 I の 4 回目。視野の端。v6 の直しの前に期待を書いた。`EXPECT_UNSEEN789`、乱数種 0〜19、誤排除も）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": {"7": UNSEEN7_CASES, "8": UNSEEN8_CASES, "9": UNSEEN9_CASES},
                                           "expect": EXPECT_UNSEEN789}, sort_keys=True, ensure_ascii=False).encode()).hexdigest()[:12]
              + "（夜の記録の 00:29 の節の値と同じか見る）", "",
              "| case | method | outcome (commands, extra, wrong reject) over seeds 0..19 | mismatching seeds |", "|---|---|---|---|"]
    mism7 = 0
    for c in {**UNSEEN7_CASES, **UNSEEN8_CASES, **UNSEEN9_CASES}:
        for k in ("baseline", "candidate_v5", "candidate_v6"):     # EXPECT_UNSEEN789["candidate"] was written for v6
            e = EXPECT_UNSEEN789["candidate" if k == "candidate_v6" else k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"])
                ok = got in [tuple(t) for t in e["any_of"]] if "any_of" in e else got == tuple(e["all"])
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]})", []).append(seed)
                if not ok:
                    bad.append(seed)
            mism7 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism7} 件（10 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism7
    v7_789 = [(c, seed) for c in {**UNSEEN7_CASES, **UNSEEN8_CASES, **UNSEEN9_CASES} for seed in range(20)
              if action_key([st["action"] for st in run_case(c, "candidate", seed)["steps"]])
              != action_key([st["action"] for st in run_case(c, "candidate_v6", seed)["steps"]])]
    lines += [f"いまの候補（v10。列 I の 7 回目の直し。点検 C42-I7 の i7-6 で版の名を直した）で初見 7〜9 を流すと、v6 と指令が違う回: {len(v7_789)} 件"
              f"{' ' + str(v7_789) if v7_789 else ''}（初見 7〜9 を見た後の版。数えない）", ""]
    lines += ["## 初見 10（列 I の 5 回目。2 個とも NG。v7 の直しの前に期待を書いた。`EXPECT_UNSEEN10`、乱数種 0〜19、誤排除・流出・false_done も）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN10_CASES, "expect": EXPECT_UNSEEN10}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 01:10 の節の値と同じか見る）", "",
              "false_done = 完了と宣言したのに、その番号の部品が箱に無い追跡番号の数（5 回目に採点に足した列。結果の決め方は変えていない）。", "",
              "| case | method | outcome (commands, extra, wrong reject, escape, false_done) over seeds 0..19 | mismatching seeds |",
              "|---|---|---|---|"]
    mism10 = 0
    for c in UNSEEN10_CASES:
        for k in ("baseline", "candidate_v6", "candidate_v7"):     # EXPECT_UNSEEN10["candidate"] was written for v7
            e = EXPECT_UNSEEN10["candidate" if k == "candidate_v7" else k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"], bool(s["escape"]), s["false_done"])
                ok = got in [tuple(t) for t in e["any_of"]] if "any_of" in e else got == tuple(e["all"])
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]}, {'escape' if got[4] else '-'}, {got[5]})", []).append(seed)
                if not ok:
                    bad.append(seed)
            mism10 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism10} 件（5 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism10
    v8_10 = {c: [run_case(c, "candidate_v8", seed)["score"]["outcome"] + f" {run_case(c, 'candidate_v8', seed)['score']['commands']}"
                 for seed in range(20)] for c in UNSEEN10_CASES}
    lines += ["v8（5 回目の 2 つ目の直し）で初見 10 を流すと（v8 は初見 10 を見た後の版。数えない）: "
              + "; ".join(f"{c} " + ", ".join(f"{g} ×{n}" for g, n in sorted({g: v.count(g) for g in v}.items()))
                          for c, v in v8_10.items()), ""]
    lines += ["## 初見 11（列 I の 5 回目の途中。点検 C42n の 1・2 の後、v8 の直しの前に期待を書いた。`EXPECT_UNSEEN11`、乱数種 0〜19）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN11_CASES, "expect": EXPECT_UNSEEN11}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 01:42 の節の値と同じか見る）。"
              "仕組みは点検役が見つけた形と同じ（値は違う）。証拠としては弱い。", "",
              "| case | method | outcome (commands, extra, wrong reject, escape, false_done) over seeds 0..19 | mismatching seeds |",
              "|---|---|---|---|"]
    mism11 = 0
    for c in UNSEEN11_CASES:
        for k in ("baseline", "candidate_v7", "candidate_v8"):     # EXPECT_UNSEEN11["candidate"] was written for v8
            e = EXPECT_UNSEEN11["candidate" if k == "candidate_v8" else k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"], bool(s["escape"]), s["false_done"])
                ok = got in [tuple(t) for t in e["any_of"]] if "any_of" in e else got == tuple(e["all"])
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]}, {'escape' if got[4] else '-'}, {got[5]})", []).append(seed)
                if not ok:
                    bad.append(seed)
            mism11 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism11} 件（4 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism11
    v9_1011 = {c: [run_case(c, "candidate_v9", seed)["score"] for seed in range(20)] for c in {**UNSEEN10_CASES, **UNSEEN11_CASES}}
    lines += ["v9（6 回目の直し）で初見 10・11 を流すと（v9 は初見 10・11 を見た後の版。数えない）: "
              + "; ".join(f"{c} " + ", ".join(f"{g} ×{n}" for g, n in sorted({g: v.count(g) for g in v}.items()))
                          for c, v in {c: [f"{s['outcome']} {s['commands']}{' escape' if s['escape'] else ''}"
                                           f"{' false_done ' + str(s['false_done']) if s['false_done'] else ''}" for s in ss]
                                       for c, ss in v9_1011.items()}.items()), ""]
    lines += ["## 初見 12（列 I の 6 回目。点検 C42n の 2 回目の後、v9 の直しと世界の直しの前に期待を書いた。`EXPECT_UNSEEN12`、乱数種 0〜19）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN12_CASES, "expect": EXPECT_UNSEEN12}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 03:55 の節の値と同じか見る）。"
              "仕組みは点検役が見つけた形と同じ（値は違う）。証拠としては弱い。", "",
              "| case | method | outcome (commands, extra, wrong reject, escape, false_done) over seeds 0..19 | mismatching seeds |",
              "|---|---|---|---|"]
    mism12 = 0
    for c in UNSEEN12_CASES:
        for k in ("baseline", "candidate_v8", "candidate_v9"):     # EXPECT_UNSEEN12["candidate"] was written for v9
            e = EXPECT_UNSEEN12["candidate" if k == "candidate_v9" else k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"], bool(s["escape"]), s["false_done"])
                ok = got in [tuple(t) for t in e["any_of"]] if "any_of" in e else got == tuple(e["all"])
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]}, {'escape' if got[4] else '-'}, {got[5]})", []).append(seed)
                if not ok:
                    bad.append(seed)
            mism12 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism12} 件（7 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism12
    lines += ["## 初見 13（列 I の 7 回目。点検 C42-I6 の i6-1・i6-2・i6-7 の後、世界に miss_from・stray_mark を足し、v10 の直しの前に期待を書いた。`EXPECT_UNSEEN13`、乱数種 0〜19）", "",
              "sha256 of the pre-registration (cases + expectations): "
              + hashlib.sha256(json.dumps({"cases": UNSEEN13_CASES, "expect": EXPECT_UNSEEN13}, sort_keys=True,
                                          ensure_ascii=False).encode()).hexdigest()[:12] + "（夜の記録の 04:32 の節の値と同じか見る）。"
              "仕組みは点検役が見つけた形と同じ（値は違う）。証拠としては弱い。v10 は i6-2 を直さない（印のある判定の無い部品の 2 ケースは誤完了のままと期待に書いた）。", "",
              "| case | method | outcome (commands, extra, wrong reject, escape, false_done) over seeds 0..19 | mismatching seeds |",
              "|---|---|---|---|"]
    mism13 = 0
    for c in UNSEEN13_CASES:
        for k in ("baseline", "candidate_v9", "candidate"):     # EXPECT_UNSEEN13["candidate"] was written for v10
            e = EXPECT_UNSEEN13[k][c]
            seen, bad = {}, []
            for seed in range(20):
                s = run_case(c, k, seed)["score"]
                got = (s["outcome"], s["commands"], s["extra_commands"], s["wrong_reject"], bool(s["escape"]), s["false_done"])
                ok = got in [tuple(t) for t in e["any_of"]] if "any_of" in e else got == tuple(e["all"])
                seen.setdefault(f"{got[0]} ({got[1]}, {got[2]}, {got[3]}, {'escape' if got[4] else '-'}, {got[5]})", []).append(seed)
                if not ok:
                    bad.append(seed)
            mism13 += len(bad)
            lines.append(f"| {c} | {k} | " + "; ".join(f"{g}: seeds {','.join(map(str, v))}" for g, v in seen.items())
                         + f" | {','.join(map(str, bad)) or '-'} |")
    lines += ["", f"期待との食い違い: {mism13} 件（8 ケース × 3 やり方 × 乱数種 20）", ""]
    mism += mism13
    lines += [f"初見の組（1・2・4・5・6・7・8・9・10・11・12・13）の期待との食い違いの合計: {mism} 件。開発用に移した旧 初見 3: {mism3} 件（別に数える）。", ""]
    lines += ["## 片方ずつ外す比較（開発用 16 + 初見 7 + 初見 2・旧 3・4・5 の 4 ケースずつ + 初見 6 の 3 + 初見 7〜9 の 10 + 初見 10 の 5 + 初見 11 の 4 + 初見 12 の 7 + 初見 13 の 8、乱数種 0〜3 の合計。no_track・no_reshoot はいまの候補 v10 から外す。合計は入れ替わりを隠すので、下のケースごとの表で読む）", "",
              "no_track = 候補から追跡（撮った絵で数・位置・印を確かめる）を外す。視野の判定・塊の大きさ・数の確かめもまとめて外れる（TRACK=False はその前で返る）。no_reshoot = 候補から撮り直しを外す。", "",
              "| method | cases | 正常完了 | 保留 | 誤完了 | wrong reject (bodies) | escape | false_done (tracks) | commands (extra) |", "|---|---|---|---|---|---|---|---|---|"]
    per_case = {}
    for group, names in (("dev", list(CASES)), ("unseen", list(UNSEEN_CASES)), ("unseen2", list(UNSEEN2_CASES)),
                         ("dev_moved (旧 初見 3)", list(UNSEEN3_CASES)), ("unseen4", list(UNSEEN4_CASES)),
                         ("unseen5", list(UNSEEN5_CASES)), ("unseen6", list(UNSEEN6_CASES)),
                         ("unseen7-9", list(UNSEEN7_CASES) + list(UNSEEN8_CASES) + list(UNSEEN9_CASES)),
                         ("unseen10", list(UNSEEN10_CASES)), ("unseen11", list(UNSEEN11_CASES)),
                         ("unseen12", list(UNSEEN12_CASES)), ("unseen13", list(UNSEEN13_CASES))):
        for k in METHODS4:
            tot = {"正常完了": 0, "保留": 0, "誤完了": 0, "記録なし": 0}
            wr = esc = fd = cmd = ext = 0
            for c in names:
                for seed in (0, 1, 2, 3):
                    s = run_case(c, k, seed)["score"]
                    tot[s["outcome"]] += 1
                    wr += s["wrong_reject"]
                    esc += bool(s["escape"])
                    fd += s["false_done"]
                    cmd += s["commands"]
                    ext += s["extra_commands"]
                    if seed == 0:
                        per_case.setdefault(c, {})[k] = s["outcome"]
            lines.append(f"| {k} | {group} {len(names)}×4 | {tot['正常完了']} | {tot['保留']} | {tot['誤完了']} | {wr} | {esc} | {fd} | {cmd} ({ext}) |")
    lines += ["", "## ケースごと（乱数種 0）", "", "| case | " + " | ".join(METHODS4) + " |", "|---|" + "---|" * len(METHODS4)]
    for c, d in per_case.items():
        tag = " (初見 1)" if c in UNSEEN_CASES else " (初見 2)" if c in UNSEEN2_CASES else " (旧 初見 3・開発用)" if c in UNSEEN3_CASES else " (初見 4)" if c in UNSEEN4_CASES else " (初見 5)" if c in UNSEEN5_CASES else " (初見 6)" if c in UNSEEN6_CASES else " (初見 7)" if c in UNSEEN7_CASES else " (初見 8)" if c in UNSEEN8_CASES else " (初見 9)" if c in UNSEEN9_CASES else " (初見 10)" if c in UNSEEN10_CASES else " (初見 11)" if c in UNSEEN11_CASES else " (初見 12)" if c in UNSEEN12_CASES else " (初見 13)" if c in UNSEEN13_CASES else ""
        lines.append(f"| {c}{tag} | " + " | ".join(d[k] for k in METHODS4) + " |")
    with open(os.path.join(out_dir, "unseen_and_ablation.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"wrote {out_dir}")
    return mism


class _Tee:
    """print to the screen and to a file (run I-4: the selftest output is written to selftest.txt, not copied by hand)."""

    def __init__(self, path, out):
        self.f, self.out = open(path, "a", encoding="utf-8"), out

    def write(self, s):
        self.out.write(s)
        self.f.write(s)

    def flush(self):
        self.out.flush()
        self.f.flush()


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(sys.argv) > 1 and sys.argv[1] == "unseen":
        unseen(os.path.join(ROOT, "outputs", "industrial", stamp + "_unseen"))
        return
    out_dir = os.path.join(ROOT, "outputs", "industrial", stamp)
    os.makedirs(out_dir, exist_ok=True)
    is_test = len(sys.argv) > 1 and sys.argv[1] == "selftest"
    if is_test:     # stdout AND stderr go to selftest.txt (review C42l-7: a traceback must not leave a cut-off file)
        path = os.path.join(out_dir, "selftest.txt")
        sys.stdout, sys.stderr = _Tee(path, sys.stdout), _Tee(path, sys.stderr)
        print("selftest " + stamp + "; sha256 (first 12): " + ", ".join(
            f + " " + hashlib.sha256(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f), "rb").read()).hexdigest()[:12]
            for f in ("cell.py", "controllers.py", "score.py", "run.py"))
            + f"; python {sys.version.split()[0]}, numpy {np.__version__}, opencv {cv2.__version__}")
    fails = None
    try:
        results = run_all()
        write(results, out_dir)
        if is_test:
            fails = selftest(results)
    finally:
        if is_test:
            print("SELFTEST STATUS:", "OK" if fails == 0 else "CRASHED (see the traceback above)" if fails is None else f"FAILED ({fails})")
            print(f"wrote {os.path.join(out_dir, 'selftest.txt')}")
            for t in (sys.stdout, sys.stderr):
                t.flush()
                t.f.close()
            sys.stdout, sys.stderr = sys.stdout.out, sys.stderr.out
    if is_test:
        sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
