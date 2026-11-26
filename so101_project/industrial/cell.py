r"""Fictional reject cell (industrial plan step 1): the WORLD side. It holds the truth and makes observations.

Two look-alike parts ride a conveyor. The inspection app (upstream, simulated) issues one event per part when it
passes x = 0: event id, tracking number, verdict (NG / OK) and, for NG, how the defect shows from above ("top_mark":
a small dark mark on the top face). The conveyor stops when the lead part reaches the pick area; a top camera sees
the pick area and the reject bin; the arm (abstract, not SO-101 kinematics yet) picks what lies at the commanded
point and drops it into the bin, then sends a done signal.

The controller (controllers.py) gets ONLY the observation dicts this file emits (inspection events, conveyor
signals, images with their capture time, robot done signals). Truth (which body is which, where it really is, what
is in the bin) stays here and goes to score.py through `truth_log`. The controller must not import this file.

Units: mm, s. Frame: +x = conveyor direction, z = across the belt, origin = inspection point on the belt centre line.
Faults (a case is a dict; see CASES): swap (bodies trade places after inspection), flip (the NG body lies mark-down),
miss (the arm closes on air n times), image_missing (re-shoots return nothing), stale (re-shoots return the last
picture taken before the pick), dup_event (the NG event arrives twice), slip (a body slides along the belt).
"""

import copy

import numpy as np

BELT_SPEED = 100.0                 # mm/s
PART = (40.0, 24.0)                # length along x, width along z [mm]
MARK_R = 3.0                       # defect mark radius [mm], at part-local (+10, 0)
GAP = 80.0                         # second body follows the first by this much [mm]
STOP_X = 500.0                     # conveyor stops when the lead body reaches this x
VIEW = (380.0, 580.0, -60.0, 160.0)   # top camera: x0, x1, z0, z1 [mm]. Belt is z -60..60, bin is z 80..160
BELT_Z = (-60.0, 60.0)
BIN = (420.0, 540.0, 80.0, 160.0)  # reject bin area seen by the camera
PX_PER_MM = 2.0
GRAB_R = 15.0                      # the arm takes a body whose centre is within this of the commanded point
T_PICK = 1.0                       # arm cycle [s]
T_SHOT = 0.2                       # camera latency [s]

CASES = {
    "normal": {},
    "swap_visible": {"swap": True},
    "swap_hidden": {"swap": True, "flip": True},
    "miss_once": {"miss": 1},
    "miss_always": {"miss": 99},
    "image_missing": {"image_missing": True},
    "stale_image": {"stale": True},
    "dup_event": {"dup_event": True},
    "dup_restamped": {"dup_restamped": True},
    "slip_small": {"slip": 8.0},
    "slip_large": {"slip": 45.0},
    # added after review C42d (expectations written before running them):
    "flip_only": {"flip": True},                 # NG body lies mark-down, nothing else wrong
    "ng_second": {"ng_second": True},            # the SECOND body is the NG one (mirrors the order)
    "first_missing": {"first_missing": True},    # the camera gives no frame at all
    "miss_push": {"miss_push": 20.0},            # the arm misses and pushes the NG body 20 mm along x
    "dup_conflict": {"dup_conflict": True},      # the NG event is re-sent with verdict OK (same event id)
}
# UNSEEN cases (night of 2026-11-19, run I-1): not used to tune the controller. Their expectations are in run.py
# (EXPECT_UNSEEN), written before these world features existed. Kept apart from CASES so tuning never sees them.
UNSEEN_CASES = {
    "u_third_part": {"third": True},             # a 3rd (good) body follows by GAP; it is outside the view at the stop
    "u_swap_offset": {"swap": True, "creep": 12.0},   # swap, then both bodies creep +12 mm (not an exact exchange)
    "u_stale_restamp": {"stale_restamp": True},  # re-shoots return the pre-pick frame, stamped with the current time
    "u_arm_in_view": {"arm_in_view": True},      # the arm's wrist shows in every picture (bright block at the belt end)
    "u_both_ng": {"both_ng": True},              # both bodies are NG
    "u_slip_back": {"slip": -30.0},              # the NG body slides 30 mm back toward the good one
    "u_dim_light": {"gain": 0.65},               # the light is dimmer: every pixel x 0.65 before the noise
}
# UNSEEN set 2 (night of 2026-11-19, run I-2, review C42h): written BEFORE the world features below existed and before
# the candidate was changed to leave out-of-view tracks uncounted. Expectations in run.py (EXPECT_UNSEEN2), with their
# sha256 put into the night log first. followers = good bodies that follow the second one (GAP apart, or follower_gap);
# ng_follower = the LAST follower is the NG one and the first two are OK; stray = a good body with NO inspection event
# lying at (x, z) when the conveyor stops (the inspection app missed it, or a person put it back).
UNSEEN2_CASES = {
    "u2_two_followers": {"followers": 2},                          # 3rd and 4th good bodies, both out of view at the stop
    "u2_follower_ng": {"followers": 1, "ng_follower": True},       # the out-of-view 3rd body is the NG one
    "u2_follower_close": {"followers": 1, "follower_gap": 50.0},   # 3rd good body only 50 mm behind -> at the view's edge
    "u2_stray_plus_follower": {"followers": 1, "stray": [460.0, 40.0]},   # out-of-view follower + untracked body in view
}
# UNSEEN set 3 (run I-2, after review C42h found that the I-2 candidate puts an untracked part that TOUCHES the NG part
# into the bin, and that the scorer did not count it). Written BEFORE the candidate fix v3 and before stray_bin existed;
# expectations in run.py (EXPECT_UNSEEN3), sha256 into the night log first. The reviewer's own positions (500,30),
# (470,20), (540,0) are NOT used here (they are seen now).
UNSEEN3_CASES = {
    "u3_stray_touch_b": {"stray": [420.0, 18.0]},                 # untracked part touching the GOOD tracked part B
    "u3_stray_at_pick": {"stray": [500.0, 0.0], "slip": 50.0},     # NG slid 50 mm on; an untracked part lies where it was
    "u3_stray_merged_ng": {"stray": [465.0, 2.0]},                # untracked part touching the NG part from behind
    "u3_stray_in_bin": {"stray_bin": True},                       # an untracked good part is already in the bin
}
# UNSEEN set 4 (run I-2, review C42h-4): the margin of the v2 view rule (10 mm). Written BEFORE follower_slip and
# enc_offset existed; expectations in run.py (EXPECT_UNSEEN4), sha256 into the night log first. No controller change
# is planned: this set only checks the current candidate (v3). follower_slip = every follower slides this much along
# +x at the stop; enc_offset = the encoder reads this much MORE travel between the inspection and the stop than the
# belt really made, the same amount for every track (review C42h round 2-5: a zero offset would cancel out, and a scale
# error would grow with the travel; neither is modelled).
UNSEEN4_CASES = {
    "u4_follower_slip15": {"followers": 1, "follower_slip": 15.0},   # 3rd at 355: still fully outside the view
    "u4_follower_slip30": {"followers": 1, "follower_slip": 30.0},   # 3rd at 370: 10 mm of it inside the view
    "u4_enc_plus25": {"followers": 1, "enc_offset": 25.0},           # predictions +25 mm: the 3rd predicted at 365 (edge)
    "u4_enc_minus12": {"followers": 1, "enc_offset": -12.0},         # predictions -12 mm: B predicted at 408 (8 mm in)
}
# UNSEEN set 5 (run I-3, 2026-11-19 23:41 PC clock): parts that OVERLAP (one blob smaller than 1.5 parts). Written BEFORE the
# candidate fix v4 and before the "strays" world feature (several untracked parts) existed; expectations in run.py
# (EXPECT_UNSEEN5, with the wrong-reject count), sha256 into the night log first. The reviewer of C42h swept ONE stray
# around the NG part at x 440..565, z -40..40 (5 mm steps) with the NG part first; those are seen. This set uses other
# forms: the NG part second (B), two strays, a stray on the GOOD part. It also replaces set 3, which was moved to the
# development side (the world was fixed after its first run). strays = [[x, z], ...] untracked good parts at the stop.
UNSEEN5_CASES = {
    "u5_ng2_overlap_behind": {"ng_second": True, "stray": [405.0, -3.0]},   # stray overlaps the NG part B from behind
    "u5_ng2_mark_covered": {"ng_second": True, "stray": [436.0, -3.0]},     # stray lies over B's defect mark
    "u5_two_strays": {"strays": [[484.0, 2.0], [404.0, -3.0]]},             # one stray on A (NG), one on B (OK)
    "u5_overlap_good_b": {"stray": [434.0, -3.0]},                          # stray overlaps the GOOD part B (NG = A)
}
# UNSEEN set 6 (run I-3, 2026-11-20 ~00:04 PC clock): the defect mark of an NG part whose stop position has jitter.
# Written BEFORE the candidate fix v5 (count the dark mark inside the FILLED outline of the blob); no new world feature.
# Expectations in run.py (EXPECT_UNSEEN6) for seeds 0..19, sha256 into the night log first. Known before writing (found
# in run I-3 on seen cases): with the NG part second, swapped or both NG, v1..v4 lose the mark on seeds
# {4,7,8,9,10,12,14,17,19} out of 0..19; a shift by a whole pixel (0.5 mm steps) keeps that set, a half pixel does not.
UNSEEN6_CASES = {
    "u6_ng2_slip10": {"ng_second": True, "slip": 10.0},     # NG = B, slid 10 mm on (20 px: same pixel phase)
    "u6_swap_creep7": {"swap": True, "creep": 7.0},          # swap, then both creep 7 mm (14 px: same phase)
    "u6_ng2_slip10q": {"ng_second": True, "slip": 10.25},   # as the first, but 20.5 px: another phase (not predicted for v4)
}
# UNSEEN sets 7, 8, 9 (run I-4, 2026-11-20 ~00:27 PC clock, review C42l). Written BEFORE the candidate fix v6 (closing with
# the outside of the picture counted as dark) and BEFORE the world features b_x / b_z existed. Expectations in run.py
# (EXPECT_UNSEEN7/8/9) for seeds 0..19 with the wrong-reject count; sha256 into the night log first. Known before writing
# (review C42j round 2, seen form {ng_second, flip, creep 58}): a part that ends 2 mm or less inside the picture's edge
# gets the dark belt strip between it and the edge into its blob (the 9 x 9 closing treats the outside as bright when it
# erodes), and that strip reads as a defect mark. b_x / b_z = the SECOND body (B) lies exactly at this x / z at the stop
# (no jitter on that axis), set after slip / swap / creep.
UNSEEN7_CASES = {   # a GOOD part at the edge of the picture
    "u7_edge59_ng2flip": {"ng_second": True, "flip": True, "creep": 59.0},   # good A ends 1 mm before the bottom edge, NG B mark down
    "u7_zedge_flip": {"flip": True, "b_z": -47.5},       # good B ends 0.5 mm inside the belt-side edge (z -60), NG A mark down
    "u7_topedge_flip": {"flip": True, "b_x": 401.5},     # good B ends 1.5 mm inside the upstream edge (x 380), NG A mark down
    "u7_edge59_ng2": {"ng_second": True, "creep": 59.0},  # as the first, but the NG mark is up (the price side of a fix)
}
UNSEEN8_CASES = {   # an NG part whose mark is not seen
    "u8_flip_slip25": {"flip": True, "slip": 25.0},       # NG A mark down, slid 25 mm on; nothing at an edge
    "u8_flip_edge59": {"flip": True, "creep": 59.0},      # NG A mark down AND 1 mm before the bottom edge
    "u8_mark_at_edge": {"creep": 66.0},                   # NG A cut by the bottom edge (546..586); mark at 573..579, 1 mm inside
}
UNSEEN9_CASES = {   # only unmarked parts in the picture
    "u9_flip_zfar": {"flip": True, "b_z": 47.5},          # good B at the far belt side (z 60 is NOT a picture edge)
    "u9_flip_close_behind": {"flip": True, "b_x": 458.0},  # good B 2 mm behind the NG A (merged by the closing)
    "u9_flip_cut_edge": {"flip": True, "creep": 64.0},    # NG A mark down, cut by the bottom edge (no strip)
}
# UNSEEN set 10 (run I-5, 2026-11-20, copied 01:10:04 PC clock, review C42n; the comment said ~01:15 by mistake). Written BEFORE the candidate fix v7 and BEFORE the
# world features flip_b / a_jit existed. Expectations in run.py (EXPECT_UNSEEN10) for seeds 0..19 with wrong reject,
# ESCAPE and false_done (a track declared complete whose own body is not in the bin; scorer column planned). Known before
# writing (review C42l-2, seen form {both_ng, creep 50..80}): with two NG tracks open, a marked part of the OTHER track
# can be matched to (or re-identified for) the first track, which is then declared complete while its own body stays on
# the belt; the run's outcome is "hold" because the other track holds. flip_b = the SECOND body lies mark-down.
# a_jit = after the stop (and creep), the FIRST body's x gets extra jitter N(0, a_jit) mm (A otherwise stops at exactly
# x 500 + creep).
UNSEEN10_CASES = {
    "u10_ng2_flip_a": {"both_ng": True, "flip": True},                  # both NG; A (first) mark-down, B mark-up
    "u10_ng2_creep72_jit": {"both_ng": True, "creep": 72.0, "a_jit": 1.5},   # A cut by the bottom edge, its mark near/over the edge; B at ~492
    "u10_ng2_flip_b": {"both_ng": True, "flip_b": True},                # both NG; B (second) mark-down
    "u10_ng2_b445_jit": {"both_ng": True, "b_x": 445.0, "a_jit": 1.5},  # both NG; B slid on to 445 (25 mm from its prediction)
    "u10_ng2_swap": {"both_ng": True, "swap": True},                    # both NG and swapped (labels cross, both go to the bin)
}
# UNSEEN set 11 (run I-5, 2026-11-20, copied 01:42:11 PC clock, review C42n items 1-2; the comment said ~01:30 by mistake). Written BEFORE the candidate fix v8; no
# new world feature. Expectations in run.py (EXPECT_UNSEEN11), seeds 0..19, sha256 into the night log first. Known before
# writing (review C42n, seen forms {both_ng, b_x 250..350 / 610..700} and {both_ng, swap, flip (+ creep 12, a_jit 1.5,
# slip 10, enc_offset -12, miss 1)}): v7 counts only OPEN in-view NG tracks, so after T1 is held for a count mismatch
# the second track re-identifies T1's marked body; and v7 checks the mark of the picked track's own blob only. The
# values below are not the reviewer's, but the mechanisms are seen: weak as evidence.
UNSEEN11_CASES = {
    "u11_ng2_b_past_end": {"both_ng": True, "b_x": 600.0},             # B slid past the view's far end (580..620)
    "u11_ng2_swap_flip_jit": {"both_ng": True, "swap": True, "flip": True, "a_jit": 2.0},   # A (mark down) swapped to ~420
    "u11_ng2_ng_follower": {"both_ng": True, "followers": 1, "ng_follower": True},   # T1 OK, T2 NG, T3 NG out of view
    "u11_ng2_creep90": {"both_ng": True, "creep": 90.0},               # A at 590 (10 mm in view: no blob), B at ~510
}
# UNSEEN set 12 (run I-6, 2026-11-20, review C42-I6). Written BEFORE the candidate fix v9 and BEFORE the world fix of
# "swap with a third / follower body" (cell.py crashed at `a, b = self.bodies`). Expectations in run.py (EXPECT_UNSEEN12),
# seeds 0..19, with wrong reject, escape and false_done; sha256 into the night log first. Known before writing (review
# C42n round 2, seen forms {both_ng, swap, b_x 600 / 300} and {both_ng, followers 1, ng_follower, follower_slip 80,
# b_x 340}): v8 holds one NG track (count) or leaves it uncounted (out of view), the rest then match one to one, and the
# held / out-of-view track's marked body is picked by POSITION for the other track. The values below are not the
# reviewer's, but the mechanism is seen: weak as evidence. Pairs 1/5 and 3/4 give the controller the same picture
# (one marked part where the other NG track is predicted), so no rule can pick in one and hold in the other.
# CORRECTION (review C42-I6 i6-3, checked): not the same picture. detect() gives the same blobs for 1/5, but the raw
# picture of 5 has A's 5 mm strip at the bottom edge; in 3/4 the marked blob's z differs by about 4-5 mm and T3 is
# predicted at 340 vs 320. The marked part lies where the track being PICKED is predicted. Only a rule that uses the
# count, position (x, within the match tolerance) and mark alone cannot tell them apart.
UNSEEN12_CASES = {
    "u12_swap_b_far": {"both_ng": True, "swap": True, "b_x": 625.0},               # A (T1) swapped to ~420, B (T2) past the far end
    "u12_swap_b_up_jit": {"both_ng": True, "swap": True, "b_x": 335.0, "a_jit": 2.5},   # B upstream, out of view; A jitters
    "u12_follower_in_view": {"both_ng": True, "followers": 1, "ng_follower": True, "follower_slip": 75.0, "b_x": 333.0},
    #                         T1 OK (A 500), T2 NG (B at 333, out of view), T3 NG (C slid on to ~415, in view)
    "u12_ng_follower_gap": {"both_ng": True, "followers": 1, "ng_follower": True, "follower_gap": 100.0},   # T3 NG at ~320, out
    "u12_a_slid_out": {"both_ng": True, "slip": 95.0},                             # A (T1) at 595: 5 mm in view, no blob
    "u12_swap_third": {"swap": True, "third": True},                               # swap with a 3rd good body (world fix)
    "u12_ng2_swap_follower": {"both_ng": True, "swap": True, "followers": 1},      # both NG swapped + a good follower out of view
}
# UNSEEN set 13 (run I-7, review C42-I6 i6-1 / i6-2 / i6-7). Written AFTER the world features miss_from / stray_mark and
# the stray_bin + creep fix, and BEFORE the candidate fix v10; expectations in run.py (EXPECT_UNSEEN13) for seeds 0..19
# with wrong reject, escape and false_done; sha256 into the night log first. Seen forms (review C42-I6): {both_ng, swap}
# with every pick from the 2nd one missing, and {flip, b_x 625, stray [420, -3]} with a marked stray. The values below
# differ; the mechanisms are the seen ones, so this set is weak evidence.
# miss_from = k: the k-th pick command and every later one close on air. stray_mark: the strays show a defect mark.
UNSEEN13_CASES = {
    "u13_ng2_swap_miss2": {"both_ng": True, "swap": True, "a_jit": 1.0, "miss_from": 2},     # i6-1 form, A jitters
    "u13_ng2_miss2_creep4": {"both_ng": True, "creep": 4.0, "miss_from": 2},                 # control: no swap
    "u13_ng2_swap_miss1_from3": {"both_ng": True, "swap": True, "miss": 1, "miss_from": 3},  # 1st pick misses once, then the 3rd on
    "u13_ng2_swap_third_miss2": {"both_ng": True, "swap": True, "third": True, "miss_from": 2},   # + a 3rd good body out of view
    "u13_mstray_b_up": {"flip": True, "b_x": 330.0, "stray": [425.0, 1.0], "stray_mark": True},    # i6-2 form, B upstream
    "u13_mstray_b_far_creep3": {"flip": True, "creep": 3.0, "b_x": 645.0, "stray": [416.0, -4.0], "stray_mark": True},
    "u13_mstray_ng_seen": {"b_x": 335.0, "stray": [428.0, -2.0], "stray_mark": True},          # control: NG mark up
    "u13_bin_creep8": {"stray_bin": True, "creep": 8.0},                                       # i6-7 world fix
}
CASE_VERSION = "2026-11-20i"   # i: + miss_from / stray_mark; creep moves belt bodies only (stray_bin + creep crashed);
#                                   + UNSEEN13_CASES (I-7). No earlier case used miss_from, stray_mark or stray_bin + creep
# was "2026-11-20h"   # c: + UNSEEN_CASES (CASES unchanged from b). d: + UNSEEN2/3 world features (I-2)
#                                e: + UNSEEN5_CASES and "strays" (several untracked parts) (I-3, review C42j-7)
#                                f: + UNSEEN7/8/9_CASES and "b_x" / "b_z" (I-4). Earlier cases unchanged
#                                g: + UNSEEN10_CASES and "flip_b" / "a_jit" (I-5). Earlier cases unchanged
#                                h: + UNSEEN12_CASES; swap now works with a 3rd / follower body (I-6). No earlier case
#                                   used that pair (it crashed), so earlier cases are unchanged


class Cell:
    def __init__(self, case, seed=0, labels=("A", "B", "C")):
        """labels: truth names of (first body, second body, third body if any). The first body is the defective one (NG)
        unless the case says ng_second / both_ng."""
        self.case = dict(case)
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.enc = 0.0                       # conveyor travel [mm], what the encoder reports
        j = lambda: float(self.rng.normal(0, 1.0))
        second = bool(self.case.get("ng_second"))
        self.bodies = [
            {"id": labels[0], "defect": not second, "x": 0.0 + j(), "z": 2.0 + j(), "mark_up": True, "where": "belt"},
            {"id": labels[1], "defect": second, "x": -GAP + j(), "z": -3.0 + j(), "mark_up": True, "where": "belt"},
        ]
        if self.case.get("both_ng"):
            self.bodies[1]["defect"] = True
        names = list(labels) + (["C"] if len(labels) < 3 else []) + ["D", "E", "F", "G"]
        if self.case.get("third"):
            self.bodies.append({"id": names[2], "defect": False, "x": -2 * GAP + j(), "z": 1.0 + j(),
                                "mark_up": True, "where": "belt"})
        nf = int(self.case.get("followers", 0))       # UNSEEN set 2: good bodies behind the second one
        x, self.follow_nominal = -GAP, []
        for i in range(nf):
            x -= GAP if i else float(self.case.get("follower_gap", GAP))
            self.follow_nominal.append(x)
            self.bodies.append({"id": names[2 + i], "defect": False, "x": x + j(), "z": 1.0 + j(), "mark_up": True,
                                "where": "belt"})
        if self.case.get("ng_follower"):              # the last follower is NG, the first two are OK
            self.bodies[0]["defect"] = False
            self.bodies[-1]["defect"] = True
        self.ng = self.bodies[-1] if self.case.get("ng_follower") else (self.bodies[1] if second else self.bodies[0])
        self.strays = []                               # untracked: no inspection event, no tracking number
        if self.case.get("stray"):
            self.strays.append({"id": "S", "defect": False, "x": None, "z": float(self.case["stray"][1]), "mark_up": True,
                                "where": "belt", "_x": float(self.case["stray"][0])})
        for i, (sx, sz) in enumerate(self.case.get("strays", [])):   # UNSEEN set 5 (run I-3): several untracked parts
            self.strays.append({"id": f"S{i + 1}", "defect": False, "x": None, "z": float(sz), "mark_up": True,
                                "where": "belt", "_x": float(sx)})
        if self.case.get("stray_mark"):   # run I-7 (review C42-I6 i6-2): the untracked parts show a defect mark from
            for st in self.strays:          # above (a defective part the inspection app missed, or one a person put back)
                st["defect"] = True
        self.bin_count = 0
        self.misses_left = int(self.case.get("miss", 0))
        self.last_image = None
        self.truth_log = [(0.0, "tracks", {f"T{i + 1}": b["id"] for i, b in enumerate(self.bodies)})]   # scorer only
        if self.case.get("stray_bin"):   # UNSEEN set 3: an untracked good part already in the bin. Added AFTER the
            # tracks entry (the first version added it before, gave it T3, and the scorer said no_decision: a world bug)
            self.bodies.append({"id": "S", "defect": False, "x": None, "z": None, "mark_up": True, "where": "bin"})
            self.bin_count = 1
        self.obs_log = []                    # everything the controller was given, in order (for replay)
        self.cmd_log = []

    # ------------------------------------------------------------------ observations
    def _emit(self, ob):
        ob = copy.deepcopy(ob)
        self.obs_log.append(ob)
        return ob

    def start(self):
        """inspection events and the conveyor run up to the stop. Returns the list of observations."""
        out = []
        v1, v2 = ("OK", "NG") if self.case.get("ng_second") else ("NG", "OK")
        if self.case.get("both_ng"):
            v2 = "NG"
        evs = [("E1", "T1", v1, 0.0, 0.0), ("E2", "T2", v2, GAP / BELT_SPEED, GAP / BELT_SPEED)]
        if self.case.get("third"):
            evs.append(("E3", "T3", "OK", 2 * GAP / BELT_SPEED, 2 * GAP / BELT_SPEED))
        if self.case.get("ng_follower"):
            evs[0] = ("E1", "T1", "OK", 0.0, 0.0)
        for i, xn in enumerate(self.follow_nominal):   # nominal place (no jitter), like E1 and E2
            tt = round(-xn / BELT_SPEED, 3)
            evs.append((f"E{3 + i}", f"T{3 + i}", "NG" if self.bodies[2 + i]["defect"] else "OK", tt, tt))
        if self.case.get("dup_event"):        # the same message delivered again later (identical content)
            evs.append(("E1", "T1", "NG", 0.0, GAP / BELT_SPEED + 0.1))
        if self.case.get("dup_restamped"):    # the app re-issues the judgement with the time and encoder of the re-send
            evs.append(("E1", "T1", "NG", GAP / BELT_SPEED + 0.1, GAP / BELT_SPEED + 0.1))
        if self.case.get("dup_conflict"):     # the same event id comes again with the other verdict
            evs.append(("E1", "T1", "OK", 0.0, GAP / BELT_SPEED + 0.1))
        for eid, tid, verdict, t, t_deliver in sorted(evs, key=lambda e: e[4]):
            ev = {"type": "inspection", "t": round(t, 3), "event_id": eid, "track_id": tid, "verdict": verdict,
                  "x_at_inspection": 0.0, "encoder_at_inspection": round(t * BELT_SPEED, 3)}
            if verdict == "NG":
                ev["defect_feature"] = "top_mark"     # what the inspection app says the defect looks like from above
            out.append(self._emit(ev))
        run = STOP_X - self.bodies[0]["x"]
        self.t = run / BELT_SPEED
        self.enc = run
        for b in self.bodies:
            if b["where"] == "belt":
                b["x"] += run
        if self.case.get("slip"):
            self.ng["x"] += self.case["slip"]               # the NG body slid forward on the belt
            self.truth_log.append((self.t, "slip", {"id": self.ng["id"], "dx": self.case["slip"]}))
        if self.case.get("swap"):
            a, b = self.bodies[0], self.bodies[1]   # run I-6: was `a, b = self.bodies` (crashed with a 3rd / follower body)
            (a["x"], a["z"]), (b["x"], b["z"]) = (b["x"], b["z"]), (a["x"], a["z"])
            self.truth_log.append((self.t, "swap", {}))
        if self.case.get("creep"):
            for b in self.bodies:
                if b["where"] == "belt":   # run I-7 (review C42-I6 i6-7): a body already in the bin (stray_bin) has x None
                    b["x"] += self.case["creep"]
            self.truth_log.append((self.t, "creep", {"dx": self.case["creep"]}))
        if self.case.get("a_jit"):                   # UNSEEN set 10 (run I-5): extra jitter on the FIRST body's x
            dx = float(self.rng.normal(0, float(self.case["a_jit"])))
            self.bodies[0]["x"] += dx
            self.truth_log.append((self.t, "a_jit", {"id": self.bodies[0]["id"], "dx": round(dx, 3)}))
        if self.case.get("follower_slip"):           # UNSEEN set 4: every follower slides along +x
            for b in self.bodies[2:2 + int(self.case.get("followers", 0))]:
                b["x"] += self.case["follower_slip"]
            self.truth_log.append((self.t, "follower_slip", {"dx": self.case["follower_slip"]}))
        for key, ax in (("b_x", "x"), ("b_z", "z")):     # UNSEEN sets 7-9 (run I-4): B lies exactly here at the stop
            if key in self.case:
                self.bodies[1][ax] = float(self.case[key])
                self.truth_log.append((self.t, key, {"id": self.bodies[1]["id"], ax: self.bodies[1][ax]}))
        for st in self.strays:                       # placed at the stop, painted after the tracked parts (on top)
            st["x"] = st.pop("_x")
            self.bodies.append(st)
            self.truth_log.append((self.t, "stray", {"id": st["id"], "x": st["x"], "z": st["z"]}))
        if self.case.get("flip"):
            self.ng["mark_up"] = False
            self.truth_log.append((self.t, "flip", {"id": self.ng["id"]}))
        if self.case.get("flip_b"):                  # UNSEEN set 10 (run I-5): the SECOND body lies mark-down
            self.bodies[1]["mark_up"] = False
            self.truth_log.append((self.t, "flip_b", {"id": self.bodies[1]["id"]}))
        self.truth_log.append((self.t, "stopped", self.snapshot()))
        enc_seen = self.enc + float(self.case.get("enc_offset", 0.0))   # UNSEEN set 4: the encoder reads wrong
        out.append(self._emit({"type": "conveyor", "t": round(self.t, 3), "state": "stopped", "encoder": round(enc_seen, 3)}))
        return out

    def snapshot(self):
        return {"bodies": copy.deepcopy(self.bodies), "bin_count": self.bin_count}

    def shoot(self, kind="first"):
        """top camera picture. kind 'first' = right after the stop, 'reshoot' = asked by the controller."""
        self.t += T_SHOT
        if self.case.get("first_missing") or (kind == "reshoot" and self.case.get("image_missing")):
            return self._emit({"type": "image", "t_request": round(self.t, 3), "image": None, "why": "no frame"})
        if kind == "reshoot" and self.case.get("stale") and self.last_image is not None:
            img, tc = self.last_image
            return self._emit({"type": "image", "t_capture": tc, "image": img, "px_per_mm": PX_PER_MM, "view_mm": list(VIEW)})
        if kind == "reshoot" and self.case.get("stale_restamp") and self.last_image is not None:
            img, _ = self.last_image          # old frame, new time stamp (a buffered frame re-sent by the camera driver)
            self.truth_log.append((self.t, "stale_restamp", {}))
            return self._emit({"type": "image", "t_capture": round(self.t, 3), "image": img, "px_per_mm": PX_PER_MM,
                               "view_mm": list(VIEW)})
        img = self.render()
        self.last_image = (img, round(self.t, 3))
        return self._emit({"type": "image", "t_capture": round(self.t, 3), "image": img, "px_per_mm": PX_PER_MM, "view_mm": list(VIEW)})

    def render(self):
        x0, x1, z0, z1 = VIEW
        W, H = int((z1 - z0) * PX_PER_MM), int((x1 - x0) * PX_PER_MM)     # image right = +z, image down = +x
        img = np.full((H, W), 40.0)
        bz0, bz1 = [int((z - z0) * PX_PER_MM) for z in BELT_Z]
        img[:, bz0:bz1] = 70.0                                            # belt
        bx0, bx1, bzz0, bzz1 = BIN
        img[int((bx0 - x0) * PX_PER_MM):int((bx1 - x0) * PX_PER_MM), int((bzz0 - z0) * PX_PER_MM):int((bzz1 - z0) * PX_PER_MM)] = 55.0
        yy, xx = np.mgrid[0:H, 0:W]
        X, Z = x0 + (yy + 0.5) / PX_PER_MM, z0 + (xx + 0.5) / PX_PER_MM
        for b in self._visible_bodies():
            m = (np.abs(X - b["x"]) <= PART[0] / 2) & (np.abs(Z - b["z"]) <= PART[1] / 2)
            img[m] = 200.0
            if b["defect"] and b["mark_up"]:
                img[(X - (b["x"] + 10)) ** 2 + (Z - b["z"]) ** 2 <= MARK_R ** 2] = 30.0
        if self.case.get("arm_in_view"):     # wrist block at the belt end, 40 x 40 mm
            img[(np.abs(X - 560.0) <= 20.0) & (np.abs(Z) <= 20.0)] = 210.0
        img *= float(self.case.get("gain", 1.0))
        img += self.rng.normal(0, 4.0, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def _visible_bodies(self):
        out = [b for b in self.bodies if b["where"] == "belt"]
        slots = [(450.0, 105.0), (450.0, 135.0), (510.0, 105.0), (510.0, 135.0)]
        for i, b in enumerate([b for b in self.bodies if b["where"] == "bin"]):
            out.append(dict(b, x=slots[i][0], z=slots[i][1]))
        return out

    # ------------------------------------------------------------------ actions
    def pick(self, cmd):
        """cmd: {'cmd_id', 'x', 'z'}. The arm goes there, closes, carries to the bin and reports done."""
        self.cmd_log.append(copy.deepcopy(cmd))
        self.t += T_PICK
        took = None
        if self.case.get("miss_push") and not any(w == "miss" for _, w, _ in self.truth_log):
            near = [b for b in self.bodies if b["where"] == "belt" and np.hypot(b["x"] - cmd["x"], b["z"] - cmd["z"]) <= GRAB_R]
            for b in near[:1]:
                b["x"] += self.case["miss_push"]
            self.truth_log.append((self.t, "miss", {"cmd_id": cmd["cmd_id"], "pushed": near[0]["id"] if near else None}))
        elif self.misses_left > 0:
            self.misses_left -= 1
            self.truth_log.append((self.t, "miss", {"cmd_id": cmd["cmd_id"]}))
        elif self.case.get("miss_from") and len(self.cmd_log) >= int(self.case["miss_from"]):
            # run I-7 (review C42-I6 i6-1): the k-th pick command and every later one close on air ("miss" only fails
            # the FIRST n picks, so a second pick that fails could not be made before)
            self.truth_log.append((self.t, "miss", {"cmd_id": cmd["cmd_id"], "from": int(self.case["miss_from"])}))
        else:
            near = [b for b in self.bodies if b["where"] == "belt" and np.hypot(b["x"] - cmd["x"], b["z"] - cmd["z"]) <= GRAB_R]
            if near:
                took = min(near, key=lambda b: np.hypot(b["x"] - cmd["x"], b["z"] - cmd["z"]))
                took["where"] = "bin"
                self.bin_count += 1
        self.truth_log.append((self.t, "pick", {"cmd_id": cmd["cmd_id"], "took": took["id"] if took else None}))
        return self._emit({"type": "robot", "t": round(self.t, 3), "cmd_id": cmd["cmd_id"], "done": True})

    def finish(self, declarations):
        self.truth_log.append((self.t, "end", {"snapshot": self.snapshot(), "declarations": copy.deepcopy(declarations),
                                               "cycle_s": round(self.t, 3)}))
