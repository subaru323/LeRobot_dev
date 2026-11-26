r"""Fictional reject cell: the CONTROLLER side. It sees only observations (no truth, no body ids, no scorer output).

Observation dicts (from cell.py through the driver in run.py):
  inspection {t, event_id, track_id, verdict, x_at_inspection, encoder_at_inspection, defect_feature?}
  conveyor   {t, state, encoder}
  image      {t_capture, image (uint8 HxW, image down = +x, right = +z), px_per_mm, view_mm [x0,x1,z0,z1]} or {image: None}
  robot      {t, cmd_id, done}
Actions returned by next_action():
  {"pick": {"cmd_id", "x", "z", "track_id"}} | {"reshoot": True} | {"end": {track_id: {"state": complete|hold|pass, "why"}}}

Baseline  = what the plan calls the base method: on an NG inspection event, pick at the position predicted from the
            conveyor encoder, and record "removed" when the arm's done signal arrives. No images.
Candidate = baseline + tracking checked against the picture (count, position, the defect mark) + a re-shoot after
            the pick (fresh picture, target gone, bin count up). Anything it cannot confirm becomes a HOLD with a reason.
Limits (plan: re-shoot / re-plan at most 2 each per move): MAX_RESHOOT, MAX_REPICK.
"""

import cv2
import numpy as np

MATCH_TOL = 20.0       # mm: a picture blob belongs to a predicted track if it is this close. GUESS (slip seen 8 mm ok)
PICK_TOL = 15.0        # mm: after the pick, a blob this close to the pick point means the part is still there
BELT_Z = (-60.0, 60.0)
BIN_Z = (80.0, 160.0)
PART_LEN = 40.0         # mm along the belt
PART_WID = 24.0         # mm across the belt
PART_AREA_MM2 = PART_LEN * PART_WID
VIEW_CLEAR = 10.0      # mm: a track whose predicted part ends this far outside the view is not expected in the picture
OUTLINE_TOL = 6.0      # mm: v4 (run I-3). A belt blob longer/wider than one part by more than this -> hold. GUESS
MAX_RESHOOT = 2
MAX_REPICK = 2
LIMITS = {"MATCH_TOL_mm": MATCH_TOL, "PICK_TOL_mm": PICK_TOL, "MAX_RESHOOT": MAX_RESHOOT, "MAX_REPICK": MAX_REPICK,
          "dark_threshold": 90, "bright_threshold": 140, "VIEW_CLEAR_mm": VIEW_CLEAR, "OUTLINE_TOL_mm": OUTLINE_TOL}


def detect(ob, mark_filled=False, border_dark=False):
    """blobs from a top picture: [{'x','z','mark','area','len_x','wid_z'}] on the belt, and the number of parts in the
    bin. mark_filled: v5 way of counting the defect mark (see below). border_dark: v6 (run I-4): the closing counts the
    outside of the picture as dark. Before v6 the erosion counted it as bright, so the dark belt strip (<= 4 px) between a
    part and the picture's edge joined the blob and read as a defect mark (review C42j round 2)."""
    img, ppm, (x0, x1, z0, z1) = ob["image"], ob["px_per_mm"], ob["view_mm"]
    bright = (img > 140).astype(np.uint8)
    if border_dark:
        bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8),
                                  borderType=cv2.BORDER_CONSTANT, borderValue=0)
    else:
        bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))   # fill the dark mark
    n, lab, st, cen = cv2.connectedComponentsWithStats(bright)
    belt, bin_n = [], 0
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA] / ppm ** 2
        if area < 0.5 * PART_AREA_MM2:
            continue
        cx, cy = cen[i]
        x, z = x0 + (cy + 0.5) / ppm, z0 + (cx + 0.5) / ppm
        if BIN_Z[0] <= z <= BIN_Z[1]:
            bin_n += 1
            continue
        if not (BELT_Z[0] <= z <= BELT_Z[1]):
            continue
        inside = lab == i
        if mark_filled:    # v5 (run I-3): the 9 x 9 closing leaves the 12 px mark as a hole on some pixel phases;
            # count the dark pixels inside the FILLED outer outline instead (area, centre and outline stay as before)
            cs, _ = cv2.findContours(inside.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            filled = np.zeros_like(bright)
            cv2.drawContours(filled, cs, -1, 1, thickness=cv2.FILLED)
            inside = filled.astype(bool)
        dark = int(((img < 90) & inside).sum())
        belt.append({"x": round(float(x), 2), "z": round(float(z), 2), "mark": dark >= 0.5 * np.pi * 3.0 ** 2 * ppm ** 2,
                     "area": round(float(area), 1),
                     "len_x": round(float(st[i, cv2.CC_STAT_HEIGHT]) / ppm, 1),     # run I-3 (v4): outline of the blob
                     "wid_z": round(float(st[i, cv2.CC_STAT_WIDTH]) / ppm, 1)})
    return belt, bin_n


class Baseline:
    name = "baseline"

    def __init__(self):
        self.events, self.enc, self.queue, self.decl, self.pending, self.n = [], None, [], {}, None, 0
        self.history = []

    def feed(self, ob):
        if ob["type"] == "inspection":
            self.events.append(ob)
            if ob["verdict"] == "NG":
                self.queue.append(ob)
        elif ob["type"] == "conveyor" and ob["state"] == "stopped":
            self.enc = ob["encoder"]
        elif ob["type"] == "robot" and self.pending:
            self.decl[self.pending] = {"state": "complete", "why": "arm done signal"}
            self.history.append({"t": ob["t"], "decide": "complete", "track": self.pending, "why": "arm done signal"})
            self.pending = None

    def next_action(self):
        if self.enc is None:
            return None
        if self.queue and not self.pending:
            ev = self.queue.pop(0)
            self.n += 1
            x = ev["x_at_inspection"] + self.enc - ev["encoder_at_inspection"]
            self.pending = ev["track_id"]
            self.history.append({"decide": "pick", "track": ev["track_id"], "x": x, "z": 0.0, "why": "predicted from encoder"})
            return {"pick": {"cmd_id": f"C{self.n}", "x": round(x, 2), "z": 0.0, "track_id": ev["track_id"]}}
        for ev in self.events:
            if ev["verdict"] == "OK":
                self.decl.setdefault(ev["track_id"], {"state": "pass", "why": "OK verdict"})
        return {"end": self.decl}


class Candidate:
    name = "candidate"
    TRACK = True       # check the picture before the pick (count, position, defect mark). False: pick at the encoder prediction
    RESHOOT = True     # re-shoot after the pick and confirm. False: complete on the arm's done signal
    VIEW_AWARE = True  # count only the tracks predicted inside the view (run I-2, review C42h). False = the I-1 version
    BLOB_CHECK = False  # run I-2 v3 (review C42h-1): a belt blob larger than 1.5 parts (parts touching) -> hold.
    #                     Replaced in v4 by OUTLINE_CHECK (an overlap of two parts stays below 1.5 parts of area)
    OUTLINE_CHECK = True  # run I-3 v4: a belt blob longer than a part + OUTLINE_TOL along x, or wider along z -> hold
    FIRST_COUNTS = True   # run I-2 v3: a re-pick checks bin/belt counts against those before the track's FIRST pick
    MARK_FILLED = True    # run I-3 v5: count the defect mark inside the filled outline of the blob (see detect)
    BORDER_DARK = True    # run I-4 v6: the closing counts the outside of the picture as dark (see detect)
    PROVISIONAL = True    # run I-7 v10 (review C42-I6 i6-1): an NG track completed while two or more NG tracks were not
    #                       complete at its pick is complete only provisionally: two marked NG parts that traded places
    #                       look the same as two that did not, so the part taken may be the other track's body. If a later
    #                       NG track is held, every provisional completion turns into a hold
    NG2_ALL = True        # run I-6 v9 (review C42n round 2): with two or more NG tracks not declared complete, pick only
    #                       when EVERY one of them is still open and predicted inside the view. A held or out-of-view NG
    #                       track's body may be the marked part that matches another track by position. Else hold
    NG2_PENDING = True    # run I-5 v8 (review C42n 1-2): the guard counts NG tracks NOT declared complete (held, out of
    #                       view, at the edge included), and asks for the mark on EVERY open in-view NG track's blob
    NG2_GUARD = True      # run I-5 v7 (review C42l-2): two or more NG tracks open in view -> pick only when every open
    #                       track matches a blob one to one and the track's own blob is marked; no re-identification
    #                       by "the only marked part" (that part may be the OTHER NG track's body). Else hold

    def __init__(self):
        self.tracks = {}            # track_id -> inspection event (first one only)
        self.seen_events = set()
        self.enc = None
        self.img = None             # latest usable picture observation
        self.decl = {}
        self.history = []
        self.n = 0
        self.job = None             # current NG job: {track, x, z, t_done, reshoots, repicks, bin_before, others}
        self.want_shot = False
        self.first_tries = 0        # re-shoots asked before the first usable picture
        self.conflict = {}          # track -> why (the same event came again with another verdict)
        self.provisional = []       # v10: tracks completed while another NG track was not complete

    def log(self, **k):
        self.history.append(k)

    def feed(self, ob):
        if ob["type"] == "inspection":
            if ob["event_id"] in self.seen_events or ob["track_id"] in self.tracks:
                first = self.tracks.get(ob["track_id"]) or next(e for e in self.tracks.values() if e["event_id"] == ob["event_id"])
                if first["verdict"] != ob["verdict"] or first["track_id"] != ob["track_id"]:
                    self.conflict[first["track_id"]] = (f"inspection event {ob['event_id']} came again with verdict {ob['verdict']}"
                                                        f" (first: {first['verdict']})")
                    self.log(t=ob["t"], decide="conflict", event=ob["event_id"], why=self.conflict[first["track_id"]])
                else:
                    self.log(t=ob["t"], decide="link", event=ob["event_id"], track=first["track_id"],
                             why="re-sent inspection event: same result as the first, no new command")
                return
            self.seen_events.add(ob["event_id"])
            self.tracks[ob["track_id"]] = ob
        elif ob["type"] == "conveyor" and ob["state"] == "stopped":
            self.enc = ob["encoder"]
        elif ob["type"] == "image":
            self.img = ob
        elif ob["type"] == "robot" and self.job and ob["cmd_id"] == self.job["cmd_id"]:
            if not self.RESHOOT:
                self.decl[self.job["track"]] = {"state": "complete", "why": "arm done signal (re-shoot off)"}
                self.log(t=ob["t"], decide="complete", track=self.job["track"], why="arm done signal (re-shoot off)")
                self.job = None
                # the next track still gets its own fresh picture (tracking needs one). Added after the first ablation
                # run (outputs/industrial/20261119_224230_unseen): without it, u_both_ng held on the PRE-pick picture.
                self.img = None
                self.first_tries = 0
                return
            self.job["t_done"] = ob["t"]
            self.job["reshoots"] = 0
            self.img = None
            self.want_shot = True

    def predict(self, ev):
        return ev["x_at_inspection"] + self.enc - ev["encoder_at_inspection"]

    def in_view(self, tid):
        """'in' (the whole part is inside the picture's x range), 'out' (it ends VIEW_CLEAR or more outside), else 'edge'.
        Uses the view_mm the picture itself reports, and the encoder prediction only (no picture content)."""
        x0, x1 = self.img["view_mm"][0], self.img["view_mm"][1]
        px, h = self.predict(self.tracks[tid]), PART_LEN / 2
        if x0 + h <= px <= x1 - h:
            return "in"
        if px + h + VIEW_CLEAR <= x0 or px - h - VIEW_CLEAR >= x1:
            return "out"
        return "edge"

    def hold(self, track, why):
        self.decl[track] = {"state": "hold", "why": why}
        self.log(decide="hold", track=track, why=why)
        self.job = None
        if self.PROVISIONAL and self.provisional and (self.tracks.get(track) or {}).get("verdict") == "NG":
            for p in self.provisional:
                if (self.decl.get(p) or {}).get("state") == "complete":
                    w = (f"provisional completion withdrawn: NG track {track} is held, and the part taken for {p} may be"
                         f" {track}'s body (two marked NG parts that traded places look the same)")
                    self.decl[p] = {"state": "hold", "why": w}
                    self.log(decide="hold", track=p, why=w)
            self.provisional = []

    def plan_pick(self, track, blob, why):
        self.n += 1
        belt, bin_n = detect(self.img, self.MARK_FILLED, self.BORDER_DARK)
        again = bool(self.job and self.job["track"] == track)
        ng_nc = [t for t, ev in self.tracks.items() if ev["verdict"] == "NG" and (self.decl.get(t) or {}).get("state") != "complete"]
        if again and self.FIRST_COUNTS:     # keep the counts from before the first pick of this track
            bin_n, n_belt = self.job["bin_before"], self.job["n_belt_before"]
        else:
            n_belt = len(belt)
        self.job = {"track": track, "cmd_id": f"C{self.n}", "x": blob["x"], "z": blob["z"], "t_done": None,
                    "reshoots": 0, "repicks": self.job["repicks"] + 1 if again else 0,
                    "bin_before": bin_n, "n_belt_before": n_belt,
                    "ng2": self.job["ng2"] if again else len(ng_nc) >= 2}
        self.log(t_image=self.img["t_capture"], decide="pick", track=track, x=blob["x"], z=blob["z"], why=why)
        return {"pick": {"cmd_id": self.job["cmd_id"], "x": blob["x"], "z": blob["z"], "track_id": track}}

    def match(self, belt):
        """predicted tracks -> blobs, one to one within MATCH_TOL. Returns {track: blob or None}."""
        out, used = {}, set()
        for tid, ev in sorted(self.tracks.items(), key=lambda kv: kv[0]):
            if tid in self.decl:
                continue
            px = self.predict(ev)
            cand = sorted(((abs(b["x"] - px) + abs(b["z"]), i) for i, b in enumerate(belt) if i not in used))
            if cand and cand[0][0] <= MATCH_TOL:
                used.add(cand[0][1])
                out[tid] = belt[cand[0][1]]
            else:
                out[tid] = None
        return out

    def next_action(self):
        if self.enc is None:
            return None
        if self.want_shot:                                   # after a pick: need a picture taken after the done signal
            ob = self.img
            if ob is None or ob.get("image") is None or ob["t_capture"] <= self.job["t_done"]:
                if ob is not None:
                    self.log(decide="reject image", why="no frame" if ob.get("image") is None else
                             f"picture from {ob['t_capture']} is not after the arm finished ({self.job['t_done']})")
                if self.job["reshoots"] >= MAX_RESHOOT:
                    why = "no confirmation picture" if (ob is None or ob.get("image") is None) else "confirmation picture is stale"
                    self.want_shot = False
                    self.hold(self.job["track"], why)
                    return self.next_action()
                self.job["reshoots"] += 1
                self.img = None
                return {"reshoot": True}
            self.want_shot = False
            return self.verify()
        for t, why in self.conflict.items():
            if t not in self.decl:
                self.hold(t, why)
        todo = [t for t, ev in sorted(self.tracks.items()) if ev["verdict"] == "NG" and t not in self.decl]
        if not todo:
            for t, ev in self.tracks.items():
                if ev["verdict"] == "OK":
                    self.decl.setdefault(t, {"state": "pass", "why": "OK verdict"})
            return {"end": self.decl}
        if self.img is None or self.img.get("image") is None:
            if self.first_tries >= MAX_RESHOOT:
                for t in todo:
                    self.hold(t, f"no picture after {self.first_tries + 1} tries")
                return self.next_action()
            self.first_tries += 1
            return {"reshoot": True}
        belt, _ = detect(self.img, self.MARK_FILLED, self.BORDER_DARK)
        tid = todo[0]
        if not self.TRACK:     # the picture is only kept as the "before" for the re-shoot check
            ev = self.tracks[tid]
            return self.plan_pick(tid, {"x": round(self.predict(ev), 2), "z": 0.0}, "predicted from encoder (tracking off)")
        open_tracks = [t for t in self.tracks if t not in self.decl]
        if self.VIEW_AWARE:     # run I-2: leave out tracks predicted clear of the view; a track at the edge -> hold
            where = {t: self.in_view(t) for t in open_tracks}
            edge = [t for t in open_tracks if where[t] == "edge"]
            if where[tid] != "in" or edge:
                why = (f"NG track {tid} predicted at x {self.predict(self.tracks[tid]):.0f}, {where[tid]} of the view"
                       if where[tid] != "in" else f"track {edge[0]} predicted at the edge of the view (x "
                       f"{self.predict(self.tracks[edge[0]]):.0f}): cannot tell whether it should be in the picture")
                self.hold(tid, why)
                return self.next_action()
            open_tracks = [t for t in open_tracks if where[t] == "in"]
        big = [x for x in belt if x["area"] > 1.5 * PART_AREA_MM2]
        if self.BLOB_CHECK and big:
            self.hold(tid, f"a part-like blob of {big[0]['area']:.0f} mm2 at x {big[0]['x']:.0f} is larger than 1.5 parts"
                           " (parts touching or overlapping, or something that is not a part)")
            return self.next_action()
        long_ = [x for x in belt if x["len_x"] > PART_LEN + OUTLINE_TOL or x["wid_z"] > PART_WID + OUTLINE_TOL]
        if self.OUTLINE_CHECK and long_:
            self.hold(tid, f"a part-like blob of {long_[0]['len_x']:.0f} x {long_[0]['wid_z']:.0f} mm at x {long_[0]['x']:.0f}"
                           f" is bigger than one part ({PART_LEN:.0f} x {PART_WID:.0f}) by more than {OUTLINE_TOL:.0f} mm"
                           " (parts touching or overlapping, or something that is not a part)")
            return self.next_action()
        if len(belt) != len(open_tracks):
            self.hold(tid, f"picture shows {len(belt)} parts, tracking expects {len(open_tracks)}")
            return self.next_action()
        m = self.match(belt)
        b = m.get(tid)
        need_mark = self.tracks[tid].get("defect_feature") == "top_mark"
        ng_open = [t for t in open_tracks if self.tracks[t]["verdict"] == "NG"]
        if self.NG2_PENDING:
            ng_open = [t for t, ev in sorted(self.tracks.items()) if ev["verdict"] == "NG"
                       and (self.decl.get(t) or {}).get("state") != "complete"]
        if self.NG2_GUARD and len(ng_open) >= 2:
            away = [t for t in ng_open if t not in open_tracks] if self.NG2_ALL else []   # held, or not inside the view
            if away:
                self.hold(tid, f"{len(ng_open)} NG tracks not complete ({', '.join(ng_open)}): NG track {away[0]} is " +
                          ("held" if (self.decl.get(away[0]) or {}).get("state") == "hold" else "not inside the view") +
                          ", so a marked part here may be its body")
                return self.next_action()
            lost = [t for t in open_tracks if m.get(t) is None]
            unmarked = ([t for t in open_tracks if self.tracks[t]["verdict"] == "NG" and m.get(t) is not None
                         and not m[t]["mark"]] if self.NG2_PENDING else [])
            if lost or b is None or (need_mark and not b["mark"]) or unmarked:
                self.hold(tid, f"{len(ng_open)} NG tracks open ({', '.join(ng_open)}): " +
                          (f"track {lost[0]} matches no part, so a marked part may belong to another NG track" if lost
                           else f"the part matched to {tid} shows no defect mark" if not b["mark"]
                           else f"the part matched to NG track {unmarked[0]} shows no defect mark"))
                return self.next_action()
        if b is not None and (b["mark"] or not need_mark):
            return self.plan_pick(tid, b, "tracked position, defect mark seen" if need_mark else "tracked position")
        marked = [x for x in belt if x["mark"]]
        if need_mark and len(marked) == 1:
            return self.plan_pick(tid, marked[0], "re-identified by the defect mark (tracking " +
                                  ("pointed at an unmarked part" if b is not None else f"found nothing within {MATCH_TOL} mm") + ")")
        self.hold(tid, "cannot tell which part is the NG one (" +
                  (f"{len(marked)} marked parts" if need_mark else "no match") + ")")
        return self.next_action()

    def verify(self):
        j = self.job
        belt, bin_n = detect(self.img, self.MARK_FILLED, self.BORDER_DARK)
        still = [b for b in belt if np.hypot(b["x"] - j["x"], b["z"] - j["z"]) <= PICK_TOL]
        if still:
            self.log(t_image=self.img["t_capture"], decide="still there", track=j["track"], why="part still at the pick point")
            if j["repicks"] >= MAX_REPICK:
                self.hold(j["track"], f"pick failed {j['repicks'] + 1} times")
                return self.next_action()
            return self.plan_pick(j["track"], still[0], "re-pick: part still at the pick point")
        if bin_n != j["bin_before"] + 1:
            self.hold(j["track"], f"pick point empty but bin count {j['bin_before']} -> {bin_n}")
            return self.next_action()
        if len(belt) != j["n_belt_before"] - 1:
            self.hold(j["track"], f"belt count {j['n_belt_before']} -> {len(belt)} (another part moved?)")
            return self.next_action()
        self.decl[j["track"]] = {"state": "complete", "why": f"fresh picture {self.img['t_capture']}: pick point empty, bin +1"}
        if self.PROVISIONAL and j.get("ng2"):
            self.provisional.append(j["track"])
            self.decl[j["track"]]["why"] += " (provisional: another NG track is not complete yet)"
        self.log(t_image=self.img["t_capture"], decide="complete", track=j["track"], why=self.decl[j["track"]]["why"])
        self.job = None
        return self.next_action()


class CandidateV9(Candidate):
    """the candidate at the end of run I-6 (v9): a completion stays even when a later NG track is held, so after two
    marked NG parts traded places a completion on the other track's body stays (review C42-I6 i6-1)."""
    name = "candidate_v9"
    PROVISIONAL = False


class CandidateV8(CandidateV9):
    """the candidate at the end of run I-5 (v8): a held or out-of-view NG track drops out of the one-to-one match, so
    its marked body can be picked by position for another NG track (review C42n round 2)."""
    name = "candidate_v8"
    NG2_ALL = False


class CandidateV7(CandidateV8):
    """the candidate after the two-NG-tracks guard of run I-5 (v7): counts only OPEN in-view NG tracks, checks only the
    picked track's own mark (review C42n 1-2)."""
    name = "candidate_v7"
    NG2_PENDING = False


class CandidateV6(CandidateV7):
    """the candidate after the picture-edge fix of run I-4 (v6), before the two-NG-tracks guard v7."""
    name = "candidate_v6"
    NG2_GUARD = False


class CandidateV5(CandidateV6):
    """the candidate after the mark fix of run I-3 (v5), before the picture-edge fix v6."""
    name = "candidate_v5"
    BORDER_DARK = False


class CandidateV4(CandidateV5):
    """the candidate after the overlap fix of run I-3 (outline check), before the mark fix v5."""
    name = "candidate_v4"
    MARK_FILLED = False


class CandidateV3(CandidateV4):
    """the candidate at the end of run I-2 (area check: a belt blob larger than 1.5 parts -> hold), before run I-3."""
    name = "candidate_v3"
    BLOB_CHECK = True
    OUTLINE_CHECK = False


class CandidateV2(CandidateV3):
    """the candidate after the first fix of run I-2 (out-of-view tracks not counted), before review C42h-1."""
    name = "candidate_v2"
    BLOB_CHECK = False
    FIRST_COUNTS = False


class CandidateV1(CandidateV2):
    """the candidate as it was in run I-1 (counts every open track against the picture, even one outside the view)."""
    name = "candidate_v1"
    VIEW_AWARE = False


class NoTrack(Candidate):
    """ablation: candidate without the tracking check (picks at the encoder prediction, still re-shoots and confirms)."""
    name = "no_track"
    TRACK = False


class NoReshoot(Candidate):
    """ablation: candidate without the re-shoot (tracks with the picture, completes on the arm's done signal)."""
    name = "no_reshoot"
    RESHOOT = False


class AllHold(Candidate):
    """negative control: holds everything. The acceptance check must fail it on the normal case."""
    name = "all_hold"

    def next_action(self):
        if self.enc is None:
            return None
        for t, ev in self.tracks.items():
            self.decl.setdefault(t, {"state": "hold" if ev["verdict"] == "NG" else "pass", "why": "always hold"})
        return {"end": self.decl}


CONTROLLERS = {"baseline": Baseline, "candidate": Candidate, "candidate_v1": CandidateV1, "candidate_v2": CandidateV2,
               "candidate_v3": CandidateV3, "candidate_v4": CandidateV4, "candidate_v5": CandidateV5, "candidate_v6": CandidateV6, "candidate_v7": CandidateV7, "candidate_v8": CandidateV8, "candidate_v9": CandidateV9, "no_track": NoTrack, "no_reshoot": NoReshoot, "all_hold": AllHold}
