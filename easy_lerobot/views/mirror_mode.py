import json
import subprocess
import sys
from pathlib import Path

import streamlit as st

from core import align, logs, procs
from core.hardware import arm_ports, present_ports
from core.ui import festival_badge, test_mode, test_mode_badge

APP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = APP_DIR.parent
MIRROR_SCRIPT = REPO_DIR / "dev_scripts" / "mirror_leader.py"
FAKE_MIRROR = APP_DIR / "core" / "fake_mirror.py"
MIRROR_LOG = logs.path("mirror")  # kept across runs and restarts (core/logs.py)
MOTOR_IDS = ["1", "2", "3", "4", "5", "6"]
# Same as the "Mirror L -> F" launch config: joint 5 (wrist_roll) moves through the 4095/0 seam, so it is
# range-mapped and driven in speed mode (see docs/LOG.md P-004). --monitor-hz 0 prints one goal line per
# second instead of a carriage-return live display, which would grow the log as a single endless line.
# The follower is always moved slowly to the leader pose first (core/align.py).
MIRROR_OPTIONS = [
    *align.ALIGN_FLAGS,
    *align.JOINT5_FLAGS,
    "--vel-max", "3000", "--vel-kp", "6",
    "--monitor-hz", "0",
]

test = test_mode()
LEADER_PORT, FOLLOWER_PORT = arm_ports()  # detected on the Pi; fixed COM ports on Windows


def follower_torque(action: str) -> dict:
    """Run core.follower_torque in a separate process and return its JSON result."""
    if test:
        # Fake motors: remember the switch in the session instead of touching the bus.
        if action != "status":
            st.session_state["fake_torque"] = action == "on"
        on = st.session_state.get("fake_torque", False)
        return {"ok": True, "torque": {i: on for i in MOTOR_IDS}, "errors": {}}
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "core.follower_torque", action, "--port", FOLLOWER_PORT],
            cwd=APP_DIR,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=15,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "torque": {}, "errors": {"timeout": "no answer within 15 s"}}
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {"ok": False, "torque": {}, "errors": {"process": (proc.stderr or proc.stdout).strip()[-500:]}}


def refresh():
    result = follower_torque("status")
    st.session_state["torque_result"] = result
    st.session_state["torque_on"] = bool(result["torque"]) and all(result["torque"].values())


def on_toggle():
    if st.session_state["torque_on"] and not test:
        # Every use of the follower's servos starts by moving it slowly to the leader pose (core/align.py).
        with st.spinner("フォロワーをリーダーの姿勢にゆっくり合わせています…"):
            ok, message = align.align_follower(LEADER_PORT, FOLLOWER_PORT)
        if not ok:
            result = follower_torque("status")
            result["ok"] = False
            result.setdefault("errors", {})["align"] = message
            st.session_state["torque_result"] = result
            st.session_state["torque_on"] = bool(result["torque"]) and all(result["torque"].values())
            return
    result = follower_torque("on" if st.session_state["torque_on"] else "off")
    st.session_state["torque_result"] = result
    # Show what the motors actually report, not what the switch asked for.
    st.session_state["torque_on"] = bool(result["torque"]) and all(result["torque"].values())


def mirror_running() -> bool:
    return procs.running("mirror")


def start_mirror():
    st.session_state["mirror_started_here"] = True  # show this run's result, not an old one
    if test:
        st.session_state["fake_torque"] = True  # mirror_leader enables the follower's torque
    procs.start(
        "mirror",
        [str(FAKE_MIRROR)] if test else
        [str(MIRROR_SCRIPT), "--leader-port", LEADER_PORT, "--follower-port", FOLLOWER_PORT, *MIRROR_OPTIONS],
        cwd=REPO_DIR,
        log_path=MIRROR_LOG,
    )


def stop_mirror():
    procs.stop("mirror", wait_s=5.0)  # mirror_leader cleans up as on Ctrl+C


def mirror_error_message() -> str:
    """Turn the last line mirror_leader printed into a message for the user."""
    try:
        lines = [line.strip() for line in MIRROR_LOG.read_text(encoding="utf-8").splitlines() if line.strip()]
    except FileNotFoundError:
        lines = []
    last = lines[-1] if lines else ""
    # Windows quotes the port ('COM19'); Linux pyserial writes "could not open port /dev/ttyACM0: ...".
    if any(f in last for f in (f"'{LEADER_PORT}'", f"open {LEADER_PORT}", f"port {LEADER_PORT}:")):
        return f"リーダー（{LEADER_PORT}）に接続できませんでした。"
    if any(f in last for f in (f"'{FOLLOWER_PORT}'", f"open {FOLLOWER_PORT}", f"port {FOLLOWER_PORT}:")):
        return f"フォロワー（{FOLLOWER_PORT}）に接続できませんでした。"
    if last.startswith("no response from"):
        return f"応答しないモーターがあります: {last.removeprefix('no response from').strip()}"
    if "poses differ too much" in last:
        return "リーダーとフォロワーの姿勢が違いすぎます。同じ姿勢にしてから開始してください。"
    return f"ミラーが止まりました: {last}"


def arm_row(role: str, what: str, present: bool) -> str:
    """One connection row: arm name, what it is, and a coloured status (no device path)."""
    state = '<span class="dot on"></span>接続中' if present else '<span class="dot"></span>未接続'
    return (f'<div class="arm-row"><div><div class="arm-name">{role}</div><div class="arm-what">{what}</div></div>'
            f'<div class="arm-state {"on" if present else ""}">{state}</div></div>')


# --- state -----------------------------------------------------------------
busy = procs.running_name()  # "mirror", "record" or None
running = busy is not None
ports = present_ports()
leader_ok = test or LEADER_PORT in ports
follower_ok = test or FOLLOWER_PORT in ports
if st.session_state.get("torque_result_test") != test:
    # Test mode was switched: the cached motor state belongs to the other mode.
    st.session_state.pop("torque_result", None)
    st.session_state["torque_result_test"] = test
if not running and follower_ok and "torque_result" not in st.session_state:
    refresh()  # the follower port is in use while mirroring or recording
if not follower_ok:
    st.session_state.pop("torque_result", None)
result = st.session_state.get("torque_result", {"ok": False, "torque": {}, "errors": {}})
torque_known = bool(result["torque"])

st.html(
    """
    <style>
    /* Cyberpunk look: near-black page with a faint grid and scanlines, neon cyan / magenta. */
    .stApp {
        background:
            repeating-linear-gradient(0deg, rgba(255,255,255,.025) 0 1px, transparent 1px 3px),
            linear-gradient(rgba(0,240,255,.05) 1px, transparent 1px) 0 0 / 32px 32px,
            linear-gradient(90deg, rgba(0,240,255,.05) 1px, transparent 1px) 0 0 / 32px 32px,
            radial-gradient(120% 80% at 50% 0%, #1a0b2e 0%, #05060d 60%) !important;
    }
    .block-container { padding-top: 2.2rem; padding-bottom: 1rem; max-width: 560px; }
    .mm-title { text-align: center; margin: .3rem 0 .1rem; }
    .mm-title h1 {
        font-size: 2.5rem; font-weight: 900; letter-spacing: .06em; margin: 0; padding: 0; color: #e8fdff;
        text-shadow: -2px 0 #ff2bd6, 2px 0 #00f0ff, 0 0 18px rgba(0,240,255,.6);
    }
    .mm-title .bar { width: 52%; height: 3px; margin: 4px auto 0;
        background: linear-gradient(90deg, transparent, #00f0ff, #ff2bd6, transparent);
        box-shadow: 0 0 12px #00f0ff; }
    .mm-sub { text-align: center; color: #9fe9f5; margin: .5rem 0 1.1rem; font-size: .95rem; letter-spacing: .04em; }
    .arm-row {
        display: flex; align-items: center; justify-content: space-between; gap: 12px;
        padding: 14px 18px; margin-bottom: 10px; color: #e8fdff;
        background: linear-gradient(90deg, rgba(0,240,255,.10), rgba(10,12,30,.85) 45%);
        border: 1px solid rgba(0,240,255,.45); border-left: 4px solid #00f0ff;
        box-shadow: 0 0 14px rgba(0,240,255,.18), inset 0 0 18px rgba(0,240,255,.06);
        clip-path: polygon(0 0, calc(100% - 16px) 0, 100% 16px, 100% 100%, 16px 100%, 0 calc(100% - 16px));
    }
    .arm-name { font-size: 1.3rem; font-weight: 800; line-height: 1.3; letter-spacing: .04em; }
    .arm-what { font-size: .82rem; color: #7fb9c4; }
    .arm-state { font-weight: 800; font-size: 1.05rem; white-space: nowrap; color: #6b7280; letter-spacing: .06em; }
    .arm-state.on { color: #39ff88; text-shadow: 0 0 10px rgba(57,255,136,.8); }
    .dot { display: inline-block; width: 12px; height: 12px; margin-right: 8px; vertical-align: 0;
        background: #4b5563; transform: rotate(45deg); }
    .dot.on { background: #39ff88; box-shadow: 0 0 12px #39ff88; }
    .mm-status { text-align: center; margin: 1.2rem 0 .2rem; font-size: 2.3rem; font-weight: 900; color: #e8fdff;
        letter-spacing: .08em; text-shadow: -2px 0 #ff2bd6, 2px 0 #00f0ff; }
    .mm-status-sub { text-align: center; color: #9fe9f5; margin-bottom: .9rem; }
    .st-key-mirror_start button, .st-key-mirror_stop button {
        height: 150px;
        border: none;
        border-radius: 0;
        clip-path: polygon(0 0, calc(100% - 26px) 0, 100% 26px, 100% 100%, 26px 100%, 0 calc(100% - 26px));
        transition: filter .15s, transform .05s;
    }
    /* !important: Streamlit's own hover / disabled backgrounds would win otherwise. */
    .st-key-mirror_start button, .st-key-mirror_start button:hover {
        background: linear-gradient(135deg, #00f0ff, #39ff88) !important; color: #031216 !important;
        box-shadow: 0 0 28px rgba(0,240,255,.55); }
    .st-key-mirror_stop button, .st-key-mirror_stop button:hover {
        background: linear-gradient(135deg, #ff2bd6, #ff3b3b) !important; color: #fff !important;
        animation: mm-pulse 1.6s ease-in-out infinite; }
    @keyframes mm-pulse { 50% { filter: brightness(1.18); } }
    .st-key-mirror_start button:hover, .st-key-mirror_stop button:hover { filter: brightness(1.1); }
    .st-key-mirror_start button:active, .st-key-mirror_stop button:active { transform: scale(.99); }
    .st-key-mirror_start button:disabled {
        background: repeating-linear-gradient(135deg, #10131f 0 14px, #151a2a 14px 28px) !important;
        color: #4f6b75 !important; box-shadow: none; }
    .st-key-mirror_start button p, .st-key-mirror_stop button p {
        font-size: 2rem;
        font-weight: 900;
        letter-spacing: .08em;
    }
    /* The stop button is the one that matters for safety: make it the biggest thing on screen. */
    .st-key-mirror_stop button { height: 230px; }
    .st-key-mirror_stop button p { font-size: 2.6rem; line-height: 1.5; text-shadow: 0 0 12px rgba(255,255,255,.6); }
    /* Bordered containers (the torque row) in the same neon frame. */
    [data-testid="stVerticalBlockBorderWrapper"] { border-color: rgba(255,43,214,.45) !important;
        box-shadow: 0 0 14px rgba(255,43,214,.15); border-radius: 0 !important; }
    </style>
    """
)

# --- header ----------------------------------------------------------------
if st.button("戻る", icon=":material/arrow_back:", type="tertiary"):
    stop_mirror()  # never leave the arm mirroring on a page nobody is watching
    st.session_state["mirror_was_running"] = False
    st.session_state.pop("torque_result", None)
    st.switch_page("views/home.py")

festival_badge()
st.html('<div class="mm-title"><h1>mirrorMODE</h1><div class="bar"></div></div>'
        '<div class="mm-sub">リーダーを動かすと、フォロワーがまねをします</div>')
test_mode_badge()
st.html(arm_row("リーダー", "手で動かす側のアーム", leader_ok) + arm_row("フォロワー", "まねをして動く側のアーム", follower_ok))


# --- mirror start / stop (main control) -------------------------------------
@st.fragment(run_every=1.0)
def mirror_control():
    now_running = mirror_running()
    if now_running != st.session_state.get("mirror_was_running", False):
        # The process started or ended since the last check: redraw the whole page
        # so the torque row follows.
        st.session_state["mirror_was_running"] = now_running
        if not now_running:
            st.session_state.pop("torque_result", None)
        st.rerun()

    if now_running:
        st.html('<div class="mm-status"><span class="dot on"></span>ミラー中</div>'
                '<div class="mm-status-sub">リーダーを手で動かしてアームを操作します</div>')
        if st.button("■  \nミラー停止", key="mirror_stop", width="stretch"):
            stop_mirror()
            st.session_state["mirror_was_running"] = False
            st.session_state.pop("torque_result", None)
            st.rerun()
        return

    recording = procs.running("record")
    ready = leader_ok and follower_ok and not recording
    if st.button("▶  ミラー開始", key="mirror_start", width="stretch", disabled=not ready):
        start_mirror()
        st.session_state["mirror_was_running"] = True
        st.rerun()
    if recording:
        st.caption("録画中はミラーを開始できません。")
    elif not ready:
        st.caption("リーダーとフォロワーの USB と電源をつなぐと開始できます。")
    elif st.session_state.get("mirror_started_here") and procs.failed("mirror"):
        st.markdown(f":red[:material/error: {mirror_error_message()}]")
    else:
        st.caption("停止してもフォロワーはその姿勢を保ちます。")


mirror_control()

# --- follower torque (secondary control) -----------------------------------
with st.container(border=True):
    label_col, toggle_col = st.columns([5, 1], vertical_alignment="center")
    with label_col:
        st.markdown("**フォロワーのトルク**")
        if running:
            st.caption(f"{'ミラー' if busy == 'mirror' else '録画'}中は切り替えできません")
        elif not follower_ok:
            st.caption("フォロワーが未接続です")
        elif "align" in result["errors"]:
            st.markdown(f":orange[{result['errors']['align']}]")
        elif result["errors"]:
            st.markdown(f":orange[応答しないモーター: {', '.join('F' + i for i in result['errors'])}]")
        else:
            st.caption("ON でリーダーの姿勢にゆっくり合わせてから保持 ・ OFF で脱力（腕が落ちます）")
    with toggle_col:
        with st.container(horizontal=True, horizontal_alignment="right", vertical_alignment="center", gap="small"):
            st.button("", icon=":material/refresh:", type="tertiary", help="状態を読み直す",
                      on_click=refresh, disabled=running or not follower_ok)
            st.toggle("ON", key="torque_on", on_change=on_toggle, label_visibility="collapsed",
                      disabled=running or not torque_known)
