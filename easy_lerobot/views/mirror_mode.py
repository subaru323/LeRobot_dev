import json
import subprocess
import sys
import tempfile
from pathlib import Path

import streamlit as st

from core import procs
from core.hardware import FOLLOWER_PORT, LEADER_PORT, present_ports
from core.ui import test_mode, test_mode_badge

APP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = APP_DIR.parent
MIRROR_SCRIPT = REPO_DIR / "dev_scripts" / "mirror_leader.py"
FAKE_MIRROR = APP_DIR / "core" / "fake_mirror.py"
MIRROR_LOG = Path(tempfile.gettempdir()) / "easy_lerobot" / "mirror.log"
MOTOR_IDS = ["1", "2", "3", "4", "5", "6"]

test = test_mode()


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
        [str(MIRROR_SCRIPT), "--leader-port", LEADER_PORT, "--follower-port", FOLLOWER_PORT],
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
    if f"'{LEADER_PORT}'" in last or f"open {LEADER_PORT}" in last:
        return f"リーダー（{LEADER_PORT}）に接続できませんでした。"
    if f"'{FOLLOWER_PORT}'" in last or f"open {FOLLOWER_PORT}" in last:
        return f"フォロワー（{FOLLOWER_PORT}）に接続できませんでした。"
    if last.startswith("no response from"):
        return f"応答しないモーターがあります: {last.removeprefix('no response from').strip()}"
    if "poses differ too much" in last:
        return "リーダーとフォロワーの姿勢が違いすぎます。同じ姿勢にしてから開始してください。"
    return f"ミラーが止まりました: {last}"


def port_badge(role: str, port: str, present: bool):
    if present:
        st.badge(f"{role}  {port}", icon=":material/link:", color="green")
    else:
        st.badge(f"{role}  {port}  未接続", icon=":material/link_off:", color="gray")


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
    .block-container { padding-top: 3rem; padding-bottom: 1rem; max-width: 680px; }
    .st-key-mirror_start button, .st-key-mirror_stop button {
        height: 150px;
        border-radius: 24px;
        border: none;
        color: #fff;
        transition: filter .15s, transform .05s;
    }
    /* !important: Streamlit's own hover / disabled backgrounds would win otherwise. */
    .st-key-mirror_start button, .st-key-mirror_start button:hover { background: #16a34a !important; }
    .st-key-mirror_stop button, .st-key-mirror_stop button:hover { background: #dc2626 !important; }
    .st-key-mirror_start button:hover, .st-key-mirror_stop button:hover { filter: brightness(1.08); color: #fff; }
    .st-key-mirror_start button:active, .st-key-mirror_stop button:active { transform: scale(.99); }
    .st-key-mirror_start button:disabled { background: #3f3f46 !important; color: #a1a1aa !important; }
    .st-key-mirror_start button p, .st-key-mirror_stop button p {
        font-size: 2rem;
        font-weight: 800;
        letter-spacing: .04em;
    }
    </style>
    """
)

# --- header ----------------------------------------------------------------
if st.button("戻る", icon=":material/arrow_back:", type="tertiary"):
    stop_mirror()  # never leave the arm mirroring on a page nobody is watching
    st.session_state["mirror_was_running"] = False
    st.session_state.pop("torque_result", None)
    st.switch_page("views/home.py")

st.title("mirrorMODE")
with st.container(horizontal=True, gap="small"):
    test_mode_badge()
    port_badge("リーダー", LEADER_PORT, leader_ok)
    port_badge("フォロワー", FOLLOWER_PORT, follower_ok)


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
        if st.button("■  停止", key="mirror_stop", width="stretch"):
            stop_mirror()
            st.session_state["mirror_was_running"] = False
            st.session_state.pop("torque_result", None)
            st.rerun()
        st.markdown(":green-badge[:material/sync: ミラー中]　リーダーを動かすとフォロワーが追従します")
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
        elif result["errors"]:
            st.markdown(f":orange[応答しないモーター: {', '.join('F' + i for i in result['errors'])}]")
        else:
            st.caption("ON で今の姿勢を保持 ・ OFF で脱力（腕が落ちます）")
    with toggle_col:
        with st.container(horizontal=True, horizontal_alignment="right", vertical_alignment="center", gap="small"):
            st.button("", icon=":material/refresh:", type="tertiary", help="状態を読み直す",
                      on_click=refresh, disabled=running or not follower_ok)
            st.toggle("ON", key="torque_on", on_change=on_toggle, label_visibility="collapsed",
                      disabled=running or not torque_known)
