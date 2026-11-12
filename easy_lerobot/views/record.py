from pathlib import Path

import streamlit as st

from core import key_input, procs, recording
from core.hardware import FOLLOWER_PORT, LEADER_PORT, present_ports
from core.ui import test_mode, test_mode_badge

REPO_DIR = Path(__file__).resolve().parents[2]

# Settings live in plain session keys; widgets use "w_" keys and copy their value
# back, because Streamlit drops widget state on pages that do not render the widget
# (LearnMODE reads the dataset name and episode count from here).
DEFAULTS = {
    "rec_task": "",
    "rec_dataset": recording.DEFAULT_DATASET,
    "rec_episodes": recording.DEFAULT_EPISODES,
    "rec_episode_s": recording.DEFAULT_EPISODE_S,
    "rec_reset_s": recording.DEFAULT_RESET_S,
}
for key, value in DEFAULTS.items():
    st.session_state.setdefault(key, value)


def keep(key: str):
    st.session_state[key] = st.session_state[f"w_{key}"]


def setting_input(widget, label: str, key: str, **kwargs):
    widget(label, key=f"w_{key}", value=st.session_state[key], on_change=keep, args=(key,), **kwargs)


test = test_mode()


def start_recording():
    name = st.session_state["rec_dataset"]
    st.session_state["rec_base"] = recording.recorded_episodes(name, test)
    st.session_state["rec_was_running"] = True
    st.session_state["rec_started_here"] = True  # show this run's result, not an old one
    procs.start(
        "record",
        recording.record_args(
            name,
            st.session_state["rec_task"].strip(),
            int(st.session_state["rec_episodes"]),
            int(st.session_state["rec_episode_s"]),
            int(st.session_state["rec_reset_s"]),
            test=test,
        ),
        cwd=REPO_DIR,
        log_path=recording.RECORD_LOG,
        # If a calibration file does not match the motors, lerobot asks whether to use it:
        # an empty line answers "use the provided file" instead of hanging on input().
        stdin_text="\n",
    )


def port_badge(role: str, port: str, present: bool):
    if present:
        st.badge(f"{role}  {port}", icon=":material/link:", color="green")
    else:
        st.badge(f"{role}  {port}  未接続", icon=":material/link_off:", color="gray")


busy = procs.running_name()
recording_now = busy == "record"
ports = present_ports()
leader_ok = test or LEADER_PORT in ports
follower_ok = test or FOLLOWER_PORT in ports

st.html(
    """
    <style>
    .block-container { padding-top: 3rem; padding-bottom: 1rem; max-width: 720px; }
    .st-key-rec_start button, .st-key-rec_stop button {
        height: 110px; border-radius: 22px; border: none; color: #fff;
    }
    /* !important: Streamlit's own hover / disabled backgrounds would win otherwise. */
    .st-key-rec_start button, .st-key-rec_start button:hover { background: #16a34a !important; }
    .st-key-rec_stop button, .st-key-rec_stop button:hover { background: #dc2626 !important; }
    .st-key-rec_start button:hover, .st-key-rec_stop button:hover { filter: brightness(1.08); color: #fff; }
    .st-key-rec_start button:disabled { background: #3f3f46 !important; color: #a1a1aa !important; }
    .st-key-rec_start button p, .st-key-rec_stop button p { font-size: 1.8rem; font-weight: 800; }
    .st-key-rec_next button, .st-key-rec_redo button { height: 64px; border-radius: 16px; }
    .st-key-rec_next button p, .st-key-rec_redo button p { font-size: 1.15rem; font-weight: 700; }
    .rec-phase { font-size: 1.6rem; font-weight: 800; margin: .2rem 0 .1rem; }
    </style>
    """
)

# --- header ----------------------------------------------------------------
if st.button("戻る", icon=":material/arrow_back:", type="tertiary", disabled=busy == "record"):
    st.switch_page("views/learn_mode.py")

st.title("1. 録画")
with st.container(horizontal=True, gap="small"):
    test_mode_badge()
    port_badge("リーダー", LEADER_PORT, leader_ok)
    port_badge("フォロワー", FOLLOWER_PORT, follower_ok)
    st.badge(f"カメラ  front (#{recording.FRONT_CAMERA_INDEX})", icon=":material/videocam:", color="gray")


@st.fragment(run_every=1.0)
def recording_panel():
    now = procs.running("record")
    if now != st.session_state.get("rec_was_running", False):
        st.session_state["rec_was_running"] = now
        st.rerun()  # started or finished: redraw the whole page
    if not now:
        return

    phase, episode = recording.session_phase()
    base = st.session_state.get("rec_base", 0)
    target = int(st.session_state["rec_episodes"])
    current = (episode - base + 1) if episode is not None else 0

    with st.container(border=True):
        if phase == "recording":
            st.html(f'<div class="rec-phase">🔴 録画中　{current} / {target} 回目</div>')
            st.caption("リーダーを動かしてお手本を見せてください。")
        elif phase == "reset":
            st.html(f'<div class="rec-phase">⏸ 片付け中　次は {min(current + 1, target)} 回目</div>')
            st.caption("物を元の位置に戻してください。時間が来ると次の録画が始まります。")
        elif phase == "redo":
            st.html(f'<div class="rec-phase">↺ 撮り直し　次も {current} 回目</div>')
            st.caption("今の回は捨てました。物を元の位置に戻してください。")
        elif phase == "saving":
            st.html('<div class="rec-phase">💾 保存中…</div>')
            st.caption("動画を書き出しています。終わるまでお待ちください。")
        else:
            st.html('<div class="rec-phase">⏳ 準備中…</div>')
            st.caption("アームとカメラに接続しています。")
        # The current episode is not finished while recording, and is thrown away on a redo.
        completed = max(current - 1, 0) if phase in ("recording", "redo") else current
        st.progress(min(completed / target, 1.0))

        can_control = phase in ("recording", "reset", "redo")
        next_col, redo_col = st.columns(2)
        with next_col:
            if st.button("この回を終える  ▶", key="rec_next", width="stretch", disabled=not can_control,
                         help="録画中なら今の回を保存して片付けへ、片付け中なら次の録画へ進みます"):
                key_input.press("right")
        with redo_col:
            if st.button("↺  撮り直す", key="rec_redo", width="stretch", disabled=phase != "recording",
                         help="今の回を捨てて、もう一度録画します"):
                key_input.press("left")

    if st.button("■  録画を終える", key="rec_stop", width="stretch", disabled=phase == "saving"):
        procs.stop("record", wait_s=None)  # lerobot-record saves the dataset on its own


if recording_now:
    recording_panel()

if not recording_now:
    name = st.session_state["rec_dataset"]
    saved = recording.recorded_episodes(name, test)
    with st.container(border=True):
        setting_input(st.text_input, "タスクの説明（英語推奨）", "rec_task",
                      placeholder="例: Pick the red cube and put it in the box")
        c1, c2, c3, c4 = st.columns(4)
        with c1:
            setting_input(st.number_input, "回数", "rec_episodes", min_value=1, max_value=200, step=1)
        with c2:
            setting_input(st.number_input, "1回の秒数", "rec_episode_s", min_value=3, max_value=300, step=1)
        with c3:
            setting_input(st.number_input, "片付けの秒数", "rec_reset_s", min_value=0, max_value=120, step=1)
        with c4:
            setting_input(st.text_input, "データ名", "rec_dataset")
        st.caption(f"このデータに録画済み **{saved}** 回。続きから追加で録画します。" if saved else
                   "新しいデータとして録画します。")

    ready = leader_ok and follower_ok and busy is None and st.session_state["rec_task"].strip() != ""
    if st.button("▶  録画開始", key="rec_start", width="stretch", disabled=not ready):
        start_recording()
        st.rerun()

    if busy == "mirror":
        st.caption("ミラー中は録画を開始できません。mirrorMODE で停止してください。")
    elif not (leader_ok and follower_ok):
        st.caption("リーダーとフォロワーの USB と電源をつなぐと開始できます。")
    elif not st.session_state["rec_task"].strip():
        st.caption("タスクの説明を入れると開始できます。")
    elif not st.session_state.get("rec_started_here"):
        pass
    elif procs.failed("record"):
        st.markdown(f":red[:material/error: {recording.error_message()}]")
    elif procs.finished_ok("record"):
        st.markdown(f":green[:material/check_circle: 保存しました。このデータは合計 {saved} 回です。]")
