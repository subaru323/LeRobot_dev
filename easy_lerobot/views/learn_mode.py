import streamlit as st

from core import recording
from core.ui import test_mode, test_mode_badge

STEPS = [
    {"key": "record", "icon": ":material/videocam:", "title": "録画", "desc": "リーダーで動きを見せて、お手本を録画します"},
    {"key": "train", "icon": ":material/model_training:", "title": "学習", "desc": "録画したお手本から、動き方を学習させます"},
    {"key": "run", "icon": ":material/smart_toy:", "title": "実行", "desc": "学習したモデルで、アームを動かします"},
]


def learn_progress() -> dict:
    """Progress of each step.

    Recording is read from the dataset on disk. Training and running are placeholders
    until projects/<name>/project.json exists (see the project-state spec).
    """
    dataset = st.session_state.get("rec_dataset", recording.DEFAULT_DATASET)
    return {
        "record": {
            "done": recording.recorded_episodes(dataset, test_mode()),
            "target": int(st.session_state.get("rec_episodes", recording.DEFAULT_EPISODES)),
        },
        "train": {"done": False},
        "run": {"trials": 0, "success": 0},
    }


def step_states(progress: dict) -> dict:
    """'done', 'ready' or 'locked' for each step; a step unlocks once the previous one is done."""
    record_done = progress["record"]["done"] >= progress["record"]["target"]
    train_done = progress["train"]["done"]
    return {
        "record": "done" if record_done else "ready",
        "train": "done" if train_done else "ready" if record_done else "locked",
        "run": "ready" if train_done else "locked",
    }


def status_badge(state: str):
    if state == "done":
        st.badge("完了", icon=":material/check_circle:", color="green")
    elif state == "ready":
        st.badge("準備OK", icon=":material/play_circle:", color="blue")
    else:
        st.badge("ロック中", icon=":material/lock:", color="gray")


def step_detail(key: str, progress: dict):
    if key == "record":
        done, target = progress["record"]["done"], progress["record"]["target"]
        st.markdown(f"お手本 **{done}** / {target} 回")
        st.progress(min(done / target, 1.0))
    elif key == "train":
        st.markdown("**学習済み**" if progress["train"]["done"] else "未学習")
        st.progress(1.0 if progress["train"]["done"] else 0.0)
    else:
        trials, success = progress["run"]["trials"], progress["run"]["success"]
        rate = f"{success / trials:.0%}" if trials else "—"
        st.markdown(f"成功率 **{rate}**")
        st.progress(success / trials if trials else 0.0)


progress = learn_progress()
states = step_states(progress)
current = next((s["key"] for s in STEPS if states[s["key"]] == "ready"), None)

st.html(
    """
    <style>
    .block-container { padding-top: 3rem; padding-bottom: 1rem; max-width: 960px; }
    [class*="st-key-learn_"] button { height: 52px; border-radius: 14px; }
    [class*="st-key-learn_"] button p { font-size: 1.05rem; font-weight: 700; }
    /* Same green "go" colour as the mirror start button. */
    [class*="st-key-learn_"] button[kind="primary"],
    [class*="st-key-learn_"] button[kind="primary"]:hover { background: #16a34a !important; border: none; color: #fff; }
    [class*="st-key-learn_"] button[kind="primary"]:hover { filter: brightness(1.08); color: #fff; }
    .learn-arrow { text-align: center; font-size: 1.8rem; opacity: .45; }
    </style>
    """
)

if st.button("戻る", icon=":material/arrow_back:", type="tertiary"):
    st.switch_page("views/home.py")

st.title("LearnMODE")
test_mode_badge()
st.caption("① 録画 → ② 学習 → ③ 実行 の順に進めます。前の段階が終わると、次の段階が開きます。")

cols = st.columns([1, 0.12, 1, 0.12, 1], vertical_alignment="center")
for n, step in enumerate(STEPS):
    if n > 0:
        cols[2 * n - 1].html('<div class="learn-arrow">→</div>')
    state = states[step["key"]]
    with cols[2 * n].container(border=True, height="stretch"):
        status_badge(state)
        st.markdown(f"### {step['icon']} {n + 1}. {step['title']}")
        st.caption(step["desc"])
        step_detail(step["key"], progress)
        label = "やり直す" if state == "done" else "始める"
        if st.button(label, key=f"learn_{step['key']}", width="stretch",
                     type="primary" if step["key"] == current else "secondary",
                     disabled=state == "locked"):
            if step["key"] == "record":
                st.switch_page("views/record.py")
            st.toast(f"「{step['title']}」の画面はこれから作ります。", icon=":material/construction:")
