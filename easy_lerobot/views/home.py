import streamlit as st

from core import startup
from core.help import help_button
from core.ui import set_test_mode, test_mode, test_mode_badge

MODES = [
    ("mirrorMODE", "views/mirror_mode.py", "リーダーの動きをフォロワーに映す"),
    ("LearnMODE", "views/learn_mode.py", "お手本を録画して学習させる"),
]

# Streamlit tags each keyed widget with a "st-key-<key>" class; use it to turn
# the mode buttons into large boxes.
st.html(
    """
    <style>
    [class*="st-key-mode_"] button {
        height: 180px;
        border-radius: 16px;
        border-width: 2px;
    }
    [class*="st-key-mode_"] button p {
        font-size: 0.95rem;
        line-height: 1.8;
    }
    [class*="st-key-mode_"] button p strong {
        font-size: 1.6rem;
    }
    </style>
    """
)


def on_test_mode_change():
    set_test_mode(st.session_state["_test_mode_toggle"])


title_col, gear_col = st.columns([6, 1], vertical_alignment="center")
with title_col:
    st.title("easy_lerobot")
with gear_col:
    with st.container(horizontal=True, horizontal_alignment="right"):
        help_button()
        with st.popover("", icon=":material/settings:", help="設定"):
            st.toggle(
                "テストモード",
                key="_test_mode_toggle",
                value=test_mode(),
                on_change=on_test_mode_change,
                help="アームをつながずに画面の流れだけを確かめます。ポート・モーター・ミラー・録画はすべて偽物になります。",
            )
            st.caption("アームなしで画面の流れを確かめるためのモードです。")

test_mode_badge()
st.write("使うモードを選んでください。")

for col, (name, page, caption) in zip(st.columns(2), MODES):
    with col:
        if st.button(f"**{name}**  \n{caption}", key=f"mode_{name}", width="stretch"):
            st.session_state["mode"] = name
            st.switch_page(page)


@st.cache_resource
def startup_log() -> startup.StartupLog:
    """One log per server process: its start time is when the screen became reachable."""
    return startup.StartupLog()


@st.fragment(run_every=2)
def readiness():
    """Boot log after power-on; refreshes on its own so nobody has to reload the page."""
    log = startup_log()
    checks = startup.run_checks()
    log.update(checks)
    with st.container(border=True):
        if startup.ready(checks):
            st.markdown(":green[:material/check_circle: **準備完了**]")
        else:
            st.markdown(":orange[:material/hourglass_top: **準備中**]　基板をつなぐと自動で進みます")
        rows = log.lines(checks)
        st.code("\n".join(f"{t:>8}  {mark}  {text}" for t, mark, text in rows), language=None, wrap_lines=True)


readiness()
