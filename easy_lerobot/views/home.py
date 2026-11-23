import streamlit as st

from core import startup
from core.ui import festival_badge, page_title, set_test_mode, test_mode, test_mode_badge

MODES = [
    ("mirrorMODE", "views/mirror_mode.py", "リーダーの動きをフォロワーに映す"),
    ("LearnMODE", "views/learn_mode.py", "お手本を録画して学習させる"),
]

# Streamlit tags each keyed widget with a "st-key-<key>" class; use it to turn
# the mode buttons into large boxes.
st.html(
    """
    <style>
    .st-key-gear_row { margin-bottom: -2.6rem; position: relative; z-index: 2; }
    [class*="st-key-mode_"] button {
        height: 180px;
        border-radius: 22px;
        border: 1px solid #2a2a36;
        background: #15151c;
        color: #fff;
    }
    [class*="st-key-mode_"] button:hover { border-color: #a78bfa; box-shadow: 0 0 20px rgba(167,139,250,.35); color: #fff; }
    [class*="st-key-mode_"] button p {
        font-size: 0.95rem;
        line-height: 1.8;
    }
    [class*="st-key-mode_"] button p strong {
        font-size: 2rem;
        font-weight: 900;
        font-style: italic;
    }
    [class*="st-key-mode_"] button { height: 150px; background: linear-gradient(180deg, #17171f, #121218);
        box-shadow: 0 6px 18px rgba(0,0,0,.35); }
    .st-key-mode_mirrorMODE button { border-color: rgba(167,139,250,.55); }
    </style>
    """
)


def on_test_mode_change():
    set_test_mode(st.session_state["_test_mode_toggle"])


# Settings gear on its own row at the top right, so it stays there on a narrow phone screen
# (side-by-side columns stack vertically on phones and pushed it under the title).
with st.container(horizontal=True, horizontal_alignment="right", key="gear_row"):
    with st.popover("", icon=":material/settings:", help="設定"):
        st.toggle(
            "テストモード",
            key="_test_mode_toggle",
            value=test_mode(),
            on_change=on_test_mode_change,
            help="アームをつながずに画面の流れだけを確かめます。ポート・モーター・ミラー・録画はすべて偽物になります。",
        )
        st.caption("アームなしで画面の流れを確かめるためのモードです。")
festival_badge()
page_title("easy_lerobot", "ロボットアームを動かしてみよう")

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
    with st.container(border=True, key="el_card_ready"):
        if startup.ready(checks):
            st.markdown(":green[:material/check_circle: **準備完了**]")
        else:
            st.markdown(":orange[:material/hourglass_top: **準備中**]　基板をつなぐと自動で進みます")
        rows = log.lines(checks)
        st.code("\n".join(f"{t:>8}  {mark}  {text}" for t, mark, text in rows), language=None, wrap_lines=True)


readiness()
