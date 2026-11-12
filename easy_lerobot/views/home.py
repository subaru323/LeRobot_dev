import streamlit as st

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
