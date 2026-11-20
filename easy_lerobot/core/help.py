"""Help button: shows the festival guide illustrations in a dialog.

The images live in easy_lerobot/assets/help/ as small JPEGs so they load quickly
even over the VPS relay on a slow Wi-Fi.
"""
from pathlib import Path

import streamlit as st

HELP_DIR = Path(__file__).resolve().parent.parent / "assets" / "help"

# (tab label, file name, short text under the picture)
PAGES = [
    ("あそびかた", "01.jpg", "係の人の合図で「ミラー開始」を押して、手前のアーム（リーダー）をゆっくり動かします。終わったら「停止」。"),
    ("してはいけないこと", "02.jpg", "動いているアーム（フォロワー）にさわらない・指を入れない。リーダーは振り回さない。ケーブルにさわらない。"),
    ("こまったとき", "03.jpg", "始める前に2台を同じポーズに。変な動き・音・においがしたら、すぐ「停止」を押して係の人へ。画面がくるくるのときは待ちます。"),
    ("係の人へ", "04.jpg", "ディスプレイはラズパイの電源を入れる前につなぐ。止めた後もフォロワーは力が入っているので、腕を支えてから電源を切る。ラズパイはボタンで切り、緑のランプが落ち着いてから抜く。"),
]


@st.dialog("ヘルプ", width="large")
def _show():
    for tab, (_, name, text) in zip(st.tabs([p[0] for p in PAGES]), PAGES):
        with tab:
            path = HELP_DIR / name
            if path.exists():
                st.image(str(path), width="stretch")
            st.write(text)


def help_button(key: str = "help_button"):
    """A '?' button that opens the guide; put it next to the settings gear."""
    if st.button("", icon=":material/help:", help="ヘルプ", key=key, type="tertiary"):
        _show()
