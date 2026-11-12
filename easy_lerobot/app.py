"""Entry point of the easy_lerobot web app.

Run from easy_lerobot/:
    uv run streamlit run app.py
"""

import streamlit as st

st.set_page_config(page_title="easy_lerobot", layout="centered")

home = st.Page("views/home.py", title="モード選択", default=True)
mirror = st.Page("views/mirror_mode.py", title="mirrorMODE", url_path="mirror")
learn = st.Page("views/learn_mode.py", title="LearnMODE", url_path="learn")
record = st.Page("views/record.py", title="録画", url_path="record")

# The sidebar menu is hidden so that users always pick a mode on the home page.
st.navigation([home, mirror, learn, record], position="hidden").run()
