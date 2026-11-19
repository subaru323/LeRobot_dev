"""Entry point of the easy_lerobot web app.

Run from easy_lerobot/:
    uv run streamlit run app.py

When .streamlit/config.toml serves the app to the LAN, anyone on the same Wi-Fi can
open it, so the pages are only registered after the password in
.streamlit/secrets.toml (app_password) has been entered.
"""

import hmac
import time

import streamlit as st

from core.ui import apply_theme, connection_guard, festival_badge, page_title

# The value shipped in secrets.toml.example; refusing it keeps the app closed until a real password is set.
EXAMPLE_PASSWORD = "change-me"

st.set_page_config(page_title="easy_lerobot", layout="centered")
# Grey the page out with a spinner while the Pi is shutting down or rebooting.
connection_guard()
apply_theme()


def password_entered() -> bool:
    """Show the password form until the right password is given; True once it was."""
    if st.session_state.get("authenticated"):
        return True
    try:
        expected = st.secrets["app_password"]
        if not str(expected).strip() or expected == EXAMPLE_PASSWORD:
            raise KeyError("app_password")
    except (KeyError, FileNotFoundError):
        festival_badge()
        page_title("easy_lerobot")
        st.error("パスワードが設定されていません（見本の値のままでは使えません）。")
        st.caption("`easy_lerobot/.streamlit/secrets.toml` に `app_password = \"...\"` を書いて、画面のサーバーを起動し直してください（ラズパイでは `sudo systemctl restart easy-lerobot`）。")
        return False
    festival_badge()
    page_title("easy_lerobot", "パスワードを入れてください")
    with st.form("login"):
        given = st.text_input("パスワード", type="password")
        submitted = st.form_submit_button("入る", width="stretch")
    if submitted:
        if hmac.compare_digest(given.encode("utf-8"), str(expected).encode("utf-8")):
            st.session_state["authenticated"] = True
            st.rerun()
        time.sleep(1.0)  # slows down guessing over the LAN
        st.error("パスワードが違います。")
    return False


if not password_entered():
    # No page is registered, so typing a page URL does not get past this point either.
    st.stop()

home = st.Page("views/home.py", title="モード選択", default=True)
mirror = st.Page("views/mirror_mode.py", title="mirrorMODE", url_path="mirror")
learn = st.Page("views/learn_mode.py", title="LearnMODE", url_path="learn")
record = st.Page("views/record.py", title="録画", url_path="record")

# The sidebar menu is hidden so that users always pick a mode on the home page.
st.navigation([home, mirror, learn, record], position="hidden").run()
