"""Small UI pieces shared by the pages."""

import tempfile
from pathlib import Path

import streamlit as st

# Test mode is a developer switch kept in a flag file: it holds for every browser
# session and survives reloads, and Streamlit re-importing this module.
TEST_MODE_FLAG = Path(tempfile.gettempdir()) / "easy_lerobot" / "test_mode.flag"


def test_mode() -> bool:
    """True while the UI runs without hardware (fake ports, motors and processes)."""
    return TEST_MODE_FLAG.is_file()


def set_test_mode(on: bool):
    if on:
        TEST_MODE_FLAG.parent.mkdir(parents=True, exist_ok=True)
        TEST_MODE_FLAG.touch()
    else:
        TEST_MODE_FLAG.unlink(missing_ok=True)


def test_mode_badge():
    if test_mode():
        st.badge("テストモード", icon=":material/science:", color="orange")


# Runs in the browser. Every second it asks the server's health endpoint whether it is
# still there; after two misses in a row (shutdown, reboot, Wi-Fi gone) it covers the
# whole page with a spinner so nobody presses buttons that can no longer reach the arms.
# When the server answers again the cover is removed; Streamlit reconnects by itself and,
# if the server was restarted, starts a fresh session (the password page).
_GUARD_JS = """
<script>
// The watcher is copied into the parent page and runs there, so it keeps going even
// when Streamlit re-creates this little iframe on a rerun.
const watcher = () => {
  const w = window, d = document;
  if (w.__easyLerobotGuard) return;  // one watcher per page, however often the app reruns
  w.__easyLerobotGuard = true;
  const css = d.createElement("style");
  css.textContent = `
    #el-offline { position: fixed; inset: 0; z-index: 2147483647; display: none;
      align-items: center; justify-content: center; flex-direction: column; gap: 18px;
      background: rgba(15, 17, 22, .86); color: #fff; font: 600 1.05rem/1.6 sans-serif;
      text-align: center; padding: 24px; }
    #el-offline.on { display: flex; }
    #el-offline .spin { width: 56px; height: 56px; border-radius: 50%;
      border: 6px solid rgba(255,255,255,.25); border-top-color: #fff;
      animation: el-spin .9s linear infinite; }
    #el-offline small { font-weight: 400; opacity: .8; }
    @keyframes el-spin { to { transform: rotate(360deg); } }`;
  d.head.appendChild(css);
  const cover = d.createElement("div");
  cover.id = "el-offline";
  cover.innerHTML = '<div class="spin"></div><div>ラズパイと接続が切れました</div>' +
    '<small>シャットダウン中・再起動中は操作できません。<br>戻ったら自動で再開します。</small>';
  d.body.appendChild(cover);
  let misses = 0;
  const check = async () => {
    let ok = false;
    try {
      const r = await w.fetch(new URL("_stcore/health", w.location.href.split("#")[0].replace(/[^/]*$/, "")),
                              { cache: "no-store", signal: AbortSignal.timeout(1500) });
      ok = r.ok;
    } catch (e) { ok = false; }
    misses = ok ? 0 : misses + 1;
    cover.classList.toggle("on", misses >= 2);
  };
  w.setInterval(check, 1000);
};
if (!window.parent.__easyLerobotGuard) {
  const tag = window.parent.document.createElement("script");
  tag.textContent = "(" + watcher.toString() + ")();";
  window.parent.document.head.appendChild(tag);
}
</script>
"""


def connection_guard():
    """Cover the page with a spinner while the server cannot be reached."""
    import streamlit.components.v1 as components

    components.html(_GUARD_JS, height=0)


# One look for every page, after the festival mock-up: near-black background with a soft
# purple glow, rounded dark cards, a purple accent, green "ok" dots, a red stop button.
_THEME_CSS = """
<style>
.stApp {
    background: radial-gradient(110% 60% at 50% -10%, rgba(124,58,237,.28) 0%, rgba(10,10,15,0) 60%), #0a0a0f !important;
}
/* Hide Streamlit's own top bar (Deploy / menu): the page should look like an app on the phone */
header[data-testid="stHeader"], [data-testid="stToolbar"], [data-testid="stDecoration"] { display: none !important; }
.block-container { padding-top: 1.2rem; }
/* Tighter vertical rhythm so the main controls fit on one phone screen */
[data-testid="stVerticalBlock"] { gap: .55rem; }
@media (max-width: 640px) { .block-container { padding-left: 1.1rem !important; padding-right: 1.1rem !important; } }
/* Page title: bold italic with a purple swoosh under it */
.el-title { text-align: center; margin: .2rem 0 0; }
.el-title h1 { font-size: clamp(2.5rem, 12vw, 3.3rem); font-weight: 900; font-style: italic; letter-spacing: -.01em;
    margin: 0; padding: 0; color: #fff; line-height: 1.15; }
.el-title .swoosh { width: 52%; max-width: 280px; height: 7px; margin: -2px auto 0; border-radius: 99px;
    transform: translateX(14%) rotate(-2deg);
    background: linear-gradient(90deg, #6d28d9, #a855f7 60%, #d8b4fe); box-shadow: 0 0 16px rgba(168,85,247,.6); }
.el-sub { text-align: center; color: #c4c4d4; margin: .5rem 0 .9rem; font-size: 1.05rem; letter-spacing: .04em; }
/* Cards: st.container(border=True, key="el_card_...") and our own rows */
[class*="st-key-el_card"], .el-card {
    background: #15151c !important; border: 1px solid #2a2a36 !important; border-radius: 18px !important;
}
.el-row { display: flex; align-items: center; justify-content: space-between; gap: 12px;
    background: linear-gradient(180deg, #17171f, #121218); border: 1px solid #2a2a36; border-radius: 20px;
    padding: 14px 18px; margin-bottom: 10px; box-shadow: 0 6px 18px rgba(0,0,0,.35); }
.el-row .name { font-size: 1.4rem; font-weight: 900; line-height: 1.25; color: #fff; }
.el-row .what { font-size: .92rem; color: #9a9aab; margin-top: 2px; }
.el-row .state { display: flex; align-items: center; gap: 12px; padding-left: 18px; border-left: 1px solid #33333f;
    min-height: 46px; font-weight: 900; font-size: 1.25rem; color: #6b6b7b; white-space: nowrap; }
.el-row .state.on { color: #4ade80; }
.el-dot { display: inline-block; width: 18px; height: 18px; border-radius: 50%; background: #4b4b5a; flex: none; }
.el-dot.on { background: #4ade80; box-shadow: 0 0 14px rgba(74,222,128,.95); }
.el-status { display: flex; justify-content: center; align-items: center; gap: 16px; margin: .9rem 0 .2rem;
    font-size: clamp(2.6rem, 13vw, 3.4rem); font-weight: 900; color: #fff; line-height: 1.1; }
.el-status .el-dot { width: 26px; height: 26px; }
.el-live { font-size: 1rem; font-weight: 800; letter-spacing: .08em; color: #86efac; padding: 5px 12px; border-radius: 12px;
    border: 1.5px solid rgba(74,222,128,.55); background: rgba(74,222,128,.12); }
.el-status-sub { text-align: center; color: #c4c4d4; margin-bottom: .8rem; font-size: 1rem; }
/* Bigger switches for fingers */
[data-testid="stToggle"] label, .stToggle label { transform: scale(1.35); transform-origin: right center; }
/* Buttons: rounded; "go" buttons purple, "stop" buttons red (pages add .st-key-* rules) */
.stButton button { border-radius: 14px; }
</style>
"""


def apply_theme():
    """Shared styles for all pages (called once per run from app.py)."""
    st.html(_THEME_CSS)


def page_title(title: str, sub: str = ""):
    """Centered page title with the purple swoosh, and an optional one-line description."""
    st.html(f'<div class="el-title"><h1>{title}</h1><div class="swoosh"></div></div>'
            + (f'<div class="el-sub">{sub}</div>' if sub else ""))


def festival_badge():
    """Festival name and date, shown at the top of the pages."""
    st.html(
        '<div style="display:flex;justify-content:center;margin:.1rem 0 .5rem">'
        '<div style="display:inline-flex;gap:.7em;align-items:center;padding:5px 16px;border-radius:99px;'
        'background:rgba(124,58,237,.16);border:1px solid rgba(167,139,250,.55);color:#e9e3ff;'
        'font-weight:800;letter-spacing:.08em;font-size:.95rem">'
        '<span>文化祭</span><span style="color:#c4b5fd">2026</span><span style="color:#c4b5fd">11/23</span>'
        '</div></div>'
    )
