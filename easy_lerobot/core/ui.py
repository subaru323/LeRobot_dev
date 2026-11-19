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


def festival_badge():
    """Neon badge with the festival name and date shown at the top of the operation pages."""
    st.html(
        '<div style="display:flex;justify-content:center;margin:.2rem 0 .4rem">'
        '<div style="display:inline-flex;gap:.6em;align-items:baseline;padding:6px 16px;color:#f5ff3b;'
        'border:1px solid #f5ff3b;box-shadow:0 0 12px rgba(245,255,59,.45),inset 0 0 10px rgba(245,255,59,.12);'
        'font-weight:900;letter-spacing:.12em;text-shadow:0 0 8px rgba(245,255,59,.8);'
        'clip-path:polygon(10px 0,100% 0,calc(100% - 10px) 100%,0 100%)">'
        '<span>ハイフェス</span><span style="color:#00f0ff;text-shadow:0 0 8px #00f0ff">2026</span>'
        '<span style="color:#ff2bd6;text-shadow:0 0 8px #ff2bd6">11/23</span></div></div>'
    )
