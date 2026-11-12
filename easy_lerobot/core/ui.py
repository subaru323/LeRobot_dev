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
