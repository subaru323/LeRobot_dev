"""Send key presses to the OS.

On Windows, lerobot-record listens for Right / Left / Esc with a global pynput
listener, so a key press injected here reaches it whichever window has focus.
"""

from pynput.keyboard import Controller, Key

_KEYS = {"right": Key.right, "left": Key.left}


def press(name: str):
    keyboard = Controller()
    key = _KEYS[name]
    keyboard.press(key)
    keyboard.release(key)
