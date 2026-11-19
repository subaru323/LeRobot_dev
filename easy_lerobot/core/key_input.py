"""Send key presses to the OS.

On Windows, lerobot-record listens for Right / Left / Esc with a global pynput
listener, so a key press injected here reaches it whichever window has focus.

pynput is imported lazily: on a Linux machine without a display (a Raspberry Pi
served to a phone) importing it fails, and that must not take the whole UI down.
"""


def available() -> bool:
    """True when key presses can be sent on this machine."""
    try:
        from pynput.keyboard import Controller

        Controller()
    except Exception:
        return False
    return True


def press(name: str):
    from pynput.keyboard import Controller, Key

    key = {"right": Key.right, "left": Key.left}[name]
    keyboard = Controller()
    keyboard.press(key)
    keyboard.release(key)
