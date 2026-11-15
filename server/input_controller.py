"""OS mouse input control, multi-monitor clamping, and global ESC safety listener."""

from __future__ import annotations

import ctypes
import sys
from typing import Callable

from pynput import keyboard
from pynput.mouse import Button, Controller as MouseController

# Virtual screen system metrics on Windows (supports negative multi-monitor offsets).
SM_XVIRTUALSCREEN = 76
SM_YVIRTUALSCREEN = 77
SM_CXVIRTUALSCREEN = 78
SM_CYVIRTUALSCREEN = 79

SCROLL_STEP = 2

BUTTON_MAP = {
    "left": Button.left,
    "right": Button.right,
    "middle": Button.middle,
}


def get_virtual_screen_bounds() -> tuple[int, int, int, int]:
    """Return (min_x, min_y, max_x, max_y) inclusive bounds across all monitors."""
    if sys.platform == "win32":
        user32 = ctypes.windll.user32
        min_x = int(user32.GetSystemMetrics(SM_XVIRTUALSCREEN))
        min_y = int(user32.GetSystemMetrics(SM_YVIRTUALSCREEN))
        width = int(user32.GetSystemMetrics(SM_CXVIRTUALSCREEN))
        height = int(user32.GetSystemMetrics(SM_CYVIRTUALSCREEN))
        if width > 0 and height > 0:
            return min_x, min_y, min_x + width - 1, min_y + height - 1

    # Fallback bounds for non-Windows or unavailable desktop metrics.
    return 0, 0, 1919, 1079


class InputController:
    """Executes mouse actions via pynput with screen-bounds clamping and ESC kill-switch."""

    def __init__(self) -> None:
        self._mouse = MouseController()
        self._pressed_buttons: set[Button] = set()
        self._esc_listener: keyboard.Listener | None = None

    def start_esc_listener(self, on_esc: Callable[[], None]) -> None:
        """Start a global keyboard listener that invokes on_esc whenever ESC is pressed."""
        if self._esc_listener is not None:
            return

        def _on_press(key: keyboard.Key | keyboard.KeyCode | None) -> None:
            if key == keyboard.Key.esc:
                on_esc()

        self._esc_listener = keyboard.Listener(on_press=_on_press)
        self._esc_listener.daemon = True
        self._esc_listener.start()

    def stop_esc_listener(self) -> None:
        """Stop the global keyboard listener if running."""
        if self._esc_listener is not None:
            self._esc_listener.stop()
            self._esc_listener = None

    def move_cursor(self, dx: int, dy: int) -> None:
        """Move the mouse cursor by relative (dx, dy), clamped to virtual desktop bounds."""
        if dx == 0 and dy == 0:
            return

        try:
            pos = self._mouse.position
            if pos is None:
                return

            cur_x, cur_y = pos
            min_x, min_y, max_x, max_y = get_virtual_screen_bounds()
            new_x = max(min_x, min(max_x, int(cur_x + dx)))
            new_y = max(min_y, min(max_y, int(cur_y + dy)))
            self._mouse.position = (new_x, new_y)
        except Exception:
            # Ignore transient OS cursor errors (e.g., UAC prompt or locked desktop).
            pass

    def handle_button(self, button_name: str, action: str = "click") -> None:
        """Execute a mouse button press/release/click or vertical scroll action."""
        try:
            if button_name == "scroll_up":
                self._mouse.scroll(0, SCROLL_STEP)
                return
            if button_name == "scroll_down":
                self._mouse.scroll(0, -SCROLL_STEP)
                return

            btn = BUTTON_MAP.get(button_name)
            if btn is None:
                return

            if action == "down":
                if btn not in self._pressed_buttons:
                    self._mouse.press(btn)
                    self._pressed_buttons.add(btn)
            elif action == "up":
                self._mouse.release(btn)
                self._pressed_buttons.discard(btn)
            elif action == "click":
                self._mouse.click(btn, 1)
        except Exception:
            pass

    def release_all(self) -> None:
        """Release any mouse buttons currently held down."""
        for btn in list(self._pressed_buttons):
            try:
                self._mouse.release(btn)
            except Exception:
                pass
        self._pressed_buttons.clear()
