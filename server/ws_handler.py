"""WebSocket handler for Phase 3 PhoneMotion control, calibration, config sync, and RTT ping."""

from __future__ import annotations

import asyncio
import json
import sys
import time
from typing import Any

from websockets.asyncio.server import ServerConnection
from websockets.exceptions import ConnectionClosed

try:
    from server.input_controller import InputController
    from server.motion_processor import CALIBRATION_DURATION_SEC, MotionProcessor
except ImportError:
    from input_controller import InputController
    from motion_processor import CALIBRATION_DURATION_SEC, MotionProcessor

# Minimum interval between terminal status refreshes to keep 60 Hz streams smooth.
PRINT_INTERVAL_SEC = 0.05

STATE_ACTIVE = "active"
STATE_PAUSED = "paused"
STATE_STOPPED = "stopped"


def _format_telemetry(
    state: str,
    is_calibrating: bool,
    rpy: tuple[float, float, float],
    data: dict[str, Any],
    rate_hz: float,
) -> str:
    """Format state, message rate, Madgwick RPY orientation, and gyro/accel values."""
    tag = "CALIB" if is_calibrating else state.upper()
    return (
        f"[{tag:7s}] [{rate_hz:5.1f} msg/s] "
        f"rpy=({rpy[0]:6.1f}, {rpy[1]:6.1f}, {rpy[2]:6.1f}) "
        f"gyro=({float(data.get('gx', 0)):6.3f}, {float(data.get('gy', 0)):6.3f}, {float(data.get('gz', 0)):6.3f}) "
        f"acc=({float(data.get('ax', 0)):5.2f}, {float(data.get('ay', 0)):5.2f}, {float(data.get('az', 0)):5.2f})"
    )


class ControllerSession:
    """Shared controller state coordinating WebSocket clients, sensor fusion, and OS mouse input."""

    def __init__(
        self,
        input_controller: InputController | None = None,
        motion_processor: MotionProcessor | None = None,
    ) -> None:
        self.input_controller = input_controller or InputController()
        self.motion_processor = motion_processor or MotionProcessor()
        self.state: str = STATE_ACTIVE
        self.last_reason: str = ""
        self.active_clients: set[ServerConnection] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Bind the running asyncio loop so background threads (ESC hook) can update state."""
        self._loop = loop

    def trigger_esc_stop(self) -> None:
        """Thread-safe emergency stop invoked by the laptop ESC keyboard listener."""
        if self._loop is None or self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self.set_state(STATE_STOPPED, "ESC pressed on laptop"))
        )

    async def _send_json(self, websocket: ServerConnection, payload: dict[str, Any]) -> None:
        """Send a JSON packet to one client, ignoring closed sockets."""
        try:
            await websocket.send(json.dumps(payload))
        except ConnectionClosed:
            pass

    async def _broadcast_json(self, payload: dict[str, Any]) -> None:
        """Broadcast a JSON packet to all connected clients."""
        for client in list(self.active_clients):
            await self._send_json(client, payload)

    async def _send_state(self, websocket: ServerConnection) -> None:
        """Push current control state to a single WebSocket client."""
        await self._send_json(
            websocket,
            {"type": "state", "state": self.state, "reason": self.last_reason},
        )

    async def broadcast_state(self) -> None:
        """Push current control state to all connected clients."""
        await self._broadcast_json(
            {"type": "state", "state": self.state, "reason": self.last_reason}
        )

    async def start_calibration(self) -> None:
        """Start 2-second stationary gyro bias calibration and notify connected clients."""
        self.motion_processor.start_calibration()
        print(
            f"\n[CALIB] Hold phone still for {CALIBRATION_DURATION_SEC:.1f}s to calibrate gyro bias...",
            flush=True,
        )
        await self._broadcast_json(
            {
                "type": "calibration",
                "status": "calibrating",
                "duration": CALIBRATION_DURATION_SEC,
            }
        )

    async def set_state(self, new_state: str, reason: str = "") -> None:
        """Transition control state, recenter/reset on pause changes, and sync clients."""
        if new_state == self.state and reason == self.last_reason:
            return

        if new_state == STATE_ACTIVE and self.state == STATE_PAUSED:
            self.motion_processor.recenter()
        elif new_state != STATE_ACTIVE or self.state != STATE_ACTIVE:
            self.motion_processor.reset()
            self.input_controller.release_all()

        self.state = new_state
        self.last_reason = reason

        suffix = f" ({reason})" if reason else ""
        print(f"\n[STATE] Control -> {self.state.upper()}{suffix}", flush=True)
        await self.broadcast_state()

    async def _handle_control(self, command: str, websocket: ServerConnection) -> None:
        """Process pause/resume/stop/calibrate/recenter commands from the phone UI."""
        if command == "pause":
            if self.state != STATE_STOPPED:
                await self.set_state(STATE_PAUSED, "Paused from phone")
        elif command == "resume":
            if self.state == STATE_PAUSED:
                await self.set_state(STATE_ACTIVE, "Resumed from phone")
        elif command == "toggle_pause":
            if self.state == STATE_ACTIVE:
                await self.set_state(STATE_PAUSED, "Paused from phone")
            elif self.state == STATE_PAUSED:
                await self.set_state(STATE_ACTIVE, "Resumed from phone")
            else:
                await self._send_state(websocket)
        elif command == "stop":
            await self.set_state(STATE_STOPPED, "Emergency Stop pressed on phone")
        elif command == "clear_stop":
            if self.state == STATE_STOPPED:
                await self.set_state(STATE_ACTIVE, "Emergency Stop cleared")
        elif command == "toggle_stop":
            if self.state == STATE_STOPPED:
                await self.set_state(STATE_ACTIVE, "Emergency Stop cleared")
            else:
                await self.set_state(STATE_STOPPED, "Emergency Stop pressed on phone")
        elif command == "calibrate":
            await self.start_calibration()
        elif command == "recenter":
            self.motion_processor.recenter()
            print("\n[CALIB] Orientation recentered.", flush=True)
            await self._send_json(websocket, {"type": "calibration", "status": "recentered"})

    async def handle_message(self, data: dict[str, Any], websocket: ServerConnection) -> str | None:
        """Dispatch a parsed JSON packet by type. Returns 'motion' when a motion packet arrived."""
        msg_type = data.get("type", "motion")

        if msg_type == "motion":
            was_calibrating = self.motion_processor.is_calibrating
            if self.state == STATE_ACTIVE or was_calibrating:
                dx, dy = self.motion_processor.process(data)
                if was_calibrating and not self.motion_processor.is_calibrating:
                    bias = self.motion_processor.gyro_bias
                    print(
                        f"\n[CALIB] Gyro bias calibrated: "
                        f"gx={bias['gx']:.4f}, gy={bias['gy']:.4f}, gz={bias['gz']:.4f} rad/s",
                        flush=True,
                    )
                    await self._broadcast_json(
                        {"type": "calibration", "status": "done", "bias": bias}
                    )
                elif self.state == STATE_ACTIVE and not self.motion_processor.is_calibrating:
                    self.input_controller.move_cursor(dx, dy)
            return "motion"

        if msg_type == "ping":
            roll, pitch, yaw = self.motion_processor.madgwick.get_euler_deg()
            await self._send_json(
                websocket,
                {
                    "type": "pong",
                    "client_t": data.get("client_t"),
                    "roll": round(roll, 2),
                    "pitch": round(pitch, 2),
                    "yaw": round(yaw, 2),
                },
            )
            return None

        if msg_type == "settings":
            s = self.motion_processor.update_settings(data)
            print(
                f"\n[SETTINGS] sens={s.sensitivity:.0f} smooth={s.smoothing:.2f} "
                f"dz={s.deadzone:.3f} axes=({s.x_axis},{s.y_axis}) "
                f"inv=({s.invert_x},{s.invert_y}) [saved]",
                flush=True,
            )
            return None

        if msg_type == "button":
            if self.state != STATE_STOPPED:
                button = str(data.get("button", ""))
                action = str(data.get("action", "click"))
                self.input_controller.handle_button(button, action)
            return None

        if msg_type == "control":
            command = str(data.get("command", ""))
            await self._handle_control(command, websocket)
            return None

        if msg_type == "diag":
            secure_ctx = bool(data.get("secureContext"))
            print(
                f"\n[DIAG] phase={data.get('phase')} secure={secure_ctx} "
                f"proto={data.get('protocol')} source={data.get('sensorSource')} "
                f"readings={data.get('hasAnyReading')} "
                f"apis=(gyro={data.get('hasGyroApi')}, accel={data.get('hasAccelApi')}, "
                f"dm={data.get('hasDeviceMotion')}, do={data.get('hasDeviceOrientation')})",
                flush=True,
            )
            if not secure_ctx:
                print(
                    "[DIAG] WARNING: Phone browser is NOT in a secure context. "
                    "Enable chrome://flags/#unsafely-treat-insecure-origin-as-secure or run with --https.",
                    flush=True,
                )
            return None

        return None

    async def handle_client(self, websocket: ServerConnection) -> None:
        """Handle a phone client's WebSocket lifecycle and incoming messages."""
        self.bind_loop(asyncio.get_running_loop())
        self.active_clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"\n[WS] Phone connected: {client_addr} (state={self.state.upper()})", flush=True)

        await self._send_state(websocket)
        await self._send_json(
            websocket,
            {"type": "settings_sync", **self.motion_processor.settings_dict()},
        )
        await self.start_calibration()

        msg_count = 0
        rate_hz = 0.0
        has_full_window = False
        window_start = time.monotonic()
        last_print = 0.0
        is_tty = sys.stdout.isatty()

        try:
            async for raw_message in websocket:
                try:
                    data = json.loads(raw_message)
                except json.JSONDecodeError:
                    print(f"\n[WS] Ignoring invalid JSON from {client_addr}: {raw_message!r}")
                    continue

                if not isinstance(data, dict):
                    continue

                kind = await self.handle_message(data, websocket)
                if kind != "motion":
                    continue

                now = time.monotonic()
                msg_count += 1
                elapsed = now - window_start
                if elapsed >= 1.0:
                    rate_hz = msg_count / elapsed
                    msg_count = 0
                    window_start = now
                    has_full_window = True
                elif not has_full_window and elapsed >= 0.2:
                    rate_hz = msg_count / elapsed

                if now - last_print >= PRINT_INTERVAL_SEC or not is_tty:
                    rpy = self.motion_processor.madgwick.get_euler_deg()
                    line = _format_telemetry(
                        self.state,
                        self.motion_processor.is_calibrating,
                        rpy,
                        data,
                        rate_hz,
                    )
                    if is_tty:
                        print(f"\r{line:<125}", end="", flush=True)
                    else:
                        print(line, flush=True)
                    last_print = now

        except ConnectionClosed as exc:
            if is_tty:
                print()
            print(f"[WS] Phone disconnected: {client_addr} (code={exc.code})")
        except Exception as exc:
            if is_tty:
                print()
            print(f"[WS] Connection error for {client_addr}: {exc}")
        else:
            if is_tty:
                print()
            print(f"[WS] Phone disconnected: {client_addr}")
        finally:
            self.active_clients.discard(websocket)
            if not self.active_clients:
                self.motion_processor.reset()
                self.input_controller.release_all()
