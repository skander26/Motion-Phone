"""Motion signal processing: Madgwick 6-DOF fusion, gyro bias calibration, config persistence, and cursor math."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

VALID_AXES = ("gx", "gy", "gz")
DEFAULT_DT_SEC = 1.0 / 60.0
MAX_DT_SEC = 0.2
CALIBRATION_DURATION_SEC = 2.0
MAX_BIAS_RAD_S = 0.5
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config.json"


def _wrap_deg(angle: float) -> float:
    """Wrap an angle in degrees to [-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0


class MadgwickFilter:
    """6-DOF Madgwick IMU filter fusing gyroscope (rad/s) and accelerometer (m/s^2)."""

    def __init__(self, beta: float = 0.08) -> None:
        self.beta = beta
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0

    def reset(self) -> None:
        """Reset quaternion orientation and reference frame to identity."""
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0

    def update_imu(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt: float,
    ) -> None:
        """Advance orientation estimate by dt seconds using gyro and accelerometer readings."""
        if dt <= 0.0:
            return

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if acc_norm > 1e-6:
            inv_acc = 1.0 / acc_norm
            ax *= inv_acc
            ay *= inv_acc
            az *= inv_acc

            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _4q0 = 4.0 * q0
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _8q1 = 8.0 * q1
            _8q2 = 8.0 * q2
            q0q0 = q0 * q0
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay

            s_norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if s_norm > 1e-9:
                inv_s = 1.0 / s_norm
                q_dot1 -= self.beta * (s0 * inv_s)
                q_dot2 -= self.beta * (s1 * inv_s)
                q_dot3 -= self.beta * (s2 * inv_s)
                q_dot4 -= self.beta * (s3 * inv_s)

        q0 += q_dot1 * dt
        q1 += q_dot2 * dt
        q2 += q_dot3 * dt
        q3 += q_dot4 * dt

        q_norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if q_norm > 1e-9:
            inv_q = 1.0 / q_norm
            self.q0 = q0 * inv_q
            self.q1 = q1 * inv_q
            self.q2 = q2 * inv_q
            self.q3 = q3 * inv_q

    def _raw_euler_deg(self) -> tuple[float, float, float]:
        """Convert current quaternion to (roll, pitch, yaw) in degrees."""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        roll = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
        sinp = max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))
        pitch = math.degrees(math.asin(sinp))
        yaw = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
        return roll, pitch, yaw

    def recenter(self) -> None:
        """Capture current orientation as the (0, 0, 0) reference."""
        self.ref_roll, self.ref_pitch, self.ref_yaw = self._raw_euler_deg()

    def get_euler_deg(self) -> tuple[float, float, float]:
        """Return (roll, pitch, yaw) in degrees relative to the last recenter."""
        roll, pitch, yaw = self._raw_euler_deg()
        return (
            _wrap_deg(roll - self.ref_roll),
            _wrap_deg(pitch - self.ref_pitch),
            _wrap_deg(yaw - self.ref_yaw),
        )


@dataclass
class MotionSettings:
    """User-tunable motion-to-cursor parameters synced live from the phone UI."""

    sensitivity: float = 1200.0
    smoothing: float = 0.35
    deadzone: float = 0.025
    x_axis: str = "gz"
    y_axis: str = "gx"
    invert_x: bool = False
    invert_y: bool = False


class MotionProcessor:
    """Converts sensor packets into calibrated, smoothed, framerate-independent pixel deltas."""

    def __init__(
        self,
        settings: MotionSettings | None = None,
        config_path: Path | None = DEFAULT_CONFIG_PATH,
    ) -> None:
        self.config_path = config_path
        self.settings = settings or MotionSettings()
        if settings is None and self.config_path is not None:
            self.load_config()

        self.madgwick = MadgwickFilter()
        self.gyro_bias: dict[str, float] = {axis: 0.0 for axis in VALID_AXES}
        self.is_calibrating: bool = False
        self._cal_start_ms: float | None = None
        self._cal_samples: list[tuple[float, float, float]] = []

        self._last_t: float | None = None
        self._ema: dict[str, float] = {axis: 0.0 for axis in VALID_AXES}
        self._rem_x: float = 0.0
        self._rem_y: float = 0.0

    def reset(self) -> None:
        """Clear filter history, time reference, and sub-pixel remainders."""
        self._last_t = None
        for axis in VALID_AXES:
            self._ema[axis] = 0.0
        self._rem_x = 0.0
        self._rem_y = 0.0

    def recenter(self) -> None:
        """Recenter orientation reference and clear residual motion accumulators."""
        self.madgwick.recenter()
        self.reset()

    def start_calibration(self) -> None:
        """Begin a 2-second stationary gyro bias calibration sequence."""
        self.is_calibrating = True
        self._cal_start_ms = None
        self._cal_samples.clear()
        self.madgwick.reset()
        self.reset()

    def settings_dict(self) -> dict[str, Any]:
        """Return settings serialized in the client JSON schema."""
        return {
            "sensitivity": self.settings.sensitivity,
            "smoothing": self.settings.smoothing,
            "deadzone": self.settings.deadzone,
            "xAxis": self.settings.x_axis,
            "yAxis": self.settings.y_axis,
            "invertX": self.settings.invert_x,
            "invertY": self.settings.invert_y,
        }

    def load_config(self) -> None:
        """Load saved settings from config.json if present."""
        if self.config_path is None or not self.config_path.is_file():
            return
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
        except Exception:
            return
        if isinstance(data, dict):
            self.update_settings(data, persist=False)

    def save_config(self) -> None:
        """Persist current settings to config.json."""
        if self.config_path is None:
            return
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            self.config_path.write_text(json.dumps(self.settings_dict(), indent=2) + "\n", encoding="utf-8")
        except Exception:
            pass

    def update_settings(self, payload: dict[str, Any], persist: bool = True) -> MotionSettings:
        """Validate and apply live tuning settings, optionally saving to config.json."""
        if "sensitivity" in payload:
            self.settings.sensitivity = max(50.0, min(5000.0, float(payload["sensitivity"])))
        if "smoothing" in payload:
            self.settings.smoothing = max(0.0, min(0.95, float(payload["smoothing"])))
        if "deadzone" in payload:
            self.settings.deadzone = max(0.0, min(1.0, float(payload["deadzone"])))

        x_axis = payload.get("xAxis", payload.get("x_axis"))
        if x_axis in VALID_AXES:
            self.settings.x_axis = x_axis

        y_axis = payload.get("yAxis", payload.get("y_axis"))
        if y_axis in VALID_AXES:
            self.settings.y_axis = y_axis

        if "invertX" in payload or "invert_x" in payload:
            self.settings.invert_x = bool(payload.get("invertX", payload.get("invert_x")))
        if "invertY" in payload or "invert_y" in payload:
            self.settings.invert_y = bool(payload.get("invertY", payload.get("invert_y")))

        if persist:
            self.save_config()

        return self.settings

    def _compute_dt(self, packet: dict[str, Any]) -> tuple[float, float | None]:
        """Compute elapsed seconds since the previous packet and return (dt, t_ms)."""
        raw_t = packet.get("t")
        if raw_t is None:
            return DEFAULT_DT_SEC, None

        try:
            t_ms = float(raw_t)
        except (TypeError, ValueError):
            return DEFAULT_DT_SEC, None

        if self._last_t is None:
            self._last_t = t_ms
            return DEFAULT_DT_SEC, t_ms

        dt = (t_ms - self._last_t) / 1000.0
        self._last_t = t_ms

        if dt <= 0.0 or dt > MAX_DT_SEC:
            return DEFAULT_DT_SEC, t_ms
        return dt, t_ms

    def _apply_deadzone(self, value: float) -> float:
        """Suppress small sensor noise below deadzone while keeping onset smooth."""
        dz = self.settings.deadzone
        if abs(value) < dz:
            return 0.0
        return math.copysign(abs(value) - dz, value)

    def _apply_ema(self, axis: str, value: float) -> float:
        """Smooth gyro rate via exponential moving average."""
        alpha = 1.0 - self.settings.smoothing
        filtered = alpha * value + (1.0 - alpha) * self._ema[axis]
        if value == 0.0 and abs(filtered) < 1e-4:
            filtered = 0.0
        self._ema[axis] = filtered
        return filtered

    @staticmethod
    def _read_float(packet: dict[str, Any], key: str) -> float:
        try:
            return float(packet.get(key, 0.0))
        except (TypeError, ValueError):
            return 0.0

    def _accumulate_calibration(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt: float,
        t_ms: float | None,
    ) -> None:
        """Collect stationary gyro samples for 2 seconds and update gyro bias offsets."""
        # Keep Madgwick aligned with gravity while holding still during bias capture.
        self.madgwick.update_imu(0.0, 0.0, 0.0, ax, ay, az, dt)
        self._cal_samples.append((gx, gy, gz))

        if t_ms is None:
            elapsed_sec = len(self._cal_samples) * DEFAULT_DT_SEC
        else:
            if self._cal_start_ms is None:
                self._cal_start_ms = t_ms
            elapsed_sec = (t_ms - self._cal_start_ms) / 1000.0
            if elapsed_sec < 0.0:
                self._cal_start_ms = t_ms
                elapsed_sec = 0.0

        if elapsed_sec >= CALIBRATION_DURATION_SEC and self._cal_samples:
            n = len(self._cal_samples)
            mean_x = sum(s[0] for s in self._cal_samples) / n
            mean_y = sum(s[1] for s in self._cal_samples) / n
            mean_z = sum(s[2] for s in self._cal_samples) / n
            self.gyro_bias["gx"] = max(-MAX_BIAS_RAD_S, min(MAX_BIAS_RAD_S, mean_x))
            self.gyro_bias["gy"] = max(-MAX_BIAS_RAD_S, min(MAX_BIAS_RAD_S, mean_y))
            self.gyro_bias["gz"] = max(-MAX_BIAS_RAD_S, min(MAX_BIAS_RAD_S, mean_z))
            self.is_calibrating = False
            self._cal_samples.clear()
            self.madgwick.recenter()
            self.reset()

    def process(self, packet: dict[str, Any]) -> tuple[int, int]:
        """Convert one motion packet into integer screen cursor deltas (dx, dy)."""
        dt, t_ms = self._compute_dt(packet)
        raw_gx = self._read_float(packet, "gx")
        raw_gy = self._read_float(packet, "gy")
        raw_gz = self._read_float(packet, "gz")
        ax = self._read_float(packet, "ax")
        ay = self._read_float(packet, "ay")
        az = self._read_float(packet, "az")

        if self.is_calibrating:
            self._accumulate_calibration(raw_gx, raw_gy, raw_gz, ax, ay, az, dt, t_ms)
            return 0, 0

        cal_gyro = {
            "gx": raw_gx - self.gyro_bias["gx"],
            "gy": raw_gy - self.gyro_bias["gy"],
            "gz": raw_gz - self.gyro_bias["gz"],
        }
        self.madgwick.update_imu(cal_gyro["gx"], cal_gyro["gy"], cal_gyro["gz"], ax, ay, az, dt)

        for axis in VALID_AXES:
            self._apply_ema(axis, self._apply_deadzone(cal_gyro[axis]))

        # Default sign convention:
        # - Rotating phone right (negative gz) moves cursor right (+X).
        # - Tilting phone up (positive gx) moves cursor up (-Y).
        rate_x = -self._ema[self.settings.x_axis]
        rate_y = -self._ema[self.settings.y_axis]

        if self.settings.invert_x:
            rate_x = -rate_x
        if self.settings.invert_y:
            rate_y = -rate_y

        dx = rate_x * self.settings.sensitivity * dt
        dy = rate_y * self.settings.sensitivity * dt

        total_x = self._rem_x + dx
        total_y = self._rem_y + dy
        int_dx = int(math.trunc(total_x))
        int_dy = int(math.trunc(total_y))

        self._rem_x = total_x - int_dx
        self._rem_y = total_y - int_dy

        return int_dx, int_dy
