"""TASK 2 — Future IMU: BNO055 adapters and tilt filtering, currently inactive.

The Mac cannot read a bare BNO055 over USB. Use either:

* a microcontroller/USB serial bridge that prints pitch values, or
* this package on a Raspberry Pi with the BNO055 connected over I2C.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from threading import Event
from typing import Callable, Iterator


@dataclass(frozen=True)
class ImuSample:
    pitch_deg: float
    source: str
    timestamp: float


@dataclass(frozen=True)
class TiltDecision:
    command: str | None
    state: str
    pitch_deg: float


class TiltCommandGate:
    """Convert stable tilt into one command, then require neutral to re-arm."""

    def __init__(
        self,
        threshold_deg: float = 15.0,
        neutral_deg: float = 7.0,
        stable_samples: int = 4,
        invert_pitch: bool = False,
    ) -> None:
        if threshold_deg <= 0:
            raise ValueError("Tilt threshold must be positive")
        if not 0 <= neutral_deg < threshold_deg:
            raise ValueError("Neutral angle must be at least 0 and below the threshold")
        if stable_samples < 1:
            raise ValueError("Stable samples must be at least 1")
        self.threshold_deg = threshold_deg
        self.neutral_deg = neutral_deg
        self.stable_samples = stable_samples
        self.invert_pitch = invert_pitch
        self._candidate: str | None = None
        self._candidate_count = 0
        self._armed = True

    def update(self, pitch_deg: float) -> TiltDecision:
        pitch = -pitch_deg if self.invert_pitch else pitch_deg
        if abs(pitch) <= self.neutral_deg:
            self._candidate = None
            self._candidate_count = 0
            self._armed = True
            return TiltDecision(None, "neutral — armed", pitch_deg)

        command: str | None = None
        if pitch >= self.threshold_deg:
            command = "forward"
        elif pitch <= -self.threshold_deg:
            command = "back"

        if command is None:
            self._candidate = None
            self._candidate_count = 0
            return TiltDecision(None, "deadband", pitch_deg)

        if command != self._candidate:
            self._candidate = command
            self._candidate_count = 1
        else:
            self._candidate_count += 1

        if self._armed and self._candidate_count >= self.stable_samples:
            self._armed = False
            return TiltDecision(command, f"{command} command", pitch_deg)

        if not self._armed:
            return TiltDecision(None, f"{command} held — return to neutral", pitch_deg)
        return TiltDecision(
            None,
            f"confirming {command} {self._candidate_count}/{self.stable_samples}",
            pitch_deg,
        )


def parse_serial_pitch(line: str) -> float:
    """Parse a number or JSON line from an Arduino/USB serial bridge.

    Accepted examples: ``12.5``, ``{"pitch": 12.5}``,
    ``{"pitch_deg": 12.5}``, or ``{"euler": [heading, roll, pitch]}``.
    """
    value = line.strip()
    if not value:
        raise ValueError("empty IMU line")
    try:
        return float(value)
    except ValueError:
        payload = json.loads(value)
        if "pitch_deg" in payload:
            return float(payload["pitch_deg"])
        if "pitch" in payload:
            return float(payload["pitch"])
        if "euler" in payload and len(payload["euler"]) >= 3:
            return float(payload["euler"][2])
        raise ValueError("JSON must contain pitch, pitch_deg, or euler[2]")


def serial_pitch_samples(
    port: str,
    baudrate: int,
    stop_event: Event,
) -> Iterator[ImuSample]:
    """Yield pitch samples from a USB serial bridge."""
    import serial

    if not port:
        raise ValueError("IMU serial port is required")
    with serial.Serial(port=port, baudrate=baudrate, timeout=0.25) as connection:
        connection.reset_input_buffer()
        while not stop_event.is_set():
            raw = connection.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            try:
                pitch = parse_serial_pitch(raw)
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
            yield ImuSample(pitch, f"serial:{port}", time.time())


def bno055_i2c_samples(stop_event: Event, period_seconds: float = 0.1) -> Iterator[ImuSample]:
    """Yield BNO055 pitch on Raspberry Pi I2C; imports hardware packages lazily."""
    try:
        import adafruit_bno055
        import board
    except ImportError as exc:
        raise RuntimeError(
            "Raspberry Pi I2C mode needs: pip install adafruit-circuitpython-bno055"
        ) from exc

    sensor = adafruit_bno055.BNO055_I2C(board.I2C())
    while not stop_event.is_set():
        euler = sensor.euler
        if euler and len(euler) >= 3 and euler[2] is not None:
            yield ImuSample(float(euler[2]), "bno055:i2c", time.time())
        stop_event.wait(period_seconds)


ImuCallback = Callable[[ImuSample, TiltDecision], None]


def monitor_imu(
    source: str,
    stop_event: Event,
    callback: ImuCallback,
    gate: TiltCommandGate,
    serial_port: str = "",
    serial_baudrate: int = 115_200,
) -> None:
    """Run the selected IMU source until stopped."""
    if source == "serial":
        samples = serial_pitch_samples(serial_port, serial_baudrate, stop_event)
    elif source == "i2c":
        samples = bno055_i2c_samples(stop_event)
    else:
        raise ValueError("IMU source must be 'serial' or 'i2c'")

    for sample in samples:
        if stop_event.is_set():
            break
        callback(sample, gate.update(sample.pitch_deg))
