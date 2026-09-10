"""TASK 2 — Joystick: hold-to-run wheel jogging with GUI release/stop events."""
from __future__ import annotations

import time
from threading import Event

from lerobot.motors.feetech import OperatingMode
from .common import create_bus, lock_and_disconnect, rpm_to_raw_speed
from .rotate_with_feedback import read_telemetry
from .task1_motor import MOTOR_NAME, build_motion_plan, _raise_on_fault


class KeyLatch:
    """Ignore key repeat; opposing controls request stop until both release."""
    def __init__(self):
        self.held = set()

    def press(self, key):
        if key not in {"w", "s"} or key in self.held:
            return None
        self.held.add(key)
        if len(self.held) > 1:
            return "stop"
        return "forward" if key == "w" else "back"

    def release(self, key):
        was_held = key in self.held
        self.held.discard(key)
        return was_held


def jog_wheel(port, servo_id, rpm, direction, invert_direction, stop_event: Event,
              callback=None, maximum_seconds=30.0):
    """Velocity mode until release, fault, or 30-second hold limit; always stop."""
    build_motion_plan(1, rpm, "cw")  # shared speed validation
    if direction not in {"forward", "back"}:
        raise ValueError("Direction must be forward or back")
    if not 0 < maximum_seconds <= 30:
        raise ValueError("Hold limit must be between 0 and 30 seconds")
    sign = 1 if direction == "forward" else -1
    if invert_direction:
        sign *= -1
    bus = create_bus(port, servo_id, name=MOTOR_NAME)
    started = time.monotonic()
    timed_out = False
    try:
        bus.connect()
        if stop_event.is_set():
            return "Released before motion"
        bus.disable_torque(MOTOR_NAME)
        bus.write("Operating_Mode", MOTOR_NAME, OperatingMode.VELOCITY.value, normalize=False)
        bus.write("Acceleration", MOTOR_NAME, 20, normalize=False)
        # Clear any stale velocity before enabling torque.
        bus.write("Goal_Velocity", MOTOR_NAME, 0, normalize=False)
        if stop_event.is_set():
            return "Released before motion"
        _raise_on_fault(read_telemetry(bus, started))
        bus.enable_torque(MOTOR_NAME)
        if not stop_event.is_set():
            bus.write("Goal_Velocity", MOTOR_NAME, sign * rpm_to_raw_speed(rpm), normalize=False)
        while not stop_event.is_set():
            telemetry = read_telemetry(bus, started)
            _raise_on_fault(telemetry)
            if callback:
                callback(telemetry)
            if time.monotonic() - started >= maximum_seconds:
                timed_out = True
                break
            stop_event.wait(0.08)
    finally:
        if bus.is_connected:
            try:
                bus.write("Goal_Velocity", MOTOR_NAME, 0, normalize=False)
            finally:
                lock_and_disconnect(bus, MOTOR_NAME, restore_position_mode=True)
    return "30-second hold limit reached" if timed_out else "Released / stopped"
