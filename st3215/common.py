"""Shared, safety-focused helpers for ST3215 examples."""

from __future__ import annotations

import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

STEPS_PER_REVOLUTION = 4096
DEFAULT_HOME = 2047


def create_bus(port: str, servo_id: int, name: str = "servo") -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={
            name: Motor(
                id=servo_id,
                model="sts3215",
                norm_mode=MotorNormMode.RANGE_M100_100,
            )
        },
    )


def rpm_to_raw_speed(rpm: float) -> int:
    if rpm <= 0:
        raise ValueError("RPM must be greater than zero")
    return round(rpm * STEPS_PER_REVOLUTION / 60)


def degrees_to_steps(degrees: float) -> int:
    return round(degrees * STEPS_PER_REVOLUTION / 360)


def read_raw(bus: FeetechMotorsBus, register: str, motor: str = "servo") -> int:
    return int(bus.read(register, motor, normalize=False))


def configure_position_mode(
    bus: FeetechMotorsBus,
    rpm: float,
    acceleration: int = 20,
    motor: str = "servo",
) -> None:
    bus.disable_torque(motor)
    bus.write("Operating_Mode", motor, OperatingMode.POSITION.value, normalize=False)
    bus.write("Acceleration", motor, acceleration, normalize=False)
    bus.write("Goal_Velocity", motor, rpm_to_raw_speed(rpm), normalize=False)


def wait_for_position(
    bus: FeetechMotorsBus,
    target: int,
    timeout: float = 10.0,
    tolerance: int = 5,
    motor: str = "servo",
) -> int:
    deadline = time.time() + timeout
    position = read_raw(bus, "Present_Position", motor)

    # A short delay avoids interpreting the pre-movement Moving flag as completion.
    time.sleep(0.05)
    while time.time() < deadline:
        position = read_raw(bus, "Present_Position", motor)
        error = target - position
        print(f"\rPosition: {position:4d} | Error: {error:+5d}", end="", flush=True)
        if abs(error) <= tolerance:
            print()
            return position
        time.sleep(0.05)

    print()
    raise TimeoutError(f"Servo did not reach {target}; last position was {position}")


def lock_and_disconnect(
    bus: FeetechMotorsBus,
    motor: str = "servo",
    restore_position_mode: bool = False,
    minimum_limit: int = 0,
    maximum_limit: int = 4095,
) -> None:
    if not bus.is_connected:
        return

    try:
        bus.disable_torque(motor)
        if restore_position_mode:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value, normalize=False)
            bus.write("Min_Position_Limit", motor, minimum_limit, normalize=False)
            bus.write("Max_Position_Limit", motor, maximum_limit, normalize=False)
        bus.write("Lock", motor, 1, normalize=False)
    finally:
        bus.disconnect(disable_torque=False)

