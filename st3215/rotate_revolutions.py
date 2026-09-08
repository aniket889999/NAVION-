"""Rotate an exact relative number of revolutions using ST3215 step mode."""

from __future__ import annotations

import argparse
import time

from lerobot.motors.feetech import OperatingMode

from .common import (
    STEPS_PER_REVOLUTION,
    create_bus,
    lock_and_disconnect,
    read_raw,
    rpm_to_raw_speed,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    parser.add_argument("--rotations", type=float, required=True)
    parser.add_argument("--rpm", type=float, required=True)
    parser.add_argument("--direction", choices=("cw", "ccw"), required=True)
    args = parser.parse_args()

    if args.rotations <= 0:
        raise SystemExit("Rotations must be greater than zero")

    direction = 1 if args.direction == "cw" else -1
    steps = round(args.rotations * STEPS_PER_REVOLUTION) * direction
    if abs(steps) > 30_000:
        raise SystemExit("This safety example limits one command to 30,000 steps")

    speed = rpm_to_raw_speed(args.rpm)
    expected_time = args.rotations * 60 / args.rpm
    bus = create_bus(args.port, args.servo_id, name="wheel")
    old_minimum, old_maximum = 0, 4095

    try:
        bus.connect()
        print("Starting position:", read_raw(bus, "Present_Position", "wheel"))
        print(
            f"Command: {args.rotations} rotations {args.direction} at {args.rpm} RPM "
            f"({steps} steps, about {expected_time:.2f} seconds)"
        )

        bus.disable_torque("wheel")
        old_minimum = read_raw(bus, "Min_Position_Limit", "wheel")
        old_maximum = read_raw(bus, "Max_Position_Limit", "wheel")
        bus.write("Min_Position_Limit", "wheel", 0, normalize=False)
        bus.write("Max_Position_Limit", "wheel", 0, normalize=False)
        bus.write("Operating_Mode", "wheel", OperatingMode.STEP.value, normalize=False)
        bus.write("Acceleration", "wheel", 30, normalize=False)
        bus.write("Goal_Velocity", "wheel", speed, normalize=False)
        bus.enable_torque("wheel")
        bus.write("Goal_Position", "wheel", steps, normalize=False)

        started = time.time()
        deadline = started + expected_time + 4
        time.sleep(0.2)
        while time.time() < deadline:
            elapsed = time.time() - started
            position = read_raw(bus, "Present_Position", "wheel")
            moving = read_raw(bus, "Moving", "wheel")
            estimate = min(args.rotations, elapsed * args.rpm / 60)
            print(
                f"\rPosition={position:4d} | Moving={moving} | "
                f"Estimated rotations={estimate:.2f}/{args.rotations}",
                end="",
                flush=True,
            )
            if elapsed >= expected_time and moving == 0:
                break
            time.sleep(0.1)
        print()
        print("Final position:", read_raw(bus, "Present_Position", "wheel"))
    except KeyboardInterrupt:
        print("\nEmergency stop requested.")
    finally:
        lock_and_disconnect(
            bus,
            "wheel",
            restore_position_mode=True,
            minimum_limit=old_minimum,
            maximum_limit=old_maximum,
        )
        print("Motor stopped; position mode and limits restored.")


if __name__ == "__main__":
    main()

