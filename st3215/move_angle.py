"""Move to a signed angle measured from the calibrated home position."""

from __future__ import annotations

import argparse

from .common import (
    DEFAULT_HOME,
    STEPS_PER_REVOLUTION,
    configure_position_mode,
    create_bus,
    degrees_to_steps,
    lock_and_disconnect,
    read_raw,
    wait_for_position,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    parser.add_argument("--degrees", type=float, required=True)
    parser.add_argument("--direction", choices=("cw", "ccw"), required=True)
    parser.add_argument("--rpm", type=float, default=10)
    parser.add_argument("--home", type=int, default=DEFAULT_HOME)
    args = parser.parse_args()

    direction = 1 if args.direction == "cw" else -1
    target = args.home + direction * degrees_to_steps(args.degrees)
    if not 0 <= target < STEPS_PER_REVOLUTION:
        raise SystemExit("Target is outside position-mode range 0..4095")

    bus = create_bus(args.port, args.servo_id)
    try:
        bus.connect()
        current = read_raw(bus, "Present_Position")
        print(f"Current={current}, home={args.home}, target={target}, speed={args.rpm} RPM")

        configure_position_mode(bus, args.rpm)
        bus.enable_torque("servo")
        bus.write("Goal_Position", "servo", target, normalize=False)
        final_position = wait_for_position(bus, target)
        angle = (final_position - args.home) * 360 / STEPS_PER_REVOLUTION
        print(f"Final position={final_position}; angle from home={angle:.2f} degrees")
    finally:
        lock_and_disconnect(bus)


if __name__ == "__main__":
    main()

