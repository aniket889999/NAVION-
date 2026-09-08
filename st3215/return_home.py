"""Return the ST3215 to its calibrated home position."""

from __future__ import annotations

import argparse

from .common import (
    DEFAULT_HOME,
    configure_position_mode,
    create_bus,
    lock_and_disconnect,
    read_raw,
    wait_for_position,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    parser.add_argument("--rpm", type=float, default=10)
    parser.add_argument("--home", type=int, default=DEFAULT_HOME)
    args = parser.parse_args()

    bus = create_bus(args.port, args.servo_id)
    try:
        bus.connect()
        print("Current position:", read_raw(bus, "Present_Position"))
        configure_position_mode(bus, args.rpm, acceleration=10)
        bus.enable_torque("servo")
        bus.write("Goal_Position", "servo", args.home, normalize=False)
        final_position = wait_for_position(bus, args.home)
        print("Home reached:", final_position)
    finally:
        lock_and_disconnect(bus)


if __name__ == "__main__":
    main()

