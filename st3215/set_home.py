"""Set the servo's current physical angle as its persistent center/home."""

from __future__ import annotations

import argparse

from lerobot.motors.feetech import OperatingMode

from .common import create_bus, lock_and_disconnect, read_raw


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    args = parser.parse_args()

    bus = create_bus(args.port, args.servo_id)
    try:
        bus.connect()
        bus.disable_torque("servo")
        bus.write("Operating_Mode", "servo", OperatingMode.POSITION.value, normalize=False)

        print("Position before homing:", read_raw(bus, "Present_Position"))
        print("Previous homing offset:", read_raw(bus, "Homing_Offset"))

        offsets = bus.set_half_turn_homings("servo")
        print("Written homing offset:", offsets["servo"])
        print("Position after homing:", read_raw(bus, "Present_Position"))
        print("Current physical angle is now home (approximately 2047).")
    finally:
        lock_and_disconnect(bus)


if __name__ == "__main__":
    main()

