"""Rotate continuously like a wheel for a bounded time."""

from __future__ import annotations

import argparse
import time

from lerobot.motors.feetech import OperatingMode

from .common import create_bus, lock_and_disconnect, read_raw, rpm_to_raw_speed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    parser.add_argument("--rpm", type=float, default=10)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--direction", choices=("cw", "ccw"), default="cw")
    args = parser.parse_args()

    direction = 1 if args.direction == "cw" else -1
    velocity = direction * rpm_to_raw_speed(args.rpm)
    bus = create_bus(args.port, args.servo_id, name="wheel")

    try:
        bus.connect()
        bus.disable_torque("wheel")
        bus.write("Operating_Mode", "wheel", OperatingMode.VELOCITY.value, normalize=False)
        bus.write("Acceleration", "wheel", 20, normalize=False)
        bus.enable_torque("wheel")
        bus.write("Goal_Velocity", "wheel", velocity, normalize=False)

        print(
            f"Rotating {args.direction} at {args.rpm} RPM for {args.seconds} seconds. "
            "Press Control+C to stop."
        )
        start = time.time()
        while time.time() - start < args.seconds:
            present = read_raw(bus, "Present_Velocity", "wheel")
            elapsed = time.time() - start
            print(f"\rVelocity={present:5d} | Time={elapsed:5.1f}s", end="", flush=True)
            time.sleep(0.2)
        print()
    except KeyboardInterrupt:
        print("\nEmergency stop requested.")
    finally:
        if bus.is_connected:
            try:
                bus.write("Goal_Velocity", "wheel", 0, normalize=False, num_retry=5)
                time.sleep(0.2)
            finally:
                lock_and_disconnect(bus, "wheel", restore_position_mode=True)
        print("Motor stopped; position mode restored.")


if __name__ == "__main__":
    main()

