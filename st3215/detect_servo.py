"""Detect ST3215 IDs and baud rates without commanding movement."""

from __future__ import annotations

import argparse

from lerobot.motors.feetech import FeetechMotorsBus


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, such as /dev/cu.usbmodem...")
    args = parser.parse_args()

    detected = FeetechMotorsBus.scan_port(args.port)
    print("Detected motors:", detected)
    if not detected:
        raise SystemExit(
            "No servo replied. Check external motor power, USB-mode jumpers, and D/V/G wiring."
        )


if __name__ == "__main__":
    main()

