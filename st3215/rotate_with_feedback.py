"""Run an exact ST3215 rotation and report live/final telemetry."""

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from lerobot.motors.feetech import OperatingMode

from .common import (
    STEPS_PER_REVOLUTION,
    create_bus,
    lock_and_disconnect,
    read_raw,
    rpm_to_raw_speed,
)


MOTOR_NAME = "wheel"
CURRENT_MILLIAMPS_PER_STEP = 6.5

# ST3215 Status register bits used by the Feetech SDK.
STATUS_FLAGS = {
    0x01: "input-voltage fault",
    0x02: "angle-sensor fault",
    0x04: "over-temperature",
    0x08: "over-current",
    0x20: "overload",
}


@dataclass(frozen=True)
class Telemetry:
    elapsed_s: float
    position_steps: int
    position_deg: float
    velocity_steps_s: int
    velocity_rpm: float
    load_raw: int
    load_percent: float
    voltage_v: float
    temperature_c: int
    current_raw: int
    current_ma: float
    moving: int
    status_raw: int
    status_text: str


def decode_status(status: int) -> str:
    faults = [name for mask, name in STATUS_FLAGS.items() if status & mask]
    unknown_bits = status & ~sum(STATUS_FLAGS)
    if unknown_bits:
        faults.append(f"unknown-bits-0x{unknown_bits:02X}")
    return ", ".join(faults) if faults else "OK"


def read_telemetry(bus, started: float) -> Telemetry:
    position = read_raw(bus, "Present_Position", MOTOR_NAME)
    velocity = read_raw(bus, "Present_Velocity", MOTOR_NAME)
    load = read_raw(bus, "Present_Load", MOTOR_NAME)
    voltage = read_raw(bus, "Present_Voltage", MOTOR_NAME)
    temperature = read_raw(bus, "Present_Temperature", MOTOR_NAME)
    current = read_raw(bus, "Present_Current", MOTOR_NAME)
    moving = read_raw(bus, "Moving", MOTOR_NAME)
    status = read_raw(bus, "Status", MOTOR_NAME)

    return Telemetry(
        elapsed_s=time.monotonic() - started,
        position_steps=position,
        position_deg=position * 360.0 / STEPS_PER_REVOLUTION,
        velocity_steps_s=velocity,
        velocity_rpm=velocity * 60.0 / STEPS_PER_REVOLUTION,
        load_raw=load,
        load_percent=load / 10.0,
        voltage_v=voltage / 10.0,
        temperature_c=temperature,
        current_raw=current,
        current_ma=current * CURRENT_MILLIAMPS_PER_STEP,
        moving=moving,
        status_raw=status,
        status_text=decode_status(status),
    )


def wrapped_position_error(start: int, end: int) -> int:
    """Return the shortest signed encoder difference after the full turn."""
    half_turn = STEPS_PER_REVOLUTION // 2
    return (end - start + half_turn) % STEPS_PER_REVOLUTION - half_turn


def save_csv(samples: list[Telemetry], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=Telemetry.__dataclass_fields__)
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)


def print_final_report(
    samples: list[Telemetry],
    start_position: int,
    rotations: float,
    direction: str,
    requested_rpm: float,
    duration: float,
) -> None:
    final = samples[-1]
    moving_samples = [sample for sample in samples if sample.moving or abs(sample.velocity_rpm) >= 0.5]
    average_speed = (
        sum(abs(sample.velocity_rpm) for sample in moving_samples) / len(moving_samples)
        if moving_samples
        else 0.0
    )

    print("\n========== FINAL ST3215 FEEDBACK ==========")
    print(f"Command             : {rotations:g} rotation(s) {direction} at {requested_rpm:g} RPM")
    print(f"Measured duration    : {duration:.2f} s")
    print(f"Start position       : {start_position} steps")
    print(f"Final position       : {final.position_steps} steps ({final.position_deg:.2f} deg)")
    print(f"Wrapped position err : {wrapped_position_error(start_position, final.position_steps):+d} steps")
    print(f"Final velocity       : {final.velocity_steps_s:+d} steps/s ({final.velocity_rpm:+.2f} RPM)")
    print(f"Average moving speed : {average_speed:.2f} RPM")
    print(f"Peak speed           : {max(abs(s.velocity_rpm) for s in samples):.2f} RPM")
    print(f"Input voltage        : {final.voltage_v:.1f} V (minimum {min(s.voltage_v for s in samples):.1f} V)")
    print(f"Temperature          : {final.temperature_c} C (maximum {max(s.temperature_c for s in samples)} C)")
    print(f"Current              : {final.current_ma:.1f} mA (peak {max(abs(s.current_ma) for s in samples):.1f} mA)")
    print(f"Load                 : {final.load_percent:+.1f}% (peak {max(abs(s.load_percent) for s in samples):.1f}%)")
    print(f"Moving flag          : {final.moving}")
    print(f"Status register      : 0x{final.status_raw:02X} ({final.status_text})")
    print("============================================")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Rotate an ST3215 exactly and print live/final motor feedback."
    )
    parser.add_argument("--port", required=True, help="macOS serial port, normally /dev/cu.usbmodem...")
    parser.add_argument("--id", type=int, default=1, dest="servo_id")
    parser.add_argument("--rotations", type=float, default=1.0)
    parser.add_argument("--rpm", type=float, default=20.0)
    parser.add_argument("--direction", choices=("cw", "ccw"), default="cw")
    parser.add_argument("--sample-period", type=float, default=0.20, help="telemetry period in seconds")
    parser.add_argument("--csv", type=Path, help="optional path for a telemetry CSV log")
    args = parser.parse_args()

    if args.rotations <= 0:
        raise SystemExit("Rotations must be greater than zero")
    if args.sample_period < 0.05:
        raise SystemExit("Sample period must be at least 0.05 seconds")

    direction_sign = 1 if args.direction == "cw" else -1
    steps = round(args.rotations * STEPS_PER_REVOLUTION) * direction_sign
    if abs(steps) > 30_000:
        raise SystemExit("Safety limit: one command cannot exceed 30,000 steps")

    speed = rpm_to_raw_speed(args.rpm)
    expected_time = args.rotations * 60.0 / args.rpm
    bus = create_bus(args.port, args.servo_id, name=MOTOR_NAME)
    old_minimum, old_maximum = 0, STEPS_PER_REVOLUTION - 1
    samples: list[Telemetry] = []
    start_position = 0
    started = 0.0
    finished = 0.0
    movement_seen = False

    try:
        print(f"Connecting to ST3215 ID {args.servo_id} on {args.port}...")
        bus.connect()
        model_number = read_raw(bus, "Model_Number", MOTOR_NAME)
        start_position = read_raw(bus, "Present_Position", MOTOR_NAME)
        print(f"Connected: model={model_number}, start position={start_position}")
        print(
            f"Command: {args.rotations:g} rotation(s) {args.direction} at {args.rpm:g} RPM "
            f"({steps:+d} steps, speed register={speed}, expected time={expected_time:.2f} s)"
        )

        bus.disable_torque(MOTOR_NAME)
        old_minimum = read_raw(bus, "Min_Position_Limit", MOTOR_NAME)
        old_maximum = read_raw(bus, "Max_Position_Limit", MOTOR_NAME)
        bus.write("Min_Position_Limit", MOTOR_NAME, 0, normalize=False)
        bus.write("Max_Position_Limit", MOTOR_NAME, 0, normalize=False)
        bus.write("Operating_Mode", MOTOR_NAME, OperatingMode.STEP.value, normalize=False)
        bus.write("Acceleration", MOTOR_NAME, 30, normalize=False)
        bus.write("Goal_Velocity", MOTOR_NAME, speed, normalize=False)
        bus.enable_torque(MOTOR_NAME)

        started = time.monotonic()
        bus.write("Goal_Position", MOTOR_NAME, steps, normalize=False)
        deadline = started + expected_time + max(4.0, expected_time * 0.5)

        print("\nLive feedback:")
        print(" time    pos    speed     voltage  temp  current  load     moving  status")
        while time.monotonic() < deadline:
            sample = read_telemetry(bus, started)
            samples.append(sample)
            movement_seen |= sample.moving == 1 or abs(sample.velocity_rpm) >= 0.5
            print(
                f"{sample.elapsed_s:5.2f}s  {sample.position_steps:4d}  "
                f"{sample.velocity_rpm:+7.2f}rpm  {sample.voltage_v:5.1f}V  "
                f"{sample.temperature_c:3d}C  {sample.current_ma:7.1f}mA  "
                f"{sample.load_percent:+6.1f}%     {sample.moving}     {sample.status_text}"
            )

            if sample.status_raw:
                raise RuntimeError(
                    f"Servo protection fault 0x{sample.status_raw:02X}: {sample.status_text}"
                )
            if sample.elapsed_s >= expected_time and sample.moving == 0:
                break
            time.sleep(args.sample_period)

        finished = time.monotonic()
        if not samples:
            raise RuntimeError("No telemetry was received from the servo")
        if not movement_seen:
            raise RuntimeError(
                "The command was sent, but no movement was detected. "
                "Check motor power, D/V/G wiring, and adapter jumpers."
            )
        if samples[-1].moving:
            raise TimeoutError("The servo did not finish before the safety timeout")

        print_final_report(
            samples,
            start_position,
            args.rotations,
            args.direction,
            args.rpm,
            finished - started,
        )
        if args.csv:
            save_csv(samples, args.csv)
            print(f"Telemetry CSV saved: {args.csv.resolve()}")
    except KeyboardInterrupt:
        print("\nEmergency stop requested by Control+C.")
    finally:
        lock_and_disconnect(
            bus,
            MOTOR_NAME,
            restore_position_mode=True,
            minimum_limit=old_minimum,
            maximum_limit=old_maximum,
        )
        print("Torque disabled; position mode and original limits restored.")


if __name__ == "__main__":
    main()
