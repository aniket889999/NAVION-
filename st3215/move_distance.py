"""TASK 1 — Distance CLI: move a 65 mm wheel by a requested distance."""

from __future__ import annotations

import argparse
import math
import time

from lerobot.motors.feetech import OperatingMode

from .common import (
    STEPS_PER_REVOLUTION,
    create_bus,
    lock_and_disconnect,
    read_raw,
    rpm_to_raw_speed,
)

from .rotate_with_feedback import read_telemetry


MOTOR_NAME = "wheel"
DEFAULT_WHEEL_DIAMETER_MM = 65.0

# Each large movement is divided into one-revolution segments.
MAX_SEGMENT_STEPS = STEPS_PER_REVOLUTION


def distance_to_motion(
    distance_cm: float,
    wheel_diameter_mm: float,
) -> tuple[int, float, float]:
    """Convert linear wheel travel into encoder steps."""

    if distance_cm <= 0:
        raise ValueError(
            "Distance must be greater than zero"
        )

    if wheel_diameter_mm <= 0:
        raise ValueError(
            "Wheel diameter must be greater than zero"
        )

    circumference_cm = (
        math.pi * wheel_diameter_mm / 10.0
    )

    rotations = (
        distance_cm / circumference_cm
    )

    steps = round(
        rotations * STEPS_PER_REVOLUTION
    )

    if steps == 0:
        raise ValueError(
            "Distance is too small to produce "
            "one encoder step"
        )

    return steps, rotations, circumference_cm


def split_steps(total_steps: int) -> list[int]:
    """Divide a large command into safe segments."""

    segments = []
    remaining = total_steps

    while remaining > 0:
        segment = min(
            remaining,
            MAX_SEGMENT_STEPS,
        )

        segments.append(segment)
        remaining -= segment

    return segments


def wrapped_error(
    actual: int,
    expected: int,
) -> int:
    """Calculate shortest encoder error."""

    half_turn = (
        STEPS_PER_REVOLUTION // 2
    )

    return (
        actual
        - expected
        + half_turn
    ) % STEPS_PER_REVOLUTION - half_turn


def run_segment(
    bus,
    signed_steps: int,
    rpm: float,
    segment_number: int,
    segment_count: int,
) -> None:
    """Execute one relative step-mode segment."""

    start_position = read_raw(
        bus,
        "Present_Position",
        MOTOR_NAME,
    )

    expected_position = (
        start_position + signed_steps
    ) % STEPS_PER_REVOLUTION

    segment_rotations = (
        abs(signed_steps)
        / STEPS_PER_REVOLUTION
    )

    expected_time = (
        segment_rotations * 60.0 / rpm
    )

    bus.write(
        "Goal_Position",
        MOTOR_NAME,
        signed_steps,
        normalize=False,
    )

    started = time.monotonic()

    deadline = (
        started
        + expected_time
        + 4.0
    )

    movement_seen = False
    final_position = start_position

    while time.monotonic() < deadline:
        telemetry = read_telemetry(
            bus,
            started,
        )

        final_position = (
            telemetry.position_steps
        )

        movement_seen |= (
            telemetry.moving == 1
            or abs(telemetry.velocity_rpm) >= 0.5
        )

        print(
            f"\rSegment "
            f"{segment_number}/{segment_count} | "
            f"Position "
            f"{telemetry.position_steps:4d} | "
            f"Speed "
            f"{telemetry.velocity_rpm:+6.2f} RPM | "
            f"Voltage "
            f"{telemetry.voltage_v:4.1f} V | "
            f"Temp "
            f"{telemetry.temperature_c:2d} C | "
            f"Current "
            f"{telemetry.current_ma:6.1f} mA | "
            f"Moving {telemetry.moving}",
            end="",
            flush=True,
        )

        if telemetry.status_raw:
            print()

            raise RuntimeError(
                "Servo fault "
                f"0x{telemetry.status_raw:02X}: "
                f"{telemetry.status_text}"
            )

        elapsed = (
            time.monotonic() - started
        )

        position_reached = (
            abs(
                wrapped_error(
                    final_position,
                    expected_position,
                )
            )
            <= 8
        )

        if (
            elapsed >= expected_time
            and telemetry.moving == 0
        ):
            if (
                not movement_seen
                and not position_reached
            ):
                print()

                raise RuntimeError(
                    "Command was sent, but motor "
                    "movement was not detected"
                )

            print()
            return

        time.sleep(0.08)

    print()

    raise TimeoutError(
        "Segment did not finish. "
        f"Final position: {final_position}. "
        f"Expected: {expected_position}"
    )


def move_distance(
    bus,
    steps: int,
    direction_sign: int,
    rpm: float,
) -> None:
    """Move all required step segments."""

    segments = split_steps(steps)

    for index, segment_steps in enumerate(
        segments,
        start=1,
    ):
        run_segment(
            bus=bus,
            signed_steps=(
                direction_sign
                * segment_steps
            ),
            rpm=rpm,
            segment_number=index,
            segment_count=len(segments),
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Move an ST3215-powered wheel "
            "by distance in centimetres."
        )
    )

    parser.add_argument(
        "--port",
        required=True,
    )

    parser.add_argument(
        "--id",
        type=int,
        default=1,
        dest="servo_id",
    )

    parser.add_argument(
        "--distance-cm",
        type=float,
        required=True,
    )

    parser.add_argument(
        "--direction",
        choices=("forward", "back"),
        required=True,
    )

    parser.add_argument(
        "--rpm",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--wheel-diameter-mm",
        type=float,
        default=DEFAULT_WHEEL_DIAMETER_MM,
    )

    parser.add_argument(
        "--round-trip",
        action="store_true",
        help=(
            "Move the requested direction, "
            "pause, then return to start."
        ),
    )

    parser.add_argument(
        "--invert",
        action="store_true",
        help=(
            "Reverse the forward/back mapping "
            "if the wheel is mounted oppositely."
        ),
    )

    args = parser.parse_args()

    if not 0 < args.rpm <= 30:
        raise SystemExit(
            "RPM must be greater than zero "
            "and no more than 30"
        )

    try:
        (
            steps,
            rotations,
            circumference_cm,
        ) = distance_to_motion(
            args.distance_cm,
            args.wheel_diameter_mm,
        )

    except ValueError as error:
        raise SystemExit(str(error)) from error

    degrees = (
        steps
        * 360.0
        / STEPS_PER_REVOLUTION
    )

    actual_distance_cm = (
        steps
        / STEPS_PER_REVOLUTION
        * circumference_cm
    )

    expected_seconds = (
        rotations
        * 60.0
        / args.rpm
    )

    # Default mapping:
    # positive/CW = forward
    # negative/CCW = back
    direction_sign = (
        1
        if args.direction == "forward"
        else -1
    )

    if args.invert:
        direction_sign *= -1

    bus = create_bus(
        args.port,
        args.servo_id,
        name=MOTOR_NAME,
    )

    print(
        "========== DISTANCE CALCULATION =========="
    )

    print(
        f"Wheel diameter      : "
        f"{args.wheel_diameter_mm:.1f} mm"
    )

    print(
        f"Wheel circumference : "
        f"{circumference_cm:.4f} cm"
    )

    print(
        f"Requested distance  : "
        f"{args.distance_cm:.4f} cm "
        f"{args.direction}"
    )

    print(
        f"Required rotations  : "
        f"{rotations:.6f}"
    )

    print(
        f"Encoder command     : "
        f"{steps} steps"
    )

    print(
        f"Wheel angle         : "
        f"{degrees:.3f} degrees"
    )

    print(
        f"Calculated distance : "
        f"{actual_distance_cm:.4f} cm"
    )

    print(
        f"Estimated time      : "
        f"{expected_seconds:.2f} seconds"
    )

    print(
        "=========================================="
    )

    try:
        bus.connect()

        start_position = read_raw(
            bus,
            "Present_Position",
            MOTOR_NAME,
        )

        print(
            f"Starting encoder position: "
            f"{start_position}"
        )

        bus.disable_torque(MOTOR_NAME)

        bus.write(
            "Min_Position_Limit",
            MOTOR_NAME,
            0,
            normalize=False,
        )

        bus.write(
            "Max_Position_Limit",
            MOTOR_NAME,
            0,
            normalize=False,
        )

        bus.write(
            "Operating_Mode",
            MOTOR_NAME,
            OperatingMode.STEP.value,
            normalize=False,
        )

        bus.write(
            "Acceleration",
            MOTOR_NAME,
            30,
            normalize=False,
        )

        bus.write(
            "Goal_Velocity",
            MOTOR_NAME,
            rpm_to_raw_speed(args.rpm),
            normalize=False,
        )

        bus.enable_torque(MOTOR_NAME)

        move_distance(
            bus,
            steps,
            direction_sign,
            args.rpm,
        )

        if args.round_trip:
            print(
                "Pausing for one second "
                "before returning..."
            )

            time.sleep(1.0)

            move_distance(
                bus,
                steps,
                -direction_sign,
                args.rpm,
            )

        final = read_telemetry(
            bus,
            time.monotonic(),
        )

        final_error = wrapped_error(
            final.position_steps,
            start_position,
        )

        print(
            "\n============== FINAL RESULT =============="
        )

        print(
            f"Final position      : "
            f"{final.position_steps} steps"
        )

        print(
            f"Change from start   : "
            f"{final_error:+d} steps"
        )

        print(
            f"Voltage             : "
            f"{final.voltage_v:.1f} V"
        )

        print(
            f"Temperature         : "
            f"{final.temperature_c} C"
        )

        print(
            f"Current             : "
            f"{final.current_ma:.1f} mA"
        )

        print(
            f"Load                : "
            f"{final.load_percent:+.1f} %"
        )

        print(
            f"Status              : "
            f"{final.status_text}"
        )

        print(
            "=========================================="
        )

    except KeyboardInterrupt:
        print(
            "\nEmergency stop requested."
        )

    finally:
        lock_and_disconnect(
            bus,
            MOTOR_NAME,
            restore_position_mode=True,
            minimum_limit=0,
            maximum_limit=(
                STEPS_PER_REVOLUTION - 1
            ),
        )

        print(
            "Torque disabled; position mode restored."
        )


if __name__ == "__main__":
    main()
