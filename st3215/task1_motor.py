"""TASK 1 — Motor controller for homing, rotations, distance and feedback."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from threading import Event
from typing import Callable

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from serial.tools import list_ports

from .common import (
    DEFAULT_HOME,
    STEPS_PER_REVOLUTION,
    create_bus,
    lock_and_disconnect,
    read_raw,
    rpm_to_raw_speed,
)
from .rotate_with_feedback import Telemetry, read_telemetry

MOTOR_NAME = "wheel"
MAX_ROTATIONS = 100
MAX_DEMO_RPM = 30.0
DEFAULT_WHEEL_DIAMETER_MM = 65.0
MAX_DISTANCE_CM = 1_000.0
HOME_TOLERANCE_STEPS = 8
SAMPLE_PERIOD_SECONDS = 0.08


class MotionCancelled(RuntimeError):
    """Raised after the GUI requests a controlled emergency stop."""


@dataclass(frozen=True)
class DeviceScan:
    port: str
    baudrate: int
    servo_ids: tuple[int, ...]


@dataclass(frozen=True)
class MotorSnapshot:
    servo_id: int
    model_number: int
    homing_offset: int
    operating_mode: int
    torque_enabled: int
    telemetry: Telemetry


@dataclass(frozen=True)
class HomeResult:
    position_before: int
    previous_offset: int
    written_offset: int
    position_after: int


@dataclass(frozen=True)
class MotionPlan:
    rotations: int
    direction: str
    rpm: float
    direction_sign: int
    steps_per_rotation: int
    total_steps: int
    total_degrees: int
    seconds_per_rotation: float
    expected_seconds: float


@dataclass(frozen=True)
class MotionUpdate:
    phase: str
    completed_rotations: int
    requested_rotations: int
    progress_percent: float
    telemetry: Telemetry


@dataclass(frozen=True)
class MotionResult:
    plan: MotionPlan
    duration_seconds: float
    start_position: int
    final_position: int
    final_error_steps: int
    final_telemetry: Telemetry


@dataclass(frozen=True)
class DistancePlan:
    distance_cm: float
    wheel_diameter_mm: float
    direction: str
    rpm: float
    direction_sign: int
    circumference_cm: float
    rotations: float
    steps: int
    degrees: float
    calculated_distance_cm: float
    expected_seconds: float


@dataclass(frozen=True)
class DistanceUpdate:
    phase: str
    progress_percent: float
    traveled_cm: float
    telemetry: Telemetry


@dataclass(frozen=True)
class DistanceResult:
    plan: DistancePlan
    round_trip: bool
    duration_seconds: float
    start_position: int
    final_position: int
    final_error_steps: int
    final_telemetry: Telemetry


UpdateCallback = Callable[[MotionUpdate], None]
DistanceCallback = Callable[[DistanceUpdate], None]


def build_motion_plan(rotations: int, rpm: float, direction: str) -> MotionPlan:
    if isinstance(rotations, bool) or not isinstance(rotations, int):
        raise ValueError("Rotations must be a whole number")
    if not 1 <= rotations <= MAX_ROTATIONS:
        raise ValueError(f"Rotations must be between 1 and {MAX_ROTATIONS}")
    if not 0 < rpm <= MAX_DEMO_RPM:
        raise ValueError(f"RPM must be greater than 0 and no more than {MAX_DEMO_RPM:g}")
    if direction not in {"cw", "ccw"}:
        raise ValueError("Direction must be 'cw' or 'ccw'")

    direction_sign = 1 if direction == "cw" else -1
    return MotionPlan(
        rotations=rotations,
        direction=direction,
        rpm=rpm,
        direction_sign=direction_sign,
        steps_per_rotation=STEPS_PER_REVOLUTION,
        total_steps=rotations * STEPS_PER_REVOLUTION,
        total_degrees=rotations * 360,
        seconds_per_rotation=60.0 / rpm,
        expected_seconds=rotations * 60.0 / rpm,
    )


def build_distance_plan(
    distance_cm: float,
    wheel_diameter_mm: float,
    rpm: float,
    direction: str,
    invert_direction: bool = False,
) -> DistancePlan:
    if not 0 < distance_cm <= MAX_DISTANCE_CM:
        raise ValueError(
            f"Distance must be greater than 0 and no more than {MAX_DISTANCE_CM:g} cm"
        )
    if not math.isfinite(wheel_diameter_mm) or wheel_diameter_mm <= 0:
        raise ValueError("Wheel diameter must be greater than zero")
    if not 0 < rpm <= MAX_DEMO_RPM:
        raise ValueError(f"RPM must be greater than 0 and no more than {MAX_DEMO_RPM:g}")
    if direction not in {"forward", "back"}:
        raise ValueError("Direction must be 'forward' or 'back'")

    circumference_cm = math.pi * wheel_diameter_mm / 10.0
    rotations = distance_cm / circumference_cm
    steps = round(rotations * STEPS_PER_REVOLUTION)
    if steps == 0:
        raise ValueError("Distance is too small to produce one encoder step")

    direction_sign = 1 if direction == "forward" else -1
    if invert_direction:
        direction_sign *= -1

    degrees = steps * 360.0 / STEPS_PER_REVOLUTION
    calculated_distance_cm = steps / STEPS_PER_REVOLUTION * circumference_cm
    return DistancePlan(
        distance_cm=distance_cm,
        wheel_diameter_mm=wheel_diameter_mm,
        direction=direction,
        rpm=rpm,
        direction_sign=direction_sign,
        circumference_cm=circumference_cm,
        rotations=rotations,
        steps=steps,
        degrees=degrees,
        calculated_distance_cm=calculated_distance_cm,
        expected_seconds=rotations * 60.0 / rpm,
    )


def discover_usb_serial_ports() -> list[str]:
    ports = list(list_ports.comports())
    preferred = [
        port.device
        for port in ports
        if (port.vid, port.pid) == (0x1A86, 0x55D3)
    ]
    other_usb = [
        port.device
        for port in ports
        if "usb" in port.device.lower() and port.device not in preferred
    ]
    return preferred + other_usb


def scan_connected_servos() -> list[DeviceScan]:
    results: list[DeviceScan] = []
    for port in discover_usb_serial_ports():
        baudrate_ids = FeetechMotorsBus.scan_port(port)
        for baudrate, servo_ids in baudrate_ids.items():
            results.append(
                DeviceScan(
                    port=port,
                    baudrate=baudrate,
                    servo_ids=tuple(servo_ids),
                )
            )
    return results


def read_motor_snapshot(port: str, servo_id: int) -> MotorSnapshot:
    bus = create_bus(port, servo_id, name=MOTOR_NAME)
    try:
        bus.connect()
        started = time.monotonic()
        return MotorSnapshot(
            servo_id=servo_id,
            model_number=read_raw(bus, "Model_Number", MOTOR_NAME),
            homing_offset=read_raw(bus, "Homing_Offset", MOTOR_NAME),
            operating_mode=read_raw(bus, "Operating_Mode", MOTOR_NAME),
            torque_enabled=read_raw(bus, "Torque_Enable", MOTOR_NAME),
            telemetry=read_telemetry(bus, started),
        )
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def set_current_as_home(port: str, servo_id: int) -> HomeResult:
    """Persistently map the current physical position to logical position 2047."""
    bus = create_bus(port, servo_id, name=MOTOR_NAME)
    try:
        bus.connect()
        bus.disable_torque(MOTOR_NAME)
        bus.write(
            "Operating_Mode",
            MOTOR_NAME,
            OperatingMode.POSITION.value,
            normalize=False,
        )
        position_before = read_raw(bus, "Present_Position", MOTOR_NAME)
        previous_offset = read_raw(bus, "Homing_Offset", MOTOR_NAME)
        offsets = bus.set_half_turn_homings(MOTOR_NAME)
        written_offset = int(offsets[MOTOR_NAME])
        position_after = read_raw(bus, "Present_Position", MOTOR_NAME)
        return HomeResult(
            position_before=position_before,
            previous_offset=previous_offset,
            written_offset=written_offset,
            position_after=position_after,
        )
    finally:
        lock_and_disconnect(bus, MOTOR_NAME)


def wrapped_position_error(position: int, target: int = DEFAULT_HOME) -> int:
    half_turn = STEPS_PER_REVOLUTION // 2
    return (position - target + half_turn) % STEPS_PER_REVOLUTION - half_turn


def _check_stop(stop_event: Event) -> None:
    if stop_event.is_set():
        raise MotionCancelled("Motion stopped by the operator")


def _emit(
    callback: UpdateCallback | None,
    phase: str,
    completed: int,
    requested: int,
    progress: float,
    telemetry: Telemetry,
) -> None:
    if callback is None:
        return
    callback(
        MotionUpdate(
            phase=phase,
            completed_rotations=completed,
            requested_rotations=requested,
            progress_percent=max(0.0, min(100.0, progress)),
            telemetry=telemetry,
        )
    )


def _raise_on_fault(telemetry: Telemetry) -> None:
    if telemetry.status_raw:
        raise RuntimeError(
            f"Servo fault 0x{telemetry.status_raw:02X}: {telemetry.status_text}"
        )


def _configure_position_mode(bus, rpm: float) -> None:
    bus.disable_torque(MOTOR_NAME)
    bus.write(
        "Operating_Mode",
        MOTOR_NAME,
        OperatingMode.POSITION.value,
        normalize=False,
    )
    bus.write("Min_Position_Limit", MOTOR_NAME, 0, normalize=False)
    bus.write(
        "Max_Position_Limit",
        MOTOR_NAME,
        STEPS_PER_REVOLUTION - 1,
        normalize=False,
    )
    bus.write("Acceleration", MOTOR_NAME, 30, normalize=False)
    bus.write("Goal_Velocity", MOTOR_NAME, rpm_to_raw_speed(rpm), normalize=False)


def _move_to_home(
    bus,
    plan: MotionPlan,
    stop_event: Event,
    callback: UpdateCallback | None,
    run_started: float,
    completed_rotations: int,
) -> Telemetry:
    _configure_position_mode(bus, plan.rpm)
    bus.enable_torque(MOTOR_NAME)
    bus.write("Goal_Position", MOTOR_NAME, DEFAULT_HOME, normalize=False)

    started = time.monotonic()
    deadline = started + max(8.0, 60.0 / plan.rpm + 4.0)
    final = read_telemetry(bus, run_started)
    while time.monotonic() < deadline:
        _check_stop(stop_event)
        final = read_telemetry(bus, run_started)
        _raise_on_fault(final)
        error = wrapped_position_error(final.position_steps)
        _emit(
            callback,
            "Returning to home",
            completed_rotations,
            plan.rotations,
            completed_rotations / plan.rotations * 100.0,
            final,
        )
        if abs(error) <= HOME_TOLERANCE_STEPS and final.moving == 0:
            return final
        time.sleep(SAMPLE_PERIOD_SECONDS)

    raise TimeoutError(
        f"Servo did not reach home {DEFAULT_HOME}; last position was {final.position_steps}"
    )


def _run_one_revolution(
    bus,
    plan: MotionPlan,
    stop_event: Event,
    callback: UpdateCallback | None,
    run_started: float,
    completed_before: int,
) -> Telemetry:
    command_steps = plan.direction_sign * STEPS_PER_REVOLUTION
    bus.write("Goal_Position", MOTOR_NAME, command_steps, normalize=False)

    segment_started = time.monotonic()
    deadline = segment_started + plan.seconds_per_rotation + 4.0
    movement_seen = False
    final = read_telemetry(bus, run_started)

    while time.monotonic() < deadline:
        _check_stop(stop_event)
        final = read_telemetry(bus, run_started)
        _raise_on_fault(final)
        segment_elapsed = time.monotonic() - segment_started
        movement_seen |= final.moving == 1 or abs(final.velocity_rpm) >= 0.5
        fractional_turn = min(1.0, segment_elapsed / plan.seconds_per_rotation)
        progress = (completed_before + fractional_turn) / plan.rotations * 100.0
        _emit(
            callback,
            f"Rotation {completed_before + 1} of {plan.rotations}",
            completed_before,
            plan.rotations,
            progress,
            final,
        )

        if segment_elapsed >= plan.seconds_per_rotation and final.moving == 0:
            if not movement_seen:
                raise RuntimeError(
                    "The rotation command was sent, but no movement was detected"
                )
            return final
        time.sleep(SAMPLE_PERIOD_SECONDS)

    raise TimeoutError(
        f"Rotation {completed_before + 1} did not finish before the safety timeout"
    )


def rotate_whole_turns_and_return_home(
    port: str,
    servo_id: int,
    rotations: int,
    rpm: float,
    direction: str,
    stop_event: Event,
    callback: UpdateCallback | None = None,
) -> MotionResult:
    """Run whole turns as safe 4096-step segments, then correct back to home."""
    plan = build_motion_plan(rotations, rpm, direction)
    bus = create_bus(port, servo_id, name=MOTOR_NAME)
    run_started = time.monotonic()
    start_position = 0
    final_telemetry: Telemetry | None = None

    try:
        bus.connect()
        _check_stop(stop_event)
        start_position = read_raw(bus, "Present_Position", MOTOR_NAME)

        # Always start from logical home so completed whole rotations end there.
        _move_to_home(bus, plan, stop_event, callback, run_started, 0)

        bus.disable_torque(MOTOR_NAME)
        bus.write("Min_Position_Limit", MOTOR_NAME, 0, normalize=False)
        bus.write("Max_Position_Limit", MOTOR_NAME, 0, normalize=False)
        bus.write(
            "Operating_Mode",
            MOTOR_NAME,
            OperatingMode.STEP.value,
            normalize=False,
        )
        bus.write("Acceleration", MOTOR_NAME, 30, normalize=False)
        bus.write(
            "Goal_Velocity",
            MOTOR_NAME,
            rpm_to_raw_speed(plan.rpm),
            normalize=False,
        )
        bus.enable_torque(MOTOR_NAME)

        for completed in range(plan.rotations):
            final_telemetry = _run_one_revolution(
                bus,
                plan,
                stop_event,
                callback,
                run_started,
                completed,
            )

        final_telemetry = _move_to_home(
            bus,
            plan,
            stop_event,
            callback,
            run_started,
            plan.rotations,
        )
        final_error = wrapped_position_error(final_telemetry.position_steps)
        _emit(
            callback,
            "Complete — stopped at home",
            plan.rotations,
            plan.rotations,
            100.0,
            final_telemetry,
        )
        return MotionResult(
            plan=plan,
            duration_seconds=time.monotonic() - run_started,
            start_position=start_position,
            final_position=final_telemetry.position_steps,
            final_error_steps=final_error,
            final_telemetry=final_telemetry,
        )
    finally:
        lock_and_disconnect(
            bus,
            MOTOR_NAME,
            restore_position_mode=True,
            minimum_limit=0,
            maximum_limit=STEPS_PER_REVOLUTION - 1,
        )


def _distance_segments(total_steps: int) -> list[int]:
    """Split distance travel into commands no larger than one revolution."""
    segments: list[int] = []
    remaining = total_steps
    while remaining:
        segment = min(remaining, STEPS_PER_REVOLUTION)
        segments.append(segment)
        remaining -= segment
    return segments


def _emit_distance(
    callback: DistanceCallback | None,
    phase: str,
    progressed_steps: float,
    total_path_steps: int,
    circumference_cm: float,
    telemetry: Telemetry,
) -> None:
    if callback is None:
        return
    callback(
        DistanceUpdate(
            phase=phase,
            progress_percent=max(
                0.0,
                min(100.0, progressed_steps / total_path_steps * 100.0),
            ),
            traveled_cm=progressed_steps / STEPS_PER_REVOLUTION * circumference_cm,
            telemetry=telemetry,
        )
    )


def _configure_step_mode(bus, rpm: float) -> None:
    bus.disable_torque(MOTOR_NAME)
    bus.write("Min_Position_Limit", MOTOR_NAME, 0, normalize=False)
    bus.write("Max_Position_Limit", MOTOR_NAME, 0, normalize=False)
    bus.write(
        "Operating_Mode",
        MOTOR_NAME,
        OperatingMode.STEP.value,
        normalize=False,
    )
    bus.write("Acceleration", MOTOR_NAME, 30, normalize=False)
    bus.write("Goal_Velocity", MOTOR_NAME, rpm_to_raw_speed(rpm), normalize=False)
    bus.enable_torque(MOTOR_NAME)


def _run_distance_segment(
    bus,
    signed_steps: int,
    rpm: float,
    stop_event: Event,
    callback: DistanceCallback | None,
    run_started: float,
    phase: str,
    progressed_before: int,
    total_path_steps: int,
    circumference_cm: float,
) -> Telemetry:
    start_position = read_raw(bus, "Present_Position", MOTOR_NAME)
    expected_position = (start_position + signed_steps) % STEPS_PER_REVOLUTION
    expected_time = abs(signed_steps) / STEPS_PER_REVOLUTION * 60.0 / rpm
    bus.write("Goal_Position", MOTOR_NAME, signed_steps, normalize=False)

    segment_started = time.monotonic()
    deadline = segment_started + expected_time + 4.0
    movement_seen = False
    final = read_telemetry(bus, run_started)

    while time.monotonic() < deadline:
        _check_stop(stop_event)
        final = read_telemetry(bus, run_started)
        _raise_on_fault(final)
        elapsed = time.monotonic() - segment_started
        movement_seen |= final.moving == 1 or abs(final.velocity_rpm) >= 0.5
        segment_progress = min(
            abs(signed_steps),
            elapsed * rpm / 60.0 * STEPS_PER_REVOLUTION,
        )
        _emit_distance(
            callback,
            phase,
            progressed_before + segment_progress,
            total_path_steps,
            circumference_cm,
            final,
        )

        if elapsed >= expected_time and final.moving == 0:
            position_reached = (
                abs(wrapped_position_error(final.position_steps, expected_position))
                <= HOME_TOLERANCE_STEPS
            )
            if not position_reached or (abs(signed_steps) >= STEPS_PER_REVOLUTION and not movement_seen):
                raise RuntimeError(
                    "Distance target was not verified: motor stopped short or no full-turn motion was detected"
                )
            return final
        time.sleep(SAMPLE_PERIOD_SECONDS)

    raise TimeoutError(
        f"Distance segment did not finish; last position was {final.position_steps}"
    )


def move_distance_and_maybe_return(
    port: str,
    servo_id: int,
    distance_cm: float,
    wheel_diameter_mm: float,
    rpm: float,
    direction: str,
    stop_event: Event,
    callback: DistanceCallback | None = None,
    round_trip: bool = False,
    invert_direction: bool = False,
) -> DistanceResult:
    """Move the wheel a calculated distance, optionally returning to the start."""
    plan = build_distance_plan(
        distance_cm,
        wheel_diameter_mm,
        rpm,
        direction,
        invert_direction,
    )
    bus = create_bus(port, servo_id, name=MOTOR_NAME)
    run_started = time.monotonic()
    start_position = 0
    final: Telemetry | None = None
    total_path_steps = plan.steps * (2 if round_trip else 1)
    progressed_steps = 0

    try:
        bus.connect()
        _check_stop(stop_event)
        start_position = read_raw(bus, "Present_Position", MOTOR_NAME)
        _configure_step_mode(bus, plan.rpm)

        for segment in _distance_segments(plan.steps):
            final = _run_distance_segment(
                bus,
                plan.direction_sign * segment,
                plan.rpm,
                stop_event,
                callback,
                run_started,
                f"Moving {plan.direction}",
                progressed_steps,
                total_path_steps,
                plan.circumference_cm,
            )
            progressed_steps += segment

        if round_trip:
            pause_deadline = time.monotonic() + 1.0
            while time.monotonic() < pause_deadline:
                _check_stop(stop_event)
                time.sleep(0.05)

            for segment in _distance_segments(plan.steps):
                final = _run_distance_segment(
                    bus,
                    -plan.direction_sign * segment,
                    plan.rpm,
                    stop_event,
                    callback,
                    run_started,
                    "Returning to starting position",
                    progressed_steps,
                    total_path_steps,
                    plan.circumference_cm,
                )
                progressed_steps += segment

        if final is None:
            raise RuntimeError("No distance command was executed")

        expected_position = start_position
        if not round_trip:
            expected_position = (
                start_position + plan.direction_sign * plan.steps
            ) % STEPS_PER_REVOLUTION
        final_error = wrapped_position_error(
            final.position_steps,
            expected_position,
        )
        _emit_distance(
            callback,
            "Round trip complete" if round_trip else f"{plan.direction.title()} move complete",
            total_path_steps,
            total_path_steps,
            plan.circumference_cm,
            final,
        )
        return DistanceResult(
            plan=plan,
            round_trip=round_trip,
            duration_seconds=time.monotonic() - run_started,
            start_position=start_position,
            final_position=final.position_steps,
            final_error_steps=final_error,
            final_telemetry=final,
        )
    finally:
        lock_and_disconnect(
            bus,
            MOTOR_NAME,
            restore_position_mode=True,
            minimum_limit=0,
            maximum_limit=STEPS_PER_REVOLUTION - 1,
        )
