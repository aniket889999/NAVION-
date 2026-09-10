"""Hardware-free tests for NAVION distance, IMU, and gesture decisions."""

from __future__ import annotations

import unittest

from st3215.camera_gesture import GestureCommandGate
from st3215.imu_control import TiltCommandGate, parse_serial_pitch
from st3215.task1_motor import build_distance_plan


class DistancePlanTests(unittest.TestCase):
    def test_ten_centimetres_with_sixty_five_mm_wheel(self) -> None:
        plan = build_distance_plan(10.0, 65.0, 20.0, "forward")
        self.assertEqual(plan.steps, 2006)
        self.assertAlmostEqual(plan.rotations, 0.4897075, places=6)
        self.assertAlmostEqual(plan.calculated_distance_cm, 10.0, places=2)


class TiltGateTests(unittest.TestCase):
    def test_requires_stability_and_neutral_rearm(self) -> None:
        gate = TiltCommandGate(15.0, 7.0, stable_samples=3)
        self.assertIsNone(gate.update(20.0).command)
        self.assertIsNone(gate.update(20.0).command)
        self.assertEqual(gate.update(20.0).command, "forward")
        self.assertIsNone(gate.update(20.0).command)
        self.assertIsNone(gate.update(0.0).command)
        self.assertIsNone(gate.update(-20.0).command)
        self.assertIsNone(gate.update(-20.0).command)
        self.assertEqual(gate.update(-20.0).command, "back")

    def test_serial_formats(self) -> None:
        self.assertEqual(parse_serial_pitch("12.5"), 12.5)
        self.assertEqual(parse_serial_pitch('{"pitch": -8.25}'), -8.25)
        self.assertEqual(parse_serial_pitch('{"euler": [1, 2, 3]}'), 3.0)


class GestureGateTests(unittest.TestCase):
    def test_thumb_up_is_debounced_and_rearmed(self) -> None:
        gate = GestureCommandGate(stable_frames=2, release_frames=2)
        self.assertIsNone(gate.update("Thumb_Up")[0])
        self.assertEqual(gate.update("Thumb_Up")[0], "forward")
        self.assertIsNone(gate.update("Thumb_Up")[0])
        gate.update("None")
        gate.update("None")
        self.assertIsNone(gate.update("Thumb_Down")[0])
        self.assertEqual(gate.update("Thumb_Down")[0], "back")


if __name__ == "__main__":
    unittest.main()
