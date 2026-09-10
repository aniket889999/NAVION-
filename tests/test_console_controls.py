"""Control regressions with no serial traffic or camera access."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from st3215.camera_gesture import GestureCommandGate, GestureUpdate
from st3215.joystick import KeyLatch, jog_wheel
from st3215.task1_gui import MotorControlApp


class Var:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class GestureRegressionTests(unittest.TestCase):
    def test_stop_is_not_blocked_after_thumb(self):
        gate = GestureCommandGate(stable_frames=2)
        gate.update("Thumb_Up")
        self.assertEqual(gate.update("Thumb_Up")[0], "forward")
        self.assertEqual(gate.update("Open_Palm")[0], "stop")

    def test_opposite_thumb_requires_stability_but_not_missing_hand(self):
        gate = GestureCommandGate(stable_frames=2)
        gate.update("Thumb_Up")
        gate.update("Thumb_Up")
        self.assertIsNone(gate.update("Thumb_Down")[0])
        self.assertEqual(gate.update("Thumb_Down")[0], "back")

    def test_hold_does_not_repeat_and_short_dropout_does_not_rearm(self):
        gate = GestureCommandGate(stable_frames=2, release_frames=8)
        gate.update("Thumb_Up")
        gate.update("Thumb_Up")
        for _ in range(4):
            gate.update("None")
        for _ in range(20):
            self.assertIsNone(gate.update("Thumb_Up")[0])


def camera_app():
    return SimpleNamespace(
        last_camera_frame=0, last_camera_image=None,
        _render_camera=Mock(), gesture_var=Var(), confidence_var=Var(),
        camera_arm_button=Mock(), camera_confidence_var=Var("0.65"),
        camera_armed=Var(True), camera_status=Var(), busy=False,
        camera_ready_after=0,
        gesture_gate=GestureCommandGate(stable_frames=2),
        _start_distance=Mock(), _request_stop=Mock(),
    )


def frame(gesture="Thumb_Up", confidence=0.95, hand_count=1, age=0):
    return GestureUpdate(gesture, confidence, "detected", None, None,
                         hand_count, 15, time.monotonic() - age)


class CameraDispatchTests(unittest.TestCase):
    def test_preview_only_never_moves(self):
        app = camera_app()
        app.camera_armed.set(False)
        for _ in range(8):
            MotorControlApp._camera_frame(app, frame())
        app._start_distance.assert_not_called()

    def test_stale_low_confidence_and_two_hands_do_not_move(self):
        for update in (frame(age=3), frame(confidence=0.4), frame(hand_count=2)):
            app = camera_app()
            for _ in range(8):
                MotorControlApp._camera_frame(app, update)
            app._start_distance.assert_not_called()

    def test_busy_motor_has_no_queued_gesture(self):
        app = camera_app()
        app.busy = True
        for _ in range(8):
            MotorControlApp._camera_frame(app, frame())
        app._start_distance.assert_not_called()

    def test_armed_stable_thumb_sends_one_move_per_hold(self):
        app = camera_app()
        MotorControlApp._camera_frame(app, frame())
        app._start_distance.assert_not_called()
        MotorControlApp._camera_frame(app, frame())
        for _ in range(10):
            MotorControlApp._camera_frame(app, frame())
        app._start_distance.assert_called_once_with("forward", source="camera")

    def test_open_palm_stops_even_when_busy_and_disarmed(self):
        app = camera_app()
        app.busy = True
        app.camera_armed.set(False)
        MotorControlApp._camera_frame(app, frame("Open_Palm"))
        app._request_stop.assert_called_once()

    def test_invalid_threshold_disarms(self):
        app = camera_app()
        app.camera_confidence_var.set("nan")
        MotorControlApp._camera_frame(app, frame())
        self.assertFalse(app.camera_armed.get())
        app._start_distance.assert_not_called()

    def test_task_completion_preserves_latch_and_records_final_position(self):
        app = camera_app()
        MotorControlApp._camera_frame(app, frame())
        MotorControlApp._camera_frame(app, frame())
        gate = app.gesture_gate
        app.active_source = "camera"
        app._set_busy = Mock()
        app._draw_stick = Mock()
        app._apply_telemetry = Mock()
        app.history_records = []
        app.history_tree = Mock()
        app.last_position_var = Var()
        app.status_var = Var()
        app.joystick_status = Var()
        app.closing = False
        before = SimpleNamespace(telemetry=SimpleNamespace(position_steps=2047))
        after = SimpleNamespace(telemetry=SimpleNamespace(position_steps=4053,
                                temperature_c=30, voltage_v=6.0, current_ma=0, status_text="OK"))
        MotorControlApp._complete(app, "Camera forward", before, after, "Complete", None)
        self.assertIs(app.gesture_gate, gate)
        self.assertEqual(app.history_records[0]["final"], 4053)
        for _ in range(8):
            MotorControlApp._camera_frame(app, frame())
        app._start_distance.assert_called_once()

    def test_native_camera_exit_stops_camera_job_without_closing_gui(self):
        session = SimpleNamespace(
            latest=lambda: None, error=lambda: None,
            process=SimpleNamespace(is_alive=lambda: False, exitcode=-6),
            stop_requested_at=None, stop_event=threading.Event(), close=Mock(),
        )
        app = SimpleNamespace(
            camera=session, last_camera_frame=time.monotonic(), camera_started_at=time.monotonic(),
            camera_armed=Var(True), active_source="camera", stop_event=threading.Event(),
            camera_arm_button=Mock(), start_camera_button=Mock(), camera_status=Var(),
            last_camera_image=None, camera_preview=Mock(),
        )
        MotorControlApp._poll_camera(app)
        self.assertTrue(app.stop_event.is_set())
        self.assertIsNone(app.camera)
        self.assertFalse(app.camera_armed.get())
        self.assertIn("exited (-6)", app.camera_status.get())
        session.close.assert_called_once()


class JoystickInputTests(unittest.TestCase):
    def test_key_repeat_never_restarts_a_task(self):
        keys = KeyLatch()
        self.assertEqual(keys.press("w"), "forward")
        for _ in range(25):
            self.assertIsNone(keys.press("w"))
        self.assertTrue(keys.release("w"))
        self.assertEqual(keys.press("w"), "forward")

    def test_conflicting_keys_stop_instead_of_reverse(self):
        keys = KeyLatch()
        keys.press("w")
        self.assertEqual(keys.press("s"), "stop")
        keys.release("w")
        self.assertIsNone(keys.press("s"))
        keys.release("s")
        self.assertEqual(keys.press("s"), "back")

    def test_typing_w_in_an_entry_never_jogs(self):
        app = SimpleNamespace(
            release_timers={}, focus_get=lambda: SimpleNamespace(winfo_class=lambda: "TEntry"),
            _start_jog=Mock(),
        )
        MotorControlApp._key_press(app, SimpleNamespace(keysym="w"))
        app._start_jog.assert_not_called()

    def test_key_release_signals_worker_stop(self):
        app = SimpleNamespace(
            release_timers={}, keys=KeyLatch(), active_source="joystick",
            stop_event=threading.Event(), joystick_status=Var(),
        )
        app.keys.press("s")
        MotorControlApp._release_key(app, "s")
        self.assertTrue(app.stop_event.is_set())

    def test_focus_loss_stops_joystick(self):
        app = SimpleNamespace(focus_get=lambda: None, active_source="joystick",
            stop_event=threading.Event(), camera_armed=Var(True), keys=KeyLatch(),
            pointer_active=True, _draw_stick=Mock())
        MotorControlApp._check_focus(app)
        self.assertTrue(app.stop_event.is_set())
        self.assertFalse(app.camera_armed.get())


class JogMotorTests(unittest.TestCase):
    def setUp(self):
        self.bus = Mock(is_connected=True)
        self.stop = threading.Event()
        self.telemetry = SimpleNamespace(status_raw=0)

    def test_release_before_connect_finishes_never_enables_torque(self):
        self.stop.set()
        with patch("st3215.joystick.create_bus", return_value=self.bus), \
             patch("st3215.joystick.lock_and_disconnect") as cleanup:
            jog_wheel("fake", 1, 10, "forward", False, self.stop)
        self.bus.enable_torque.assert_not_called()
        cleanup.assert_called_once()

    def test_release_stops_speed_and_cleans_up(self):
        with patch("st3215.joystick.create_bus", return_value=self.bus), \
             patch("st3215.joystick.read_telemetry", return_value=self.telemetry), \
             patch("st3215.joystick.lock_and_disconnect") as cleanup:
            jog_wheel("fake", 1, 10, "back", False, self.stop,
                      callback=lambda _: self.stop.set())
        speeds = [c.args[2] for c in self.bus.write.call_args_list if c.args[0] == "Goal_Velocity"]
        self.assertLess(speeds[-2], 0)
        self.assertEqual(speeds[-1], 0)
        cleanup.assert_called_once()

    def test_serial_error_still_attempts_stop_and_cleanup(self):
        with patch("st3215.joystick.create_bus", return_value=self.bus), \
             patch("st3215.joystick.read_telemetry", side_effect=[self.telemetry, OSError("lost USB")]), \
             patch("st3215.joystick.lock_and_disconnect") as cleanup:
            with self.assertRaisesRegex(OSError, "lost USB"):
                jog_wheel("fake", 1, 10, "forward", False, self.stop)
        self.bus.write.assert_called_with("Goal_Velocity", "wheel", 0, normalize=False)
        cleanup.assert_called_once()

    def test_invalid_speed_never_connects(self):
        with patch("st3215.joystick.create_bus") as create:
            with self.assertRaises(ValueError):
                jog_wheel("fake", 1, 300, "forward", False, self.stop)
        create.assert_not_called()


if __name__ == "__main__":
    unittest.main()
