"""TASK 3 — Camera gestures: MediaPipe tracking and isolated camera processing."""

from __future__ import annotations

import os
import multiprocessing
import queue
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "gesture_recognizer.task"


@dataclass(frozen=True)
class GestureUpdate:
    gesture: str
    confidence: float
    state: str
    command: str | None
    frame_rgb: Any
    hand_count: int = 0
    fps: float = 0.0
    captured_at: float = 0.0


class GestureCommandGate:
    """Emit one command after a stable gesture, then require release to re-arm."""

    COMMANDS = {
        "Thumb_Up": "forward",
        "Thumb_Down": "back",
        "Open_Palm": "stop",
    }

    def __init__(self, stable_frames: int = 4, release_frames: int = 4) -> None:
        if stable_frames < 1 or release_frames < 1:
            raise ValueError("Frame counts must be at least 1")
        self.stable_frames = stable_frames
        self.release_frames = release_frames
        self._candidate = ""
        self._candidate_count = 0
        self._release_count = 0
        self._armed = True
        self._last_command = None

    def update(self, gesture: str) -> tuple[str | None, str]:
        command = self.COMMANDS.get(gesture)
        # Stop must bypass the movement latch, even immediately after a thumb.
        if command == "stop":
            self._armed = False
            self._candidate_count = 0
            self._last_command = "stop"
            return "stop", "Open palm: STOP"
        if command is None:
            self._release_count += 1
            if self._release_count >= 2:
                self._candidate = ""
                self._candidate_count = 0
            if self._release_count >= self.release_frames:
                self._armed = True
                self._last_command = None
            return None, "ready" if self._armed else "release gesture to re-arm"

        self._release_count = 0
        if gesture != self._candidate:
            self._candidate = gesture
            self._candidate_count = 1
        else:
            self._candidate_count += 1

        if (self._armed or command != self._last_command) and self._candidate_count >= self.stable_frames:
            self._armed = False
            self._last_command = command
            return command, f"issued {command}"
        if not self._armed:
            return None, "Move sent; relax hand or change thumb direction"
        return None, f"confirming {self._candidate_count}/{self.stable_frames}"


GestureCallback = Callable[[GestureUpdate], None]


def monitor_camera(
    camera_index: int,
    model_path: str | Path,
    minimum_confidence: float,
    stop_event: Event,
    callback: GestureCallback,
    stable_frames: int = 4,
) -> None:
    """Recognize gestures until stopped and send RGB preview frames to callback."""
    if not 0.0 <= minimum_confidence <= 1.0:
        raise ValueError("Camera confidence must be between 0 and 1")

    # MediaPipe imports matplotlib during startup. A writable cache avoids a noisy
    # warning on machines where ~/.matplotlib is read-only.
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/navion-matplotlib")
    import cv2
    import mediapipe as mp

    model = Path(model_path).expanduser().resolve()
    if not model.is_file():
        raise FileNotFoundError(
            f"Gesture model not found: {model}. Reinstall project dependencies/assets."
        )

    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    camera = cv2.VideoCapture(camera_index, backend)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        raise RuntimeError(
            f"Could not open camera {camera_index}. Allow camera access for Terminal in "
            "System Settings > Privacy & Security > Camera."
        )

    try:
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        options = recognizer_options(mp, model)
        with mp.tasks.vision.GestureRecognizer.create_from_options(options) as recognizer:
            timestamp_ms = 0
            missed_frames = 0
            while not stop_event.is_set():
                loop_started = time.monotonic()
                ok, frame_bgr = camera.read()
                if not ok:
                    missed_frames += 1
                    if missed_frames >= 15:
                        raise RuntimeError("Camera stopped returning frames; close other camera apps and retry")
                    stop_event.wait(0.1)
                    continue
                missed_frames = 0
                # Mirror before recognition, so the overlay and preview agree.
                frame_rgb = cv2.cvtColor(cv2.flip(frame_bgr, 1), cv2.COLOR_BGR2RGB)
                captured_at = time.monotonic()
                timestamp_ms = max(timestamp_ms + 1, int(captured_at * 1000))
                result = recognizer.recognize_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb), timestamp_ms,
                )

                gesture = "None"
                confidence = 0.0
                if result.gestures and result.gestures[0]:
                    top = result.gestures[0][0]
                    confidence = float(top.score)
                    gesture = top.category_name

                hand_count = len(result.hand_landmarks)
                # Movement gating belongs to the GUI, which knows arming/busy state.
                command = None
                state = f"{gesture.replace('_', ' ')} recognized"
                if hand_count == 0:
                    state = "No hand detected — show your whole hand in good light"
                elif hand_count > 1:
                    state = "Show one hand at a time"
                elif confidence < minimum_confidence:
                    state = f"Uncertain {gesture} ({confidence:.0%}); hold thumb upright, fingers folded"
                draw_landmarks(cv2, frame_rgb, result.hand_landmarks)
                label = f"{gesture} {confidence:.0%}"
                cv2.putText(
                    frame_rgb,
                    label,
                    (14, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (25, 220, 80),
                    2,
                    cv2.LINE_AA,
                )
                elapsed = max(time.monotonic() - loop_started, 0.001)
                callback(GestureUpdate(gesture, confidence, state, command, frame_rgb,
                                       hand_count, 1 / max(elapsed, 1 / 15), captured_at))

                remaining = 1 / 15 - (time.monotonic() - loop_started)
                if remaining > 0:
                    stop_event.wait(remaining)
    finally:
        camera.release()


def recognizer_options(mp, model):
    """Track video hands; classification confidence is filtered independently."""
    return mp.tasks.vision.GestureRecognizerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(model),
                                         delegate=mp.tasks.BaseOptions.Delegate.CPU),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )


def draw_landmarks(cv2, frame, hands):
    edges = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
             (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
             (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20))
    height, width = frame.shape[:2]
    for hand in hands:
        points = [(int(p.x * width), int(p.y * height)) for p in hand]
        for a, b in edges:
            cv2.line(frame, points[a], points[b], (70, 225, 190), 2)
        for point in points:
            cv2.circle(frame, point, 4, (245, 245, 245), -1)


def _camera_process(frames, messages, stop, config, log_path):
    """Native MediaPipe/macOS failures must not terminate the motor GUI."""
    with open(log_path, "w", buffering=1) as log:
        os.dup2(log.fileno(), 2)
        try:
            def publish(update):
                try:
                    frames.put_nowait(update)
                except queue.Full:
                    pass  # bounded queue; never accumulate seconds of old frames
            monitor_camera(stop_event=stop, callback=publish, **config)
        except Exception as exc:
            messages.put(str(exc))
        finally:
            frames.cancel_join_thread()


class CameraSession:
    """A disposable camera process, polled by the Tk main thread."""
    def __init__(self, camera_index=0, minimum_confidence=0.65):
        context = multiprocessing.get_context("spawn")
        self.frames = context.Queue(maxsize=2)
        self.messages = context.Queue(maxsize=4)
        self.stop_event = context.Event()
        self.log_path = PROJECT_ROOT / ".navion" / "camera.log"
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.process = context.Process(
            target=_camera_process,
            args=(self.frames, self.messages, self.stop_event,
                  dict(camera_index=camera_index, model_path=DEFAULT_MODEL_PATH,
                       minimum_confidence=minimum_confidence), str(self.log_path)),
            daemon=True,
        )
        self.stop_requested_at = None

    def start(self):
        self.process.start()

    def latest(self):
        latest = None
        try:
            while True:
                latest = self.frames.get_nowait()
        except queue.Empty:
            return latest

    def error(self):
        try:
            return self.messages.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.stop_event.set()
        if self.stop_requested_at is None:
            self.stop_requested_at = time.monotonic()

    def close(self):
        self.stop()
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=0.5)
        self.frames.close()
        self.messages.close()
