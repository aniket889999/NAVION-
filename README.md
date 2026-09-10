# NAVION ST3215 Control Tools

Safety-focused command-line utilities for detecting, homing, positioning, and rotating a Waveshare/Feetech ST3215 smart serial-bus servo from macOS.

## Code labels and file guide

Start the current application with `python -m st3215.task1_gui`.
The original filenames are preserved so existing terminal commands continue to work.

| Label | Code file | What it does |
| --- | --- | --- |
| APP — Motion console | [task1_gui.py](st3215/task1_gui.py) | Tkinter dashboard combining motor travel, W/S joystick, camera gestures, feedback and position history. |
| TASK 1 — Motor controller | [task1_motor.py](st3215/task1_motor.py) | Device reads, home calibration, whole rotations, centimetre travel and motor feedback for the GUI. |
| TASK 1 — Distance CLI | [move_distance.py](st3215/move_distance.py) | Standalone forward/back distance commands for the 65 mm wheel, with optional return. |
| TASK 2 — Joystick | [joystick.py](st3215/joystick.py) | Hold W/S or the on-screen stick to jog; stop on release, fault or the hold-time limit. |
| TASK 2 — Future IMU | [imu_control.py](st3215/imu_control.py) | Optional BNO055 serial/I2C readers and tilt filtering. Retained for later; inactive in the current GUI. |
| TASK 3 — Camera gestures | [camera_gesture.py](st3215/camera_gesture.py) | Hand tracking, gesture filtering and a separate camera process; thumb up/down and open-palm Stop. |
| CORE — Bus helpers | [common.py](st3215/common.py) | Shared servo connection, encoder conversions, speed conversion and cleanup helpers. |
| CLI 01 — Detect servo | [detect_servo.py](st3215/detect_servo.py) | Find servo IDs and baud rates on a USB adapter. |
| CLI 02 — Set home | [set_home.py](st3215/set_home.py) | Save the current physical shaft angle as logical home. |
| CLI 03 — Move angle | [move_angle.py](st3215/move_angle.py) | Command a selected angle and direction. |
| CLI 04 — Return home | [return_home.py](st3215/return_home.py) | Return to the saved logical home position. |
| CLI 05 — Timed velocity | [continuous_velocity.py](st3215/continuous_velocity.py) | Run the wheel at a selected speed for a bounded time. |
| CLI 06 — Revolutions | [rotate_revolutions.py](st3215/rotate_revolutions.py) | Command a selected number of revolutions. |
| CLI 07 — Rotation feedback | [rotate_with_feedback.py](st3215/rotate_with_feedback.py) | Rotation with live telemetry, a final health report and CSV recording. |
| PACKAGE — Module entry | [__init__.py](st3215/__init__.py) | Identifies the ST3215 Python package. |
| TESTS — Control regressions | [test_console_controls.py](tests/test_console_controls.py) | Mocked joystick, stop, camera dispatch, crash handling and history tests. |
| TESTS — Plans and filters | [test_sensor_controls.py](tests/test_sensor_controls.py) | Distance conversion, IMU parsing and gesture-filter tests. |

Supporting files: [desktop dependencies](requirements.txt),
[optional Raspberry Pi dependencies](requirements-rpi.txt),
[camera model provenance](models/README.md), and [work log](CHANGELOG.md).

Latest software checks: **24 hardware-free tests passed**. Live GUI layout,
camera recognition and physical motor operation still need local verification
for the current revision; passing mocked tests does not establish hardware performance.

## Hardware

- Mac connected to the bus-servo adapter by USB-C.
- Adapter jumpers set to **B/USB**.
- ST3215 connected to the adapter's bus-servo socket with `D`, `V`, and `G` aligned.
- Separate motor supply connected to the adapter. The supply voltage must match the exact servo variant.
- Keep the shaft and wiring clear before enabling torque or continuous rotation.

## Installation

```bash
brew install python@3.12
"$(brew --prefix python@3.12)/bin/python3.12" -m venv ~/lerobot-env
source ~/lerobot-env/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The examples below use this adapter port and servo ID:

```bash
export ST3215_PORT=/dev/cu.usbmodem5A7C1165021
export ST3215_ID=1
```

Replace them if the hardware changes.

## Commands

Detect motors without moving them:

```bash
python -m st3215.detect_servo --port "$ST3215_PORT"
```

Set the current physical angle as center/home (`2047`). This writes persistent calibration:

```bash
python -m st3215.set_home --port "$ST3215_PORT" --id "$ST3215_ID"
```

Move 20 degrees clockwise from home at 10 RPM:

```bash
python -m st3215.move_angle \
  --port "$ST3215_PORT" --id "$ST3215_ID" \
  --degrees 20 --direction cw --rpm 10
```

Return to home:

```bash
python -m st3215.return_home \
  --port "$ST3215_PORT" --id "$ST3215_ID" --rpm 10
```

Continuous wheel rotation for 10 seconds:

```bash
python -m st3215.continuous_velocity \
  --port "$ST3215_PORT" --id "$ST3215_ID" \
  --direction cw --rpm 10 --seconds 10
```

Rotate 361 degrees counterclockwise at 10 RPM:

```bash
python -m st3215.rotate_revolutions \
  --port "$ST3215_PORT" --id "$ST3215_ID" \
  --rotations 1.0027778 --direction ccw --rpm 10
```

Perform five clockwise revolutions at 20 RPM:

```bash
python -m st3215.rotate_revolutions \
  --port "$ST3215_PORT" --id "$ST3215_ID" \
  --rotations 5 --direction cw --rpm 20
```

Perform one exact clockwise revolution at 20 RPM, show live feedback, print a
final health report, and save every sample to CSV:

```bash
python -m st3215.rotate_with_feedback \
  --port "$ST3215_PORT" --id "$ST3215_ID" \
  --rotations 1 --direction cw --rpm 20 \
  --csv st3215-feedback.csv
```

The report includes position, speed, voltage, temperature, current, estimated
load, movement state, protection status, and peak/minimum values recorded during
the rotation. One revolution at 20 RPM should take approximately three seconds.

## NAVION graphical robot control

Install Tkinter once for the same Python 3.12 used by `lerobot-env`:

```bash
brew install python-tk@3.12
```

Start the graphical interface:

```bash
source ~/lerobot-env/bin/activate
cd ~/NAVION-
python -m st3215.task1_gui
```

The GUI provides these tabs:

- **Drive & joystick:** scan the adapter/ID, read feedback, persist the current shaft
  angle as logical home (`2047`), execute 1–100 whole rotations at 1–30 RPM,
  and move a calculated distance using a configurable wheel diameter (65 mm by
  default). Large rotation commands are divided into safe one-turn segments.
  Enable the joystick, then hold **W** for forward or **S** for back. Release
  the key to stop. Drag the on-screen stick up/down for the same control.
  **Space / Esc / STOP** stop and disarm controls. Key repeat cannot launch
  extra jobs; conflicting keys, lost window focus and editing an input stop
  joystick motion. A hold is capped at 30 seconds; release and press again.
- **Camera gestures:** preview is independent of motor arming. After the preview
  starts, enable camera movement, then hold a thumb up/down.
  Each stable thumb gesture sends the chosen distance at the shared RPM.
  Holding the same thumb does not repeat a move. Relax the hand or switch thumb
  direction before the next move. An open palm stops and
  disarms immediately, including after a previous thumb command.
- **Position history:** stores actual encoder readings before and after each
  motor task, including joystick release and interrupted tasks. The final read
  occurs after the controller's cleanup. Failed reads are marked unknown.
  CSV includes temperature, supply voltage, current and fault status.
- **BNO055 · later:** inactive information tab. The optional IMU code remains
  available, but no IMU is connected or started by the current GUI.

The controller disables torque after completion or an emergency stop.

### BNO055 connection

A bare BNO055 cannot be connected directly to a Mac USB-C port. These adapters
remain in `st3215/imu_control.py` for later integration:

1. **serial** (Mac): connect the sensor to an Arduino/RP2040 or another USB
   serial bridge. Send one pitch sample per line at 115200 baud as a number,
   `{"pitch": 12.5}`, `{"pitch_deg": 12.5}`, or
   `{"euler": [heading, roll, pitch]}`. Use a different serial port from the
   ST3215 adapter.
2. **i2c** (Raspberry Pi): wire the BNO055 to the Pi I2C pins, then install the
   hardware dependency with `python -m pip install -r requirements-rpi.txt`.

### Camera setup

The MediaPipe gesture model is stored at
`models/gesture_recognizer.task`. On first use, macOS may ask for Terminal camera
permission. If necessary, enable it in **System Settings → Privacy & Security →
Camera** and restart the GUI.

The camera uses MediaPipe VIDEO tracking with separate hand-detection and gesture
thresholds, a mirrored preview and landmark overlays. Gesture confidence defaults
to 0.65; raw labels/confidence remain visible even below the threshold. The GUI
does not execute stale frames or stack commands while the motor is busy. Camera
loss disarms movement and stops camera-originated travel.

MediaPipe runs in a separate process: native camera/graphics failures no longer
terminate the motor console. **View camera diagnostic log** displays errors from
`.navion/camera.log`. Frames are processed locally and not saved. Stop/restart
the preview after changing the camera index. Camera and joystick cannot be
armed together. Forward/back sign depends on wheel mounting; use **Invert
forward / back** after checking the direction.

Reference: [MediaPipe Gesture Recognizer Python guide](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python).

Run the hardware-free control tests with:

```bash
python -m unittest discover -s tests -v
```

Press `Control+C` to request an emergency stop during continuous or multi-revolution commands.
