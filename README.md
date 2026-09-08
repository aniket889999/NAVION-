# NAVION ST3215 Control Tools

Safety-focused command-line utilities for detecting, homing, positioning, and rotating a Waveshare/Feetech ST3215 smart serial-bus servo from macOS.

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

Press `Control+C` to request an emergency stop during continuous or multi-revolution commands.
