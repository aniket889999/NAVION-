"""APP — Motion console: manual travel, hold-to-run joystick and camera."""
from __future__ import annotations

import argparse
import csv
import math
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .camera_gesture import CameraSession, GestureCommandGate
from .common import STEPS_PER_REVOLUTION
from .joystick import KeyLatch, jog_wheel
from .task1_motor import (
    MotionCancelled, build_distance_plan, build_motion_plan,
    move_distance_and_maybe_return, read_motor_snapshot,
    rotate_whole_turns_and_return_home, scan_connected_servos,
    set_current_as_home,
)

BG = "#101923"
CARD = "#182532"
FIELD = "#223443"
INK = "#edf4f7"
MUTED = "#9aafbd"
ACCENT = "#5ce0bd"
RED = "#ff777c"
DEFAULT_PORT = "/dev/cu.usbmodem5A7C1165021"


class MotorControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NAVION · Motion Console")
        width = min(1180, self.winfo_screenwidth() - 70)
        height = min(870, self.winfo_screenheight() - 85)
        self.geometry(f"{width}x{height}+25+25")
        self.minsize(940, 680)
        self.configure(bg=BG)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None
        self.busy = False
        self.active_source = ""
        self.closing = False
        self.camera = None
        self.camera_started_at = 0.0
        self.last_camera_frame = 0.0
        self.last_camera_image = None
        self.camera_photo = None
        self.gesture_gate = GestureCommandGate(stable_frames=4, release_frames=8)
        self.camera_ready_after = 0.0
        self.keys = KeyLatch()
        self.release_timers = {}
        self.pointer_active = False
        self.pointer_direction = None
        self.history_records = []
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.id_var = tk.StringVar(value="1")
        self.rpm_var = tk.StringVar(value="10")
        self.rotations_var = tk.StringVar(value="10")
        self.direction_var = tk.StringVar(value="cw")
        self.distance_var = tk.StringVar(value="10")
        self.wheel_diameter_var = tk.StringVar(value="65")
        self.invert_var = tk.BooleanVar(value=False)
        self.joystick_enabled = tk.BooleanVar(value=False)
        self.camera_armed = tk.BooleanVar(value=False)
        self.camera_index_var = tk.StringVar(value="0")
        self.camera_confidence_var = tk.StringVar(value="0.65")
        self.sensor_distance_var = tk.StringVar(value="10")
        self.camera_status = tk.StringVar(value="Preview is off")
        self.gesture_var = tk.StringVar(value="Waiting for camera")
        self.confidence_var = tk.DoubleVar(value=0)
        self.joystick_status = tk.StringVar(value="Enable joystick to use W / S")
        self.status_var = tk.StringVar(value="Ready. Read the motor to see its current position.")
        self.connection_var = tk.StringVar(value="NOT READ")
        self.last_position_var = tk.StringVar(value="No task completed in this session")
        self.progress_var = tk.DoubleVar(value=0)
        self.plan_var = tk.StringVar()
        self.distance_plan_var = tk.StringVar()
        self.metrics = {key: tk.StringVar(value="—") for key in
                        ("position", "angle", "speed", "temperature", "voltage", "current", "load", "fault")}
        self.manual_widgets = []
        self._style()
        self._build_ui()
        for var in (self.rpm_var, self.rotations_var, self.distance_var,
                    self.wheel_diameter_var, self.direction_var, self.invert_var):
            var.trace_add("write", lambda *_: self._refresh_plans())
        self._refresh_plans()
        self.bind("<KeyPress>", self._key_press)
        self.bind("<KeyRelease>", self._key_release)
        self.bind("<FocusOut>", lambda _: self.after_idle(self._check_focus))
        self.bind("<ButtonRelease-1>", self._pointer_release)
        self.bind("<Escape>", lambda _: self._request_stop())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(40, self._poll)

    def _style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", font=("Helvetica", 11), background=BG, foreground=INK)
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD)
        style.configure("TLabel", background=BG, foreground=INK)
        style.configure("Card.TLabel", background=CARD)
        style.configure("Muted.TLabel", foreground=MUTED, background=CARD)
        style.configure("Title.TLabel", font=("Helvetica", 26, "bold"))
        style.configure("Heading.TLabel", background=CARD, font=("Helvetica", 14, "bold"))
        style.configure("Metric.TLabel", background=CARD, foreground=ACCENT, font=("Menlo", 19, "bold"))
        style.configure("TButton", background=FIELD, foreground=INK, padding=(12, 9), borderwidth=0)
        style.map("TButton", background=[("active", "#334c60"), ("disabled", CARD)],
                  foreground=[("disabled", "#607888")])
        style.configure("Primary.TButton", background=ACCENT, foreground=BG, font=("Helvetica", 11, "bold"))
        style.map("Primary.TButton", background=[("active", "#95f1d8"), ("disabled", FIELD)],
                  foreground=[("disabled", MUTED)])
        style.configure("Stop.TButton", background=RED, foreground=BG, font=("Helvetica", 12, "bold"))
        style.map("Stop.TButton", background=[("active", "#ffa8ac")])
        style.configure("TEntry", fieldbackground=FIELD, foreground=INK, insertcolor=INK, padding=6)
        style.configure("TCombobox", fieldbackground=FIELD, foreground=INK, padding=6)
        style.map("TCombobox", fieldbackground=[("readonly", FIELD)], foreground=[("readonly", INK)])
        style.configure("TCheckbutton", background=CARD, foreground=INK)
        style.map("TCheckbutton", background=[("active", CARD)], foreground=[("disabled", MUTED)])
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=BG, padding=(20, 10), foreground=MUTED)
        style.map("TNotebook.Tab", background=[("selected", CARD)], foreground=[("selected", ACCENT)])
        style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor=FIELD, borderwidth=0)
        style.configure("Treeview", background=CARD, fieldbackground=CARD, foreground=INK,
                        rowheight=30, borderwidth=0)
        style.configure("Treeview.Heading", background=FIELD, foreground=MUTED, padding=8)
        style.map("Treeview", background=[("selected", "#315a68")])

    def _card(self, parent, title):
        card = ttk.Frame(parent, style="Card.TFrame", padding=16)
        if title:
            ttk.Label(card, text=title, style="Heading.TLabel").pack(anchor="w", pady=(0, 10))
        return card

    def _entry(self, parent, label, variable, width=10):
        box = ttk.Frame(parent, style="Card.TFrame")
        ttk.Label(box, text=label, style="Muted.TLabel").pack(anchor="w", pady=(0, 5))
        entry = ttk.Entry(box, textvariable=variable, width=width)
        entry.pack(fill="x")
        self.manual_widgets.append(entry)
        return box

    def _button(self, parent, text, command, primary=False):
        button = ttk.Button(parent, text=text, command=command,
                            style="Primary.TButton" if primary else "TButton")
        self.manual_widgets.append(button)
        return button

    def _build_ui(self):
        shell = ttk.Frame(self, padding=18)
        shell.pack(fill="both", expand=True)
        header = ttk.Frame(shell)
        header.pack(fill="x", pady=(0, 12))
        ttk.Label(header, text="NAVION", style="Title.TLabel").pack(side="left")
        ttk.Label(header, text="  /  MOTION CONSOLE", foreground=MUTED).pack(side="left")
        ttk.Label(header, textvariable=self.connection_var, foreground=ACCENT).pack(side="right")
        footer = ttk.Frame(shell)
        footer.pack(side="bottom", fill="x", pady=(10, 0))
        ttk.Button(footer, text="STOP  ·  SPACE / ESC", command=self._request_stop,
                   style="Stop.TButton").pack(side="right", padx=(12, 0))
        ttk.Label(footer, textvariable=self.status_var, wraplength=740).pack(side="left", fill="x", expand=True)
        ttk.Progressbar(shell, variable=self.progress_var, maximum=100).pack(side="bottom", fill="x", pady=(10, 0))

        device = self._card(shell, "")
        device.pack(fill="x", pady=(0, 10))
        ttk.Label(device, text="USB", style="Muted.TLabel").pack(side="left", padx=(0, 8))
        port = ttk.Entry(device, textvariable=self.port_var, width=36)
        port.pack(side="left", fill="x", expand=True)
        self.manual_widgets.append(port)
        ttk.Label(device, text="ID", style="Muted.TLabel").pack(side="left", padx=(12, 8))
        identity = ttk.Entry(device, textvariable=self.id_var, width=4)
        identity.pack(side="left")
        self.manual_widgets.append(identity)
        self._button(device, "Scan", self._scan).pack(side="left", padx=8)
        self._button(device, "Read motor", self._read_motor, True).pack(side="left")

        strip = ttk.Frame(shell)
        strip.pack(fill="x", pady=(0, 12))
        for column, (key, label) in enumerate((
            ("position", "ENCODER"), ("angle", "ANGLE"), ("speed", "SPEED"),
            ("temperature", "TEMPERATURE"), ("voltage", "SUPPLY"),
        )):
            strip.columnconfigure(column, weight=1)
            tile = self._card(strip, "")
            tile.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 8, 0))
            ttk.Label(tile, text=label, style="Muted.TLabel").pack(anchor="w")
            ttk.Label(tile, textvariable=self.metrics[key], style="Metric.TLabel").pack(anchor="w", pady=(5, 0))

        details = ttk.Frame(shell)
        details.pack(fill="x", pady=(0, 8))
        for key, label in (("current", "Current"), ("load", "Load"), ("fault", "Servo status")):
            ttk.Label(details, text=f"{label}:", foreground=MUTED).pack(side="left", padx=(0, 6))
            ttk.Label(details, textvariable=self.metrics[key]).pack(side="left", padx=(0, 22))

        self.notebook = ttk.Notebook(shell)
        self.notebook.pack(fill="both", expand=True)
        self.drive_tab = ttk.Frame(self.notebook, padding=(0, 12))
        camera_tab = ttk.Frame(self.notebook, padding=(0, 12))
        history_tab = ttk.Frame(self.notebook, padding=(0, 12))
        imu_tab = ttk.Frame(self.notebook, padding=(0, 12))
        self.notebook.add(self.drive_tab, text="Drive & joystick")
        self.notebook.add(camera_tab, text="Camera gestures")
        self.notebook.add(history_tab, text="Position history")
        self.notebook.add(imu_tab, text="BNO055 · later")
        self.notebook.bind("<<NotebookTabChanged>>", self._tab_changed)
        self._build_drive()
        self._build_camera(camera_tab)
        self._build_history(history_tab)
        info = self._card(imu_tab, "BNO055 is not connected yet")
        info.pack(fill="both", expand=True)
        ttk.Label(info, text="W / S and the on-screen joystick are ready for this stage.\n\n"
                  "The BNO055 adapter code is retained for later setup. No IMU is opened automatically.",
                  style="Muted.TLabel", wraplength=760).pack(anchor="w")

    def _build_drive(self):
        self.drive_tab.columnconfigure(0, weight=2, minsize=310)
        self.drive_tab.columnconfigure(1, weight=3)
        self.drive_tab.rowconfigure(0, weight=1)
        left = self._card(self.drive_tab, "Joystick")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        ttk.Checkbutton(left, text="Enable W / S + on-screen stick", variable=self.joystick_enabled,
                        command=self._toggle_joystick).pack(anchor="w")
        ttk.Label(left, text="Hold W = forward · hold S = back\nRelease = stop · Space = STOP",
                  style="Muted.TLabel").pack(anchor="w", pady=(8, 5))
        self.stick = tk.Canvas(left, width=280, height=210, bg=CARD, highlightthickness=0, takefocus=True)
        self.stick.pack(fill="both", expand=True)
        self.stick.bind("<Configure>", lambda _: self._draw_stick())
        self.stick.bind("<ButtonPress-1>", self._pointer_press)
        self.stick.bind("<B1-Motion>", self._pointer_move)
        ttk.Label(left, textvariable=self.joystick_status, style="Muted.TLabel", wraplength=310).pack(anchor="w")
        ttk.Label(left, text="Single wheel: forward / back only.\nEach hold is limited to 30 seconds.",
                  style="Muted.TLabel").pack(anchor="w", pady=(8, 0))

        right = ttk.Frame(self.drive_tab)
        right.grid(row=0, column=1, sticky="nsew")
        shared = self._card(right, "")
        shared.pack(fill="x", pady=(0, 10))
        self._entry(shared, "RPM · all controls", self.rpm_var, 7).pack(side="left", padx=(0, 16))
        self._entry(shared, "Wheel diameter · mm", self.wheel_diameter_var, 9).pack(side="left", padx=(0, 16))
        invert = ttk.Checkbutton(shared, text="Invert forward / back", variable=self.invert_var)
        invert.pack(side="left", anchor="s", pady=7)
        self.manual_widgets.append(invert)
        tasks = self._card(right, "Precise travel")
        tasks.pack(fill="both", expand=True)
        turn = ttk.Frame(tasks, style="Card.TFrame")
        turn.pack(fill="x")
        self._entry(turn, "Whole rotations", self.rotations_var, 7).pack(side="left", padx=(0, 10))
        direction = ttk.Combobox(turn, textvariable=self.direction_var, values=("cw", "ccw"),
                                 width=6, state="readonly")
        direction.pack(side="left", anchor="s", padx=(0, 10))
        self.manual_widgets.append(direction)
        self._button(turn, "Run rotations", self._start_rotation, True).pack(side="right", anchor="s")
        ttk.Label(tasks, textvariable=self.plan_var, style="Muted.TLabel", wraplength=540).pack(anchor="w", pady=(8, 12))
        distance = ttk.Frame(tasks, style="Card.TFrame")
        distance.pack(fill="x")
        self._entry(distance, "Travel · cm", self.distance_var, 7).pack(side="left", padx=(0, 10))
        self._button(distance, "Forward", lambda: self._start_distance("forward")).pack(side="left", anchor="s", padx=3)
        self._button(distance, "Back", lambda: self._start_distance("back")).pack(side="left", anchor="s", padx=3)
        self._button(distance, "Out + return", lambda: self._start_distance("forward", True)).pack(side="right", anchor="s")
        ttk.Label(tasks, textvariable=self.distance_plan_var, style="Muted.TLabel", wraplength=540).pack(anchor="w", pady=8)
        self._button(tasks, "Set current angle as home · 2047", self._set_home).pack(anchor="w", pady=(6, 0))
        ttk.Label(tasks, text="Distance is estimated wheel travel; slip is not measured.",
                  style="Muted.TLabel").pack(anchor="w", pady=(8, 0))

    def _build_camera(self, tab):
        tab.columnconfigure(0, weight=3)
        tab.columnconfigure(1, weight=2, minsize=315)
        tab.rowconfigure(0, weight=1)
        preview = self._card(tab, "Live hand tracking")
        preview.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.camera_preview = tk.Canvas(preview, bg="#0b121a", highlightthickness=0)
        self.camera_preview.pack(fill="both", expand=True)
        self.camera_preview.bind("<Configure>", lambda _: self._render_camera())
        self.camera_preview.create_text(200, 120, text="Start preview to see your hand", fill=MUTED)
        controls = self._card(tab, "Gesture control")
        controls.grid(row=0, column=1, sticky="nsew")
        row = ttk.Frame(controls, style="Card.TFrame")
        row.pack(fill="x")
        self._entry(row, "Camera index", self.camera_index_var, 5).pack(side="left", padx=(0, 12))
        self._entry(row, "Gesture confidence", self.camera_confidence_var, 7).pack(side="left")
        buttons = ttk.Frame(controls, style="Card.TFrame")
        buttons.pack(fill="x", pady=12)
        self.start_camera_button = ttk.Button(buttons, text="Start preview", command=self._start_camera, style="Primary.TButton")
        self.start_camera_button.pack(side="left", padx=(0, 7))
        ttk.Button(buttons, text="Stop camera", command=self._stop_camera).pack(side="left")
        self._entry(controls, "Distance per gesture · cm", self.sensor_distance_var, 9).pack(anchor="w", pady=(0, 10))
        self.camera_arm_button = ttk.Checkbutton(
            controls, text="Enable camera movement", variable=self.camera_armed,
            command=self._toggle_camera_arm, state="disabled")
        self.camera_arm_button.pack(anchor="w")
        ttk.Label(controls, text="Thumb up → forward\nThumb down → back\nOpen palm → stop + disarm",
                  style="Card.TLabel").pack(anchor="w", pady=(12, 10))
        ttk.Label(controls, textvariable=self.gesture_var, style="Heading.TLabel", wraplength=320).pack(anchor="w")
        ttk.Progressbar(controls, variable=self.confidence_var, maximum=100).pack(fill="x", pady=8)
        ttk.Label(controls, textvariable=self.camera_status, style="Muted.TLabel", wraplength=310).pack(anchor="w")
        ttk.Label(controls, text="One move per gesture. Show your full hand.\n"
                  "Fold four fingers; keep your thumb upright.\nCamera runs locally. No frames are saved.",
                  style="Muted.TLabel", wraplength=310).pack(anchor="w", pady=(12, 0))
        ttk.Button(controls, text="View camera diagnostic log", command=self._show_camera_log).pack(anchor="w", pady=(10, 0))

    def _build_history(self, tab):
        card = self._card(tab, "Position after every motor task")
        card.pack(fill="both", expand=True)
        ttk.Label(card, textvariable=self.last_position_var, style="Muted.TLabel").pack(anchor="w", pady=(0, 10))
        row = ttk.Frame(card, style="Card.TFrame")
        row.pack(side="bottom", fill="x", pady=(10, 0))
        ttk.Button(row, text="Export CSV", command=self._export_history).pack(side="right")
        ttk.Label(row, text="Encoder is a shaft angle, not a cumulative revolution count.",
                  style="Muted.TLabel").pack(side="left")
        table = ttk.Frame(card, style="Card.TFrame")
        table.pack(fill="both", expand=True)
        columns = ("time", "task", "start", "final", "angle", "result")
        self.history_tree = ttk.Treeview(table, columns=columns, show="headings")
        for col, title, width in zip(columns,
             ("Time", "Task", "Start", "After task", "Angle °", "Result"),
             (140, 245, 65, 80, 75, 285)):
            self.history_tree.heading(col, text=title)
            self.history_tree.column(col, width=width, minwidth=55)
        scroll = ttk.Scrollbar(table, command=self.history_tree.yview)
        self.history_tree.configure(yscrollcommand=scroll.set)
        self.history_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _refresh_plans(self):
        try:
            plan = build_motion_plan(int(self.rotations_var.get()), float(self.rpm_var.get()), self.direction_var.get())
            self.plan_var.set(f"{plan.total_degrees:,}° · {plan.total_steps:,} counts · ~{plan.expected_seconds:.1f}s · finishes at home")
            plan = build_distance_plan(float(self.distance_var.get()), float(self.wheel_diameter_var.get()),
                                       float(self.rpm_var.get()), "forward", self.invert_var.get())
            self.distance_plan_var.set(f"{plan.rotations:.4f} turns · {plan.steps:,} counts · {plan.degrees:.2f}°")
        except (ValueError, OverflowError):
            self.plan_var.set("Whole turns: 1–100. Speed: greater than 0, up to 30 RPM.")
            self.distance_plan_var.set("Enter a positive distance and wheel diameter.")

    def _device(self):
        port = self.port_var.get().strip()
        identity = int(self.id_var.get())
        if not port or not 0 <= identity <= 253:
            raise ValueError("Enter a USB port and servo ID from 0 to 253")
        return port, identity

    def _error(self, exc):
        self.status_var.set(str(exc))
        messagebox.showerror("NAVION", str(exc), parent=self)

    def _set_busy(self, busy):
        self.busy = busy
        for widget in self.manual_widgets:
            if isinstance(widget, ttk.Combobox):
                widget.configure(state="disabled" if busy else "readonly")
            else:
                widget.configure(state="disabled" if busy else "normal")

    def _job(self, task, action, source="manual", motion=False):
        if self.busy or self.closing:
            return False
        try:
            port, identity = self._device()
        except ValueError as exc:
            self._error(exc)
            return False
        self.stop_event.clear()
        self.active_source = source
        self._set_busy(True)
        self.progress_var.set(0)
        self.status_var.set(task)
        if source != "camera":
            self.camera_armed.set(False)
        def runner():
            before = after = None
            result = "Complete"
            error = None
            try:
                before = read_motor_snapshot(port, identity)
                self.events.put(("telemetry", before.telemetry))
                if motion and self.stop_event.is_set():
                    result = "Stopped before motion"
                else:
                    value = action(port, identity)
                    if isinstance(value, str):
                        result = value
            except MotionCancelled:
                result = "Stopped by operator"
            except Exception as exc:
                result = f"Failed: {exc}"
                error = str(exc)
            finally:
                # Read after torque disable/mode restoration, including stopped tasks.
                # An unreadable final value is never presented as the current angle.
                try:
                    after = read_motor_snapshot(port, identity)
                except Exception as exc:
                    result += f"; final position unavailable: {exc}"
                self.events.put(("complete", task, before, after, result, error))
        self.worker = threading.Thread(target=runner, daemon=True)
        self.worker.start()
        return True

    def _complete(self, task, before, after, result, error):
        self._set_busy(False)
        was_camera = self.active_source == "camera"
        self.active_source = ""
        self.pointer_active = False
        self._draw_stick()
        if after:
            self._apply_telemetry(after.telemetry)
        else:
            self.metrics["position"].set("Unknown")
            self.metrics["angle"].set("Unknown")
        start = before.telemetry.position_steps if before else "unknown"
        final = after.telemetry.position_steps if after else "unknown"
        angle = f"{final * 360 / STEPS_PER_REVOLUTION:.2f}" if isinstance(final, int) else "unknown"
        record = dict(time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"), task=task,
                      start=start, final=final, angle=angle, result=result,
                      temperature_c=after.telemetry.temperature_c if after else "",
                      voltage_v=after.telemetry.voltage_v if after else "",
                      current_ma=after.telemetry.current_ma if after else "",
                      status=after.telemetry.status_text if after else "")
        self.history_records.append(record)
        self.history_tree.insert("", 0, values=[record[k] for k in ("time", "task", "start", "final", "angle", "result")])
        self.last_position_var.set(f"{task}  ·  start {start} → after {final} counts  ·  {angle}°")
        self.status_var.set(f"{result} · final encoder {final}")
        self.joystick_status.set("Release all keys before the next hold")
        self.camera_ready_after = time.monotonic()
        # Preserve the last camera gesture across its motion, so holding the same
        # thumb does not immediately run the move again after completion.
        if not was_camera:
            self.gesture_gate = GestureCommandGate(stable_frames=4, release_frames=8)
        if error:
            self.camera_armed.set(False)
            self.joystick_enabled.set(False)
        if not self.closing and error:
            self._error(error)

    def _update(self, update):
        self.events.put(("telemetry", update.telemetry))
        self.events.put(("progress", update.progress_percent, update.phase))

    def _apply_telemetry(self, telemetry):
        self.connection_var.set(f"ID {self.id_var.get()} · CONNECTED")
        self.metrics["position"].set(f"{telemetry.position_steps}")
        self.metrics["angle"].set(f"{telemetry.position_deg:.1f}°")
        self.metrics["speed"].set(f"{telemetry.velocity_rpm:.1f} rpm")
        self.metrics["temperature"].set(f"{telemetry.temperature_c} °C")
        self.metrics["voltage"].set(f"{telemetry.voltage_v:.1f} V")
        self.metrics["current"].set(f"{telemetry.current_ma:.0f} mA")
        self.metrics["load"].set(f"{telemetry.load_percent:.1f}%")
        self.metrics["fault"].set(telemetry.status_text)

    def _read_motor(self):
        self._job("Read motor", lambda *_: None)

    def _scan(self):
        if self.busy or self.closing:
            return
        self.camera_armed.set(False)
        self._set_busy(True)
        self.active_source = "scan"
        self.status_var.set("Scanning USB adapters and servo IDs…")
        def runner():
            try:
                scans = scan_connected_servos()
                found = next((s for s in scans if s.servo_ids and s.baudrate == 1_000_000), None)
                if found is None:
                    raise RuntimeError("No servo found at 1,000,000 baud")
                self.events.put(("scan", found))
            except Exception as exc:
                self.events.put(("scan_error", str(exc)))
        self.worker = threading.Thread(target=runner, daemon=True)
        self.worker.start()

    def _set_home(self):
        if messagebox.askyesno("Set home", "Save the current shaft angle as home (2047)?\n"
                               "This changes the motor's persistent homing offset.", parent=self):
            self._job("Set current angle as home", lambda p, i: set_current_as_home(p, i))

    def _start_rotation(self):
        try:
            plan = build_motion_plan(int(self.rotations_var.get()), float(self.rpm_var.get()), self.direction_var.get())
        except ValueError as exc:
            self._error(exc)
            return
        if not messagebox.askyesno("Run rotations",
            f"Run {plan.rotations} {plan.direction.upper()} turns at {plan.rpm:g} RPM?\n"
            "The wheel first moves to home, then finishes at home.", parent=self):
            return
        self._job(f"{plan.rotations} rotations {plan.direction.upper()}",
                  lambda p, i: rotate_whole_turns_and_return_home(p, i, plan.rotations, plan.rpm,
                      plan.direction, self.stop_event, self._update), motion=True)

    def _start_distance(self, direction, round_trip=False, source="manual"):
        try:
            distance = float(self.sensor_distance_var.get() if source == "camera" else self.distance_var.get())
            invert = self.invert_var.get()
            plan = build_distance_plan(distance, float(self.wheel_diameter_var.get()),
                                       float(self.rpm_var.get()), direction, invert)
        except ValueError as exc:
            self.camera_armed.set(False)
            self._error(exc)
            return
        self._job(f"{source.title()} · {direction} {distance:g} cm" + (" + return" if round_trip else ""),
                  lambda p, i: move_distance_and_maybe_return(p, i, plan.distance_cm, plan.wheel_diameter_mm,
                      plan.rpm, direction, self.stop_event, self._update, round_trip, invert),
                  source=source, motion=True)

    def _toggle_joystick(self):
        if self.joystick_enabled.get():
            self.camera_armed.set(False)
            self.stick.focus_set()
            self.joystick_status.set("Ready · hold W / S, or drag the stick")
        else:
            if self.active_source == "joystick":
                self.stop_event.set()
            self.joystick_status.set("Joystick disabled")

    def _start_jog(self, direction):
        if not self.joystick_enabled.get() or self.closing:
            self.joystick_status.set("Enable joystick first")
            return False
        if self.busy:
            return False
        try:
            rpm = float(self.rpm_var.get())
            build_motion_plan(1, rpm, "cw")
            invert = self.invert_var.get()
        except ValueError as exc:
            self._error(exc)
            return False
        self.joystick_status.set(f"{direction.upper()} · release to stop")
        self._draw_stick(direction)
        return self._job(f"Joystick · {direction}", lambda p, i: jog_wheel(
            p, i, rpm, direction, invert, self.stop_event,
            callback=lambda t: self.events.put(("telemetry", t))),
            source="joystick", motion=True)

    def _key_press(self, event):
        key = event.keysym.lower()
        if key in ("space", "escape"):
            self._request_stop()
            return "break"
        if key not in ("w", "s"):
            return
        if key in self.release_timers:
            self.after_cancel(self.release_timers.pop(key))
        focus = self.focus_get()
        if focus and focus.winfo_class() in ("TEntry", "Entry", "TCombobox", "Text", "TSpinbox"):
            return
        if self.notebook.select() != str(self.drive_tab) or not self.joystick_enabled.get():
            return
        command = self.keys.press(key)
        if command == "stop":
            self.stop_event.set()
            self.joystick_status.set("Both keys held — release both")
        elif command:
            self._start_jog(command)
        return "break"

    def _key_release(self, event):
        key = event.keysym.lower()
        if key not in ("w", "s"):
            return
        if key in self.release_timers:
            self.after_cancel(self.release_timers[key])
        # Cancelled by a repeated keypress on systems that synthesize releases.
        self.release_timers[key] = self.after(25, lambda: self._release_key(key))

    def _release_key(self, key):
        self.release_timers.pop(key, None)
        if self.keys.release(key) and self.active_source == "joystick":
            self.stop_event.set()
            self.joystick_status.set("Released — stopping")

    def _draw_stick(self, direction=None):
        if not hasattr(self, "stick"):
            return
        canvas = self.stick
        canvas.delete("all")
        x, y = canvas.winfo_width() / 2, canvas.winfo_height() / 2
        radius = max(45, min(76, y - 24))
        canvas.create_oval(x-radius, y-radius, x+radius, y+radius, fill=BG, outline="#35505f", width=2)
        canvas.create_line(x, y-radius+10, x, y+radius-10, fill="#35505f", width=2)
        canvas.create_text(x+radius+22, y-radius+12, text="W", fill=ACCENT, font=("Menlo", 15, "bold"))
        canvas.create_text(x+radius+22, y+radius-12, text="S", fill=ACCENT, font=("Menlo", 15, "bold"))
        offset = -radius*.55 if direction == "forward" else radius*.55 if direction == "back" else 0
        canvas.create_oval(x-25, y+offset-25, x+25, y+offset+25, fill=ACCENT, outline="")
        canvas.create_text(x, y+offset, text="↑" if direction == "forward" else "↓" if direction == "back" else "•",
                           fill=BG, font=("Helvetica", 20, "bold"))

    def _pointer_press(self, event):
        self.stick.focus_set()
        self.pointer_active = True
        self.pointer_direction = None
        self._pointer_move(event)

    def _pointer_move(self, event):
        if not self.pointer_active:
            return
        delta = event.y - self.stick.winfo_height() / 2
        direction = "forward" if delta < -20 else "back" if delta > 20 else None
        if self.active_source == "joystick" and direction != self.pointer_direction:
            self.stop_event.set()
            self.pointer_active = False
            self.joystick_status.set("Direction changed · release and press again")
            self._draw_stick()
            return
        if direction and not self.busy:
            if self._start_jog(direction):
                self.pointer_direction = direction
                self._draw_stick(direction)
        elif direction is None and self.active_source == "joystick":
            self.stop_event.set()
            self._draw_stick()
        # Reversing requires returning to neutral and starting a new hold.

    def _pointer_release(self, _event=None):
        if self.pointer_active:
            self.pointer_active = False
            if self.active_source == "joystick":
                self.stop_event.set()
            self._draw_stick()

    def _check_focus(self):
        focus = self.focus_get()
        editing = focus and focus.winfo_class() in ("TEntry", "Entry", "TCombobox", "Text", "TSpinbox")
        if focus is None or editing:
            if self.active_source == "joystick":
                self.stop_event.set()
            if focus is None:
                self.camera_armed.set(False)
            self.keys.held.clear()
            self.pointer_active = False
            self._draw_stick()

    def _tab_changed(self, _event=None):
        if self.notebook.select() != str(self.drive_tab):
            self.joystick_enabled.set(False)
            if self.active_source == "joystick":
                self.stop_event.set()

    def _request_stop(self):
        self.stop_event.set()
        self.camera_armed.set(False)
        self.joystick_enabled.set(False)
        self.status_var.set("STOP requested" if self.busy else "Stopped · controls disarmed")
        self.joystick_status.set("Stopped · enable joystick to resume")
        self._draw_stick()

    def _start_camera(self):
        if self.camera is not None or self.closing:
            return
        try:
            index = int(self.camera_index_var.get())
            confidence = float(self.camera_confidence_var.get())
            if index < 0 or not 0.3 <= confidence <= 1:
                raise ValueError("Camera index must be >= 0; confidence must be 0.30–1.00")
            self.camera = CameraSession(index, confidence)
            self.camera.start()
        except Exception as exc:
            self.camera = None
            self._error(exc)
            return
        self.camera_started_at = time.monotonic()
        self.last_camera_frame = 0
        self.camera_armed.set(False)
        self.start_camera_button.configure(state="disabled")
        self.camera_status.set("Starting preview… allow Camera access for Terminal if macOS asks.")

    def _stop_camera(self):
        self.camera_armed.set(False)
        if self.active_source == "camera":
            self.stop_event.set()
        if self.camera:
            self.camera.stop()
            self.camera_status.set("Stopping camera…")

    def _toggle_camera_arm(self):
        if self.camera_armed.get():
            if not self.camera or time.monotonic() - self.last_camera_frame > 1 or self.busy:
                self.camera_armed.set(False)
                self.camera_status.set("Start a live preview and finish the current motor task first.")
                return
            try:
                build_distance_plan(float(self.sensor_distance_var.get()), float(self.wheel_diameter_var.get()),
                                    float(self.rpm_var.get()), "forward")
            except ValueError as exc:
                self.camera_armed.set(False)
                self._error(exc)
                return
            self.joystick_enabled.set(False)
            self.camera_ready_after = time.monotonic()
            self.gesture_gate = GestureCommandGate(stable_frames=4, release_frames=8)
            self.camera_status.set("Armed · hold thumb up/down to move once; open palm to stop.")
        elif self.active_source == "camera":
            self.stop_event.set()

    def _camera_frame(self, update):
        self.last_camera_frame = time.monotonic()
        self.last_camera_image = update.frame_rgb
        self._render_camera()
        self.gesture_var.set(f"{update.gesture.replace('_', ' ')} · {update.confidence:.0%}")
        self.confidence_var.set(update.confidence * 100)
        self.camera_arm_button.configure(state="normal")
        fresh = self.last_camera_frame - update.captured_at < 0.6
        try:
            threshold = float(self.camera_confidence_var.get())
            if not math.isfinite(threshold) or not 0.3 <= threshold <= 1:
                raise ValueError
        except ValueError:
            self.camera_armed.set(False)
            self.camera_status.set("Enter a confidence threshold from 0.30 to 1.00.")
            return
        accepted = update.gesture if update.hand_count == 1 and update.confidence >= threshold and fresh else "None"
        if accepted == "Open_Palm":
            # Stop is independent of arming and cannot be swallowed by debouncing.
            if self.camera_armed.get() or self.busy:
                self._request_stop()
            self.camera_status.set("Open palm detected · stopped and disarmed")
            return
        if self.busy:
            self.camera_status.set("Motor task active · new thumb commands are ignored")
            return
        if not self.camera_armed.get():
            self.camera_status.set(f"PREVIEW ONLY · {update.state} · {update.fps:.0f} FPS")
            return
        if not fresh or update.captured_at <= self.camera_ready_after:
            self.camera_status.set("Waiting for a fresh camera frame")
            return
        command, state = self.gesture_gate.update(accepted)
        self.camera_status.set(f"ARMED · {state}")
        if command in ("forward", "back"):
            self._start_distance(command, source="camera")

    def _render_camera(self):
        if self.last_camera_image is None or not hasattr(self, "camera_preview"):
            return
        from PIL import Image, ImageTk
        canvas = self.camera_preview
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        image = Image.fromarray(self.last_camera_image)
        image.thumbnail((width, height))
        self.camera_photo = ImageTk.PhotoImage(image)
        canvas.delete("all")
        canvas.create_image(width/2, height/2, image=self.camera_photo, anchor="center")

    def _poll_camera(self):
        if not self.camera:
            return
        session = self.camera
        update = session.latest()
        if update and not session.stop_event.is_set():
            self._camera_frame(update)
        error = session.error()
        dead = not session.process.is_alive()
        stop_elapsed = time.monotonic() - session.stop_requested_at if session.stop_requested_at else 0
        age = time.monotonic() - (self.last_camera_frame or self.camera_started_at)
        if self.camera_armed.get() and age > 1.5:
            self.camera_armed.set(False)
            if self.active_source == "camera":
                self.stop_event.set()
            self.camera_status.set("Camera stalled · movement disarmed")
        if age > 30 and not session.stop_event.is_set():
            error = "Camera startup/frame timeout. Check permissions and the diagnostic log."
            session.stop()
        if error or dead or stop_elapsed > 2:
            exitcode = session.process.exitcode
            expected = session.stop_event.is_set()
            if self.active_source == "camera":
                self.stop_event.set()
            session.close()
            self.camera = None
            self.camera_armed.set(False)
            self.camera_arm_button.configure(state="disabled")
            self.start_camera_button.configure(state="normal")
            self.camera_status.set(error or ("Camera stopped" if expected else
                f"Camera process exited ({exitcode}). Open diagnostic log; the motor console is still running."))
            self.last_camera_image = None
            self.camera_preview.delete("all")
            self.camera_preview.create_text(180, 100, text="Camera offline", fill=MUTED)

    def _show_camera_log(self):
        path = Path(__file__).resolve().parents[1] / ".navion" / "camera.log"
        window = tk.Toplevel(self)
        window.title("Camera diagnostic log")
        window.geometry("820x430")
        text = tk.Text(window, bg=BG, fg=INK, wrap="word", font=("Menlo", 10))
        text.pack(fill="both", expand=True)
        try:
            content = path.read_text(errors="replace")[-20000:]
        except OSError:
            content = "No diagnostic log yet. Start the camera preview first."
        text.insert("1.0", content)
        text.configure(state="disabled")

    def _export_history(self):
        if not self.history_records:
            self.status_var.set("No task history to export yet")
            return
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv",
            initialfile=f"navion-history-{datetime.now():%Y%m%d-%H%M%S}.csv", filetypes=[("CSV", "*.csv")])
        if path:
            try:
                with open(path, "w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(self.history_records[0]))
                    writer.writeheader()
                    writer.writerows(self.history_records)
                self.status_var.set(f"History exported to {path}")
            except OSError as exc:
                self._error(exc)

    def _poll(self):
        try:
            for _ in range(120):
                event = self.events.get_nowait()
                if event[0] == "telemetry":
                    self._apply_telemetry(event[1])
                elif event[0] == "progress":
                    self.progress_var.set(event[1])
                    self.status_var.set(event[2])
                elif event[0] == "complete":
                    self._complete(*event[1:])
                elif event[0] == "scan":
                    self._set_busy(False)
                    self.active_source = ""
                    self.port_var.set(event[1].port)
                    self.id_var.set(str(event[1].servo_ids[0]))
                    self.status_var.set(f"Detected ID {event[1].servo_ids[0]} at {event[1].baudrate:,} baud")
                elif event[0] == "scan_error":
                    self._set_busy(False)
                    self.active_source = ""
                    if not self.closing:
                        self._error(event[1])
        except queue.Empty:
            pass
        self._poll_camera()
        if self.closing and not self.busy and self.camera is None:
            self.destroy()
            return
        self.after(40, self._poll)

    def _on_close(self):
        self.closing = True
        self._request_stop()
        self._stop_camera()
        self.status_var.set("Stopping motor and releasing camera…")


def main():
    parser = argparse.ArgumentParser(description="NAVION motor / joystick / camera console")
    parser.add_argument("--ui-check", action="store_true", help="Open layout briefly without connecting devices")
    args = parser.parse_args()
    app = MotorControlApp()
    if args.ui_check:
        app.after(1500, app._on_close)
    app.mainloop()


if __name__ == "__main__":
    main()
