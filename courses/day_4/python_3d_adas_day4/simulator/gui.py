"""Tkinter application for the cumulative Day 4 robust ADAS simulator."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .controllers import ManualController, ReferenceController
from .faults import SCENARIOS
from .model import ControlCommand
from .renderer3d import Primitive, SceneBuilder
from .simulation import Simulation


def load_student_controller():
    import student_controller

    importlib.invalidate_caches()
    module = importlib.reload(student_controller)
    return module.StudentController()


class SimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Python 3D ADAS simulator — Day 4 robustness")
        self.root.geometry("1320x800")
        self.root.minsize(980, 650)
        self.running = True
        self.key_state: set[str] = set()
        self.controller_name = tk.StringVar(value="Reference")
        self.scenario_name = tk.StringVar(value="nominal")
        self.target_speed = tk.DoubleVar(value=14.0)
        self.status_text = tk.StringVar(value="Ready")
        self.simulation = Simulation(ReferenceController())

        self._build_toolbar()
        self.canvas = tk.Canvas(
            root,
            background="#78a9d1",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)
        self.root.after(20, self._tick)

    def _build_toolbar(self) -> None:
        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(fill=tk.X)
        self.run_button = ttk.Button(
            toolbar,
            text="Pause",
            command=self.toggle_running,
            width=10,
        )
        self.run_button.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(
            toolbar,
            text="Reset",
            command=self.reset,
            width=10,
        ).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Label(toolbar, text="Controller:").pack(side=tk.LEFT)
        selector = ttk.Combobox(
            toolbar,
            textvariable=self.controller_name,
            values=("Reference", "Student", "Manual"),
            state="readonly",
            width=12,
        )
        selector.pack(side=tk.LEFT, padx=(5, 14))
        selector.bind("<<ComboboxSelected>>", lambda _event: self.reset())

        ttk.Label(toolbar, text="Cruise speed:").pack(side=tk.LEFT)
        slider = ttk.Scale(
            toolbar,
            from_=8.0,
            to=20.0,
            variable=self.target_speed,
            command=self._target_changed,
            length=180,
        )
        slider.pack(side=tk.LEFT, padx=(5, 5))
        self.target_label = ttk.Label(toolbar, text="14.0 m/s", width=9)
        self.target_label.pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(toolbar, text="Scenario:").pack(side=tk.LEFT)
        scenario_selector = ttk.Combobox(
            toolbar,
            textvariable=self.scenario_name,
            values=tuple(SCENARIOS),
            state="readonly",
            width=15,
        )
        scenario_selector.pack(side=tk.LEFT, padx=(5, 12))
        scenario_selector.bind(
            "<<ComboboxSelected>>",
            lambda _event: self.reset(),
        )

        ttk.Button(
            toolbar,
            text="Save CSV",
            command=self.save_csv,
            width=11,
        ).pack(side=tk.LEFT)
        ttk.Label(
            toolbar,
            textvariable=self.status_text,
            foreground="#555555",
        ).pack(side=tk.RIGHT)

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._key_down)
        self.root.bind("<KeyRelease>", self._key_up)

    def _key_down(self, event: tk.Event) -> None:
        key = str(event.keysym).lower()
        self.key_state.add(key)
        if key == "space":
            self.toggle_running()
        elif key == "r":
            self.reset()

    def _key_up(self, event: tk.Event) -> None:
        self.key_state.discard(str(event.keysym).lower())

    def _target_changed(self, _value: str = "") -> None:
        value = round(float(self.target_speed.get()), 1)
        self.simulation.target_speed = value
        self.target_label.configure(text=f"{value:.1f} m/s")

    def toggle_running(self) -> None:
        self.running = not self.running
        self.run_button.configure(text="Pause" if self.running else "Run")
        self.status_text.set("Running" if self.running else "Paused")

    def _make_controller(self):
        name = self.controller_name.get()
        if name == "Student":
            return load_student_controller()
        if name == "Manual":
            return ManualController()
        return ReferenceController()

    def reset(self) -> None:
        try:
            self.simulation = Simulation(
                self._make_controller(),
                target_speed=float(self.target_speed.get()),
                scenario=self.scenario_name.get(),
            )
            self.status_text.set(
                f"{self.controller_name.get()} controller loaded"
            )
        except Exception as error:
            messagebox.showerror(
                "Controller error",
                f"The controller could not be loaded:\n\n{error}",
            )
            self.controller_name.set("Reference")
            self.simulation = Simulation(ReferenceController())

    def _manual_command(self) -> ControlCommand:
        throttle = 0.72 if "up" in self.key_state else 0.0
        brake = 0.75 if "down" in self.key_state else 0.0
        steering = 0.0
        if "left" in self.key_state:
            steering += math.radians(18.0)
        if "right" in self.key_state:
            steering -= math.radians(18.0)
        return ControlCommand(
            throttle,
            brake,
            steering,
            selected_target_speed=float(self.target_speed.get()),
            mode="MANUAL",
        )

    def _tick(self) -> None:
        if not self.root.winfo_exists():
            return
        if self.running:
            command = (
                self._manual_command()
                if self.controller_name.get() == "Manual"
                else None
            )
            for _ in range(2):
                self.simulation.step(1.0 / 60.0, command)
        self._draw()
        self.root.after(33, self._tick)

    def _draw(self) -> None:
        canvas = self.canvas
        width = max(320, canvas.winfo_width())
        height = max(240, canvas.winfo_height())
        canvas.delete("all")

        horizon = int(0.54 * height)
        canvas.create_rectangle(0, 0, width, horizon, fill="#78a9d1", outline="")
        canvas.create_rectangle(0, horizon, width, height, fill="#5f8953", outline="")
        canvas.create_oval(
            width - 115,
            38,
            width - 55,
            98,
            fill="#ffe58b",
            outline="",
        )

        observation = self.simulation.last_observation
        slope = observation.slope_radians if observation else 0.0
        lead_pose = self.simulation.track.pose_at(
            self.simulation.lead_progress,
            self.simulation.track.lane_offset,
        )
        primitives = SceneBuilder(width, height).build(
            self.simulation.track,
            self.simulation.vehicle.state,
            slope,
            self.simulation.lead_vehicle,
            lead_pose.slope,
        )
        for primitive in primitives:
            self._draw_primitive(primitive)

        self._draw_dashboard(height)
        self._draw_minimap(width)
        self._draw_gap_plot(width, height)

    def _draw_primitive(self, primitive: Primitive) -> None:
        coordinates = [
            coordinate for point in primitive.points for coordinate in point
        ]
        if primitive.kind == "line":
            self.canvas.create_line(
                *coordinates,
                fill=primitive.fill,
                width=primitive.width,
            )
        else:
            self.canvas.create_polygon(
                *coordinates,
                fill=primitive.fill,
                outline=primitive.outline,
                width=primitive.width,
            )

    def _draw_dashboard(self, height: int) -> None:
        observation = self.simulation.last_observation
        if observation is None:
            return
        command = self.simulation.last_command
        selected = (
            command.selected_target_speed
            if command.selected_target_speed is not None
            else observation.target_speed
        )
        desired = command.desired_gap or 0.0
        ttc_text = (
            f"{observation.time_to_collision:5.2f} s"
            if math.isfinite(observation.time_to_collision)
            else "   inf"
        )
        gap_text = (
            f"{observation.lead_distance:5.1f} m"
            if observation.lead_detected
            else "not detected"
        )
        panel_x = 18
        panel_y = 18
        panel_w = 318
        panel_h = 398
        self.canvas.create_rectangle(
            panel_x,
            panel_y,
            panel_x + panel_w,
            panel_y + panel_h,
            fill="#10202c",
            outline="#b4cad7",
        )
        rows = [
            ("EGO SPEED", f"{observation.speed:5.1f} m/s"),
            ("CRUISE SETTING", f"{observation.target_speed:5.1f} m/s"),
            ("SELECTED TARGET", f"{selected:5.1f} m/s"),
            ("LEAD SPEED", f"{observation.lead_speed:5.1f} m/s"),
            ("RANGE / GAP", gap_text),
            ("DESIRED GAP", f"{desired:5.1f} m"),
            ("CLOSING SPEED", f"{observation.closing_speed:+5.1f} m/s"),
            ("TTC", ttc_text),
            ("SUPERVISOR MODE", command.mode),
            (
                "RANGE SENSOR",
                (
                    "HEALTHY"
                    if observation.range_sensor_healthy
                    else f"FAILED ({observation.range_measurement_age:.1f} s)"
                ),
            ),
            ("ACTIVE FAULT", observation.active_fault),
            ("ACCELERATION", f"{observation.acceleration:+5.2f} m/s²"),
            ("LANE ERROR", f"{observation.cross_track_error:+5.2f} m"),
            ("LOOK-AHEAD", f"{observation.preview_distance:5.1f} m"),
            ("THROTTLE", f"{command.throttle:4.2f}"),
            ("BRAKE", f"{command.brake:4.2f}"),
            ("STEERING", f"{math.degrees(command.steering):+5.1f}°"),
        ]
        mode_colors = {
            "CRUISE": "#7fd1ff",
            "FOLLOW": "#86efac",
            "BRAKE": "#facc15",
            "EMERGENCY": "#ff6b6b",
            "SAFE_STOP": "#ff9f43",
            "MANUAL": "#d8b4fe",
        }
        self.canvas.create_text(
            panel_x + 12,
            panel_y + 12,
            anchor="nw",
            text=f"DAY 4 ROBUSTNESS LAB  •  {self.controller_name.get().upper()}",
            fill=mode_colors.get(command.mode, "#7fd1ff"),
            font=("TkDefaultFont", 11, "bold"),
        )
        for index, (label, value) in enumerate(rows):
            y = panel_y + 40 + index * 20
            self.canvas.create_text(
                panel_x + 12,
                y,
                anchor="nw",
                text=label,
                fill="#a9bdc9",
                font=("TkFixedFont", 9),
            )
            self.canvas.create_text(
                panel_x + panel_w - 12,
                y,
                anchor="ne",
                text=value,
                fill="#ffffff",
                font=("TkFixedFont", 10, "bold"),
            )

        if self.simulation.true_gap() <= 0.0:
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                42,
                text="COLLISION — RESET THE EXPERIMENT",
                fill="#ff4f4f",
                font=("TkDefaultFont", 17, "bold"),
            )
        elif command.mode == "EMERGENCY":
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                42,
                text="EMERGENCY BRAKING",
                fill="#ffdf5d",
                font=("TkDefaultFont", 17, "bold"),
            )
        elif command.mode == "SAFE_STOP":
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                42,
                text="SENSOR FAULT — CONTROLLED SAFE STOP",
                fill="#ffdf5d",
                font=("TkDefaultFont", 17, "bold"),
            )
        if observation.off_road:
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                72,
                text="OFF ROAD — TRACTION REDUCED",
                fill="#ffdf5d",
                font=("TkDefaultFont", 15, "bold"),
            )
        if self.controller_name.get() == "Manual":
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                height - 20,
                text="Arrow keys: throttle • brake • steering",
                fill="#ffffff",
                font=("TkDefaultFont", 10, "bold"),
            )

    def _draw_minimap(self, width: int) -> None:
        map_w = 260
        map_h = 160
        x0 = width - map_w - 18
        y0 = 18
        self.canvas.create_rectangle(
            x0,
            y0,
            x0 + map_w,
            y0 + map_h,
            fill="#173124",
            outline="#b4cad7",
        )
        track = self.simulation.track
        xs = [pose.x for pose in track.samples]
        ys = [pose.y for pose in track.samples]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        scale_value = min(
            (map_w - 24) / (max_x - min_x),
            (map_h - 28) / (max_y - min_y),
        )

        def map_point(x: float, y: float) -> tuple[float, float]:
            return (
                x0 + 12 + (x - min_x) * scale_value,
                y0 + map_h - 12 - (y - min_y) * scale_value,
            )

        points = [
            coordinate
            for pose in track.samples + [track.samples[0]]
            for coordinate in map_point(pose.x, pose.y)
        ]
        self.canvas.create_line(*points, fill="#a7adb0", width=8, smooth=True)
        self.canvas.create_line(*points, fill="#33383c", width=5, smooth=True)

        ego = self.simulation.vehicle.state
        lead = self.simulation.lead_vehicle
        ego_x, ego_y = map_point(ego.x, ego.y)
        lead_x, lead_y = map_point(lead.x, lead.y)
        observation = self.simulation.last_observation
        if observation is not None:
            target_x, target_y = map_point(
                observation.preview_x,
                observation.preview_y,
            )
            self.canvas.create_line(
                ego_x,
                ego_y,
                target_x,
                target_y,
                fill="#73e6ff",
                width=1,
                dash=(3, 2),
            )
            if observation.lead_detected:
                self.canvas.create_line(
                    ego_x,
                    ego_y,
                    lead_x,
                    lead_y,
                    fill="#ffd166",
                    width=2,
                    dash=(5, 3),
                )
        self.canvas.create_oval(
            lead_x - 5,
            lead_y - 5,
            lead_x + 5,
            lead_y + 5,
            fill="#ef5350",
            outline="#ffffff",
        )
        self.canvas.create_oval(
            ego_x - 5,
            ego_y - 5,
            ego_x + 5,
            ego_y + 5,
            fill="#42a5ff",
            outline="#ffffff",
        )
        self.canvas.create_text(
            x0 + 10,
            y0 + 8,
            anchor="nw",
            text=f"BLUE: EGO  •  RED: LEAD  •  LAP {self.simulation.lap}",
            fill="#ffffff",
            font=("TkDefaultFont", 8, "bold"),
        )

    def _draw_gap_plot(self, width: int, height: int) -> None:
        plot_w = 390
        plot_h = 135
        x0 = width - plot_w - 18
        y0 = height - plot_h - 18
        self.canvas.create_rectangle(
            x0,
            y0,
            x0 + plot_w,
            y0 + plot_h,
            fill="#101820",
            outline="#b4cad7",
        )
        history = self.simulation.history[-480:]
        if len(history) < 2:
            return
        max_gap = max(
            20.0,
            max(
                max(
                    float(row["gap_m"]),
                    float(row["desired_gap_m"]),
                )
                for row in history
            )
            * 1.08,
        )

        def point(index: int, value: float) -> tuple[float, float]:
            return (
                x0 + 8 + index / max(1, len(history) - 1) * (plot_w - 16),
                y0 + plot_h - 10 - value / max_gap * (plot_h - 30),
            )

        gap_points = [
            coordinate
            for index, row in enumerate(history)
            for coordinate in point(index, float(row["gap_m"]))
        ]
        desired_points = [
            coordinate
            for index, row in enumerate(history)
            for coordinate in point(index, float(row["desired_gap_m"]))
        ]
        self.canvas.create_line(*gap_points, fill="#ffd166", width=2)
        self.canvas.create_line(
            *desired_points,
            fill="#86efac",
            width=2,
            dash=(5, 3),
        )
        self.canvas.create_text(
            x0 + 10,
            y0 + 8,
            anchor="nw",
            text="FOLLOWING GAP  — actual  --- desired",
            fill="#ffffff",
            font=("TkDefaultFont", 9, "bold"),
        )

    def save_csv(self) -> None:
        if not self.simulation.history:
            messagebox.showinfo(
                "Save CSV",
                "Run the simulation before saving a result.",
            )
            return
        path = filedialog.asksaveasfilename(
            title="Save Day 4 experiment log",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
            initialfile=f"day4_{self.scenario_name.get()}_experiment.csv",
        )
        if not path:
            return
        try:
            output = self.simulation.save_csv(Path(path))
            self.status_text.set(f"Saved {output.name}")
        except Exception as error:
            messagebox.showerror("Save error", str(error))


def launch() -> None:
    root = tk.Tk()
    SimulatorApp(root)
    root.mainloop()
