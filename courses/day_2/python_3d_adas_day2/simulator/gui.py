"""Tkinter application for the Day 2 lateral-control simulator."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .controllers import ManualController, ReferenceController
from .model import ControlCommand, clamp
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
        self.root.title("Python 3D ADAS simulator — Day 2 lateral control")
        self.root.geometry("1280x780")
        self.root.minsize(900, 600)
        self.running = True
        self.key_state: set[str] = set()
        self.controller_name = tk.StringVar(value="Reference")
        self.target_speed = tk.DoubleVar(value=12.0)
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

        ttk.Label(toolbar, text="Target speed:").pack(side=tk.LEFT)
        slider = ttk.Scale(
            toolbar,
            from_=5.0,
            to=25.0,
            variable=self.target_speed,
            command=self._target_changed,
            length=180,
        )
        slider.pack(side=tk.LEFT, padx=(5, 5))
        self.target_label = ttk.Label(toolbar, text="12.0 m/s", width=9)
        self.target_label.pack(side=tk.LEFT, padx=(0, 12))

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
        return ControlCommand(throttle, brake, steering)

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
        builder = SceneBuilder(width, height)
        primitives = builder.build(
            self.simulation.track,
            self.simulation.vehicle.state,
            slope,
        )
        for primitive in primitives:
            self._draw_primitive(primitive)

        self._draw_dashboard(width, height)
        self._draw_minimap(width)
        self._draw_lateral_plot(width, height)

    def _draw_primitive(self, primitive: Primitive) -> None:
        coordinates = [
            coordinate
            for point in primitive.points
            for coordinate in point
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

    def _draw_dashboard(self, width: int, height: int) -> None:
        del width
        observation = self.simulation.last_observation
        if observation is None:
            return
        command = self.simulation.last_command
        panel_x = 18
        panel_y = 18
        panel_w = 270
        panel_h = 280
        self.canvas.create_rectangle(
            panel_x,
            panel_y,
            panel_x + panel_w,
            panel_y + panel_h,
            fill="#10202c",
            outline="#b4cad7",
            width=1,
        )
        rows = [
            ("SPEED", f"{observation.speed:5.1f} m/s"),
            ("TARGET", f"{observation.target_speed:5.1f} m/s"),
            ("ACCELERATION", f"{observation.acceleration:+5.2f} m/s²"),
            ("ROAD SLOPE", f"{math.degrees(observation.slope_radians):+4.1f}°"),
            ("ROAD GRADE", f"{observation.grade_percent:+5.2f}%"),
            ("LANE ERROR", f"{observation.cross_track_error:+5.2f} m"),
            ("HEADING ERROR", f"{math.degrees(observation.heading_error):+5.1f}°"),
            ("LOOK-AHEAD", f"{observation.preview_distance:5.1f} m"),
            ("THROTTLE", f"{command.throttle:4.2f}"),
            ("BRAKE", f"{command.brake:4.2f}"),
            ("STEERING", f"{math.degrees(command.steering):+5.1f}°"),
        ]
        self.canvas.create_text(
            panel_x + 12,
            panel_y + 12,
            anchor="nw",
            text=f"DAY 2 LATERAL LAB  •  {self.controller_name.get().upper()}",
            fill="#7fd1ff",
            font=("TkDefaultFont", 11, "bold"),
        )
        for index, (label, value) in enumerate(rows):
            y = panel_y + 40 + index * 19
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

        if observation.off_road:
            self.canvas.create_text(
                0.5 * self.canvas.winfo_width(),
                40,
                text="OFF ROAD — TRACTION REDUCED",
                fill="#ffdf5d",
                font=("TkDefaultFont", 16, "bold"),
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
        map_w = 250
        map_h = 155
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
        state = self.simulation.vehicle.state
        car_x, car_y = map_point(state.x, state.y)
        observation = self.simulation.last_observation
        if observation is not None:
            target_x, target_y = map_point(
                observation.preview_x,
                observation.preview_y,
            )
            self.canvas.create_line(
                car_x,
                car_y,
                target_x,
                target_y,
                fill="#73e6ff",
                width=1,
                dash=(3, 2),
            )
            self.canvas.create_oval(
                target_x - 4,
                target_y - 4,
                target_x + 4,
                target_y + 4,
                fill="#73e6ff",
                outline="#ffffff",
            )
        self.canvas.create_oval(
            car_x - 5,
            car_y - 5,
            car_x + 5,
            car_y + 5,
            fill="#42a5ff",
            outline="#ffffff",
        )
        self.canvas.create_text(
            x0 + 10,
            y0 + 8,
            anchor="nw",
            text=f"CIRCUIT  •  LAP {self.simulation.lap}",
            fill="#ffffff",
            font=("TkDefaultFont", 9, "bold"),
        )

    def _draw_lateral_plot(self, width: int, height: int) -> None:
        plot_w = 360
        plot_h = 118
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
        history = self.simulation.history[-360:]
        if len(history) < 2:
            return
        max_error = max(
            1.0,
            max(
                abs(float(row["cross_track_error_m"]))
                for row in history
            )
            * 1.15,
        )

        def point(index: int, error: float) -> tuple[float, float]:
            return (
                x0 + 8 + index / max(1, len(history) - 1) * (plot_w - 16),
                y0 + 0.5 * plot_h
                - error / max_error * 0.38 * plot_h,
            )

        error_points = [
            coordinate
            for index, row in enumerate(history)
            for coordinate in point(
                index,
                float(row["cross_track_error_m"]),
            )
        ]
        self.canvas.create_line(
            x0 + 8,
            y0 + 0.5 * plot_h,
            x0 + plot_w - 8,
            y0 + 0.5 * plot_h,
            fill="#f7c948",
            width=1,
            dash=(5, 3),
        )
        self.canvas.create_line(*error_points, fill="#55b7ff", width=2)
        self.canvas.create_text(
            x0 + 10,
            y0 + 8,
            anchor="nw",
            text="LATERAL ERROR HISTORY  — centreline  — error",
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
            title="Save experiment log",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
            initialfile="adas_experiment.csv",
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
