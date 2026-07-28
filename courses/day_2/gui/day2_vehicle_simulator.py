"""Interactive PyQt6 laboratory for Day 2 steering and path following.

Run from the package root:

    python day_2/gui/day2_vehicle_simulator.py

The GUI uses exactly the same bicycle model and Pure Pursuit functions as the
headless demonstrations and student exercises.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.bicycle import (  # noqa: E402
    VehicleParameters,
    VehicleState,
    bicycle_step,
    normalize_angle,
)
from simulator.metrics import calculate_path_metrics  # noqa: E402
from simulator.paths import (  # noqa: E402
    ReferencePath,
    make_reference_path,
    offset_from_path,
)
from simulator.tracking import (  # noqa: E402
    apply_lateral_offset,
    pure_pursuit,
    tracking_errors,
)

try:
    from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
    from PyQt6.QtGui import (
        QBrush,
        QColor,
        QFont,
        QPainter,
        QPainterPath,
        QPen,
        QPolygonF,
    )
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSizePolicy,
        QVBoxLayout,
        QWidget,
    )
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local GUI
    raise SystemExit(
        "PyQt6 is not installed. Run: python -m pip install -r requirements.txt"
    ) from exc


BLUE = QColor("#2476d8")
RED = QColor("#e05252")
GREEN = QColor("#2a9d6f")
ORANGE = QColor("#f59e0b")
INK = QColor("#17233c")
SLATE = QColor("#64748b")
PANEL = QColor("#ffffff")
BACKGROUND = QColor("#eef4f8")
ROAD = QColor("#586472")


@dataclass
class GuiSettings:
    path_kind: str = "training"
    controller: str = "Pure Pursuit — fixed"
    speed: float = 9.0
    wheelbase: float = 2.7
    constant_steering_degrees: float = 12.0
    base_lookahead: float = 5.0
    speed_gain: float = 0.0
    initial_offset: float = 1.5
    initial_heading_degrees: float = 4.0
    road_half_width: float = 3.5
    measurement_noise: float = 0.0
    dt: float = 0.05


class SimulationEngine:
    """GUI-facing wrapper around the common numerical model."""

    def __init__(self) -> None:
        self.settings = GuiSettings()
        self.path = make_reference_path(self.settings.path_kind)
        self.parameters = VehicleParameters(
            wheelbase=self.settings.wheelbase
        )
        self.random = np.random.default_rng(7)
        self.state = VehicleState()
        self.time = 0.0
        self.previous_index = 0
        self.target_index = 0
        self.target_x = float(self.path.x[0])
        self.target_y = float(self.path.y[0])
        self.current_cross_track_error = 0.0
        self.current_heading_error = 0.0
        self.current_command = 0.0
        self.current_steering_rate = 0.0
        self.current_lateral_acceleration = 0.0
        self.current_lookahead = self.settings.base_lookahead
        self.completed = False
        self.disturbance_count = 0
        self.history: dict[str, list[float]] = {}
        self.reset()

    def configure(self, settings: GuiSettings) -> None:
        path_changed = settings.path_kind != self.settings.path_kind
        self.settings = settings
        if path_changed:
            self.path = make_reference_path(settings.path_kind)
        self.parameters = VehicleParameters(wheelbase=settings.wheelbase)
        self.reset()

    def reset(self) -> None:
        x, y, heading = offset_from_path(
            self.path,
            distance=0.0,
            lateral_offset=self.settings.initial_offset,
            heading_offset_degrees=self.settings.initial_heading_degrees,
        )
        self.state = VehicleState(
            x=x,
            y=y,
            heading=heading,
            speed=self.settings.speed,
        )
        self.time = 0.0
        self.previous_index = 0
        self.target_index = 0
        self.target_x = float(self.path.x[0])
        self.target_y = float(self.path.y[0])
        self.current_cross_track_error = self.settings.initial_offset
        self.current_heading_error = math.radians(
            self.settings.initial_heading_degrees
        )
        self.current_command = 0.0
        self.current_steering_rate = 0.0
        self.current_lateral_acceleration = 0.0
        self.current_lookahead = self.settings.base_lookahead
        self.completed = False
        self.disturbance_count = 0
        self.history = {
            key: []
            for key in (
                "time",
                "x",
                "y",
                "heading",
                "speed",
                "steering",
                "steering_rate",
                "cross_track",
                "heading_error",
                "lateral_acceleration",
                "nearest_index",
            )
        }

    @property
    def completion_percent(self) -> float:
        return (
            100.0
            * self.path.distance[self.previous_index]
            / self.path.length
        )

    def inject_disturbance(self, magnitude: float = 2.0) -> None:
        apply_lateral_offset(self.state, magnitude)
        self.disturbance_count += 1

    def _measured_state(self) -> VehicleState:
        measured = self.state.copy()
        noise = self.settings.measurement_noise
        if noise > 0.0:
            measured.x += self.random.normal(0.0, noise)
            measured.y += self.random.normal(0.0, noise)
        return measured

    def _controller_output(
        self,
        measured: VehicleState,
    ) -> tuple[float, int, int, float, float, float, float, float]:
        mode = self.settings.controller
        errors = tracking_errors(
            measured,
            self.path,
            previous_index=self.previous_index,
        )
        if mode == "Constant steering":
            command = math.radians(
                self.settings.constant_steering_degrees
            )
            nearest = errors.nearest_index
            target = nearest
            target_x = float(self.path.x[target])
            target_y = float(self.path.y[target])
            lookahead = 0.0
        elif mode == "Nearest-point P":
            command = (
                -0.32 * errors.cross_track_error
                - 0.85 * errors.heading_error
            )
            nearest = errors.nearest_index
            target = nearest
            target_x = float(self.path.x[target])
            target_y = float(self.path.y[target])
            lookahead = 0.0
        else:
            gain = (
                self.settings.speed_gain
                if mode == "Pure Pursuit — adaptive"
                else 0.0
            )
            output = pure_pursuit(
                measured,
                self.path,
                vehicle=self.parameters,
                base_lookahead=self.settings.base_lookahead,
                speed_lookahead_gain=gain,
                previous_index=self.previous_index,
            )
            return (
                output.steering,
                output.nearest_index,
                output.target_index,
                output.target_x,
                output.target_y,
                output.lookahead_distance,
                output.cross_track_error,
                output.heading_error,
            )
        return (
            command,
            nearest,
            target,
            target_x,
            target_y,
            lookahead,
            errors.cross_track_error,
            errors.heading_error,
        )

    def step(self) -> None:
        if self.completed:
            return
        measured = self._measured_state()
        (
            command,
            nearest,
            target,
            target_x,
            target_y,
            lookahead,
            cross_track,
            heading_error,
        ) = self._controller_output(measured)
        sample = bicycle_step(
            self.state,
            command,
            parameters=self.parameters,
            dt=self.settings.dt,
            speed=self.settings.speed,
            enable_rate_limit=True,
        )
        self.previous_index = nearest
        self.target_index = target
        self.target_x = target_x
        self.target_y = target_y
        self.current_cross_track_error = cross_track
        self.current_heading_error = heading_error
        self.current_command = command
        self.current_steering_rate = sample.steering_rate
        self.current_lateral_acceleration = sample.lateral_acceleration
        self.current_lookahead = lookahead
        self.time += self.settings.dt

        values = {
            "time": self.time,
            "x": self.state.x,
            "y": self.state.y,
            "heading": self.state.heading,
            "speed": self.state.speed,
            "steering": self.state.steering,
            "steering_rate": sample.steering_rate,
            "cross_track": cross_track,
            "heading_error": heading_error,
            "lateral_acceleration": sample.lateral_acceleration,
            "nearest_index": float(nearest),
        }
        for key, value in values.items():
            self.history[key].append(value)
        if nearest >= len(self.path.x) - 3:
            self.completed = True


class MetricCard(QFrame):
    def __init__(self, title: str, unit: str = "") -> None:
        super().__init__()
        self.setObjectName("metricCard")
        self.title_label = QLabel(title)
        self.title_label.setObjectName("metricTitle")
        self.value_label = QLabel("—")
        self.value_label.setObjectName("metricValue")
        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("metricUnit")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(0)
        layout.addWidget(self.title_label)
        line = QHBoxLayout()
        line.addWidget(self.value_label)
        line.addWidget(self.unit_label)
        line.addStretch()
        layout.addLayout(line)

    def set_value(self, value: float, decimals: int = 2) -> None:
        self.value_label.setText(f"{value:.{decimals}f}")


class TraceWidget(QWidget):
    """Small dependency-free scrolling chart drawn with QPainter."""

    def __init__(
        self,
        title: str,
        traces: tuple[tuple[str, QColor], ...],
        symmetric_limit: float,
    ) -> None:
        super().__init__()
        self.title = title
        self.trace_definitions = traces
        self.symmetric_limit = symmetric_limit
        self.values: dict[str, list[float]] = {
            name: [] for name, _ in traces
        }
        self.setMinimumHeight(126)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )

    def update_values(self, values: dict[str, list[float]]) -> None:
        self.values = values
        self.update()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), PANEL)
        plot = QRectF(45.0, 25.0, self.width() - 58.0, self.height() - 42.0)
        painter.setPen(QPen(QColor("#cbd5e1"), 1.0))
        painter.drawRect(plot)
        middle_y = plot.center().y()
        painter.setPen(QPen(QColor("#dbe3ea"), 1.0, Qt.PenStyle.DashLine))
        painter.drawLine(
            QPointF(plot.left(), middle_y),
            QPointF(plot.right(), middle_y),
        )
        painter.setPen(INK)
        painter.drawText(10, 17, self.title)
        painter.setPen(SLATE)
        painter.drawText(7, int(plot.top() + 6), f"+{self.symmetric_limit:g}")
        painter.drawText(18, int(middle_y + 5), "0")
        painter.drawText(
            7,
            int(plot.bottom()),
            f"−{self.symmetric_limit:g}",
        )

        legend_x = int(plot.right() - 150)
        for index, (name, color) in enumerate(self.trace_definitions):
            painter.setPen(QPen(color, 2.0))
            painter.drawLine(
                legend_x + index * 78,
                14,
                legend_x + 18 + index * 78,
                14,
            )
            painter.setPen(INK)
            painter.drawText(
                legend_x + 22 + index * 78,
                18,
                name,
            )

        maximum_samples = 350
        for name, color in self.trace_definitions:
            samples = self.values.get(name, [])[-maximum_samples:]
            if len(samples) < 2:
                continue
            points = []
            for index, value in enumerate(samples):
                x = plot.left() + index / (len(samples) - 1) * plot.width()
                normalized = np.clip(
                    value / self.symmetric_limit,
                    -1.0,
                    1.0,
                )
                y = middle_y - normalized * 0.5 * plot.height()
                points.append(QPointF(float(x), float(y)))
            painter.setPen(QPen(color, 2.0))
            painter.drawPolyline(QPolygonF(points))


class RoadView(QWidget):
    """Top-down moving-camera road and pursuit-geometry view."""

    def __init__(self, engine: SimulationEngine) -> None:
        super().__init__()
        self.engine = engine
        self.show_geometry = True
        self.show_waypoints = True
        self.show_trail = True
        self.setMinimumSize(760, 470)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )

    def world_to_screen(self, x: float, y: float) -> QPointF:
        state = self.engine.state
        scale = min(max(self.width() / 115.0, 6.0), 10.0)
        centre_x = self.width() * 0.36
        centre_y = self.height() * 0.53
        return QPointF(
            centre_x + (x - state.x) * scale,
            centre_y - (y - state.y) * scale,
        )

    def _polyline(
        self,
        x_values: np.ndarray | list[float],
        y_values: np.ndarray | list[float],
    ) -> QPolygonF:
        return QPolygonF(
            [
                self.world_to_screen(float(x), float(y))
                for x, y in zip(x_values, y_values)
            ]
        )

    def _draw_vehicle(self, painter: QPainter) -> None:
        state = self.engine.state
        centre = self.world_to_screen(state.x, state.y)
        scale = min(max(self.width() / 115.0, 6.0), 10.0)
        painter.save()
        painter.translate(centre)
        painter.rotate(-math.degrees(state.heading))
        length = 4.5 * scale
        width = 1.85 * scale
        painter.setPen(QPen(INK, 2.0))
        painter.setBrush(QBrush(BLUE))
        painter.drawRoundedRect(
            QRectF(-0.38 * length, -0.5 * width, length, width),
            5.0,
            5.0,
        )
        painter.setBrush(QBrush(QColor("#bde3ff")))
        painter.drawRoundedRect(
            QRectF(-0.05 * length, -0.38 * width, 0.42 * length, 0.76 * width),
            3.0,
            3.0,
        )
        # Wheel rectangles make steering direction visually explicit.
        painter.setBrush(QBrush(INK))
        wheel_length = 0.65 * scale
        wheel_width = 0.22 * scale
        rear_x = -0.25 * length
        front_x = 0.47 * length
        for axle_x, steering in (
            (rear_x, 0.0),
            (front_x, state.steering),
        ):
            for axle_y in (-0.52 * width, 0.52 * width):
                painter.save()
                painter.translate(axle_x, axle_y)
                painter.rotate(-math.degrees(steering))
                painter.drawRect(
                    QRectF(
                        -0.5 * wheel_length,
                        -0.5 * wheel_width,
                        wheel_length,
                        wheel_width,
                    )
                )
                painter.restore()
        painter.setBrush(QBrush(ORANGE))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(
            QPolygonF(
                [
                    QPointF(0.67 * length, 0.0),
                    QPointF(0.47 * length, -0.18 * width),
                    QPointF(0.47 * length, 0.18 * width),
                ]
            )
        )
        painter.restore()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), BACKGROUND)
        path = self.engine.path
        half_width = self.engine.settings.road_half_width
        normal_x = -np.sin(path.heading)
        normal_y = np.cos(path.heading)
        left_x = path.x + half_width * normal_x
        left_y = path.y + half_width * normal_y
        right_x = path.x - half_width * normal_x
        right_y = path.y - half_width * normal_y

        # Draw the road as a thick centreline, then add exact boundaries.
        painter.setPen(
            QPen(
                ROAD,
                2.0 * half_width * 8.0,
                Qt.PenStyle.SolidLine,
                Qt.PenCapStyle.RoundCap,
                Qt.PenJoinStyle.RoundJoin,
            )
        )
        painter.drawPolyline(self._polyline(path.x, path.y))
        painter.setPen(QPen(QColor("#f8fafc"), 2.0, Qt.PenStyle.DashLine))
        painter.drawPolyline(self._polyline(path.x, path.y))
        painter.setPen(QPen(QColor("#f8fafc"), 1.3))
        painter.drawPolyline(self._polyline(left_x, left_y))
        painter.drawPolyline(self._polyline(right_x, right_y))

        if self.show_waypoints:
            painter.setPen(QPen(QColor("#dbe3ea"), 1.0))
            painter.setBrush(QBrush(QColor("#f8fafc")))
            for x, y in zip(path.waypoint_x, path.waypoint_y):
                point = self.world_to_screen(float(x), float(y))
                painter.drawEllipse(point, 4.5, 4.5)

        history = self.engine.history
        if self.show_trail and len(history["x"]) > 1:
            painter.setPen(QPen(QColor("#7dd3fc"), 3.0))
            painter.drawPolyline(
                self._polyline(history["x"], history["y"])
            )

        if self.show_geometry:
            state_point = self.world_to_screen(
                self.engine.state.x,
                self.engine.state.y,
            )
            nearest = self.engine.previous_index
            nearest_point = self.world_to_screen(
                float(path.x[nearest]),
                float(path.y[nearest]),
            )
            target_point = self.world_to_screen(
                self.engine.target_x,
                self.engine.target_y,
            )
            painter.setPen(QPen(RED, 2.0, Qt.PenStyle.DashLine))
            painter.drawLine(state_point, nearest_point)
            painter.setBrush(QBrush(RED))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(nearest_point, 5.0, 5.0)
            if self.engine.current_lookahead > 0.0:
                painter.setPen(QPen(ORANGE, 2.0))
                painter.drawLine(state_point, target_point)
                painter.setBrush(QBrush(ORANGE))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawEllipse(target_point, 6.0, 6.0)
                scale = min(max(self.width() / 115.0, 6.0), 10.0)
                radius = self.engine.current_lookahead * scale
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.setPen(
                    QPen(ORANGE, 1.3, Qt.PenStyle.DotLine)
                )
                painter.drawEllipse(state_point, radius, radius)

        self._draw_vehicle(painter)

        painter.fillRect(QRectF(14, 14, 250, 88), QColor(255, 255, 255, 225))
        painter.setPen(INK)
        painter.setFont(QFont("Arial", 10))
        painter.drawText(27, 38, self.engine.settings.controller)
        painter.setPen(SLATE)
        painter.drawText(
            27,
            59,
            f"Orange: target   Red: nearest point",
        )
        painter.drawText(
            27,
            81,
            f"Path: {path.name}   t = {self.engine.time:.1f} s",
        )


class ControlPanel(QWidget):
    def __init__(self, window: "MainWindow") -> None:
        super().__init__()
        self.window = window
        self.setFixedWidth(330)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        preset_group = QGroupBox("Teaching scenario")
        preset_layout = QVBoxLayout(preset_group)
        self.preset = QComboBox()
        self.preset.addItems(
            [
                "Lesson 1 — constant steering",
                "Lesson 3 — tracking geometry",
                "Lesson 4 — short look-ahead",
                "Lesson 4 — balanced",
                "Lesson 4 — long look-ahead",
                "Lesson 6 — high-speed fixed",
                "Lesson 6 — adaptive + disturbance",
            ]
        )
        preset_layout.addWidget(self.preset)
        layout.addWidget(preset_group)

        model_group = QGroupBox("Model and controller")
        form = QFormLayout(model_group)
        self.path_kind = QComboBox()
        self.path_kind.addItems(["gentle", "training", "tight"])
        self.controller = QComboBox()
        self.controller.addItems(
            [
                "Constant steering",
                "Nearest-point P",
                "Pure Pursuit — fixed",
                "Pure Pursuit — adaptive",
            ]
        )
        self.speed = self._spin(2.0, 20.0, 9.0, 0.5, " m/s")
        self.wheelbase = self._spin(1.5, 5.0, 2.7, 0.1, " m")
        self.constant_steering = self._spin(
            -30.0,
            30.0,
            12.0,
            1.0,
            "°",
        )
        form.addRow("Path", self.path_kind)
        form.addRow("Controller", self.controller)
        form.addRow("Speed", self.speed)
        form.addRow("Wheelbase", self.wheelbase)
        form.addRow("Constant steering", self.constant_steering)
        layout.addWidget(model_group)

        pursuit_group = QGroupBox("Pure Pursuit and test conditions")
        pursuit_form = QFormLayout(pursuit_group)
        self.base_lookahead = self._spin(
            1.0,
            15.0,
            5.0,
            0.5,
            " m",
        )
        self.speed_gain = self._spin(0.0, 1.0, 0.0, 0.05, " s")
        self.initial_offset = self._spin(
            -4.0,
            4.0,
            1.5,
            0.25,
            " m",
        )
        self.heading_offset = self._spin(
            -30.0,
            30.0,
            4.0,
            1.0,
            "°",
        )
        self.noise = self._spin(0.0, 1.0, 0.0, 0.05, " m")
        pursuit_form.addRow("Base look-ahead", self.base_lookahead)
        pursuit_form.addRow("Speed gain", self.speed_gain)
        pursuit_form.addRow("Initial lateral offset", self.initial_offset)
        pursuit_form.addRow("Initial heading error", self.heading_offset)
        pursuit_form.addRow("Position noise", self.noise)
        layout.addWidget(pursuit_group)

        view_group = QGroupBox("View")
        view_layout = QHBoxLayout(view_group)
        self.geometry = QCheckBox("Geometry")
        self.geometry.setChecked(True)
        self.waypoints = QCheckBox("Waypoints")
        self.waypoints.setChecked(True)
        self.trail = QCheckBox("Trail")
        self.trail.setChecked(True)
        view_layout.addWidget(self.geometry)
        view_layout.addWidget(self.waypoints)
        view_layout.addWidget(self.trail)
        layout.addWidget(view_group)

        button_grid = QGridLayout()
        self.run_button = QPushButton("Pause")
        self.reset_button = QPushButton("Reset")
        self.step_button = QPushButton("Single step")
        self.apply_button = QPushButton("Apply and reset")
        self.disturb_button = QPushButton("Push vehicle +2 m")
        self.disturb_button.setObjectName("dangerButton")
        button_grid.addWidget(self.run_button, 0, 0)
        button_grid.addWidget(self.reset_button, 0, 1)
        button_grid.addWidget(self.step_button, 1, 0)
        button_grid.addWidget(self.apply_button, 1, 1)
        button_grid.addWidget(self.disturb_button, 2, 0, 1, 2)
        layout.addLayout(button_grid)

        note = QLabel(
            "Classroom workflow:\n"
            "1. Pause and predict.\n"
            "2. Change one value.\n"
            "3. Apply and observe.\n"
            "4. Compare the metrics."
        )
        note.setWordWrap(True)
        note.setObjectName("teacherNote")
        layout.addWidget(note)
        layout.addStretch()

        self.preset.currentIndexChanged.connect(window.apply_preset)
        self.run_button.clicked.connect(window.toggle_running)
        self.reset_button.clicked.connect(window.reset_simulation)
        self.step_button.clicked.connect(window.single_step)
        self.apply_button.clicked.connect(window.apply_controls)
        self.disturb_button.clicked.connect(window.inject_disturbance)
        self.geometry.toggled.connect(window.update_view_options)
        self.waypoints.toggled.connect(window.update_view_options)
        self.trail.toggled.connect(window.update_view_options)

    @staticmethod
    def _spin(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setSingleStep(step)
        widget.setSuffix(suffix)
        widget.setDecimals(2)
        return widget

    def settings(self) -> GuiSettings:
        return GuiSettings(
            path_kind=self.path_kind.currentText(),
            controller=self.controller.currentText(),
            speed=self.speed.value(),
            wheelbase=self.wheelbase.value(),
            constant_steering_degrees=self.constant_steering.value(),
            base_lookahead=self.base_lookahead.value(),
            speed_gain=self.speed_gain.value(),
            initial_offset=self.initial_offset.value(),
            initial_heading_degrees=self.heading_offset.value(),
            measurement_noise=self.noise.value(),
        )


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(
            "Smart Vehicles Control — Day 2 Path-Following Laboratory"
        )
        self.resize(1320, 850)
        self.engine = SimulationEngine()
        self.road_view = RoadView(self.engine)
        self.error_plot = TraceWidget(
            "Tracking errors",
            (("e_y [m]", BLUE), ("e_ψ [rad]", RED)),
            symmetric_limit=4.0,
        )
        self.motion_plot = TraceWidget(
            "Control and vehicle response",
            (("δ [rad]", ORANGE), ("a_y/10", GREEN)),
            symmetric_limit=0.65,
        )
        self.control_panel = ControlPanel(self)

        self.cards = {
            "speed": MetricCard("Speed", "m/s"),
            "cross_track": MetricCard("Cross-track error", "m"),
            "heading": MetricCard("Heading error", "deg"),
            "steering": MetricCard("Steering", "deg"),
            "lookahead": MetricCard("Look-ahead", "m"),
            "completion": MetricCard("Path complete", "%"),
        }
        card_layout = QHBoxLayout()
        for card in self.cards.values():
            card_layout.addWidget(card)

        plots = QHBoxLayout()
        plots.addWidget(self.error_plot)
        plots.addWidget(self.motion_plot)
        left = QVBoxLayout()
        left.addLayout(card_layout)
        left.addWidget(self.road_view, 1)
        left.addLayout(plots)
        left_widget = QWidget()
        left_widget.setLayout(left)

        central_layout = QHBoxLayout()
        central_layout.addWidget(left_widget, 1)
        central_layout.addWidget(self.control_panel)
        central = QWidget()
        central.setLayout(central_layout)
        self.setCentralWidget(central)

        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #eef4f8;
                color: #17233c;
                font-family: Arial;
                font-size: 10pt;
            }
            QGroupBox {
                background: white;
                border: 1px solid #cbd5e1;
                border-radius: 7px;
                margin-top: 10px;
                padding-top: 8px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QFrame#metricCard {
                background: white;
                border: 1px solid #cbd5e1;
                border-radius: 7px;
            }
            QLabel#metricTitle { color: #64748b; font-size: 9pt; }
            QLabel#metricValue {
                color: #17233c;
                font-size: 17pt;
                font-weight: bold;
            }
            QLabel#metricUnit { color: #64748b; }
            QLabel#teacherNote {
                background: #dbeafe;
                border-radius: 6px;
                padding: 9px;
            }
            QPushButton {
                background: #2476d8;
                color: white;
                border: 0;
                border-radius: 5px;
                padding: 7px;
                font-weight: bold;
            }
            QPushButton:hover { background: #1d63b6; }
            QPushButton#dangerButton { background: #e05252; }
            QComboBox, QDoubleSpinBox {
                background: white;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                padding: 4px;
            }
            """
        )

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.advance)
        self.timer.start(25)
        self.update_dashboard()

    def apply_preset(self, index: int) -> None:
        panel = self.control_panel
        presets = [
            # controller, speed, base, gain, steer, offset, heading, path
            ("Constant steering", 8.0, 5.0, 0.0, 12.0, 0.0, 0.0, "gentle"),
            ("Pure Pursuit — fixed", 8.0, 6.0, 0.0, 0.0, 2.2, 18.0, "training"),
            ("Pure Pursuit — fixed", 9.0, 2.0, 0.0, 0.0, 1.5, 4.0, "training"),
            ("Pure Pursuit — fixed", 9.0, 5.0, 0.0, 0.0, 1.5, 4.0, "training"),
            ("Pure Pursuit — fixed", 9.0, 10.0, 0.0, 0.0, 1.5, 4.0, "training"),
            ("Pure Pursuit — fixed", 14.0, 4.0, 0.0, 0.0, 0.5, 0.0, "training"),
            ("Pure Pursuit — adaptive", 14.0, 2.2, 0.32, 0.0, 0.5, 0.0, "training"),
        ]
        (
            controller,
            speed,
            base,
            gain,
            steering,
            offset,
            heading,
            path,
        ) = presets[index]
        panel.controller.setCurrentText(controller)
        panel.speed.setValue(speed)
        panel.base_lookahead.setValue(base)
        panel.speed_gain.setValue(gain)
        panel.constant_steering.setValue(steering)
        panel.initial_offset.setValue(offset)
        panel.heading_offset.setValue(heading)
        panel.path_kind.setCurrentText(path)
        self.apply_controls()
        if index == 6:
            # Apply after a short delay so the recovery is visible immediately.
            QTimer.singleShot(1500, self.inject_disturbance)

    def apply_controls(self) -> None:
        self.engine.configure(self.control_panel.settings())
        self.update_dashboard()
        self.road_view.update()

    def reset_simulation(self) -> None:
        self.engine.reset()
        self.update_dashboard()
        self.road_view.update()

    def toggle_running(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self.control_panel.run_button.setText("Run")
        else:
            self.timer.start(25)
            self.control_panel.run_button.setText("Pause")

    def single_step(self) -> None:
        self.engine.step()
        self.update_dashboard()

    def inject_disturbance(self) -> None:
        self.engine.inject_disturbance(2.0)
        self.road_view.update()

    def update_view_options(self) -> None:
        panel = self.control_panel
        self.road_view.show_geometry = panel.geometry.isChecked()
        self.road_view.show_waypoints = panel.waypoints.isChecked()
        self.road_view.show_trail = panel.trail.isChecked()
        self.road_view.update()

    def advance(self) -> None:
        # Two numerical updates per visual frame gives approximately real-time
        # motion at dt=0.05 s and a 25 ms GUI timer.
        self.engine.step()
        self.engine.step()
        self.update_dashboard()
        if self.engine.completed:
            self.timer.stop()
            self.control_panel.run_button.setText("Run")

    def update_dashboard(self) -> None:
        engine = self.engine
        self.cards["speed"].set_value(engine.state.speed, 1)
        self.cards["cross_track"].set_value(
            engine.current_cross_track_error,
            2,
        )
        self.cards["heading"].set_value(
            math.degrees(engine.current_heading_error),
            1,
        )
        self.cards["steering"].set_value(
            math.degrees(engine.state.steering),
            1,
        )
        self.cards["lookahead"].set_value(
            engine.current_lookahead,
            1,
        )
        self.cards["completion"].set_value(
            engine.completion_percent,
            1,
        )
        history = engine.history
        self.error_plot.update_values(
            {
                "e_y [m]": history["cross_track"],
                "e_ψ [rad]": history["heading_error"],
            }
        )
        self.motion_plot.update_values(
            {
                "δ [rad]": history["steering"],
                "a_y/10": [
                    value / 10.0
                    for value in history["lateral_acceleration"]
                ],
            }
        )
        self.road_view.update()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--offscreen",
        action="store_true",
        help="use Qt's offscreen platform for automated smoke tests",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="save a PNG of the running GUI and exit",
    )
    parser.add_argument(
        "--preset",
        type=int,
        choices=range(0, 7),
        default=3,
        help="initial preset index (0 to 6)",
    )
    parser.add_argument(
        "--runtime-ms",
        type=int,
        default=1200,
        help="offscreen runtime before screenshot/exit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.offscreen or args.screenshot is not None:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    application = QApplication(sys.argv)
    window = MainWindow()
    window.control_panel.preset.setCurrentIndex(args.preset)
    window.apply_preset(args.preset)
    window.show()

    if args.offscreen or args.screenshot is not None:
        def finish() -> None:
            if args.screenshot is not None:
                args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                if not window.grab().save(str(args.screenshot)):
                    raise RuntimeError("failed to save GUI screenshot")
                print(f"Saved screenshot: {args.screenshot}")
            application.quit()

        QTimer.singleShot(max(100, args.runtime_ms), finish)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
