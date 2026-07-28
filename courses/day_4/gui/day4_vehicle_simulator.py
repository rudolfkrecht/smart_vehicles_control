"""Interactive PyQt robustness laboratory for all six Day 4 lessons.

Run from the package root:

    python day_4/gui/day4_vehicle_simulator.py
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPen,
    QPolygonF,
)
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from simulator.faults import FaultParameters
from simulator.integrated import IntegratedScenario
from simulator.robustness import (
    RobustScenario,
    RobustSimulation,
    RobustSnapshot,
)
from simulator.traffic import BehaviourState


NAVY = "#17233c"
BLUE = "#2476d8"
RED = "#d9534f"
GREEN = "#2ca25f"
AMBER = "#f59e0b"
PURPLE = "#7057a3"
BACKGROUND = "#edf3f7"

STATE_COLORS = {
    BehaviourState.CRUISE: BLUE,
    BehaviourState.FOLLOW: GREEN,
    BehaviourState.BRAKE: AMBER,
    BehaviourState.EMERGENCY: RED,
}


class TraceBuffer:
    def __init__(self, maximum_length: int = 500) -> None:
        self.values: deque[float] = deque(maxlen=maximum_length)

    def clear(self) -> None:
        self.values.clear()

    def append(self, value: float) -> None:
        self.values.append(float(value))


class TracePlot(QWidget):
    """Small dependency-free scrolling plot drawn with QPainter."""

    def __init__(
        self,
        title: str,
        y_minimum: float,
        y_maximum: float,
        series: list[tuple[str, str, TraceBuffer]],
    ) -> None:
        super().__init__()
        self.title = title
        self.y_minimum = y_minimum
        self.y_maximum = y_maximum
        self.series = series
        self.setMinimumHeight(135)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
        painter.setPen(QColor(NAVY))
        painter.setFont(QFont("Arial", 9, QFont.Weight.DemiBold))
        painter.drawText(10, 18, self.title)

        plot = QRectF(38, 28, self.width() - 48, self.height() - 45)
        painter.setPen(QPen(QColor("#e2e8f0"), 1))
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = plot.top() + fraction * plot.height()
            painter.drawLine(
                QPointF(plot.left(), y),
                QPointF(plot.right(), y),
            )
        painter.setFont(QFont("Arial", 7))
        painter.setPen(QColor("#64748b"))
        painter.drawText(
            3,
            int(plot.top() + 8),
            f"{self.y_maximum:g}",
        )
        painter.drawText(
            3,
            int(plot.bottom()),
            f"{self.y_minimum:g}",
        )

        for label_index, (label, color, buffer) in enumerate(self.series):
            values = list(buffer.values)
            if len(values) >= 2:
                points = []
                for index, value in enumerate(values):
                    x = (
                        plot.left()
                        + index / max(len(values) - 1, 1) * plot.width()
                    )
                    clipped = min(
                        max(value, self.y_minimum),
                        self.y_maximum,
                    )
                    fraction = (
                        clipped - self.y_minimum
                    ) / (self.y_maximum - self.y_minimum)
                    y = plot.bottom() - fraction * plot.height()
                    points.append(QPointF(x, y))
                painter.setPen(QPen(QColor(color), 2))
                painter.drawPolyline(QPolygonF(points))

            legend_x = 45 + label_index * 108
            painter.setPen(QPen(QColor(color), 3))
            painter.drawLine(
                QPointF(legend_x, self.height() - 9),
                QPointF(legend_x + 16, self.height() - 9),
            )
            painter.setPen(QColor(NAVY))
            painter.drawText(
                legend_x + 20,
                self.height() - 5,
                label,
            )


class MetricCard(QLabel):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        self.setMinimumHeight(58)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet(
            "QLabel {background: white; border: 1px solid #cbd5e1;"
            "border-radius: 6px; color: #17233c; padding: 5px;}"
        )
        self.set_value("—")

    def set_value(self, value: str, color: str = NAVY) -> None:
        self.setText(
            f"<span style='font-size:8pt'>{self.title}</span><br>"
            f"<span style='font-size:14pt; font-weight:600; "
            f"color:{color}'>{value}</span>"
        )


class RoadCanvas(QWidget):
    """Moving road view with true/measured pose and fault overlays."""

    def __init__(self) -> None:
        super().__init__()
        self.simulation = RobustSimulation()
        self.snapshot: RobustSnapshot | None = None
        self.show_curvature = True
        self.show_geometry = True
        self.setMinimumSize(760, 390)

    def set_simulation(self, simulation: RobustSimulation) -> None:
        self.simulation = simulation
        self.snapshot = None
        self.update()

    def world_to_screen(self, x: float, y: float) -> QPointF:
        state = self.simulation.vehicle
        scale = min(6.2, max(4.2, self.width() / 185.0))
        return QPointF(
            0.30 * self.width() + (x - state.x) * scale,
            0.54 * self.height() - (y - state.y) * scale,
        )

    def _draw_vehicle(
        self,
        painter: QPainter,
        *,
        x: float,
        y: float,
        heading: float,
        color: str,
        label: str,
    ) -> None:
        centre = self.world_to_screen(x, y)
        painter.save()
        painter.translate(centre)
        painter.rotate(-math.degrees(heading))
        painter.setPen(QPen(QColor(NAVY), 2))
        painter.setBrush(QBrush(QColor(color)))
        painter.drawRoundedRect(QRectF(-20, -10, 40, 20), 5, 5)
        painter.setBrush(QColor("#c9e8ff"))
        painter.drawRoundedRect(QRectF(-5, -8, 13, 16), 2, 2)
        painter.setBrush(QColor("#111827"))
        for wheel_x in (-13, 13):
            painter.drawRect(QRectF(wheel_x - 4, -13, 8, 4))
            painter.drawRect(QRectF(wheel_x - 4, 9, 8, 4))
        painter.restore()
        painter.setPen(QColor(NAVY))
        painter.setFont(QFont("Arial", 8, QFont.Weight.DemiBold))
        painter.drawText(centre + QPointF(-19, -15), label)

    def _draw_road(self, painter: QPainter) -> None:
        path = self.simulation.path
        points = [
            self.world_to_screen(float(x), float(y))
            for x, y in zip(path.x, path.y)
        ]
        polygon = QPolygonF(points)
        painter.setPen(
            QPen(
                QColor("#586271"),
                47,
                Qt.PenStyle.SolidLine,
                Qt.PenCapStyle.RoundCap,
                Qt.PenJoinStyle.RoundJoin,
            )
        )
        painter.drawPolyline(polygon)
        painter.setPen(
            QPen(
                QColor("#f8fafc"),
                2,
                Qt.PenStyle.DashLine,
            )
        )
        painter.drawPolyline(polygon)

        if self.show_curvature:
            profile = self.simulation.profile
            for index in range(0, len(path.x) - 1, 2):
                speed_fraction = (
                    profile.planned_speed[index]
                    / profile.parameters.global_speed_limit
                )
                if speed_fraction < 0.55:
                    color = RED
                elif speed_fraction < 0.78:
                    color = AMBER
                else:
                    color = GREEN
                painter.setPen(QPen(QColor(color), 4))
                painter.drawLine(points[index], points[index + 1])

    def _draw_geometry(self, painter: QPainter) -> None:
        if not self.snapshot or not self.show_geometry:
            return
        snapshot = self.snapshot
        vehicle_point = self.world_to_screen(
            snapshot.true_vehicle.x,
            snapshot.true_vehicle.y,
        )
        target_point = self.world_to_screen(
            snapshot.lateral.target_x,
            snapshot.lateral.target_y,
        )
        painter.setPen(QPen(QColor(AMBER), 2, Qt.PenStyle.DashLine))
        painter.drawLine(vehicle_point, target_point)
        painter.setBrush(QColor(AMBER))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(target_point, 6, 6)

        if self.simulation.scenario.enable_traffic:
            lead_point = self.world_to_screen(
                snapshot.lead_x,
                snapshot.lead_y,
            )
            state_color = STATE_COLORS[snapshot.traffic.state]
            painter.setPen(
                QPen(QColor(state_color), 4, Qt.PenStyle.DotLine)
            )
            painter.drawLine(vehicle_point, lead_point)

    def _draw_hud(self, painter: QPainter) -> None:
        snapshot = self.snapshot
        state = (
            snapshot.traffic.state
            if snapshot
            else BehaviourState.CRUISE
        )
        state_color = STATE_COLORS[state]
        box = QRectF(15, 14, 285, 124)
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.setBrush(QColor(255, 255, 255, 235))
        painter.drawRoundedRect(box, 7, 7)
        painter.setFont(QFont("Arial", 10, QFont.Weight.DemiBold))
        painter.setPen(QColor(NAVY))
        painter.drawText(28, 38, "DAY 4 — ROBUSTNESS TEST LAB")
        painter.setBrush(QColor(state_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(QRectF(27, 49, 104, 25), 5, 5)
        painter.setPen(QColor("white"))
        painter.drawText(42, 67, state.value)
        painter.setPen(QColor(NAVY))
        painter.setFont(QFont("Arial", 9))
        if snapshot:
            lines = (
                f"True speed: {snapshot.true_vehicle.speed:4.1f} m/s",
                f"Selected target: {snapshot.selected_target_speed:4.1f} m/s",
                (
                    f"True gap: {snapshot.true_gap:4.1f} m"
                    if math.isfinite(snapshot.true_gap)
                    else "Gap: traffic disabled"
                ),
            )
        else:
            lines = ("Ready", "Press Run or Step", "")
        for index, line in enumerate(lines):
            painter.drawText(145, 60 + index * 20, line)

        legend = (
            ("true pose", BLUE),
            ("measured", AMBER),
            ("fault", RED),
        )
        painter.setFont(QFont("Arial", 8))
        for index, (label, color) in enumerate(legend):
            x = 27 + index * 88
            painter.setBrush(QColor(color))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(QRectF(x, 110, 14, 5))
            painter.setPen(QColor(NAVY))
            painter.drawText(x + 18, 117, label)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(BACKGROUND))
        self._draw_road(painter)
        self._draw_geometry(painter)
        state = self.simulation.vehicle
        self._draw_vehicle(
            painter,
            x=state.x,
            y=state.y,
            heading=state.heading,
            color=BLUE,
            label="EGO",
        )
        if self.snapshot:
            self._draw_vehicle(
                painter,
                x=self.snapshot.measured_vehicle.x,
                y=self.snapshot.measured_vehicle.y,
                heading=self.snapshot.measured_vehicle.heading,
                color=AMBER,
                label="MEASURED",
            )
        if self.simulation.scenario.enable_traffic:
            if self.snapshot:
                lead_x = self.snapshot.lead_x
                lead_y = self.snapshot.lead_y
                lead_heading = self.snapshot.lead_heading
            else:
                from simulator.paths import point_at_distance

                lead_x, lead_y, lead_heading = point_at_distance(
                    self.simulation.path,
                    self.simulation.lead_distance,
                )
            self._draw_vehicle(
                painter,
                x=lead_x,
                y=lead_y,
                heading=lead_heading,
                color=RED,
                label="LEAD",
            )
        self._draw_hud(painter)


class ControlPanel(QWidget):
    def __init__(self, window: "Day4Window") -> None:
        super().__init__()
        self.window = window
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(8)

        title = QLabel("Teaching controls")
        title.setStyleSheet(
            "font-size: 15px; font-weight: 600; color: #17233c;"
        )
        layout.addWidget(title)

        self.preset = QComboBox()
        self.preset.addItems(
            [
                "Lesson 1 — nominal success",
                "Lesson 1 — sensor noise",
                "Lesson 1 — sensor and actuator delay",
                "Lesson 1 — steering bias",
                "Lesson 1 — weak braking",
                "Lesson 3 — lateral push",
                "Lesson 3 — combined disturbance",
                "Lesson 4 — aggressive practice",
                "Lesson 4 — balanced practice",
                "Lesson 5 — unfamiliar evaluation road",
                "Lesson 5 — combined evaluation",
            ]
        )
        self.preset.currentIndexChanged.connect(self.apply_preset)
        layout.addWidget(self.preset)

        architecture = QGroupBox("Scenario")
        switches = QFormLayout(architecture)
        self.path_kind = QComboBox()
        self.path_kind.addItems(["practice", "evaluation_a", "evaluation_b"])
        self.curve_speed = QCheckBox()
        self.traffic = QCheckBox()
        self.state_machine = QCheckBox()
        switches.addRow("Test road", self.path_kind)
        switches.addRow("Curve-aware speed", self.curve_speed)
        switches.addRow("Lead vehicle / ACC", self.traffic)
        switches.addRow("Behaviour states", self.state_machine)
        layout.addWidget(architecture)

        road = QGroupBox("Road-speed planning")
        road_form = QFormLayout(road)
        self.speed_limit = self._double(3.0, 24.0, 15.0, 0.5, " m/s")
        self.max_lateral = self._double(0.8, 6.0, 2.5, 0.1, " m/s²")
        self.preview = self._double(0.0, 35.0, 14.0, 1.0, " m")
        self.smoothing = QSpinBox()
        self.smoothing.setRange(1, 31)
        self.smoothing.setSingleStep(2)
        self.smoothing.setValue(7)
        road_form.addRow("Global speed limit", self.speed_limit)
        road_form.addRow("Max lateral accel.", self.max_lateral)
        road_form.addRow("Curve preview", self.preview)
        road_form.addRow("Smoothing samples", self.smoothing)
        layout.addWidget(road)

        lateral = QGroupBox("Path following")
        lateral_form = QFormLayout(lateral)
        self.base_lookahead = self._double(1.0, 12.0, 3.0, 0.5, " m")
        self.speed_gain = self._double(0.0, 0.8, 0.25, 0.05, " s")
        lateral_form.addRow("Base look-ahead", self.base_lookahead)
        lateral_form.addRow("Speed gain", self.speed_gain)
        layout.addWidget(lateral)

        acc = QGroupBox("ACC and safety")
        acc_form = QFormLayout(acc)
        self.headway = self._double(0.4, 4.0, 1.5, 0.1, " s")
        self.standstill_gap = self._double(1.0, 15.0, 5.0, 0.5, " m")
        self.emergency_ttc = self._double(0.4, 4.0, 1.25, 0.05, " s")
        self.emergency_gap = self._double(0.5, 10.0, 3.0, 0.5, " m")
        self.lead_preset = QComboBox()
        self.lead_preset.addItems(
            ["stop_and_go", "evaluation", "steady", "slow", "late_brake"]
        )
        acc_form.addRow("Time headway", self.headway)
        acc_form.addRow("Standstill gap", self.standstill_gap)
        acc_form.addRow("Emergency TTC", self.emergency_ttc)
        acc_form.addRow("Emergency gap", self.emergency_gap)
        acc_form.addRow("Lead scenario", self.lead_preset)
        layout.addWidget(acc)

        sensing = QGroupBox("Measurement faults")
        sensing_form = QFormLayout(sensing)
        self.position_noise = self._double(0.0, 1.0, 0.0, 0.02, " m")
        self.heading_noise = self._double(0.0, 6.0, 0.0, 0.1, "°")
        self.speed_noise = self._double(0.0, 1.5, 0.0, 0.05, " m/s")
        self.range_noise = self._double(0.0, 2.0, 0.0, 0.05, " m")
        self.sensor_delay = self._double(0.0, 0.6, 0.0, 0.05, " s")
        sensing_form.addRow("Position noise σ", self.position_noise)
        sensing_form.addRow("Heading noise σ", self.heading_noise)
        sensing_form.addRow("Speed noise σ", self.speed_noise)
        sensing_form.addRow("Range noise σ", self.range_noise)
        sensing_form.addRow("Sensor delay", self.sensor_delay)
        layout.addWidget(sensing)

        actuation = QGroupBox("Actuator faults and disturbance")
        actuator_form = QFormLayout(actuation)
        self.fault_start = self._double(0.0, 30.0, 0.0, 1.0, " s")
        self.actuator_delay = self._double(0.0, 0.6, 0.0, 0.05, " s")
        self.steering_bias = self._double(-6.0, 6.0, 0.0, 0.2, "°")
        self.steering_authority = self._double(0.2, 1.0, 1.0, 0.05, "")
        self.braking_efficiency = self._double(0.2, 1.0, 1.0, 0.05, "")
        self.push_time = self._double(0.0, 30.0, 13.0, 1.0, " s")
        self.push_size = self._double(-3.0, 3.0, 0.0, 0.25, " m")
        self.random_seed = QSpinBox()
        self.random_seed.setRange(0, 9999)
        self.random_seed.setValue(7)
        actuator_form.addRow("Fault starts", self.fault_start)
        actuator_form.addRow("Actuator delay", self.actuator_delay)
        actuator_form.addRow("Steering bias", self.steering_bias)
        actuator_form.addRow("Steering authority", self.steering_authority)
        actuator_form.addRow("Braking efficiency", self.braking_efficiency)
        actuator_form.addRow("Push time", self.push_time)
        actuator_form.addRow("Lateral push", self.push_size)
        actuator_form.addRow("Random seed", self.random_seed)
        layout.addWidget(actuation)

        overlays = QGroupBox("Visual overlays")
        overlay_form = QFormLayout(overlays)
        self.show_curvature = QCheckBox()
        self.show_curvature.setChecked(True)
        self.show_geometry = QCheckBox()
        self.show_geometry.setChecked(True)
        overlay_form.addRow("Colour road by speed", self.show_curvature)
        overlay_form.addRow("Controller geometry", self.show_geometry)
        layout.addWidget(overlays)

        apply_button = QPushButton("Apply and reset scenario")
        apply_button.setStyleSheet(
            "QPushButton {background:#2476d8; color:white; padding:7px;"
            "border-radius:5px; font-weight:600;}"
        )
        apply_button.clicked.connect(window.apply_controls)
        layout.addWidget(apply_button)
        hint = QLabel(
            "Change one parameter at a time. Predict first, then apply, "
            "run, inspect metrics and explain the result."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#475569; font-size:9pt;")
        layout.addWidget(hint)
        layout.addStretch(1)

    @staticmethod
    def _double(
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

    def apply_preset(self, index: int) -> None:
        # Presets use the same fault values as the scripts and lesson pages.
        presets = [
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="stop_and_go"),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="stop_and_go", pos=.12, heading=.8, speed_noise=.20, range=.35, seed=21),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="steady", sensor=.25, actuator=.15, authority=.78, start=6, seed=104),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="stop_and_go", bias=1.4, authority=.9, start=7, seed=41),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="evaluation", brake=.65, start=10, seed=51),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="stop_and_go", push=1.5, push_time=13, seed=12),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="evaluation", pos=.10, heading=.65, speed_noise=.18, range=.30, sensor=.10, actuator=.05, bias=-.8, authority=.9, brake=.75, push=1.0, push_time=16, start=5, seed=75),
            dict(path="practice", speed=18, lat=4.2, preview=4, base=2.0, gain=.08, headway=.8, gap=2, ttc=.75, lead="stop_and_go"),
            dict(path="practice", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="stop_and_go"),
            dict(path="evaluation_a", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="evaluation", seed=71),
            dict(path="evaluation_b", speed=15, lat=2.5, preview=14, base=3.0, gain=.25, headway=1.5, gap=5, ttc=1.25, lead="evaluation", pos=.10, heading=.65, speed_noise=.18, range=.30, sensor=.10, actuator=.05, bias=-.8, authority=.9, brake=.75, push=1.0, push_time=16, start=5, seed=75),
        ]
        values = presets[index]
        self.curve_speed.setChecked(True)
        self.traffic.setChecked(True)
        self.state_machine.setChecked(True)
        self.path_kind.setCurrentText(values["path"])
        self.speed_limit.setValue(values["speed"])
        self.max_lateral.setValue(values["lat"])
        self.preview.setValue(values["preview"])
        self.base_lookahead.setValue(values["base"])
        self.speed_gain.setValue(values["gain"])
        self.headway.setValue(values["headway"])
        self.standstill_gap.setValue(values["gap"])
        self.emergency_ttc.setValue(values["ttc"])
        self.emergency_gap.setValue(3.0 if values["ttc"] >= 1.0 else 1.2)
        self.lead_preset.setCurrentText(values["lead"])
        self.position_noise.setValue(values.get("pos", 0.0))
        self.heading_noise.setValue(values.get("heading", 0.0))
        self.speed_noise.setValue(values.get("speed_noise", 0.0))
        self.range_noise.setValue(values.get("range", 0.0))
        self.sensor_delay.setValue(values.get("sensor", 0.0))
        self.actuator_delay.setValue(values.get("actuator", 0.0))
        self.steering_bias.setValue(values.get("bias", 0.0))
        self.steering_authority.setValue(values.get("authority", 1.0))
        self.braking_efficiency.setValue(values.get("brake", 1.0))
        self.push_time.setValue(values.get("push_time", 13.0))
        self.push_size.setValue(values.get("push", 0.0))
        self.fault_start.setValue(values.get("start", 0.0))
        self.random_seed.setValue(values.get("seed", 7))
        self.window.apply_controls()


class Day4Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(
            "Smart Vehicles Control — Day 4 Robustness and Testing Laboratory"
        )
        self.resize(1420, 900)
        self.setStyleSheet(
            "QMainWindow, QWidget {font-family: Arial; color:#17233c;}"
            "QGroupBox {font-weight:600; border:1px solid #cbd5e1;"
            "border-radius:6px; margin-top:8px; padding-top:8px;}"
            "QGroupBox::title {subcontrol-origin:margin; left:8px;"
            "padding:0 4px;} QDoubleSpinBox, QSpinBox, QComboBox {"
            "padding:3px; min-height:22px;}"
        )
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.advance)
        self.steps_per_frame = 2
        self.simulation = RobustSimulation()
        self.speed_trace = TraceBuffer()
        self.measured_speed_trace = TraceBuffer()
        self.target_trace = TraceBuffer()
        self.gap_trace = TraceBuffer()
        self.measured_gap_trace = TraceBuffer()
        self.requested_steering_trace = TraceBuffer()
        self.applied_steering_trace = TraceBuffer()
        self.error_trace = TraceBuffer()
        self.measured_error_trace = TraceBuffer()
        self.minimum_gap = math.inf
        self.maximum_path_error = 0.0
        self.peak_jerk = 0.0
        self.collision_seen = False

        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        self.setCentralWidget(central)

        left = QVBoxLayout()
        controls_row = QHBoxLayout()
        self.run_button = QPushButton("Run")
        self.step_button = QPushButton("Single step")
        self.reset_button = QPushButton("Reset")
        self.run_button.clicked.connect(self.toggle_run)
        self.step_button.clicked.connect(self.single_step)
        self.reset_button.clicked.connect(self.reset_simulation)
        for button in (self.run_button, self.step_button, self.reset_button):
            button.setMinimumHeight(32)
            controls_row.addWidget(button)
        self.status = QLabel("Ready")
        self.status.setStyleSheet("padding-left:12px; font-weight:600;")
        controls_row.addWidget(self.status, 1)
        left.addLayout(controls_row)

        self.road = RoadCanvas()
        left.addWidget(self.road, 3)

        metrics = QGridLayout()
        self.state_card = MetricCard("FAULT STATUS")
        self.gap_card = MetricCard("MINIMUM GAP")
        self.path_card = MetricCard("MAX PATH ERROR")
        self.lateral_card = MetricCard("PEAK JERK")
        metrics.addWidget(self.state_card, 0, 0)
        metrics.addWidget(self.gap_card, 0, 1)
        metrics.addWidget(self.path_card, 0, 2)
        metrics.addWidget(self.lateral_card, 0, 3)
        left.addLayout(metrics)

        plots = QGridLayout()
        self.speed_plot = TracePlot(
            "True and measured speed",
            0.0,
            20.0,
            [
                ("true", BLUE, self.speed_trace),
                ("measured", AMBER, self.measured_speed_trace),
                ("target", PURPLE, self.target_trace),
            ],
        )
        self.gap_plot = TracePlot(
            "True and measured following gap",
            0.0,
            55.0,
            [
                ("true", BLUE, self.gap_trace),
                ("measured", AMBER, self.measured_gap_trace),
            ],
        )
        self.accel_plot = TracePlot(
            "Requested and applied steering",
            -35.0,
            35.0,
            [
                ("requested", PURPLE, self.requested_steering_trace),
                ("applied", BLUE, self.applied_steering_trace),
            ],
        )
        self.error_plot = TracePlot(
            "True and measured cross-track error",
            -3.5,
            3.5,
            [
                ("true", GREEN, self.error_trace),
                ("measured", AMBER, self.measured_error_trace),
            ],
        )
        plots.addWidget(self.speed_plot, 0, 0)
        plots.addWidget(self.gap_plot, 0, 1)
        plots.addWidget(self.accel_plot, 1, 0)
        plots.addWidget(self.error_plot, 1, 1)
        left.addLayout(plots, 2)
        root.addLayout(left, 1)

        self.panel = ControlPanel(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(340)
        scroll.setMaximumWidth(390)
        scroll.setWidget(self.panel)
        root.addWidget(scroll)
        self.panel.apply_preset(0)

    def build_scenario(self) -> RobustScenario:
        base = IntegratedScenario(
            duration=42.0,
            path_kind=self.panel.path_kind.currentText(),
            enable_curve_speed=self.panel.curve_speed.isChecked(),
            enable_traffic=self.panel.traffic.isChecked(),
            enable_state_machine=self.panel.state_machine.isChecked(),
            lead_preset=self.panel.lead_preset.currentText(),
            base_lookahead=self.panel.base_lookahead.value(),
            speed_lookahead_gain=self.panel.speed_gain.value(),
        )
        smoothing = self.panel.smoothing.value()
        if smoothing % 2 == 0:
            smoothing += 1
        controller = replace(
            base,
            speed_profile=replace(
                base.speed_profile,
                global_speed_limit=self.panel.speed_limit.value(),
                maximum_lateral_acceleration=self.panel.max_lateral.value(),
                preview_distance=self.panel.preview.value(),
                smoothing_window=smoothing,
            ),
            acc=replace(
                base.acc,
                time_headway=self.panel.headway.value(),
                standstill_gap=self.panel.standstill_gap.value(),
                emergency_ttc=self.panel.emergency_ttc.value(),
                emergency_gap=self.panel.emergency_gap.value(),
            ),
        )
        faults = FaultParameters(
            start_time=self.panel.fault_start.value(),
            position_noise_std=self.panel.position_noise.value(),
            heading_noise_std_degrees=self.panel.heading_noise.value(),
            speed_noise_std=self.panel.speed_noise.value(),
            range_noise_std=self.panel.range_noise.value(),
            sensor_delay=self.panel.sensor_delay.value(),
            actuator_delay=self.panel.actuator_delay.value(),
            steering_bias_degrees=self.panel.steering_bias.value(),
            steering_authority=self.panel.steering_authority.value(),
            braking_efficiency=self.panel.braking_efficiency.value(),
            lateral_push_time=self.panel.push_time.value(),
            lateral_push_m=self.panel.push_size.value(),
            random_seed=self.panel.random_seed.value(),
        )
        return RobustScenario(
            controller=controller,
            faults=faults,
            name=self.panel.preset.currentText(),
            test_id=f"gui_preset_{self.panel.preset.currentIndex()}",
        )

    def apply_controls(self) -> None:
        self.timer.stop()
        self.run_button.setText("Run")
        self.simulation = RobustSimulation(self.build_scenario())
        self.road.set_simulation(self.simulation)
        self.road.show_curvature = self.panel.show_curvature.isChecked()
        self.road.show_geometry = self.panel.show_geometry.isChecked()
        self.clear_traces()
        self.status.setText("Applied controls — predict, then run")

    def clear_traces(self) -> None:
        for buffer in (
            self.speed_trace,
            self.measured_speed_trace,
            self.target_trace,
            self.gap_trace,
            self.measured_gap_trace,
            self.requested_steering_trace,
            self.applied_steering_trace,
            self.error_trace,
            self.measured_error_trace,
        ):
            buffer.clear()
        self.minimum_gap = math.inf
        self.maximum_path_error = 0.0
        self.peak_jerk = 0.0
        self.collision_seen = False
        self.state_card.set_value("READY")
        self.gap_card.set_value("—")
        self.path_card.set_value("—")
        self.lateral_card.set_value("—")
        self.update_plots()

    def toggle_run(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self.run_button.setText("Run")
            self.status.setText("Paused")
        else:
            if self.simulation.complete:
                self.reset_simulation()
            self.timer.start()
            self.run_button.setText("Pause")
            self.status.setText("Running")

    def reset_simulation(self) -> None:
        self.timer.stop()
        self.run_button.setText("Run")
        self.simulation.reset()
        self.road.snapshot = None
        self.clear_traces()
        self.road.update()
        self.status.setText("Reset")

    def single_step(self) -> None:
        self.timer.stop()
        self.run_button.setText("Run")
        self.advance(one_step=True)
        self.status.setText("Advanced one control interval")

    def advance(self, one_step: bool = False) -> None:
        count = 1 if one_step else self.steps_per_frame
        snapshot: RobustSnapshot | None = None
        for _ in range(count):
            if self.simulation.complete:
                self.timer.stop()
                self.run_button.setText("Run")
                self.status.setText("Scenario complete")
                break
            snapshot = self.simulation.step()
            self.capture(snapshot)
        if snapshot:
            self.road.snapshot = snapshot
            self.road.update()
            self.update_cards(snapshot)
            self.update_plots()

    def capture(self, snapshot: RobustSnapshot) -> None:
        self.speed_trace.append(snapshot.true_vehicle.speed)
        self.measured_speed_trace.append(snapshot.measured_vehicle.speed)
        self.target_trace.append(snapshot.selected_target_speed)
        if math.isfinite(snapshot.true_gap):
            self.gap_trace.append(snapshot.true_gap)
            self.measured_gap_trace.append(snapshot.measured_gap)
            self.minimum_gap = min(self.minimum_gap, snapshot.true_gap)
        self.requested_steering_trace.append(
            math.degrees(snapshot.requested_steering)
        )
        self.applied_steering_trace.append(
            math.degrees(snapshot.applied_steering)
        )
        self.error_trace.append(snapshot.true_cross_track_error)
        self.measured_error_trace.append(
            snapshot.lateral.cross_track_error
        )
        self.maximum_path_error = max(
            self.maximum_path_error,
            abs(snapshot.true_cross_track_error),
        )
        self.peak_jerk = max(self.peak_jerk, abs(snapshot.jerk))
        self.collision_seen = self.collision_seen or snapshot.collision

    def update_cards(self, snapshot: RobustSnapshot) -> None:
        if snapshot.fault_active:
            self.state_card.set_value("ACTIVE", RED)
        else:
            self.state_card.set_value("NOMINAL", GREEN)
        if math.isfinite(self.minimum_gap):
            gap_color = (
                RED
                if self.minimum_gap <= 0
                else AMBER
                if self.minimum_gap < 3
                else GREEN
            )
            self.gap_card.set_value(
                f"{self.minimum_gap:.1f} m",
                gap_color,
            )
        else:
            self.gap_card.set_value("N/A")
        path_color = (
            GREEN
            if self.maximum_path_error
            <= self.simulation.scenario.road_half_width
            else RED
        )
        self.path_card.set_value(
            f"{self.maximum_path_error:.2f} m",
            path_color,
        )
        lateral_color = GREEN if self.peak_jerk <= 8.0 else AMBER
        self.lateral_card.set_value(
            f"{self.peak_jerk:.1f} m/s³",
            lateral_color,
        )
        if self.collision_seen:
            self.status.setText("COLLISION — inspect headway and thresholds")

    def update_plots(self) -> None:
        for plot in (
            self.speed_plot,
            self.gap_plot,
            self.accel_plot,
            self.error_plot,
        ):
            plot.update()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--offscreen",
        action="store_true",
        help="run a short hidden smoke test and exit",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="save the rendered window to a PNG after startup",
    )
    args = parser.parse_args()
    if args.offscreen or args.screenshot:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    application = QApplication(sys.argv)
    window = Day4Window()
    window.show()

    if args.offscreen or args.screenshot:
        window.timer.start()

        def finish() -> None:
            if args.screenshot:
                args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                window.grab().save(str(args.screenshot))
            application.quit()

        QTimer.singleShot(1200, finish)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
