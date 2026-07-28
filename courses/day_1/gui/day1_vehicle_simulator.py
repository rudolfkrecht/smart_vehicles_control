"""Interactive PyQt simulator for Day 1 longitudinal-control concepts.

Run from the repository root:

    python day_1_longitudinal/gui/day1_vehicle_simulator.py
"""

from __future__ import annotations

import argparse
from collections import deque
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
    from PyQt5.QtGui import (
        QColor,
        QFont,
        QLinearGradient,
        QPainter,
        QPainterPath,
        QPen,
        QPolygonF,
    )
    from PyQt5.QtWidgets import (
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
except ImportError as exc:  # pragma: no cover - helpful message for new setups
    raise SystemExit(
        "PyQt5 is required for the graphical simulator.\n"
        "Install the course dependencies with:\n"
        "    python -m pip install -r requirements.txt"
    ) from exc

from simulator import OpenLoopController, PController, PIController  # noqa: E402
from simulator.realtime import (  # noqa: E402
    RealtimeLongitudinalVehicle,
    RealtimeState,
)


BLUE = QColor("#1565C0")
LIGHT_BLUE = QColor("#42A5F5")
ORANGE = QColor("#EF6C00")
GREEN = QColor("#2E7D32")
RED = QColor("#C62828")
INK = QColor("#18324A")
MUTED = QColor("#607D8B")
PANEL = QColor("#FFFFFF")
BACKGROUND = QColor("#EEF4F8")
GRID = QColor("#D8E3EA")


class RoadView(QWidget):
    """Side-view road scene whose markings move with vehicle position."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.position = 0.0
        self.speed = 0.0
        self.acceleration = 0.0
        self.hill_active = False
        self.target_speed = 15.0

    def set_state(self, state: RealtimeState | None, target_speed: float) -> None:
        self.target_speed = target_speed
        if state is None:
            self.position = 0.0
            self.speed = 0.0
            self.acceleration = 0.0
            self.hill_active = False
        else:
            self.position = state.position
            self.speed = state.speed
            self.acceleration = state.acceleration
            self.hill_active = state.hill_force > 0.0
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        width = float(self.width())
        height = float(self.height())

        sky = QLinearGradient(0.0, 0.0, 0.0, height)
        sky.setColorAt(0.0, QColor("#D9F0FF"))
        sky.setColorAt(0.68, QColor("#F8FCFF"))
        sky.setColorAt(1.0, QColor("#E8F2E1"))
        painter.fillRect(self.rect(), sky)

        self._draw_background(painter, width, height)
        self._draw_road(painter, width, height)
        self._draw_vehicle(painter, width, height)
        self._draw_scene_labels(painter, width)

    def _draw_background(self, painter: QPainter, width: float, height: float) -> None:
        horizon = height * 0.56
        offset = (self.position * 0.45) % 280.0
        painter.setPen(Qt.NoPen)

        painter.setBrush(QColor("#B9D8C0"))
        mountains = QPainterPath()
        mountains.moveTo(0.0, horizon + 10.0)
        for index in range(-1, 7):
            x = index * 220.0 - offset
            mountains.lineTo(x + 70.0, horizon - 65.0)
            mountains.lineTo(x + 145.0, horizon - 15.0)
            mountains.lineTo(x + 220.0, horizon - 75.0)
        mountains.lineTo(width, horizon + 25.0)
        mountains.lineTo(0.0, horizon + 25.0)
        painter.drawPath(mountains)

        painter.setBrush(QColor("#78B47E"))
        painter.drawRect(QRectF(0.0, horizon, width, height - horizon))

        # Simple moving trees create an intuitive sense of forward speed.
        tree_spacing = 170.0
        tree_offset = (self.position * 2.0) % tree_spacing
        for index in range(-1, int(width / tree_spacing) + 2):
            x = index * tree_spacing - tree_offset
            painter.setBrush(QColor("#6D4C41"))
            painter.drawRect(QRectF(x + 12.0, horizon - 8.0, 7.0, 30.0))
            painter.setBrush(QColor("#388E3C"))
            painter.drawEllipse(QRectF(x - 3.0, horizon - 42.0, 38.0, 42.0))

    def _road_y(self, x: float, width: float, height: float) -> float:
        base = height * 0.77
        if not self.hill_active:
            return base
        # The visual slope indicates a load disturbance; the dynamics still use
        # the explicitly displayed hill force.
        return base - 0.075 * (x - width * 0.5)

    def _draw_road(self, painter: QPainter, width: float, height: float) -> None:
        road_top_left = self._road_y(0.0, width, height)
        road_top_right = self._road_y(width, width, height)
        road = QPolygonF(
            [
                QPointF(0.0, road_top_left),
                QPointF(width, road_top_right),
                QPointF(width, height),
                QPointF(0.0, height),
            ]
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#455A64"))
        painter.drawPolygon(road)

        painter.setPen(QPen(QColor("#ECEFF1"), 3.0))
        painter.drawLine(
            QPointF(0.0, road_top_left),
            QPointF(width, road_top_right),
        )

        stripe_y_shift = height * 0.13
        stripe_length = 68.0
        stripe_gap = 52.0
        period = stripe_length + stripe_gap
        offset = (self.position * 3.2) % period
        painter.setPen(QPen(QColor("#FFD54F"), 6.0, Qt.SolidLine))
        for index in range(-1, int(width / period) + 2):
            x1 = index * period - offset
            x2 = x1 + stripe_length
            y1 = self._road_y(x1, width, height) + stripe_y_shift
            y2 = self._road_y(x2, width, height) + stripe_y_shift
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def _draw_vehicle(self, painter: QPainter, width: float, height: float) -> None:
        car_x = width * 0.37
        road_y = self._road_y(car_x, width, height)
        car_y = road_y - 48.0
        angle = -4.3 if self.hill_active else 0.0

        painter.save()
        painter.translate(car_x, car_y + 26.0)
        painter.rotate(angle)
        painter.translate(-car_x, -(car_y + 26.0))

        shadow = QRectF(car_x - 78.0, car_y + 34.0, 164.0, 19.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(20, 35, 45, 55))
        painter.drawEllipse(shadow)

        body = QPainterPath()
        body.moveTo(car_x - 82.0, car_y + 24.0)
        body.quadTo(car_x - 79.0, car_y + 2.0, car_x - 58.0, car_y - 1.0)
        body.lineTo(car_x - 29.0, car_y - 30.0)
        body.quadTo(car_x - 22.0, car_y - 37.0, car_x - 10.0, car_y - 37.0)
        body.lineTo(car_x + 31.0, car_y - 37.0)
        body.quadTo(car_x + 43.0, car_y - 35.0, car_x + 58.0, car_y - 10.0)
        body.lineTo(car_x + 78.0, car_y - 3.0)
        body.quadTo(car_x + 88.0, car_y + 4.0, car_x + 86.0, car_y + 25.0)
        body.closeSubpath()
        painter.setPen(QPen(QColor("#0D47A1"), 2.0))
        painter.setBrush(BLUE)
        painter.drawPath(body)

        windows = QPolygonF(
            [
                QPointF(car_x - 25.0, car_y - 28.0),
                QPointF(car_x - 7.0, car_y - 28.0),
                QPointF(car_x - 7.0, car_y - 7.0),
                QPointF(car_x - 47.0, car_y - 7.0),
            ]
        )
        painter.setBrush(QColor("#B3E5FC"))
        painter.setPen(QPen(QColor("#E3F2FD"), 1.0))
        painter.drawPolygon(windows)
        painter.drawPolygon(
            QPolygonF(
                [
                    QPointF(car_x + 1.0, car_y - 28.0),
                    QPointF(car_x + 27.0, car_y - 28.0),
                    QPointF(car_x + 48.0, car_y - 7.0),
                    QPointF(car_x + 1.0, car_y - 7.0),
                ]
            )
        )

        painter.setBrush(QColor("#263238"))
        painter.setPen(QPen(QColor("#102027"), 2.0))
        for wheel_x in (car_x - 50.0, car_x + 51.0):
            painter.drawEllipse(QRectF(wheel_x - 15.0, car_y + 13.0, 30.0, 30.0))
            painter.setBrush(QColor("#B0BEC5"))
            painter.drawEllipse(QRectF(wheel_x - 7.0, car_y + 21.0, 14.0, 14.0))
            painter.setBrush(QColor("#263238"))

        painter.setBrush(QColor("#FFECB3"))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(
            QRectF(car_x + 76.0, car_y + 4.0, 10.0, 10.0),
            3.0,
            3.0,
        )
        painter.restore()

    def _draw_scene_labels(self, painter: QPainter, width: float) -> None:
        del width
        painter.setPen(INK)
        painter.setFont(QFont("Segoe UI", 11, QFont.DemiBold))
        painter.drawText(
            QRectF(18.0, 14.0, 260.0, 28.0),
            Qt.AlignLeft,
            f"Distance  {self.position:7.1f} m",
        )
        painter.setFont(QFont("Segoe UI", 10))
        state_text = "HILL DISTURBANCE" if self.hill_active else "FLAT ROAD"
        state_color = ORANGE if self.hill_active else GREEN
        painter.setPen(state_color)
        painter.drawText(
            QRectF(18.0, 42.0, 260.0, 24.0),
            Qt.AlignLeft,
            state_text,
        )


class FeedbackStrip(QWidget):
    """Live visual representation of the closed feedback loop."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(105)
        self.values = {
            "Reference": "15.00 m/s",
            "Error": "15.00 m/s",
            "Controller": "PI",
            "Command": "+0.00",
            "Vehicle": "0.00 m/s",
        }

    def set_values(
        self,
        *,
        target: float,
        error: float,
        controller: str,
        command: float,
        speed: float,
    ) -> None:
        self.values = {
            "Reference": f"{target:.2f} m/s",
            "Error": f"{error:+.2f} m/s",
            "Controller": controller,
            "Command": f"{command:+.2f}",
            "Vehicle": f"{speed:.2f} m/s",
        }
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        labels = list(self.values)
        margin = 14.0
        arrow_space = 25.0
        box_width = (
            self.width() - 2.0 * margin - arrow_space * (len(labels) - 1)
        ) / len(labels)
        box_height = 68.0
        y = 12.0

        for index, label in enumerate(labels):
            x = margin + index * (box_width + arrow_space)
            color = BLUE
            if label == "Error":
                color = ORANGE
            elif label == "Command":
                color = GREEN

            painter.setPen(QPen(color, 1.5))
            painter.setBrush(QColor(color.red(), color.green(), color.blue(), 18))
            painter.drawRoundedRect(QRectF(x, y, box_width, box_height), 9.0, 9.0)

            painter.setPen(MUTED)
            painter.setFont(QFont("Segoe UI", 8))
            painter.drawText(
                QRectF(x + 5.0, y + 8.0, box_width - 10.0, 18.0),
                Qt.AlignCenter,
                label.upper(),
            )
            painter.setPen(INK)
            painter.setFont(QFont("Segoe UI", 11, QFont.DemiBold))
            painter.drawText(
                QRectF(x + 5.0, y + 31.0, box_width - 10.0, 24.0),
                Qt.AlignCenter,
                self.values[label],
            )

            if index < len(labels) - 1:
                arrow_x1 = x + box_width + 4.0
                arrow_x2 = arrow_x1 + arrow_space - 8.0
                arrow_y = y + box_height / 2.0
                painter.setPen(QPen(MUTED, 1.8))
                painter.drawLine(
                    QPointF(arrow_x1, arrow_y),
                    QPointF(arrow_x2, arrow_y),
                )
                painter.drawLine(
                    QPointF(arrow_x2 - 5.0, arrow_y - 4.0),
                    QPointF(arrow_x2, arrow_y),
                )
                painter.drawLine(
                    QPointF(arrow_x2 - 5.0, arrow_y + 4.0),
                    QPointF(arrow_x2, arrow_y),
                )

        painter.setPen(QPen(BLUE, 1.3, Qt.DashLine))
        feedback_y = y + box_height + 12.0
        vehicle_center = (
            margin + 4.0 * (box_width + arrow_space) + box_width / 2.0
        )
        error_center = margin + box_width + arrow_space + box_width / 2.0
        painter.drawLine(
            QPointF(vehicle_center, y + box_height),
            QPointF(vehicle_center, feedback_y),
        )
        painter.drawLine(
            QPointF(vehicle_center, feedback_y),
            QPointF(error_center, feedback_y),
        )
        painter.drawLine(
            QPointF(error_center, feedback_y),
            QPointF(error_center, y + box_height),
        )
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(
            QRectF(error_center + 8.0, feedback_y - 16.0, 120.0, 16.0),
            Qt.AlignLeft,
            "measured speed",
        )


class HistoryPlot(QWidget):
    """Dependency-free QPainter plot of live speed and command histories."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(250)
        self.times: deque[float] = deque(maxlen=500)
        self.speeds: deque[float] = deque(maxlen=500)
        self.targets: deque[float] = deque(maxlen=500)
        self.commands: deque[float] = deque(maxlen=500)
        self.errors: deque[float] = deque(maxlen=500)
        self.window_seconds = 20.0

    def clear(self) -> None:
        self.times.clear()
        self.speeds.clear()
        self.targets.clear()
        self.commands.clear()
        self.errors.clear()
        self.update()

    def append(self, state: RealtimeState) -> None:
        self.times.append(state.time)
        self.speeds.append(state.speed)
        self.targets.append(state.target_speed)
        self.commands.append(state.command)
        self.errors.append(state.error)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), PANEL)

        margin_left = 55.0
        margin_right = 18.0
        margin_top = 22.0
        gap = 30.0
        plot_height = (self.height() - margin_top - 32.0 - gap) / 2.0
        speed_rect = QRectF(
            margin_left,
            margin_top,
            self.width() - margin_left - margin_right,
            plot_height,
        )
        command_rect = QRectF(
            margin_left,
            margin_top + plot_height + gap,
            self.width() - margin_left - margin_right,
            plot_height,
        )

        speed_max = max(
            20.0,
            (max(self.targets, default=15.0) + 4.0),
            (max(self.speeds, default=0.0) + 2.0),
        )
        self._draw_axes(painter, speed_rect, 0.0, speed_max, "Speed [m/s]")
        self._draw_axes(painter, command_rect, -1.0, 1.0, "Command [-]")

        if len(self.times) < 2:
            painter.setPen(MUTED)
            painter.setFont(QFont("Segoe UI", 10))
            painter.drawText(
                speed_rect,
                Qt.AlignCenter,
                "Press Start to generate live histories",
            )
            return

        t_max = self.times[-1]
        t_min = max(0.0, t_max - self.window_seconds)
        self._draw_curve(
            painter,
            speed_rect,
            self.times,
            self.targets,
            t_min,
            t_max,
            0.0,
            speed_max,
            QColor("#263238"),
            dashed=True,
        )
        self._draw_curve(
            painter,
            speed_rect,
            self.times,
            self.speeds,
            t_min,
            t_max,
            0.0,
            speed_max,
            BLUE,
        )
        self._draw_curve(
            painter,
            command_rect,
            self.times,
            self.commands,
            t_min,
            t_max,
            -1.0,
            1.0,
            ORANGE,
        )

        painter.setFont(QFont("Segoe UI", 8))
        painter.setPen(QColor("#263238"))
        painter.drawText(
            QRectF(speed_rect.right() - 170.0, speed_rect.top() + 3.0, 165.0, 16.0),
            Qt.AlignRight,
            "— speed    -- target",
        )
        painter.setPen(MUTED)
        painter.drawText(
            QRectF(command_rect.left(), command_rect.bottom() + 5.0, command_rect.width(), 18.0),
            Qt.AlignCenter,
            f"Time window: {t_min:.1f}–{t_max:.1f} s",
        )

    def _draw_axes(
        self,
        painter: QPainter,
        rect: QRectF,
        y_min: float,
        y_max: float,
        label: str,
    ) -> None:
        painter.setFont(QFont("Segoe UI", 8))
        for index in range(5):
            fraction = index / 4.0
            y = rect.bottom() - fraction * rect.height()
            value = y_min + fraction * (y_max - y_min)
            painter.setPen(QPen(GRID, 1.0))
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.setPen(MUTED)
            painter.drawText(
                QRectF(2.0, y - 9.0, rect.left() - 8.0, 18.0),
                Qt.AlignRight | Qt.AlignVCenter,
                f"{value:.0f}" if y_max > 2.0 else f"{value:+.1f}",
            )
        painter.setPen(QPen(MUTED, 1.0))
        painter.drawRect(rect)
        painter.save()
        painter.translate(14.0, rect.center().y())
        painter.rotate(-90.0)
        painter.drawText(
            QRectF(-rect.height() / 2.0, -10.0, rect.height(), 18.0),
            Qt.AlignCenter,
            label,
        )
        painter.restore()

    def _draw_curve(
        self,
        painter: QPainter,
        rect: QRectF,
        times: deque[float],
        values: deque[float],
        t_min: float,
        t_max: float,
        y_min: float,
        y_max: float,
        color: QColor,
        *,
        dashed: bool = False,
    ) -> None:
        time_span = max(t_max - t_min, 1.0)
        y_span = max(y_max - y_min, 1e-9)
        path = QPainterPath()
        started = False
        for time, value in zip(times, values, strict=True):
            if time < t_min:
                continue
            x = rect.left() + (time - t_min) / time_span * rect.width()
            clipped = min(max(value, y_min), y_max)
            y = rect.bottom() - (clipped - y_min) / y_span * rect.height()
            if not started:
                path.moveTo(x, y)
                started = True
            else:
                path.lineTo(x, y)
        style = Qt.DashLine if dashed else Qt.SolidLine
        painter.setPen(QPen(color, 2.0, style))
        painter.drawPath(path)


class ValueCard(QFrame):
    """Small live numeric display used above the scene."""

    def __init__(self, title: str, value: str, color: str) -> None:
        super().__init__()
        self.setObjectName("valueCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 8, 13, 8)
        layout.setSpacing(2)
        title_label = QLabel(title.upper())
        title_label.setObjectName("cardTitle")
        self.value_label = QLabel(value)
        self.value_label.setObjectName("cardValue")
        self.value_label.setStyleSheet(f"color: {color};")
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)

    def set_value(self, value: str) -> None:
        self.value_label.setText(value)


class DayOneSimulator(QMainWindow):
    """Main interactive teaching simulator window."""

    DT = 0.05

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Day 1 • Longitudinal Vehicle Control Lab")
        self.resize(1480, 920)
        self.setMinimumSize(1120, 720)

        self.vehicle = RealtimeLongitudinalVehicle()
        self.controller = PIController(0.35, 0.10, anti_windup=True)
        self.last_state: RealtimeState | None = None
        self.running = False

        self.timer = QTimer(self)
        self.timer.setInterval(round(self.DT * 1000.0))
        self.timer.timeout.connect(self._advance_simulation)

        self._build_ui()
        self._apply_style()
        self._on_controller_changed()
        self._refresh_displays()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(18, 14, 18, 16)
        outer.setSpacing(12)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title = QLabel("LONGITUDINAL CONTROL LAB")
        title.setObjectName("mainTitle")
        subtitle = QLabel(
            "Change the controller and disturbance while the vehicle is moving"
        )
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()

        self.start_button = QPushButton("▶  Start")
        self.start_button.setObjectName("startButton")
        self.start_button.clicked.connect(self._toggle_running)
        self.reset_button = QPushButton("↺  Reset")
        self.reset_button.clicked.connect(self.reset_simulation)
        header.addWidget(self.start_button)
        header.addWidget(self.reset_button)
        outer.addLayout(header)

        cards = QHBoxLayout()
        cards.setSpacing(9)
        self.time_card = ValueCard("Simulation time", "0.0 s", "#1565C0")
        self.speed_card = ValueCard("Vehicle speed", "0.00 m/s", "#1565C0")
        self.target_card = ValueCard("Target speed", "15.00 m/s", "#263238")
        self.error_card = ValueCard("Speed error", "+15.00 m/s", "#EF6C00")
        self.command_card = ValueCard("Command", "+0.00", "#2E7D32")
        self.accel_card = ValueCard("Acceleration", "+0.00 m/s²", "#6A1B9A")
        for card in (
            self.time_card,
            self.speed_card,
            self.target_card,
            self.error_card,
            self.command_card,
            self.accel_card,
        ):
            cards.addWidget(card)
        outer.addLayout(cards)

        content = QHBoxLayout()
        content.setSpacing(14)
        left = QVBoxLayout()
        left.setSpacing(10)
        self.road_view = RoadView()
        self.feedback_strip = FeedbackStrip()
        self.history_plot = HistoryPlot()
        left.addWidget(self.road_view, 5)
        left.addWidget(self.feedback_strip, 0)
        left.addWidget(self.history_plot, 5)
        content.addLayout(left, 4)
        content.addWidget(self._build_control_panel(), 0)
        outer.addLayout(content, 1)

        self.status_label = QLabel(
            "Ready • PI feedback controller • Scheduled hill: 12–24 s"
        )
        self.status_label.setObjectName("status")
        outer.addWidget(self.status_label)

    def _build_control_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("controlPanel")
        panel.setFixedWidth(350)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(11)

        controller_group = QGroupBox("Controller")
        controller_form = QFormLayout(controller_group)
        self.controller_combo = QComboBox()
        self.controller_combo.addItems(
            ["Open loop", "P controller", "PI controller"]
        )
        self.controller_combo.setCurrentIndex(2)
        self.controller_combo.currentIndexChanged.connect(
            self._on_controller_changed
        )
        self.open_loop_spin = self._spin(-1.0, 1.0, 0.35, 0.05, 2)
        self.kp_spin = self._spin(0.0, 4.0, 0.35, 0.05, 2)
        self.ki_spin = self._spin(0.0, 1.0, 0.10, 0.01, 2)
        self.anti_windup_check = QCheckBox("Anti-windup")
        self.anti_windup_check.setChecked(True)
        self.open_loop_spin.valueChanged.connect(self._update_controller_parameters)
        self.kp_spin.valueChanged.connect(self._update_controller_parameters)
        self.ki_spin.valueChanged.connect(self._update_controller_parameters)
        self.anti_windup_check.toggled.connect(self._update_controller_parameters)
        controller_form.addRow("Mode", self.controller_combo)
        controller_form.addRow("Fixed command", self.open_loop_spin)
        controller_form.addRow("Kp", self.kp_spin)
        controller_form.addRow("Ki", self.ki_spin)
        controller_form.addRow("", self.anti_windup_check)
        layout.addWidget(controller_group)

        reference_group = QGroupBox("Reference")
        reference_form = QFormLayout(reference_group)
        self.target_spin = self._spin(0.0, 30.0, 15.0, 1.0, 1, " m/s")
        self.target_spin.valueChanged.connect(self._reference_changed)
        self.target_kmh = QLabel("54.0 km/h")
        self.target_kmh.setObjectName("calculatedValue")
        reference_form.addRow("Target speed", self.target_spin)
        reference_form.addRow("Equivalent", self.target_kmh)
        layout.addWidget(reference_group)

        hill_group = QGroupBox("Road disturbance")
        hill_form = QFormLayout(hill_group)
        self.hill_enabled = QCheckBox("Enable scheduled hill")
        self.hill_enabled.setChecked(True)
        self.hill_start_spin = self._spin(0.0, 120.0, 12.0, 1.0, 1, " s")
        self.hill_end_spin = self._spin(0.5, 180.0, 24.0, 1.0, 1, " s")
        self.hill_force_spin = self._spin(
            0.0,
            6000.0,
            1500.0,
            100.0,
            0,
            " N",
        )
        self.hill_enabled.toggled.connect(self._disturbance_changed)
        self.hill_start_spin.valueChanged.connect(self._disturbance_changed)
        self.hill_end_spin.valueChanged.connect(self._disturbance_changed)
        self.hill_force_spin.valueChanged.connect(self._disturbance_changed)
        hill_form.addRow("", self.hill_enabled)
        hill_form.addRow("Start time", self.hill_start_spin)
        hill_form.addRow("End time", self.hill_end_spin)
        hill_form.addRow("Opposing force", self.hill_force_spin)
        layout.addWidget(hill_group)

        presets_group = QGroupBox("Prepared comparisons")
        presets_layout = QGridLayout(presets_group)
        flat_button = QPushButton("Flat road")
        p_button = QPushButton("P + hill")
        pi_button = QPushButton("PI + hill")
        windup_button = QPushButton("Windup case")
        flat_button.clicked.connect(self._preset_flat)
        p_button.clicked.connect(self._preset_p)
        pi_button.clicked.connect(self._preset_pi)
        windup_button.clicked.connect(self._preset_windup)
        presets_layout.addWidget(flat_button, 0, 0)
        presets_layout.addWidget(p_button, 0, 1)
        presets_layout.addWidget(pi_button, 1, 0)
        presets_layout.addWidget(windup_button, 1, 1)
        layout.addWidget(presets_group)

        question = QLabel(
            "<b>Predict before running</b><br>"
            "When the hill begins, which controller will restore zero "
            "steady-state speed error?"
        )
        question.setWordWrap(True)
        question.setObjectName("questionBox")
        layout.addWidget(question)
        layout.addStretch()
        return panel

    @staticmethod
    def _spin(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        decimals: int,
        suffix: str = "",
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #EEF4F8;
                color: #18324A;
                font-family: "Segoe UI", "Arial";
                font-size: 10pt;
            }
            QLabel#mainTitle {
                color: #0D47A1;
                font-size: 20pt;
                font-weight: 700;
                letter-spacing: 1px;
            }
            QLabel#subtitle { color: #607D8B; font-size: 10pt; }
            QFrame#valueCard, QFrame#controlPanel {
                background: white;
                border: 1px solid #D8E3EA;
                border-radius: 9px;
            }
            QLabel#cardTitle {
                color: #78909C;
                font-size: 7.5pt;
                font-weight: 600;
            }
            QLabel#cardValue { font-size: 14pt; font-weight: 700; }
            QGroupBox {
                background: #FAFCFD;
                border: 1px solid #D8E3EA;
                border-radius: 7px;
                margin-top: 10px;
                padding-top: 8px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 9px;
                padding: 0 4px;
                color: #0D47A1;
            }
            QPushButton {
                background: white;
                border: 1px solid #B9CAD5;
                border-radius: 6px;
                padding: 7px 11px;
                font-weight: 600;
            }
            QPushButton:hover { background: #E3F2FD; border-color: #42A5F5; }
            QPushButton#startButton {
                color: white;
                background: #1565C0;
                border-color: #1565C0;
                min-width: 88px;
            }
            QPushButton#startButton:hover { background: #0D47A1; }
            QComboBox, QDoubleSpinBox {
                background: white;
                border: 1px solid #B9CAD5;
                border-radius: 4px;
                padding: 4px;
                min-height: 21px;
            }
            QLabel#calculatedValue { color: #607D8B; }
            QLabel#questionBox {
                background: #FFF8E1;
                border: 1px solid #FFE082;
                border-radius: 7px;
                padding: 9px;
                color: #5D4037;
            }
            QLabel#status {
                background: #E3F2FD;
                border-radius: 5px;
                color: #0D47A1;
                padding: 6px 10px;
            }
            """
        )

    def _toggle_running(self) -> None:
        self.running = not self.running
        if self.running:
            self.timer.start()
            self.start_button.setText("Ⅱ  Pause")
            self.status_label.setText("Running • change parameters at any time")
        else:
            self.timer.stop()
            self.start_button.setText("▶  Start")
            self.status_label.setText("Paused • inspect the live histories")

    def reset_simulation(self) -> None:
        self.timer.stop()
        self.running = False
        self.start_button.setText("▶  Start")
        self.vehicle.reset()
        self.controller.reset()
        self.last_state = None
        self.history_plot.clear()
        self.status_label.setText("Reset complete • ready to run")
        self._refresh_displays()

    def _on_controller_changed(self) -> None:
        mode = self.controller_combo.currentIndex()
        if mode == 0:
            self.controller = OpenLoopController(self.open_loop_spin.value())
        elif mode == 1:
            self.controller = PController(self.kp_spin.value())
        else:
            self.controller = PIController(
                self.kp_spin.value(),
                self.ki_spin.value(),
                anti_windup=self.anti_windup_check.isChecked(),
            )
        self.open_loop_spin.setEnabled(mode == 0)
        self.kp_spin.setEnabled(mode in (1, 2))
        self.ki_spin.setEnabled(mode == 2)
        self.anti_windup_check.setEnabled(mode == 2)
        self._refresh_displays()

    def _update_controller_parameters(self) -> None:
        if isinstance(self.controller, OpenLoopController):
            self.controller.command = self.open_loop_spin.value()
        elif isinstance(self.controller, PController):
            self.controller.kp = self.kp_spin.value()
        elif isinstance(self.controller, PIController):
            self.controller.kp = self.kp_spin.value()
            self.controller.ki = self.ki_spin.value()
            self.controller.anti_windup = self.anti_windup_check.isChecked()

    def _reference_changed(self) -> None:
        self.target_kmh.setText(f"{self.target_spin.value() * 3.6:.1f} km/h")
        self._refresh_displays()

    def _disturbance_changed(self) -> None:
        enabled = self.hill_enabled.isChecked()
        self.hill_start_spin.setEnabled(enabled)
        self.hill_end_spin.setEnabled(enabled)
        self.hill_force_spin.setEnabled(enabled)
        self._refresh_displays()

    def _current_hill_force(self) -> float:
        if not self.hill_enabled.isChecked():
            return 0.0
        if (
            self.vehicle.time >= self.hill_start_spin.value()
            and self.vehicle.time < self.hill_end_spin.value()
        ):
            return self.hill_force_spin.value()
        return 0.0

    def _advance_simulation(self) -> None:
        self._step_once(refresh=True)

    def _step_once(self, *, refresh: bool) -> None:
        self.last_state = self.vehicle.step(
            self.controller,
            target_speed=self.target_spin.value(),
            dt=self.DT,
            hill_force=self._current_hill_force(),
        )
        self.history_plot.append(self.last_state)
        if refresh:
            self._refresh_displays()

    def _refresh_displays(self) -> None:
        state = self.last_state
        target = self.target_spin.value()
        speed = state.speed if state else 0.0
        error = state.error if state else target - speed
        command = state.command if state else 0.0
        acceleration = state.acceleration if state else 0.0
        time = state.time if state else 0.0
        mode = ("OPEN", "P", "PI")[self.controller_combo.currentIndex()]

        self.time_card.set_value(f"{time:.1f} s")
        self.speed_card.set_value(f"{speed:.2f} m/s")
        self.target_card.set_value(f"{target:.2f} m/s")
        self.error_card.set_value(f"{error:+.2f} m/s")
        self.command_card.set_value(f"{command:+.2f}")
        self.accel_card.set_value(f"{acceleration:+.2f} m/s²")
        self.road_view.set_state(state, target)
        self.feedback_strip.set_values(
            target=target,
            error=error,
            controller=mode,
            command=command,
            speed=speed,
        )

    def _preset_flat(self) -> None:
        self.controller_combo.setCurrentIndex(2)
        self.kp_spin.setValue(0.35)
        self.ki_spin.setValue(0.10)
        self.anti_windup_check.setChecked(True)
        self.hill_enabled.setChecked(False)
        self.reset_simulation()

    def _preset_p(self) -> None:
        self.controller_combo.setCurrentIndex(1)
        self.kp_spin.setValue(0.35)
        self.hill_enabled.setChecked(True)
        self.hill_start_spin.setValue(12.0)
        self.hill_end_spin.setValue(24.0)
        self.hill_force_spin.setValue(1500.0)
        self.reset_simulation()

    def _preset_pi(self) -> None:
        self.controller_combo.setCurrentIndex(2)
        self.kp_spin.setValue(0.35)
        self.ki_spin.setValue(0.10)
        self.anti_windup_check.setChecked(True)
        self.hill_enabled.setChecked(True)
        self.hill_start_spin.setValue(12.0)
        self.hill_end_spin.setValue(24.0)
        self.hill_force_spin.setValue(1500.0)
        self.reset_simulation()

    def _preset_windup(self) -> None:
        self.controller_combo.setCurrentIndex(2)
        self.kp_spin.setValue(0.30)
        self.ki_spin.setValue(0.14)
        self.anti_windup_check.setChecked(False)
        self.hill_enabled.setChecked(True)
        self.hill_start_spin.setValue(8.0)
        self.hill_end_spin.setValue(20.0)
        self.hill_force_spin.setValue(5000.0)
        self.target_spin.setValue(15.0)
        self.reset_simulation()

    def simulate_for_demo(self, seconds: float) -> None:
        """Advance quickly without the real-time timer (used for screenshots)."""

        self.reset_simulation()
        steps = max(0, int(math.ceil(seconds / self.DT)))
        for _ in range(steps):
            self._step_once(refresh=False)
        self._refresh_displays()
        self.status_label.setText(
            "Demonstration snapshot • PI controller recovering on the hill"
        )


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="Save a GUI screenshot and exit (used to build the course website).",
    )
    parser.add_argument(
        "--demo-seconds",
        type=float,
        default=18.0,
        help="Simulation time shown in a generated screenshot.",
    )
    return parser.parse_args()


def main() -> None:
    args = _arguments()
    app = QApplication(sys.argv[:1])
    window = DayOneSimulator()
    window.show()

    if args.screenshot is not None:
        window.simulate_for_demo(args.demo_seconds)

        def save_and_exit() -> None:
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(args.screenshot)):
                raise RuntimeError(f"Could not save screenshot: {args.screenshot}")
            print(f"Saved screenshot: {args.screenshot}")
            app.quit()

        QTimer.singleShot(350, save_and_exit)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
