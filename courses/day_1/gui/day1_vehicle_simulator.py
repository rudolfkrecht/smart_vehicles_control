"""Interactive Day 1 longitudinal-control laboratory."""

from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator import OpenLoopController, PController, PIController
from simulator.realtime import RealtimeLongitudinalVehicle

try:
    from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
    from PyQt6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
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
        QVBoxLayout,
        QWidget,
    )
except ImportError as error:
    raise SystemExit(
        "PyQt6 is required. Run: python -m pip install -r requirements.txt"
    ) from error


class HistoryPlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(720, 430)
        self.clear()

    def clear(self) -> None:
        self.time: list[float] = []
        self.speed: list[float] = []
        self.target: list[float] = []
        self.command: list[float] = []
        self.update()

    def append(
        self,
        time: float,
        speed: float,
        target: float,
        command: float,
    ) -> None:
        self.time.append(time)
        self.speed.append(speed)
        self.target.append(target)
        self.command.append(command)
        self.update()

    @staticmethod
    def _polyline(
        painter: QPainter,
        rect: QRectF,
        values: list[float],
        *,
        minimum: float,
        maximum: float,
        color: str,
    ) -> None:
        if len(values) < 2:
            return
        span = max(maximum - minimum, 1e-9)
        path = QPainterPath()
        for index, value in enumerate(values):
            x = rect.left() + index / (len(values) - 1) * rect.width()
            y = rect.bottom() - (value - minimum) / span * rect.height()
            point = QPointF(x, y)
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        painter.setPen(QPen(QColor(color), 2.2))
        painter.drawPath(path)

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f7fafc"))
        speed_rect = QRectF(58, 38, self.width() - 82, 235)
        command_rect = QRectF(58, 318, self.width() - 82, 82)
        painter.setPen(QPen(QColor("#94a3b8"), 1))
        painter.drawRect(speed_rect)
        painter.drawRect(command_rect)
        painter.setPen(QColor("#17233c"))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(58, 24, "Speed response")
        painter.drawText(58, 306, "Controller command")
        painter.drawText(7, 52, "m/s")
        painter.drawText(16, 332, "1")
        painter.drawText(12, 399, "-1")
        upper_speed = max(20.0, *(self.target or [0.0]), *(self.speed or [0.0]))
        upper_speed *= 1.1
        self._polyline(
            painter,
            speed_rect,
            self.target,
            minimum=0.0,
            maximum=upper_speed,
            color="#d94841",
        )
        self._polyline(
            painter,
            speed_rect,
            self.speed,
            minimum=0.0,
            maximum=upper_speed,
            color="#1565c0",
        )
        self._polyline(
            painter,
            command_rect,
            self.command,
            minimum=-1.0,
            maximum=1.0,
            color="#ef6c00",
        )
        painter.setPen(QColor("#1565c0"))
        painter.drawText(self.width() - 230, 24, "speed")
        painter.setPen(QColor("#d94841"))
        painter.drawText(self.width() - 170, 24, "target")
        painter.setPen(QColor("#ef6c00"))
        painter.drawText(self.width() - 103, 306, "command")


class DayOneWindow(QMainWindow):
    DT = 0.05

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Smart Vehicles Control — Day 1")
        self.resize(1120, 660)
        self.vehicle = RealtimeLongitudinalVehicle(dt=self.DT)
        self.controller = OpenLoopController(0.24)
        self.timer = QTimer(self)
        self.timer.setInterval(int(self.DT * 1000))
        self.timer.timeout.connect(self.tick)
        self._build_ui()
        self.reset()

    @staticmethod
    def spin(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str = "",
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setSingleStep(step)
        widget.setDecimals(3)
        widget.setSuffix(suffix)
        return widget

    def _build_ui(self) -> None:
        central = QWidget()
        layout = QHBoxLayout(central)
        self.plot = HistoryPlot()
        layout.addWidget(self.plot, 1)

        panel = QVBoxLayout()
        settings = QGroupBox("Experiment settings")
        form = QFormLayout(settings)
        self.controller_box = QComboBox()
        self.controller_box.addItems(["Open loop", "P controller", "PI controller"])
        self.target = self.spin(0.0, 35.0, 15.0, 1.0, " m/s")
        self.initial = self.spin(0.0, 35.0, 0.0, 1.0, " m/s")
        self.duration = self.spin(2.0, 120.0, 35.0, 5.0, " s")
        self.open_command = self.spin(-1.0, 1.0, 0.24, 0.02)
        self.kp = self.spin(0.0, 2.0, 0.35, 0.05, " s/m")
        self.ki = self.spin(0.0, 1.0, 0.10, 0.02, " 1/m")
        self.hill_angle = self.spin(0.0, 15.0, 5.0, 0.5, " deg")
        self.hill_start = self.spin(0.0, 100.0, 15.0, 1.0, " s")
        self.hill_end = self.spin(0.0, 120.0, 35.0, 1.0, " s")
        self.anti_windup = QCheckBox("enabled")
        self.anti_windup.setChecked(True)
        for label, widget in (
            ("Controller", self.controller_box),
            ("Target speed", self.target),
            ("Initial speed", self.initial),
            ("Duration", self.duration),
            ("Open-loop command", self.open_command),
            ("Kp", self.kp),
            ("Ki", self.ki),
            ("Hill angle", self.hill_angle),
            ("Hill begins", self.hill_start),
            ("Hill ends", self.hill_end),
            ("Anti-windup", self.anti_windup),
        ):
            form.addRow(label, widget)
        panel.addWidget(settings)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.pause_button = QPushButton("Pause")
        self.reset_button = QPushButton("Reset")
        self.start_button.clicked.connect(self.start)
        self.pause_button.clicked.connect(self.timer.stop)
        self.reset_button.clicked.connect(self.reset)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.pause_button)
        buttons.addWidget(self.reset_button)
        panel.addLayout(buttons)

        status = QGroupBox("Live result")
        grid = QGridLayout(status)
        self.time_label = QLabel()
        self.speed_label = QLabel()
        self.error_label = QLabel()
        self.command_label = QLabel()
        self.position_label = QLabel()
        labels = [
            ("Time", self.time_label),
            ("Speed", self.speed_label),
            ("Error", self.error_label),
            ("Command", self.command_label),
            ("Position", self.position_label),
        ]
        for row, (name, value) in enumerate(labels):
            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(value, row, 1)
        panel.addWidget(status)
        panel.addStretch()
        layout.addLayout(panel)
        self.setCentralWidget(central)

    def _new_controller(self):
        selected = self.controller_box.currentText()
        if selected == "Open loop":
            return OpenLoopController(self.open_command.value())
        if selected == "P controller":
            return PController(self.kp.value())
        return PIController(
            self.kp.value(),
            self.ki.value(),
            anti_windup=self.anti_windup.isChecked(),
        )

    def reset(self) -> None:
        self.timer.stop()
        self.controller = self._new_controller()
        self.controller.reset()
        state = self.vehicle.reset(initial_speed=self.initial.value())
        self.plot.clear()
        self.plot.append(0.0, state.speed, self.target.value(), 0.0)
        self._show_state(state, 0.0)

    def start(self) -> None:
        if self.vehicle.time >= self.duration.value():
            self.reset()
        self.timer.start()

    def hill_force(self) -> float:
        active = (
            self.vehicle.time >= self.hill_start.value()
            and self.vehicle.time < self.hill_end.value()
        )
        if not active:
            return 0.0
        return (
            self.vehicle.parameters.mass
            * 9.81
            * math.sin(math.radians(self.hill_angle.value()))
        )

    def tick(self) -> None:
        target = self.target.value()
        output = self.controller.update(
            target,
            self.vehicle.speed,
            self.DT,
        )
        state = self.vehicle.step(
            output.command,
            target_speed=target,
            hill_force=self.hill_force(),
        )
        self.plot.append(
            state.time,
            state.speed,
            state.target_speed,
            state.command,
        )
        self._show_state(state, target - state.speed)
        if state.time >= self.duration.value():
            self.timer.stop()

    def _show_state(self, state, error: float) -> None:
        self.time_label.setText(f"{state.time:5.1f} s")
        self.speed_label.setText(f"{state.speed:5.2f} m/s")
        self.error_label.setText(f"{error:+5.2f} m/s")
        self.command_label.setText(f"{state.command:+5.3f}")
        self.position_label.setText(f"{state.position:6.1f} m")


def main() -> None:
    app = QApplication(sys.argv)
    window = DayOneWindow()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
