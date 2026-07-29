"""Simulation orchestration, logging and performance metrics."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Protocol

from .controllers import ReferenceController
from .model import ControlCommand, Observation, VehicleModel
from .track import ClosedHighwayTrack


class Controller(Protocol):
    def reset(self) -> None: ...

    def update(self, observation: Observation) -> ControlCommand: ...


class Simulation:
    def __init__(
        self,
        controller: Controller | None = None,
        *,
        target_speed: float = 15.0,
        track: ClosedHighwayTrack | None = None,
        vehicle: VehicleModel | None = None,
    ) -> None:
        self.track = track or ClosedHighwayTrack()
        self.vehicle = vehicle or VehicleModel()
        self.controller: Controller = controller or ReferenceController()
        self.target_speed = float(target_speed)
        self.time = 0.0
        self.track_progress = 0.0
        self.last_track_s = 0.0
        self.lap = 0
        self.history: list[dict[str, float | int | bool]] = []
        self.last_observation: Observation | None = None
        self.last_command = ControlCommand()
        self.reset()

    def reset(self) -> None:
        start_s = 8.0
        start = self.track.pose_at(start_s, self.track.lane_offset)
        self.vehicle.reset(start.x, start.y, start.z, start.heading)
        self.controller.reset()
        self.time = 0.0
        self.track_progress = start_s
        self.last_track_s = start_s
        self.lap = 0
        self.history = []
        self.last_command = ControlCommand()
        self.last_observation = self.observe(1.0 / 60.0)

    def set_controller(self, controller: Controller) -> None:
        self.controller = controller
        self.reset()

    def observe(self, dt: float) -> Observation:
        state = self.vehicle.state
        tracking = self.track.nearest(state.x, state.y, state.heading)
        preview_distance = max(8.0, 0.75 * state.speed)
        preview = self.track.pose_at(
            tracking.pose.s + preview_distance,
            self.track.lane_offset,
        )
        grade = 100.0 * math.tan(tracking.pose.slope)
        return Observation(
            time=self.time,
            dt=dt,
            x=state.x,
            y=state.y,
            z=state.z,
            heading=state.heading,
            speed=state.speed,
            acceleration=state.acceleration,
            target_speed=self.target_speed,
            track_s=tracking.pose.s,
            track_heading=tracking.pose.heading,
            cross_track_error=tracking.cross_track_error,
            heading_error=tracking.heading_error,
            slope_radians=tracking.pose.slope,
            grade_percent=grade,
            preview_x=preview.x,
            preview_y=preview.y,
            preview_z=preview.z,
            lap=self.lap,
            off_road=(
                abs(tracking.cross_track_error)
                > 0.5 * self.track.road_width
            ),
        )

    def step(
        self,
        dt: float,
        command_override: ControlCommand | None = None,
    ) -> Observation:
        before = self.observe(dt)
        command = (
            command_override
            if command_override is not None
            else self.controller.update(before)
        )
        command = command.limited(self.vehicle.parameters.maximum_steering)
        self.vehicle.step(
            command,
            before.slope_radians,
            dt,
            off_road=before.off_road,
        )

        after_tracking = self.track.nearest(
            self.vehicle.state.x,
            self.vehicle.state.y,
            self.vehicle.state.heading,
        )
        self.vehicle.state.z = after_tracking.pose.z
        delta_s = after_tracking.pose.s - self.last_track_s
        half_length = 0.5 * self.track.total_length
        if delta_s < -half_length:
            delta_s += self.track.total_length
        elif delta_s > half_length:
            delta_s -= self.track.total_length
        self.track_progress += delta_s
        self.last_track_s = after_tracking.pose.s
        self.lap = max(0, int(self.track_progress / self.track.total_length))
        self.time += dt

        after = self.observe(dt)
        self.last_observation = after
        self.last_command = command
        self.history.append(
            {
                "time_s": self.time,
                "x_m": after.x,
                "y_m": after.y,
                "z_m": after.z,
                "speed_mps": after.speed,
                "target_speed_mps": after.target_speed,
                "acceleration_mps2": after.acceleration,
                "throttle": command.throttle,
                "brake": command.brake,
                "steering_rad": command.steering,
                "cross_track_error_m": after.cross_track_error,
                "heading_error_rad": after.heading_error,
                "slope_deg": math.degrees(after.slope_radians),
                "grade_percent": after.grade_percent,
                "track_s_m": after.track_s,
                "lap": after.lap,
                "off_road": after.off_road,
            }
        )
        return after

    def run(self, duration: float, dt: float = 1.0 / 60.0) -> None:
        steps = max(0, int(math.ceil(duration / dt)))
        for _ in range(steps):
            self.step(dt)

    def save_csv(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not self.history:
            raise ValueError("The simulation history is empty.")
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.history[0]))
            writer.writeheader()
            writer.writerows(self.history)
        return output

    def metrics(self) -> dict[str, float | None]:
        if not self.history:
            return {
                "rise_time_s": None,
                "overshoot_percent": None,
                "final_error_mps": None,
                "rmse_mps": None,
                "maximum_cross_track_error_m": None,
            }

        target = self.target_speed
        speeds = [float(row["speed_mps"]) for row in self.history]
        times = [float(row["time_s"]) for row in self.history]
        errors = [target - speed for speed in speeds]
        rise_time = next(
            (time for time, speed in zip(times, speeds) if speed >= 0.9 * target),
            None,
        )
        overshoot = (
            max(0.0, (max(speeds) - target) / target * 100.0)
            if target > 0.0
            else 0.0
        )
        sample_dt = (
            times[-1] - times[-2]
            if len(times) > 1
            else times[0]
        )
        tail_count = max(1, int(2.0 / max(1e-9, sample_dt)))
        tail = speeds[-tail_count:]
        final_error = target - sum(tail) / len(tail)
        rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
        maximum_cte = max(
            abs(float(row["cross_track_error_m"])) for row in self.history
        )
        return {
            "rise_time_s": rise_time,
            "overshoot_percent": overshoot,
            "final_error_mps": final_error,
            "rmse_mps": rmse,
            "maximum_cross_track_error_m": maximum_cte,
        }
