"""Day 3 simulation orchestration, traffic sensing, logging and metrics."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Protocol

from .controllers import ReferenceController
from .model import ControlCommand, Observation, VehicleModel, VehicleState
from .track import ClosedHighwayTrack


class Controller(Protocol):
    def reset(self) -> None: ...

    def update(self, observation: Observation) -> ControlCommand: ...


def lead_speed_schedule(time_s: float) -> float:
    """Deterministic stop-and-go traffic used in every Day 3 experiment."""

    if time_s < 18.0:
        return 10.5
    if time_s < 28.0:
        return 10.5 - 0.45 * (time_s - 18.0)
    if time_s < 42.0:
        return 6.0
    if time_s < 48.0:
        return 6.0 - (time_s - 42.0)
    if time_s < 56.0:
        return 0.0
    if time_s < 66.0:
        return 1.1 * (time_s - 56.0)
    if time_s < 82.0:
        return 11.0
    if time_s < 90.0:
        return 11.0 - 0.5 * (time_s - 82.0)
    if time_s < 100.0:
        return 7.0
    if time_s < 106.0:
        return 7.0 + 0.5 * (time_s - 100.0)
    return 10.0


class Simulation:
    def __init__(
        self,
        controller: Controller | None = None,
        *,
        target_speed: float = 14.0,
        track: ClosedHighwayTrack | None = None,
        vehicle: VehicleModel | None = None,
        sensor_range: float = 85.0,
        initial_lead_distance: float = 95.0,
        lead_vehicle_length: float = 4.4,
    ) -> None:
        self.track = track or ClosedHighwayTrack()
        self.vehicle = vehicle or VehicleModel()
        self.controller: Controller = controller or ReferenceController()
        self.target_speed = float(target_speed)
        self.sensor_range = float(sensor_range)
        self.initial_lead_distance = float(initial_lead_distance)
        self.lead_vehicle_length = float(lead_vehicle_length)
        self.initial_track_s = 8.0
        self.time = 0.0
        self.track_progress = 0.0
        self.last_track_s = 0.0
        self.lead_progress = 0.0
        self.lead_speed = 0.0
        self.lead_vehicle = VehicleState()
        self.lap = 0
        self.history: list[dict[str, float | int | bool | str]] = []
        self.last_observation: Observation | None = None
        self.last_command = ControlCommand()
        self.reset()

    def reset(self) -> None:
        start_s = self.initial_track_s
        start = self.track.pose_at(start_s, self.track.lane_offset)
        self.vehicle.reset(start.x, start.y, start.z, start.heading)
        self.controller.reset()
        self.time = 0.0
        self.track_progress = start_s
        self.last_track_s = start_s
        self.lead_progress = start_s + self.initial_lead_distance
        self.lead_speed = lead_speed_schedule(0.0)
        self._update_lead_pose()
        self.lap = 0
        self.history = []
        self.last_command = ControlCommand(
            selected_target_speed=self.target_speed,
            desired_gap=0.0,
            mode="CRUISE",
        )
        self.last_observation = self.observe(1.0 / 60.0)

    def _update_lead_pose(self) -> None:
        pose = self.track.pose_at(
            self.lead_progress,
            self.track.lane_offset,
        )
        self.lead_vehicle = VehicleState(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            heading=pose.heading,
            speed=self.lead_speed,
        )

    def true_gap(self) -> float:
        return (
            self.lead_progress
            - self.track_progress
            - self.lead_vehicle_length
        )

    def _preview_distance(self, speed: float) -> float:
        method = getattr(self.controller, "preview_distance", None)
        if callable(method):
            requested = float(method(speed))
        else:
            requested = max(8.0, 0.75 * speed)
        if not math.isfinite(requested):
            raise ValueError("preview distance must be finite")
        return max(3.0, min(30.0, requested))

    def set_controller(self, controller: Controller) -> None:
        self.controller = controller
        self.reset()

    def observe(self, dt: float) -> Observation:
        state = self.vehicle.state
        tracking = self.track.nearest(state.x, state.y, state.heading)
        preview_distance = self._preview_distance(state.speed)
        preview = self.track.pose_at(
            tracking.pose.s + preview_distance,
            self.track.lane_offset,
        )
        grade = 100.0 * math.tan(tracking.pose.slope)
        true_gap = self.true_gap()
        detected = 0.0 < true_gap <= self.sensor_range
        measured_gap = true_gap if detected else math.inf
        measured_lead_speed = self.lead_speed if detected else state.speed
        closing_speed = (
            state.speed - measured_lead_speed if detected else 0.0
        )
        time_to_collision = (
            measured_gap / closing_speed
            if detected and closing_speed > 1e-6
            else math.inf
        )
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
            preview_distance=preview_distance,
            lap=self.lap,
            off_road=(
                abs(tracking.cross_track_error)
                > 0.5 * self.track.road_width
            ),
            lead_detected=detected,
            lead_distance=measured_gap,
            lead_speed=measured_lead_speed,
            closing_speed=closing_speed,
            time_to_collision=time_to_collision,
            sensor_range=self.sensor_range,
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
        if command.selected_target_speed is None:
            command.selected_target_speed = self.target_speed
        if command.desired_gap is None:
            command.desired_gap = 0.0
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

        self.lead_speed = lead_speed_schedule(self.time)
        self.lead_progress += self.lead_speed * dt
        self._update_lead_pose()
        self.time += dt

        after = self.observe(dt)
        self.last_observation = after
        self.last_command = command
        true_gap = self.true_gap()
        selected_target = float(command.selected_target_speed)
        desired_gap = float(command.desired_gap)
        actual_ttc = (
            true_gap / (after.speed - self.lead_speed)
            if after.speed > self.lead_speed + 1e-6
            else math.inf
        )
        self.history.append(
            {
                "time_s": self.time,
                "x_m": after.x,
                "y_m": after.y,
                "z_m": after.z,
                "speed_mps": after.speed,
                "cruise_speed_mps": self.target_speed,
                "selected_target_speed_mps": selected_target,
                "acceleration_mps2": after.acceleration,
                "throttle": command.throttle,
                "brake": command.brake,
                "steering_rad": command.steering,
                "cross_track_error_m": after.cross_track_error,
                "heading_error_rad": after.heading_error,
                "preview_distance_m": after.preview_distance,
                "slope_deg": math.degrees(after.slope_radians),
                "grade_percent": after.grade_percent,
                "track_s_m": after.track_s,
                "track_progress_m": self.track_progress - self.initial_track_s,
                "lap": after.lap,
                "off_road": after.off_road,
                "lead_detected": after.lead_detected,
                "lead_speed_mps": self.lead_speed,
                "gap_m": true_gap,
                "desired_gap_m": desired_gap,
                "closing_speed_mps": after.speed - self.lead_speed,
                "time_to_collision_s": actual_ttc,
                "mode": command.mode,
                "collision": true_gap <= 0.0,
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

    def metrics(self) -> dict[str, float | int | None]:
        if not self.history:
            return {
                "minimum_gap_m": None,
                "minimum_ttc_s": None,
                "collision_samples": 0,
                "mean_absolute_gap_error_m": None,
                "speed_target_rmse_mps": None,
                "peak_deceleration_mps2": None,
                "peak_absolute_jerk_mps3": None,
                "maximum_cross_track_error_m": None,
                "outside_road_percent": None,
                "lap_progress_percent": None,
                "emergency_samples": 0,
            }

        gaps = [float(row["gap_m"]) for row in self.history]
        finite_ttc = [
            float(row["time_to_collision_s"])
            for row in self.history
            if math.isfinite(float(row["time_to_collision_s"]))
            and float(row["time_to_collision_s"]) >= 0.0
        ]
        gap_errors = [
            float(row["gap_m"]) - float(row["desired_gap_m"])
            for row in self.history
            if bool(row["lead_detected"])
            and float(row["desired_gap_m"]) > 0.0
        ]
        speed_errors = [
            float(row["selected_target_speed_mps"])
            - float(row["speed_mps"])
            for row in self.history
        ]
        accelerations = [
            float(row["acceleration_mps2"]) for row in self.history
        ]
        jerks = []
        for previous, current in zip(self.history, self.history[1:]):
            step_dt = float(current["time_s"]) - float(previous["time_s"])
            if step_dt > 0.0:
                jerks.append(
                    (
                        float(current["acceleration_mps2"])
                        - float(previous["acceleration_mps2"])
                    )
                    / step_dt
                )
        distance_travelled = max(
            0.0,
            float(self.history[-1]["track_progress_m"]),
        )
        return {
            "minimum_gap_m": min(gaps),
            "minimum_ttc_s": min(finite_ttc) if finite_ttc else None,
            "collision_samples": sum(
                bool(row["collision"]) for row in self.history
            ),
            "mean_absolute_gap_error_m": (
                sum(abs(value) for value in gap_errors) / len(gap_errors)
                if gap_errors
                else None
            ),
            "speed_target_rmse_mps": math.sqrt(
                sum(value * value for value in speed_errors)
                / len(speed_errors)
            ),
            "peak_deceleration_mps2": max(
                0.0,
                -min(accelerations),
            ),
            "peak_absolute_jerk_mps3": (
                max(abs(value) for value in jerks) if jerks else 0.0
            ),
            "maximum_cross_track_error_m": max(
                abs(float(row["cross_track_error_m"]))
                for row in self.history
            ),
            "outside_road_percent": 100.0 * sum(
                bool(row["off_road"]) for row in self.history
            ) / len(self.history),
            "lap_progress_percent": (
                100.0 * distance_travelled / self.track.total_length
            ),
            "emergency_samples": sum(
                str(row["mode"]) == "EMERGENCY" for row in self.history
            ),
        }
