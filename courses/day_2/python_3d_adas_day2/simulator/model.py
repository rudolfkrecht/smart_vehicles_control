"""Vehicle dynamics and public controller data types."""

from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ControlCommand:
    """Normalized longitudinal commands and a steering request in radians."""

    throttle: float = 0.0
    brake: float = 0.0
    steering: float = 0.0

    def limited(self, max_steering: float) -> "ControlCommand":
        throttle = clamp(float(self.throttle), 0.0, 1.0)
        brake = clamp(float(self.brake), 0.0, 1.0)
        if throttle > 0.0 and brake > 0.0:
            if throttle >= brake:
                brake = 0.0
            else:
                throttle = 0.0
        return ControlCommand(
            throttle=throttle,
            brake=brake,
            steering=clamp(float(self.steering), -max_steering, max_steering),
        )


@dataclass(frozen=True)
class Observation:
    """Measurements made available to a controller at one simulation step."""

    time: float
    dt: float
    x: float
    y: float
    z: float
    heading: float
    speed: float
    acceleration: float
    target_speed: float
    track_s: float
    track_heading: float
    cross_track_error: float
    heading_error: float
    slope_radians: float
    grade_percent: float
    preview_x: float
    preview_y: float
    preview_z: float
    preview_distance: float
    lap: int
    off_road: bool


@dataclass
class VehicleParameters:
    mass: float = 1200.0
    maximum_drive_force: float = 4500.0
    maximum_brake_force: float = 8000.0
    rolling_resistance: float = 180.0
    drag_coefficient: float = 4.0
    wheelbase: float = 2.8
    maximum_steering: float = math.radians(28.0)
    powertrain_time_constant: float = 0.35
    steering_time_constant: float = 0.18
    gravity: float = 9.81


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    acceleration: float = 0.0
    steering: float = 0.0
    longitudinal_force: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


class VehicleModel:
    """Force-balance longitudinal model plus a kinematic bicycle model."""

    def __init__(self, parameters: VehicleParameters | None = None) -> None:
        self.parameters = parameters or VehicleParameters()
        self.state = VehicleState()

    def reset(self, x: float, y: float, z: float, heading: float) -> None:
        self.state = VehicleState(x=x, y=y, z=z, heading=heading)

    def step(
        self,
        command: ControlCommand,
        slope_radians: float,
        dt: float,
        *,
        off_road: bool = False,
    ) -> None:
        p = self.parameters
        s = self.state
        command = command.limited(p.maximum_steering)

        drive_scale = 0.48 if off_road else 1.0
        rolling_scale = 4.0 if off_road else 1.0
        requested_force = (
            command.throttle * p.maximum_drive_force * drive_scale
            - command.brake * p.maximum_brake_force
        )
        force_alpha = 1.0 - math.exp(-dt / p.powertrain_time_constant)
        s.longitudinal_force += force_alpha * (
            requested_force - s.longitudinal_force
        )

        steer_alpha = 1.0 - math.exp(-dt / p.steering_time_constant)
        s.steering += steer_alpha * (command.steering - s.steering)

        drag = p.drag_coefficient * s.speed * s.speed
        rolling_limit = p.rolling_resistance * rolling_scale
        if s.speed > 0.05:
            rolling = rolling_limit
        else:
            nonrolling_force = (
                s.longitudinal_force
                - p.mass * p.gravity * math.sin(slope_radians)
            )
            rolling = clamp(nonrolling_force, -rolling_limit, rolling_limit)

        hill_force = p.mass * p.gravity * math.sin(slope_radians)
        net_force = s.longitudinal_force - drag - rolling - hill_force
        acceleration = net_force / p.mass

        new_speed = s.speed + acceleration * dt
        if new_speed < 0.0:
            new_speed = 0.0
            if acceleration < 0.0:
                acceleration = 0.0

        average_speed = 0.5 * (s.speed + new_speed)
        yaw_rate = (
            average_speed / p.wheelbase * math.tan(s.steering)
            if average_speed > 0.0
            else 0.0
        )
        middle_heading = s.heading + 0.5 * yaw_rate * dt
        s.x += average_speed * math.cos(middle_heading) * dt
        s.y += average_speed * math.sin(middle_heading) * dt
        s.heading = wrap_angle(s.heading + yaw_rate * dt)
        s.speed = new_speed
        s.acceleration = acceleration
        s.throttle = command.throttle
        s.brake = command.brake
