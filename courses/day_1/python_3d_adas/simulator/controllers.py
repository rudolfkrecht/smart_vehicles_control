"""Reference longitudinal controller and supplied lateral controller."""

from __future__ import annotations

import math

from .model import ControlCommand, Observation, clamp, wrap_angle


def pure_pursuit_steering(
    observation: Observation,
    *,
    wheelbase: float = 2.8,
    maximum_steering: float = math.radians(28.0),
) -> float:
    """Return steering for the preview point contained in the observation."""

    dx = observation.preview_x - observation.x
    dy = observation.preview_y - observation.y
    forward = (
        dx * math.cos(observation.heading)
        + dy * math.sin(observation.heading)
    )
    left = (
        -dx * math.sin(observation.heading)
        + dy * math.cos(observation.heading)
    )
    lookahead = max(1.0, math.hypot(forward, left))
    alpha = math.atan2(left, forward)
    steering = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead)
    return clamp(steering, -maximum_steering, maximum_steering)


class ReferenceController:
    """PI cruise control with conditional-integration anti-windup."""

    def __init__(self, kp: float = 0.15, ki: float = 0.005) -> None:
        self.kp = kp
        self.ki = ki
        self.integral_error = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0

    def update(self, observation: Observation) -> ControlCommand:
        error = observation.target_speed - observation.speed
        candidate_integral = self.integral_error + error * observation.dt
        candidate_raw = self.kp * error + self.ki * candidate_integral

        if (
            -1.0 <= candidate_raw <= 1.0
            or (candidate_raw > 1.0 and error < 0.0)
            or (candidate_raw < -1.0 and error > 0.0)
        ):
            self.integral_error = candidate_integral

        signed_command = clamp(
            self.kp * error + self.ki * self.integral_error,
            -1.0,
            1.0,
        )
        steering = pure_pursuit_steering(observation)
        if signed_command >= 0.0:
            return ControlCommand(
                throttle=signed_command,
                brake=0.0,
                steering=steering,
            )
        return ControlCommand(
            throttle=0.0,
            brake=-signed_command,
            steering=steering,
        )


class ManualController:
    def reset(self) -> None:
        return None

    def update(self, observation: Observation) -> ControlCommand:
        del observation
        return ControlCommand()
