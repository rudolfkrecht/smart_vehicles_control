"""Reference controllers for the cumulative Day 3 ADAS simulator."""

from __future__ import annotations

import math

from .model import ControlCommand, Observation, clamp


def pure_pursuit_steering(
    observation: Observation,
    *,
    wheelbase: float = 2.8,
    maximum_steering: float = math.radians(28.0),
) -> float:
    """Return the validated Day 2 Pure Pursuit steering command."""

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


def desired_following_gap(
    speed: float,
    standstill_gap: float,
    time_headway: float,
) -> float:
    """Constant-time-headway spacing policy."""

    return standstill_gap + time_headway * max(0.0, speed)


def adaptive_cruise_target(
    observation: Observation,
    *,
    standstill_gap: float,
    time_headway: float,
    gap_gain: float,
    closing_gain: float,
) -> tuple[float, float, str]:
    """Return selected speed, desired gap and behaviour label."""

    cruise_target = max(0.0, observation.target_speed)
    desired_gap = desired_following_gap(
        observation.speed,
        standstill_gap,
        time_headway,
    )
    if not observation.lead_detected:
        return cruise_target, desired_gap, "CRUISE"

    gap_error = observation.lead_distance - desired_gap
    traffic_target = (
        observation.lead_speed
        + gap_gain * gap_error
        - closing_gain * max(0.0, observation.closing_speed)
    )
    selected_target = clamp(traffic_target, 0.0, cruise_target)

    if (
        observation.lead_distance <= 3.0
        or observation.time_to_collision <= 1.2
    ):
        return 0.0, desired_gap, "EMERGENCY"
    if (
        observation.lead_distance < 0.72 * desired_gap
        or observation.time_to_collision < 2.5
        or selected_target < observation.speed - 1.0
    ):
        return selected_target, desired_gap, "BRAKE"
    return selected_target, desired_gap, "FOLLOW"


class ReferenceController:
    """Day 1 PI + Day 2 Pure Pursuit + Day 3 ACC."""

    def __init__(
        self,
        kp: float = 0.15,
        ki: float = 0.005,
        base_lookahead: float = 4.0,
        speed_lookahead_gain: float = 0.35,
        standstill_gap: float = 6.0,
        time_headway: float = 1.7,
        gap_gain: float = 0.22,
        closing_gain: float = 0.65,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.base_lookahead = base_lookahead
        self.speed_lookahead_gain = speed_lookahead_gain
        self.standstill_gap = standstill_gap
        self.time_headway = time_headway
        self.gap_gain = gap_gain
        self.closing_gain = closing_gain
        self.integral_error = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0

    def preview_distance(self, speed: float) -> float:
        return clamp(
            self.base_lookahead + self.speed_lookahead_gain * speed,
            3.0,
            24.0,
        )

    def update(self, observation: Observation) -> ControlCommand:
        selected_target, desired_gap, mode = adaptive_cruise_target(
            observation,
            standstill_gap=self.standstill_gap,
            time_headway=self.time_headway,
            gap_gain=self.gap_gain,
            closing_gain=self.closing_gain,
        )

        speed_error = selected_target - observation.speed
        candidate_integral = (
            self.integral_error + speed_error * observation.dt
        )
        candidate_raw = self.kp * speed_error + self.ki * candidate_integral
        if (
            -1.0 <= candidate_raw <= 1.0
            or (candidate_raw > 1.0 and speed_error < 0.0)
            or (candidate_raw < -1.0 and speed_error > 0.0)
        ):
            self.integral_error = candidate_integral

        signed_command = clamp(
            self.kp * speed_error + self.ki * self.integral_error,
            -1.0,
            1.0,
        )
        if mode == "BRAKE":
            signed_command = min(signed_command, -0.22)
        elif mode == "EMERGENCY":
            signed_command = min(signed_command, -0.90)

        steering = pure_pursuit_steering(observation)
        if signed_command >= 0.0:
            return ControlCommand(
                throttle=signed_command,
                brake=0.0,
                steering=steering,
                selected_target_speed=selected_target,
                desired_gap=desired_gap,
                mode=mode,
            )
        return ControlCommand(
            throttle=0.0,
            brake=-signed_command,
            steering=steering,
            selected_target_speed=selected_target,
            desired_gap=desired_gap,
            mode=mode,
        )


class ManualController:
    def reset(self) -> None:
        return None

    def update(self, observation: Observation) -> ControlCommand:
        return ControlCommand(
            selected_target_speed=observation.target_speed,
            mode="MANUAL",
        )
