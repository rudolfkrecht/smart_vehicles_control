"""Worked solution for the cumulative Day 3 ACC project."""

import math

from simulator.model import ControlCommand, Observation, clamp


class StudentController:
    def __init__(self) -> None:
        self.KP = 0.15
        self.KI = 0.005
        self.integral_error = 0.0

        self.BASE_LOOKAHEAD_M = 4.0
        self.SPEED_GAIN_S = 0.35
        self.WHEELBASE_M = 2.8
        self.MAX_STEERING_RAD = math.radians(28.0)

        self.STANDSTILL_GAP_M = 6.0
        self.TIME_HEADWAY_S = 1.7
        self.GAP_GAIN_PER_S = 0.22
        self.CLOSING_GAIN = 0.65

    def reset(self) -> None:
        self.integral_error = 0.0

    def preview_distance(self, speed: float) -> float:
        return clamp(
            self.BASE_LOOKAHEAD_M + self.SPEED_GAIN_S * speed,
            3.0,
            24.0,
        )

    def update(self, observation: Observation) -> ControlCommand:
        desired_gap = (
            self.STANDSTILL_GAP_M
            + self.TIME_HEADWAY_S * observation.speed
        )
        selected_target = observation.target_speed
        mode = "CRUISE"

        if observation.lead_detected:
            gap_error = observation.lead_distance - desired_gap
            traffic_target = (
                observation.lead_speed
                + self.GAP_GAIN_PER_S * gap_error
                - self.CLOSING_GAIN
                * max(0.0, observation.closing_speed)
            )
            selected_target = clamp(
                traffic_target,
                0.0,
                observation.target_speed,
            )
            mode = "FOLLOW"
            if (
                observation.lead_distance < 0.72 * desired_gap
                or observation.time_to_collision < 2.5
                or selected_target < observation.speed - 1.0
            ):
                mode = "BRAKE"
            if (
                observation.lead_distance <= 3.0
                or observation.time_to_collision <= 1.2
            ):
                selected_target = 0.0
                mode = "EMERGENCY"

        speed_error = selected_target - observation.speed
        candidate_integral = (
            self.integral_error + speed_error * observation.dt
        )
        candidate_raw = (
            self.KP * speed_error + self.KI * candidate_integral
        )
        if (
            -1.0 <= candidate_raw <= 1.0
            or (candidate_raw > 1.0 and speed_error < 0.0)
            or (candidate_raw < -1.0 and speed_error > 0.0)
        ):
            self.integral_error = candidate_integral
        signed_command = clamp(
            self.KP * speed_error + self.KI * self.integral_error,
            -1.0,
            1.0,
        )
        if mode == "BRAKE":
            signed_command = min(signed_command, -0.22)
        elif mode == "EMERGENCY":
            signed_command = min(signed_command, -0.90)

        dx = observation.preview_x - observation.x
        dy = observation.preview_y - observation.y
        target_bearing = math.atan2(dy, dx)
        alpha = (
            target_bearing - observation.heading + math.pi
        ) % (2.0 * math.pi) - math.pi
        geometric_lookahead = max(1.0, math.hypot(dx, dy))
        steering = math.atan2(
            2.0 * self.WHEELBASE_M * math.sin(alpha),
            geometric_lookahead,
        )
        steering = clamp(
            steering,
            -self.MAX_STEERING_RAD,
            self.MAX_STEERING_RAD,
        )

        if signed_command >= 0.0:
            throttle = signed_command
            brake = 0.0
        else:
            throttle = 0.0
            brake = -signed_command

        return ControlCommand(
            throttle=throttle,
            brake=brake,
            steering=steering,
            selected_target_speed=selected_target,
            desired_gap=desired_gap,
            mode=mode,
        )
