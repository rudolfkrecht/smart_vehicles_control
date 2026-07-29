"""Worked Day 2 controller solution.

Copy the lateral-control parts into ``student_controller.py`` only after you
have attempted the guided implementation and tuning tasks.
"""

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

    def reset(self) -> None:
        self.integral_error = 0.0

    def preview_distance(self, speed: float) -> float:
        return clamp(
            self.BASE_LOOKAHEAD_M + self.SPEED_GAIN_S * speed,
            3.0,
            24.0,
        )

    def update(self, observation: Observation) -> ControlCommand:
        speed_error = observation.target_speed - observation.speed
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
