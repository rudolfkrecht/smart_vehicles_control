"""Student-editable Day 2 controller.

The longitudinal PI controller is the completed Day 1 result. During Day 2,
implement and tune the Pure Pursuit lateral controller in this file. Save the
file and press Reset in the simulator to reload it.
"""

import math

from simulator.model import ControlCommand, Observation, clamp


class StudentController:
    def __init__(self) -> None:
        # Supplied Day 1 longitudinal controller. Do not retune it on Day 2.
        self.KP = 0.15
        self.KI = 0.005
        self.integral_error = 0.0

        # Day 2 lateral parameters. Start with a fixed look-ahead, then add a
        # non-zero speed gain during the independent project.
        self.BASE_LOOKAHEAD_M = 6.0
        self.SPEED_GAIN_S = 0.0
        self.WHEELBASE_M = 2.8
        self.MAX_STEERING_RAD = math.radians(28.0)

    def reset(self) -> None:
        self.integral_error = 0.0

    def preview_distance(self, speed: float) -> float:
        """Tell the simulator how far ahead to place the target point."""

        return clamp(
            self.BASE_LOOKAHEAD_M + self.SPEED_GAIN_S * speed,
            3.0,
            24.0,
        )

    def update(self, observation: Observation) -> ControlCommand:
        # Completed Day 1 PI controller with conditional anti-windup.
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

        raw_command = self.KP * speed_error + self.KI * self.integral_error
        signed_command = clamp(raw_command, -1.0, 1.0)

        # Day 2 TODO: replace zero steering with the Pure Pursuit calculation.
        #
        # dx = observation.preview_x - observation.x
        # dy = observation.preview_y - observation.y
        # target_bearing = math.atan2(dy, dx)
        # alpha = (
        #     target_bearing - observation.heading + math.pi
        # ) % (2.0 * math.pi) - math.pi
        # geometric_lookahead = max(1.0, math.hypot(dx, dy))
        # steering = math.atan2(
        #     2.0 * self.WHEELBASE_M * math.sin(alpha),
        #     geometric_lookahead,
        # )
        # steering = clamp(
        #     steering,
        #     -self.MAX_STEERING_RAD,
        #     self.MAX_STEERING_RAD,
        # )
        steering = 0.0

        # The signed command is split so throttle and brake cannot be active
        # simultaneously.
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
        )
