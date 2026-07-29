"""Student-editable controller used by Lessons 5 and 6.

Edit only this file during the first exercise. Save it and press Reset in the
simulator to reload the latest version.
"""

from simulator.controllers import pure_pursuit_steering
from simulator.model import ControlCommand, Observation, clamp


class StudentController:
    def __init__(self) -> None:
        # Lesson 5: tune KP, then introduce a non-zero KI.
        self.KP = 0.08
        self.KI = 0.00
        self.integral_error = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0

    def update(self, observation: Observation) -> ControlCommand:
        speed_error = observation.target_speed - observation.speed

        # TODO 1: add integral action.
        # candidate_integral = (
        #     self.integral_error + speed_error * observation.dt
        # )

        # TODO 2: accept or reject candidate_integral to prevent windup.
        # self.integral_error = candidate_integral

        raw_command = (
            self.KP * speed_error
            + self.KI * self.integral_error
        )
        signed_command = clamp(raw_command, -1.0, 1.0)

        # The lateral controller is supplied for Day 1. Students develop or
        # replace it during a later lateral-control lesson.
        steering = pure_pursuit_steering(observation)

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

