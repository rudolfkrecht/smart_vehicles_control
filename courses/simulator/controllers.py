"""Longitudinal controllers used by the Day 1 exercises."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ControllerOutput:
    """Values returned by a controller at one simulation step."""

    command: float
    raw_command: float
    error: float
    integral_error: float = 0.0


class OpenLoopController:
    """Apply a fixed accelerator or brake command without speed feedback."""

    def __init__(self, command: float) -> None:
        self.command = float(np.clip(command, -1.0, 1.0))

    def reset(self) -> None:
        """Reset internal controller state."""

    def update(
        self,
        target_speed: float,
        measured_speed: float,
        dt: float,
    ) -> ControllerOutput:
        del dt
        error = target_speed - measured_speed
        return ControllerOutput(
            command=self.command,
            raw_command=self.command,
            error=error,
        )


class PController:
    """Proportional speed controller with command saturation."""

    def __init__(self, kp: float) -> None:
        if kp < 0.0:
            raise ValueError("kp must be non-negative.")
        self.kp = float(kp)

    def reset(self) -> None:
        """Reset internal controller state."""

    def update(
        self,
        target_speed: float,
        measured_speed: float,
        dt: float,
    ) -> ControllerOutput:
        del dt
        error = target_speed - measured_speed
        raw_command = self.kp * error
        command = float(np.clip(raw_command, -1.0, 1.0))
        return ControllerOutput(
            command=command,
            raw_command=raw_command,
            error=error,
        )


class PIController:
    """PI speed controller with optional conditional anti-windup."""

    def __init__(
        self,
        kp: float,
        ki: float,
        *,
        anti_windup: bool = True,
        integral_limit: float = 100.0,
    ) -> None:
        if kp < 0.0 or ki < 0.0:
            raise ValueError("kp and ki must be non-negative.")
        if integral_limit <= 0.0:
            raise ValueError("integral_limit must be positive.")

        self.kp = float(kp)
        self.ki = float(ki)
        self.anti_windup = anti_windup
        self.integral_limit = float(integral_limit)
        self.integral_error = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0

    def update(
        self,
        target_speed: float,
        measured_speed: float,
        dt: float,
    ) -> ControllerOutput:
        error = target_speed - measured_speed
        candidate_integral = float(
            np.clip(
                self.integral_error + error * dt,
                -self.integral_limit,
                self.integral_limit,
            )
        )
        candidate_raw = self.kp * error + self.ki * candidate_integral
        candidate_command = float(np.clip(candidate_raw, -1.0, 1.0))

        # Stop integrating only if saturation and the current error would push
        # the controller farther into saturation. This is conditional
        # integration, a simple anti-windup method.
        pushes_further_into_saturation = (
            candidate_raw > 1.0 and error > 0.0
        ) or (
            candidate_raw < -1.0 and error < 0.0
        )

        if not (self.anti_windup and pushes_further_into_saturation):
            self.integral_error = candidate_integral

        raw_command = self.kp * error + self.ki * self.integral_error
        command = float(np.clip(raw_command, -1.0, 1.0))
        return ControllerOutput(
            command=command,
            raw_command=raw_command,
            error=error,
            integral_error=self.integral_error,
        )
