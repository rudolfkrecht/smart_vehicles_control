"""Deterministic sensor, actuator and disturbance models for Day 4.

The models are intentionally simple and visible.  They are not intended to
replace a vehicle-dynamics package; they let students test whether a controller
that works nominally also tolerates noise, delay, bias and reduced authority.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .bicycle import VehicleState, normalize_angle


@dataclass(frozen=True)
class FaultParameters:
    """Parameters for one repeatable fault injection."""

    start_time: float = 0.0
    position_noise_std: float = 0.0
    heading_noise_std_degrees: float = 0.0
    speed_noise_std: float = 0.0
    range_noise_std: float = 0.0
    sensor_delay: float = 0.0
    actuator_delay: float = 0.0
    steering_bias_degrees: float = 0.0
    steering_authority: float = 1.0
    acceleration_efficiency: float = 1.0
    braking_efficiency: float = 1.0
    lateral_push_time: float | None = None
    lateral_push_m: float = 0.0
    random_seed: int = 7

    def __post_init__(self) -> None:
        nonnegative = (
            self.start_time,
            self.position_noise_std,
            self.heading_noise_std_degrees,
            self.speed_noise_std,
            self.range_noise_std,
            self.sensor_delay,
            self.actuator_delay,
        )
        if any(value < 0.0 for value in nonnegative):
            raise ValueError("noise, delay and start time cannot be negative")
        if not 0.0 <= self.steering_authority <= 1.0:
            raise ValueError("steering_authority must be between zero and one")
        if not 0.0 <= self.acceleration_efficiency <= 1.0:
            raise ValueError(
                "acceleration_efficiency must be between zero and one"
            )
        if not 0.0 <= self.braking_efficiency <= 1.0:
            raise ValueError(
                "braking_efficiency must be between zero and one"
            )
        if self.lateral_push_time is not None and self.lateral_push_time < 0.0:
            raise ValueError("lateral_push_time cannot be negative")

    @property
    def enabled(self) -> bool:
        return any(
            (
                self.position_noise_std > 0.0,
                self.heading_noise_std_degrees > 0.0,
                self.speed_noise_std > 0.0,
                self.range_noise_std > 0.0,
                self.sensor_delay > 0.0,
                self.actuator_delay > 0.0,
                abs(self.steering_bias_degrees) > 0.0,
                self.steering_authority < 1.0,
                self.acceleration_efficiency < 1.0,
                self.braking_efficiency < 1.0,
                abs(self.lateral_push_m) > 0.0,
            )
        )

    def label(self) -> str:
        labels: list[str] = []
        if self.position_noise_std or self.heading_noise_std_degrees:
            labels.append("pose noise")
        if self.speed_noise_std or self.range_noise_std:
            labels.append("measurement noise")
        if self.sensor_delay:
            labels.append(f"{1000 * self.sensor_delay:.0f} ms sensor delay")
        if self.actuator_delay:
            labels.append(
                f"{1000 * self.actuator_delay:.0f} ms actuator delay"
            )
        if self.steering_bias_degrees:
            labels.append(
                f"{self.steering_bias_degrees:+.1f}° steering bias"
            )
        if self.steering_authority < 1.0:
            labels.append(
                f"{100 * self.steering_authority:.0f}% steering authority"
            )
        if self.braking_efficiency < 1.0:
            labels.append(
                f"{100 * self.braking_efficiency:.0f}% braking"
            )
        if self.acceleration_efficiency < 1.0:
            labels.append(
                f"{100 * self.acceleration_efficiency:.0f}% acceleration"
            )
        if self.lateral_push_m:
            labels.append(f"{self.lateral_push_m:+.1f} m lateral push")
        return ", ".join(labels) if labels else "nominal"


class FaultInjector:
    """Stateful, seeded fault pipeline shared by batch runs and the GUI."""

    def __init__(self, parameters: FaultParameters, dt: float) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self.parameters = parameters
        self.dt = dt
        self.rng = np.random.default_rng(parameters.random_seed)
        self.state_history: deque[VehicleState] = deque()
        self.steering_history: deque[float] = deque()
        self.acceleration_history: deque[float] = deque()
        self.push_applied = False

    def reset(self, initial_state: VehicleState) -> None:
        self.rng = np.random.default_rng(self.parameters.random_seed)
        self.state_history.clear()
        self.steering_history.clear()
        self.acceleration_history.clear()
        self.state_history.append(initial_state.copy())
        self.push_applied = False

    def active(self, time: float) -> bool:
        return self.parameters.enabled and time >= self.parameters.start_time

    @staticmethod
    def _delayed_value(values: deque, delay_steps: int):
        if not values:
            raise ValueError("delay buffer is empty")
        index = max(0, len(values) - 1 - delay_steps)
        return list(values)[index]

    def observe_state(self, state: VehicleState, time: float) -> VehicleState:
        self.state_history.append(state.copy())
        maximum = int(math.ceil(self.parameters.sensor_delay / self.dt)) + 3
        while len(self.state_history) > maximum:
            self.state_history.popleft()
        if not self.active(time):
            return state.copy()

        delay_steps = int(round(self.parameters.sensor_delay / self.dt))
        observed = self._delayed_value(self.state_history, delay_steps).copy()
        p = self.parameters
        observed.x += float(self.rng.normal(0.0, p.position_noise_std))
        observed.y += float(self.rng.normal(0.0, p.position_noise_std))
        observed.heading = normalize_angle(
            observed.heading
            + float(
                self.rng.normal(
                    0.0,
                    math.radians(p.heading_noise_std_degrees),
                )
            )
        )
        observed.speed = max(
            0.0,
            observed.speed + float(self.rng.normal(0.0, p.speed_noise_std)),
        )
        return observed

    def observe_gap(self, gap: float, time: float) -> float:
        if not self.active(time) or not math.isfinite(gap):
            return gap
        return gap + float(
            self.rng.normal(0.0, self.parameters.range_noise_std)
        )

    def actuator_commands(
        self,
        requested_steering: float,
        requested_acceleration: float,
        time: float,
    ) -> tuple[float, float]:
        self.steering_history.append(float(requested_steering))
        self.acceleration_history.append(float(requested_acceleration))
        maximum = int(math.ceil(self.parameters.actuator_delay / self.dt)) + 3
        while len(self.steering_history) > maximum:
            self.steering_history.popleft()
            self.acceleration_history.popleft()
        if not self.active(time):
            return float(requested_steering), float(requested_acceleration)

        delay_steps = int(round(self.parameters.actuator_delay / self.dt))
        steering = float(
            self._delayed_value(self.steering_history, delay_steps)
        )
        acceleration = float(
            self._delayed_value(self.acceleration_history, delay_steps)
        )
        p = self.parameters
        steering = (
            p.steering_authority * steering
            + math.radians(p.steering_bias_degrees)
        )
        efficiency = (
            p.braking_efficiency
            if acceleration < 0.0
            else p.acceleration_efficiency
        )
        return steering, efficiency * acceleration

    def maybe_apply_lateral_push(
        self,
        state: VehicleState,
        *,
        time: float,
        path_heading: float,
    ) -> bool:
        p = self.parameters
        if (
            self.push_applied
            or p.lateral_push_time is None
            or time < p.lateral_push_time
            or abs(p.lateral_push_m) <= 0.0
        ):
            return False
        state.x -= p.lateral_push_m * math.sin(path_heading)
        state.y += p.lateral_push_m * math.cos(path_heading)
        self.push_applied = True
        return True

