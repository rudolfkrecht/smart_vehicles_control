"""Incremental Day 1 vehicle model used by the PyQt simulator."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .longitudinal import VehicleParameters


@dataclass(frozen=True)
class RealtimeState:
    time: float
    target_speed: float
    measured_speed: float
    speed: float
    position: float
    command: float
    acceleration: float
    actuator_force: float
    resistance_force: float
    hill_force: float


class RealtimeLongitudinalVehicle:
    """Stateful version of the deterministic Day 1 longitudinal model."""

    def __init__(
        self,
        parameters: VehicleParameters | None = None,
        *,
        dt: float = 0.05,
        random_seed: int = 7,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self.parameters = parameters or VehicleParameters()
        self.dt = float(dt)
        self._rng = np.random.default_rng(random_seed)
        self.reset()

    def reset(self, *, initial_speed: float = 0.0) -> RealtimeState:
        if initial_speed < 0.0:
            raise ValueError("initial_speed must be non-negative")
        self.time = 0.0
        self.speed = float(initial_speed)
        self.position = 0.0
        self.actuator_force = 0.0
        return self.snapshot(target_speed=0.0)

    def snapshot(
        self,
        *,
        target_speed: float,
        command: float = 0.0,
        acceleration: float = 0.0,
        hill_force: float = 0.0,
        measurement_noise_std: float = 0.0,
    ) -> RealtimeState:
        measured = max(
            0.0,
            self.speed
            + float(self._rng.normal(0.0, measurement_noise_std)),
        )
        resistance = (
            self.parameters.rolling_resistance
            + self.parameters.aerodynamic_drag * self.speed**2
            if self.speed > 1e-6
            else 0.0
        )
        return RealtimeState(
            time=self.time,
            target_speed=float(target_speed),
            measured_speed=measured,
            speed=self.speed,
            position=self.position,
            command=float(np.clip(command, -1.0, 1.0)),
            acceleration=float(acceleration),
            actuator_force=self.actuator_force,
            resistance_force=resistance,
            hill_force=float(hill_force),
        )

    def step(
        self,
        command: float,
        *,
        target_speed: float,
        hill_force: float = 0.0,
        measurement_noise_std: float = 0.0,
    ) -> RealtimeState:
        vehicle = self.parameters
        applied_command = float(np.clip(command, -1.0, 1.0))
        desired_force = (
            applied_command * vehicle.max_drive_force
            if applied_command >= 0.0
            else applied_command * vehicle.max_brake_force
        )
        actuator_fraction = min(
            self.dt / vehicle.actuator_time_constant,
            1.0,
        )
        self.actuator_force += actuator_fraction * (
            desired_force - self.actuator_force
        )
        resistance = (
            vehicle.rolling_resistance
            + vehicle.aerodynamic_drag * self.speed**2
            if self.speed > 1e-6
            else 0.0
        )
        acceleration = (
            self.actuator_force - resistance - float(hill_force)
        ) / vehicle.mass
        if self.speed <= 0.0 and acceleration < 0.0:
            acceleration = 0.0
        previous_speed = self.speed
        self.speed = max(0.0, self.speed + acceleration * self.dt)
        self.position += 0.5 * (previous_speed + self.speed) * self.dt
        self.time += self.dt
        return self.snapshot(
            target_speed=target_speed,
            command=applied_command,
            acceleration=acceleration,
            hill_force=hill_force,
            measurement_noise_std=measurement_noise_std,
        )
