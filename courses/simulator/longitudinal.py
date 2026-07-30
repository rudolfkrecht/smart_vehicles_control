"""Longitudinal speed control and actuator dynamics reused on Day 4.

The controller intentionally remains close to the PI controller from Day 1,
but it now receives a time-varying target from the road-speed planner and the
traffic-behaviour layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .controllers import ControllerOutput


@dataclass(frozen=True)
class LongitudinalParameters:
    proportional_gain: float = 1.0
    integral_gain: float = 0.18
    maximum_acceleration: float = 2.5
    maximum_braking: float = 5.5
    maximum_jerk: float = 5.0
    integral_limit: float = 12.0
    enable_integral: bool = True
    enable_anti_windup: bool = True

    def __post_init__(self) -> None:
        if self.proportional_gain < 0.0 or self.integral_gain < 0.0:
            raise ValueError("controller gains cannot be negative")
        if self.maximum_acceleration <= 0.0:
            raise ValueError("maximum_acceleration must be positive")
        if self.maximum_braking <= 0.0:
            raise ValueError("maximum_braking must be positive")
        if self.maximum_jerk <= 0.0:
            raise ValueError("maximum_jerk must be positive")
        if self.integral_limit <= 0.0:
            raise ValueError("integral_limit must be positive")


@dataclass(frozen=True)
class LongitudinalOutput:
    speed_error: float
    proportional_term: float
    integral_term: float
    requested_acceleration: float
    saturated_acceleration: float
    applied_acceleration: float
    saturated: bool
    jerk_limited: bool


class SpeedController:
    """Small stateful PI controller with saturation and jerk limiting."""

    def __init__(
        self,
        parameters: LongitudinalParameters | None = None,
    ) -> None:
        self.parameters = parameters or LongitudinalParameters()
        self.integral_error = 0.0
        self.previous_acceleration = 0.0

    def reset(self, *, acceleration: float = 0.0) -> None:
        self.integral_error = 0.0
        self.previous_acceleration = float(acceleration)

    def update(
        self,
        *,
        target_speed: float,
        measured_speed: float,
        dt: float,
        acceleration_override: float | None = None,
    ) -> LongitudinalOutput:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        p = self.parameters
        error = float(target_speed - measured_speed)
        proportional = p.proportional_gain * error

        candidate_integral = self.integral_error
        if p.enable_integral:
            candidate_integral = float(
                np.clip(
                    self.integral_error + error * dt,
                    -p.integral_limit,
                    p.integral_limit,
                )
            )
        integral_term = p.integral_gain * candidate_integral
        requested = proportional + integral_term
        if acceleration_override is not None:
            requested = min(requested, float(acceleration_override))

        saturated = float(
            np.clip(
                requested,
                -p.maximum_braking,
                p.maximum_acceleration,
            )
        )
        is_saturated = not np.isclose(saturated, requested)

        # Conditional integration: keep integrating only when it helps the
        # command return from saturation.
        if p.enable_integral:
            helpful = (
                not is_saturated
                or (saturated >= p.maximum_acceleration and error < 0.0)
                or (saturated <= -p.maximum_braking and error > 0.0)
            )
            if helpful or not p.enable_anti_windup:
                self.integral_error = candidate_integral
        else:
            self.integral_error = 0.0

        maximum_change = p.maximum_jerk * dt
        applied = float(
            np.clip(
                saturated,
                self.previous_acceleration - maximum_change,
                self.previous_acceleration + maximum_change,
            )
        )
        jerk_limited = not np.isclose(applied, saturated)
        self.previous_acceleration = applied
        return LongitudinalOutput(
            speed_error=error,
            proportional_term=proportional,
            integral_term=p.integral_gain * self.integral_error,
            requested_acceleration=requested,
            saturated_acceleration=saturated,
            applied_acceleration=applied,
            saturated=is_saturated,
            jerk_limited=jerk_limited,
        )


def integrate_speed(
    speed: float,
    acceleration: float,
    dt: float,
) -> float:
    """Integrate speed while preventing reverse motion."""

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    return max(0.0, float(speed + acceleration * dt))


# Day 1 teaching model

class Controller(Protocol):
    """Interface implemented by all Day 1 controllers."""

    def reset(self) -> None: ...

    def update(
        self,
        target_speed: float,
        measured_speed: float,
        dt: float,
    ) -> ControllerOutput: ...


@dataclass(frozen=True)
class VehicleParameters:
    """Parameters of the simplified passenger-vehicle model."""

    mass: float = 1_200.0
    max_drive_force: float = 4_500.0
    max_brake_force: float = 7_000.0
    rolling_resistance: float = 180.0
    aerodynamic_drag: float = 4.0
    actuator_time_constant: float = 0.35

    def __post_init__(self) -> None:
        positive_values = (
            self.mass,
            self.max_drive_force,
            self.max_brake_force,
            self.actuator_time_constant,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError("Mass, force limits and actuator time must be positive.")
        if self.rolling_resistance < 0.0 or self.aerodynamic_drag < 0.0:
            raise ValueError("Resistance parameters must be non-negative.")


@dataclass(frozen=True)
class Scenario:
    """Simulation settings and optional disturbances."""

    duration: float = 30.0
    dt: float = 0.05
    target_speed: float = 15.0
    target_changes: tuple[tuple[float, float], ...] = ()
    initial_speed: float = 0.0
    hill_start: float | None = None
    hill_end: float | None = None
    hill_force: float = 0.0
    measurement_noise_std: float = 0.0
    random_seed: int = 7

    def __post_init__(self) -> None:
        if self.duration <= 0.0 or self.dt <= 0.0:
            raise ValueError("duration and dt must be positive.")
        if self.target_speed < 0.0 or self.initial_speed < 0.0:
            raise ValueError("Speeds must be non-negative.")
        if self.measurement_noise_std < 0.0:
            raise ValueError("measurement_noise_std must be non-negative.")
        if self.hill_force < 0.0:
            raise ValueError("hill_force must be non-negative.")
        if self.hill_end is not None and self.hill_start is None:
            raise ValueError("hill_end requires hill_start.")
        if (
            self.hill_start is not None
            and self.hill_end is not None
            and self.hill_end <= self.hill_start
        ):
            raise ValueError("hill_end must be later than hill_start.")

    def target_at(self, time: float) -> float:
        target = self.target_speed
        for change_time, new_target in sorted(self.target_changes):
            if time >= change_time:
                target = new_target
        return float(target)

    def hill_force_at(self, time: float) -> float:
        if self.hill_start is None or time < self.hill_start:
            return 0.0
        if self.hill_end is not None and time >= self.hill_end:
            return 0.0
        return self.hill_force


@dataclass(frozen=True)
class SimulationResult:
    """Time histories generated by one simulation run."""

    time: np.ndarray
    target_speed: np.ndarray
    measured_speed: np.ndarray
    speed: np.ndarray
    position: np.ndarray
    command: np.ndarray
    raw_command: np.ndarray
    acceleration: np.ndarray
    actuator_force: np.ndarray
    resistance_force: np.ndarray
    hill_force: np.ndarray
    error: np.ndarray
    integral_error: np.ndarray
    scenario: Scenario = field(repr=False)
    vehicle: VehicleParameters = field(repr=False)


def _desired_actuator_force(command: float, vehicle: VehicleParameters) -> float:
    if command >= 0.0:
        return command * vehicle.max_drive_force
    return command * vehicle.max_brake_force


def run_simulation(
    controller: Controller,
    *,
    scenario: Scenario | None = None,
    vehicle: VehicleParameters | None = None,
) -> SimulationResult:
    """Simulate one controller and return all time histories."""

    scenario = scenario or Scenario()
    vehicle = vehicle or VehicleParameters()
    controller.reset()

    number_of_steps = int(round(scenario.duration / scenario.dt)) + 1
    time = np.arange(number_of_steps, dtype=float) * scenario.dt
    rng = np.random.default_rng(scenario.random_seed)

    target_speed = np.zeros(number_of_steps)
    measured_speed = np.zeros(number_of_steps)
    speed = np.zeros(number_of_steps)
    position = np.zeros(number_of_steps)
    command = np.zeros(number_of_steps)
    raw_command = np.zeros(number_of_steps)
    acceleration = np.zeros(number_of_steps)
    actuator_force = np.zeros(number_of_steps)
    resistance_force = np.zeros(number_of_steps)
    hill_force = np.zeros(number_of_steps)
    error = np.zeros(number_of_steps)
    integral_error = np.zeros(number_of_steps)

    speed[0] = scenario.initial_speed
    current_actuator_force = 0.0

    for index, current_time in enumerate(time):
        target_speed[index] = scenario.target_at(current_time)
        noise = rng.normal(0.0, scenario.measurement_noise_std)
        measured_speed[index] = max(0.0, speed[index] + noise)

        output = controller.update(
            target_speed[index],
            measured_speed[index],
            scenario.dt,
        )
        command[index] = float(np.clip(output.command, -1.0, 1.0))
        raw_command[index] = output.raw_command
        error[index] = output.error
        integral_error[index] = output.integral_error

        desired_force = _desired_actuator_force(command[index], vehicle)
        actuator_fraction = min(
            scenario.dt / vehicle.actuator_time_constant,
            1.0,
        )
        current_actuator_force += actuator_fraction * (
            desired_force - current_actuator_force
        )
        actuator_force[index] = current_actuator_force

        if speed[index] > 1e-6:
            resistance_force[index] = (
                vehicle.rolling_resistance
                + vehicle.aerodynamic_drag * speed[index] ** 2
            )
        hill_force[index] = scenario.hill_force_at(current_time)

        net_force = (
            actuator_force[index]
            - resistance_force[index]
            - hill_force[index]
        )
        acceleration[index] = net_force / vehicle.mass

        # A stationary vehicle cannot be accelerated backwards by the road
        # resistance or hill term in this one-direction teaching model.
        if speed[index] <= 0.0 and acceleration[index] < 0.0:
            acceleration[index] = 0.0

        if index < number_of_steps - 1:
            next_speed = max(
                0.0,
                speed[index] + acceleration[index] * scenario.dt,
            )
            speed[index + 1] = next_speed
            position[index + 1] = position[index] + (
                0.5 * (speed[index] + next_speed) * scenario.dt
            )

    return SimulationResult(
        time=time,
        target_speed=target_speed,
        measured_speed=measured_speed,
        speed=speed,
        position=position,
        command=command,
        raw_command=raw_command,
        acceleration=acceleration,
        actuator_force=actuator_force,
        resistance_force=resistance_force,
        hill_force=hill_force,
        error=error,
        integral_error=integral_error,
        scenario=scenario,
        vehicle=vehicle,
    )
