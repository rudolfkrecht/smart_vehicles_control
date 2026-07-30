"""Kinematic bicycle model reused for robustness testing on Day 4.

The implementation deliberately exposes the update order and physical limits.
It is more realistic than directly changing ``x``, ``y`` and heading, while
remaining small enough for students to read during a 45-minute lesson.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class VehicleState:
    """Pose and motion state at the rear-axle reference point."""

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 8.0
    steering: float = 0.0

    def copy(self) -> "VehicleState":
        return VehicleState(
            x=self.x,
            y=self.y,
            heading=self.heading,
            speed=self.speed,
            steering=self.steering,
        )


@dataclass(frozen=True)
class VehicleParameters:
    """Geometric and actuator limits for a passenger car."""

    wheelbase: float = 2.7
    maximum_steering: float = math.radians(32.0)
    maximum_steering_rate: float = math.radians(70.0)

    def __post_init__(self) -> None:
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.maximum_steering <= 0.0:
            raise ValueError("maximum_steering must be positive")
        if self.maximum_steering_rate <= 0.0:
            raise ValueError("maximum_steering_rate must be positive")


@dataclass(frozen=True)
class MotionSample:
    """Diagnostic values returned by one model update."""

    commanded_steering: float
    applied_steering: float
    steering_rate: float
    yaw_rate: float
    turning_radius: float
    lateral_acceleration: float


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval ``[-pi, pi)``."""

    return math.atan2(math.sin(angle), math.cos(angle))


def turning_radius(wheelbase: float, steering: float) -> float:
    """Return geometric turning radius, or infinity for straight motion."""

    if abs(steering) < 1e-9:
        return math.inf
    return wheelbase / math.tan(steering)


def bicycle_step(
    state: VehicleState,
    commanded_steering: float,
    *,
    parameters: VehicleParameters,
    dt: float,
    speed: float | None = None,
    enable_rate_limit: bool = True,
) -> MotionSample:
    """Advance ``state`` by one explicit-Euler integration step.

    If ``speed`` is supplied, the caller has already updated it using the
    longitudinal controller. The bicycle model then advances pose and heading.
    """

    if dt <= 0.0:
        raise ValueError("dt must be positive")

    if speed is not None:
        if speed < 0.0:
            raise ValueError("speed cannot be negative")
        state.speed = float(speed)

    saturated_command = float(
        np.clip(
            commanded_steering,
            -parameters.maximum_steering,
            parameters.maximum_steering,
        )
    )
    requested_change = saturated_command - state.steering
    if enable_rate_limit:
        maximum_change = parameters.maximum_steering_rate * dt
        applied_change = float(
            np.clip(requested_change, -maximum_change, maximum_change)
        )
    else:
        applied_change = requested_change

    state.steering += applied_change
    steering_rate = applied_change / dt
    yaw_rate = (
        state.speed
        / parameters.wheelbase
        * math.tan(state.steering)
    )

    # Position is integrated with the current heading; heading is then updated.
    state.x += state.speed * math.cos(state.heading) * dt
    state.y += state.speed * math.sin(state.heading) * dt
    state.heading = normalize_angle(state.heading + yaw_rate * dt)

    radius = turning_radius(parameters.wheelbase, state.steering)
    lateral_acceleration = state.speed * yaw_rate
    return MotionSample(
        commanded_steering=saturated_command,
        applied_steering=state.steering,
        steering_rate=steering_rate,
        yaw_rate=yaw_rate,
        turning_radius=radius,
        lateral_acceleration=lateral_acceleration,
    )


def simulate_constant_steering(
    *,
    steering_degrees: float,
    speed: float = 8.0,
    wheelbase: float = 2.7,
    duration: float = 6.0,
    dt: float = 0.02,
    enable_rate_limit: bool = False,
) -> dict[str, np.ndarray]:
    """Run an open-loop constant-steering experiment."""

    if duration <= 0.0:
        raise ValueError("duration must be positive")

    parameters = VehicleParameters(wheelbase=wheelbase)
    state = VehicleState(speed=speed)
    command = math.radians(steering_degrees)
    time = np.arange(0.0, duration + 0.5 * dt, dt)
    history = {
        key: np.zeros_like(time)
        for key in (
            "x",
            "y",
            "heading",
            "steering",
            "yaw_rate",
            "lateral_acceleration",
        )
    }

    for index, _ in enumerate(time):
        history["x"][index] = state.x
        history["y"][index] = state.y
        history["heading"][index] = state.heading
        history["steering"][index] = state.steering
        sample = bicycle_step(
            state,
            command,
            parameters=parameters,
            dt=dt,
            speed=speed,
            enable_rate_limit=enable_rate_limit,
        )
        history["yaw_rate"][index] = sample.yaw_rate
        history["lateral_acceleration"][index] = (
            sample.lateral_acceleration
        )

    history["time"] = time
    return history
