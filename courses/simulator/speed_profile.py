"""Curvature-based road-speed planning reused on Day 4."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .paths import ReferencePath


@dataclass(frozen=True)
class SpeedProfileParameters:
    global_speed_limit: float = 15.0
    maximum_lateral_acceleration: float = 2.5
    minimum_curve_speed: float = 2.0
    preview_distance: float = 14.0
    smoothing_window: int = 7
    maximum_acceleration: float = 2.5
    maximum_braking: float = 4.0
    curvature_floor: float = 1e-4

    def __post_init__(self) -> None:
        if self.global_speed_limit <= 0.0:
            raise ValueError("global_speed_limit must be positive")
        if self.maximum_lateral_acceleration <= 0.0:
            raise ValueError("maximum_lateral_acceleration must be positive")
        if self.minimum_curve_speed < 0.0:
            raise ValueError("minimum_curve_speed cannot be negative")
        if self.preview_distance < 0.0:
            raise ValueError("preview_distance cannot be negative")
        if self.smoothing_window < 1:
            raise ValueError("smoothing_window must be at least one")
        if self.maximum_acceleration <= 0.0:
            raise ValueError("maximum_acceleration must be positive")
        if self.maximum_braking <= 0.0:
            raise ValueError("maximum_braking must be positive")


@dataclass(frozen=True)
class SpeedProfile:
    distance: np.ndarray
    raw_curve_speed: np.ndarray
    preview_speed: np.ndarray
    planned_speed: np.ndarray
    lateral_acceleration_limit: np.ndarray
    parameters: SpeedProfileParameters = field(repr=False)

    def speed_at(self, distance: float) -> float:
        return float(
            np.interp(
                float(distance),
                self.distance,
                self.planned_speed,
            )
        )


def safe_cornering_speed(
    curvature: float | np.ndarray,
    maximum_lateral_acceleration: float,
    *,
    speed_limit: float = math.inf,
    curvature_floor: float = 1e-4,
) -> float | np.ndarray:
    """Evaluate ``sqrt(a_y,max / |kappa|)`` with straight-road protection."""

    if maximum_lateral_acceleration <= 0.0:
        raise ValueError("maximum_lateral_acceleration must be positive")
    values = np.asarray(curvature, dtype=float)
    safe = np.sqrt(
        maximum_lateral_acceleration
        / np.maximum(np.abs(values), curvature_floor)
    )
    safe = np.minimum(safe, speed_limit)
    if np.ndim(curvature) == 0:
        return float(safe)
    return safe


def _forward_minimum(
    values: np.ndarray,
    distance: np.ndarray,
    preview_distance: float,
) -> np.ndarray:
    """Move a future speed reduction earlier by a spatial preview distance."""

    if preview_distance <= 0.0:
        return values.copy()
    result = np.empty_like(values)
    stop = 0
    for index, position in enumerate(distance):
        stop = max(stop, index)
        while (
            stop + 1 < len(distance)
            and distance[stop + 1] <= position + preview_distance
        ):
            stop += 1
        result[index] = np.min(values[index : stop + 1])
    return result


def _smooth(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        window += 1
    half = window // 2
    padded = np.pad(values, (half, half), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


def _enforce_acceleration_limits(
    values: np.ndarray,
    distance: np.ndarray,
    *,
    maximum_acceleration: float,
    maximum_braking: float,
) -> np.ndarray:
    """Apply forward acceleration and backward braking feasibility passes."""

    constrained = values.copy()
    for index in range(1, len(constrained)):
        ds = distance[index] - distance[index - 1]
        reachable = math.sqrt(
            max(
                0.0,
                constrained[index - 1] ** 2
                + 2.0 * maximum_acceleration * ds,
            )
        )
        constrained[index] = min(constrained[index], reachable)

    for index in range(len(constrained) - 2, -1, -1):
        ds = distance[index + 1] - distance[index]
        brake_reachable = math.sqrt(
            max(
                0.0,
                constrained[index + 1] ** 2
                + 2.0 * maximum_braking * ds,
            )
        )
        constrained[index] = min(constrained[index], brake_reachable)
    return constrained


def build_speed_profile(
    path: ReferencePath,
    parameters: SpeedProfileParameters | None = None,
) -> SpeedProfile:
    """Create raw, previewed and actuator-feasible road-speed profiles."""

    parameters = parameters or SpeedProfileParameters()
    raw = np.asarray(
        safe_cornering_speed(
            path.curvature,
            parameters.maximum_lateral_acceleration,
            speed_limit=parameters.global_speed_limit,
            curvature_floor=parameters.curvature_floor,
        ),
        dtype=float,
    )
    raw = np.clip(
        raw,
        parameters.minimum_curve_speed,
        parameters.global_speed_limit,
    )
    preview = _forward_minimum(
        raw,
        path.distance,
        parameters.preview_distance,
    )
    smoothed = np.minimum(_smooth(preview, parameters.smoothing_window), raw)
    planned = _enforce_acceleration_limits(
        smoothed,
        path.distance,
        maximum_acceleration=parameters.maximum_acceleration,
        maximum_braking=parameters.maximum_braking,
    )
    lateral_limit = np.sqrt(
        parameters.maximum_lateral_acceleration
        / np.maximum(np.abs(path.curvature), parameters.curvature_floor)
    )
    return SpeedProfile(
        distance=path.distance.copy(),
        raw_curve_speed=raw,
        preview_speed=preview,
        planned_speed=planned,
        lateral_acceleration_limit=lateral_limit,
        parameters=parameters,
    )
