"""Base objective metrics for integrated Day 4 scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .integrated import IntegratedResult
from .longitudinal import SimulationResult
from .tracking import PathFollowingResult


@dataclass(frozen=True)
class IntegratedMetrics:
    mean_path_error_m: float
    maximum_path_error_m: float
    road_departure_percent: float
    speed_rmse_mps: float
    peak_lateral_acceleration_mps2: float
    rms_acceleration_mps2: float
    peak_jerk_mps3: float
    minimum_gap_m: float
    minimum_ttc_s: float
    collision_samples: int
    completion_percent: float
    completion_time_s: float
    cruise_percent: float
    follow_percent: float
    brake_percent: float
    emergency_percent: float

    @property
    def safe(self) -> bool:
        return (
            self.collision_samples == 0
            and self.road_departure_percent == 0.0
            and self.minimum_gap_m > 0.0
        )

    def as_row(self, label: str) -> list[str]:
        return [
            label,
            f"{self.mean_path_error_m:.2f}",
            f"{self.maximum_path_error_m:.2f}",
            f"{self.road_departure_percent:.1f}",
            f"{self.speed_rmse_mps:.2f}",
            f"{self.minimum_gap_m:.1f}",
            _display_ttc(self.minimum_ttc_s),
            f"{self.peak_jerk_mps3:.1f}",
            f"{self.completion_percent:.1f}",
            "YES" if self.safe else "NO",
        ]


def _display_ttc(value: float) -> str:
    return "∞" if not math.isfinite(value) else f"{value:.2f}"


def calculate_integrated_metrics(
    result: IntegratedResult,
) -> IntegratedMetrics:
    """Summarise tracking, speed, safety, comfort and task completion."""

    if len(result.time) == 0:
        raise ValueError("result is empty")
    absolute_error = np.abs(result.cross_track_error)
    speed_error = result.selected_target_speed - result.speed
    finite_ttc = result.time_to_collision[
        np.isfinite(result.time_to_collision)
    ]
    state = result.behaviour_state

    def state_percent(name: str) -> float:
        return 100.0 * float(np.mean(state == name))

    return IntegratedMetrics(
        mean_path_error_m=float(np.mean(absolute_error)),
        maximum_path_error_m=float(np.max(absolute_error)),
        road_departure_percent=float(
            100.0
            * np.mean(
                absolute_error > result.scenario.road_half_width
            )
        ),
        speed_rmse_mps=float(np.sqrt(np.mean(speed_error**2))),
        peak_lateral_acceleration_mps2=float(
            np.max(np.abs(result.lateral_acceleration))
        ),
        rms_acceleration_mps2=float(
            np.sqrt(np.mean(result.acceleration**2))
        ),
        peak_jerk_mps3=float(np.max(np.abs(result.jerk))),
        minimum_gap_m=(
            float(np.min(result.gap[np.isfinite(result.gap)]))
            if np.any(np.isfinite(result.gap))
            else math.inf
        ),
        minimum_ttc_s=(
            float(np.min(finite_ttc)) if len(finite_ttc) else math.inf
        ),
        collision_samples=int(np.count_nonzero(result.collision)),
        completion_percent=100.0 * result.completion_fraction,
        completion_time_s=float(result.time[-1]),
        cruise_percent=state_percent("CRUISE"),
        follow_percent=state_percent("FOLLOW"),
        brake_percent=state_percent("BRAKE"),
        emergency_percent=state_percent("EMERGENCY"),
    )


def print_integrated_metrics_table(
    named_results: dict[str, IntegratedResult],
) -> None:
    """Print an aligned multi-objective classroom comparison."""

    headers = [
        "Configuration",
        "Mean |e_y|",
        "Max |e_y|",
        "Outside [%]",
        "Speed RMSE",
        "Min gap",
        "Min TTC",
        "Peak jerk",
        "Complete [%]",
        "Safe",
    ]
    rows = [
        calculate_integrated_metrics(result).as_row(label)
        for label, result in named_results.items()
    ]
    widths = [
        max(len(headers[column]), *(len(row[column]) for row in rows))
        for column in range(len(headers))
    ]

    def render(row: list[str]) -> str:
        return " | ".join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        )

    print(render(headers))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(render(row))


def weighted_workshop_score(result: IntegratedResult) -> float:
    """Simple score for ranking only configurations that remain safe."""

    metric = calculate_integrated_metrics(result)
    if not metric.safe:
        return math.inf
    return (
        1.5 * metric.mean_path_error_m
        + 0.25 * metric.speed_rmse_mps
        + 0.03 * metric.peak_jerk_mps3
        + 0.4 * max(0.0, 8.0 - metric.minimum_gap_m)
        + 0.015 * (100.0 - metric.completion_percent)
    )


# Day 2 path-following metrics

@dataclass(frozen=True)
class PathMetrics:
    mean_absolute_error_m: float
    rms_error_m: float
    maximum_absolute_error_m: float
    final_absolute_error_m: float
    completion_percent: float
    road_departure_percent: float
    peak_steering_degrees: float
    rms_steering_rate_degrees_s: float
    peak_lateral_acceleration_mps2: float
    recovery_time_s: float

    def as_row(self, label: str) -> list[str]:
        return [
            label,
            f"{self.mean_absolute_error_m:.2f}",
            f"{self.maximum_absolute_error_m:.2f}",
            f"{self.completion_percent:.1f}",
            f"{self.road_departure_percent:.1f}",
            f"{self.peak_steering_degrees:.1f}",
            f"{self.rms_steering_rate_degrees_s:.1f}",
            _display(self.recovery_time_s),
        ]


def _display(value: float) -> str:
    return "—" if not math.isfinite(value) else f"{value:.2f}"


def _recovery_time(
    result: PathFollowingResult,
    *,
    tolerance: float,
    sustained_time_s: float,
) -> float:
    disturbance_indices = np.flatnonzero(result.disturbed)
    if len(disturbance_indices) == 0:
        return math.nan
    start = int(disturbance_indices[0])
    required = max(
        1,
        int(round(sustained_time_s / result.scenario.dt)),
    )
    in_band = np.abs(result.cross_track_error) <= tolerance
    consecutive = 0
    for index in range(start, len(in_band)):
        if in_band[index]:
            consecutive += 1
            if consecutive >= required:
                first = index - required + 1
                return float(result.time[first] - result.time[start])
        else:
            consecutive = 0
    return math.nan


def calculate_path_metrics(
    result: PathFollowingResult,
    *,
    recovery_tolerance_m: float = 0.5,
    sustained_time_s: float = 0.75,
) -> PathMetrics:
    """Summarise accuracy, safety, smoothness and recovery."""

    absolute_error = np.abs(result.cross_track_error)
    if len(absolute_error) == 0:
        raise ValueError("result is empty")
    return PathMetrics(
        mean_absolute_error_m=float(np.mean(absolute_error)),
        rms_error_m=float(np.sqrt(np.mean(absolute_error**2))),
        maximum_absolute_error_m=float(np.max(absolute_error)),
        final_absolute_error_m=float(np.mean(absolute_error[-20:])),
        completion_percent=100.0 * result.completion_fraction,
        road_departure_percent=float(
            100.0
            * np.mean(absolute_error > result.scenario.road_half_width)
        ),
        peak_steering_degrees=float(
            np.degrees(np.max(np.abs(result.steering)))
        ),
        rms_steering_rate_degrees_s=float(
            np.degrees(
                np.sqrt(np.mean(result.steering_rate**2))
            )
        ),
        peak_lateral_acceleration_mps2=float(
            np.max(np.abs(result.lateral_acceleration))
        ),
        recovery_time_s=_recovery_time(
            result,
            tolerance=recovery_tolerance_m,
            sustained_time_s=sustained_time_s,
        ),
    )


def print_path_metrics_table(
    named_results: dict[str, PathFollowingResult],
) -> None:
    """Print an aligned classroom comparison table."""

    headers = [
        "Configuration",
        "Mean |e_y| [m]",
        "Max |e_y| [m]",
        "Complete [%]",
        "Outside road [%]",
        "Peak steer [deg]",
        "Steer-rate RMS [deg/s]",
        "Recovery [s]",
    ]
    rows = [
        calculate_path_metrics(result).as_row(label)
        for label, result in named_results.items()
    ]
    widths = [
        max(len(headers[column]), *(len(row[column]) for row in rows))
        for column in range(len(headers))
    ]

    def render(row: list[str]) -> str:
        return " | ".join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        )

    print(render(headers))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(render(row))


# Day 1 longitudinal metrics

@dataclass(frozen=True)
class PerformanceMetrics:
    """Compact numerical summary of one longitudinal simulation."""

    rise_time_s: float
    overshoot_percent: float
    settling_time_s: float
    final_error_mps: float
    rmse_mps: float
    maximum_acceleration_mps2: float
    saturation_percent: float
    recovery_time_s: float

    def as_row(self, label: str) -> list[str]:
        return [
            label,
            _display(self.rise_time_s),
            f"{self.overshoot_percent:.1f}",
            _display(self.settling_time_s),
            f"{self.final_error_mps:.2f}",
            f"{self.rmse_mps:.2f}",
            f"{self.saturation_percent:.1f}",
            _display(self.recovery_time_s),
        ]


def _display(value: float) -> str:
    return "—" if not math.isfinite(value) else f"{value:.2f}"


def _first_sustained_true(
    mask: np.ndarray,
    *,
    start_index: int,
    samples_required: int,
) -> int | None:
    consecutive = 0
    for index in range(start_index, len(mask)):
        if mask[index]:
            consecutive += 1
            if consecutive >= samples_required:
                return index - samples_required + 1
        else:
            consecutive = 0
    return None


def calculate_metrics(
    result: SimulationResult,
    *,
    tolerance_fraction: float = 0.05,
    sustained_time_s: float = 1.0,
) -> PerformanceMetrics:
    """Calculate common step-response and disturbance-recovery metrics."""

    if tolerance_fraction <= 0.0:
        raise ValueError("tolerance_fraction must be positive.")

    target = float(np.max(result.target_speed))
    if target <= 0.0:
        raise ValueError("Metrics require a positive target speed.")

    tolerance = tolerance_fraction * target
    in_band = np.abs(result.target_speed - result.speed) <= tolerance
    samples_required = max(
        1,
        int(round(sustained_time_s / result.scenario.dt)),
    )

    rise_indices = np.flatnonzero(result.speed >= 0.9 * target)
    rise_time = (
        float(result.time[rise_indices[0]]) if len(rise_indices) else math.nan
    )

    overshoot = max(0.0, float(np.max(result.speed) - target))
    overshoot_percent = 100.0 * overshoot / target

    last_outside = np.flatnonzero(~in_band)
    if len(last_outside) == 0:
        settling_time = 0.0
    elif last_outside[-1] >= len(result.time) - 1:
        settling_time = math.nan
    else:
        settling_time = float(result.time[last_outside[-1] + 1])

    final_samples = max(1, int(round(2.0 / result.scenario.dt)))
    final_error = float(
        np.mean(result.target_speed[-final_samples:] - result.speed[-final_samples:])
    )
    rmse = float(np.sqrt(np.mean((result.target_speed - result.speed) ** 2)))
    maximum_acceleration = float(np.max(np.abs(result.acceleration)))
    saturation_percent = float(
        100.0 * np.mean(np.abs(result.command) >= 0.999)
    )

    recovery_time = math.nan
    hill_start = result.scenario.hill_start
    if hill_start is not None and result.scenario.hill_force > 0.0:
        hill_index = int(np.searchsorted(result.time, hill_start))
        outside_after_hill = np.flatnonzero(~in_band[hill_index:])
        if len(outside_after_hill) == 0:
            recovery_time = 0.0
        else:
            # Search only after the largest speed loss. Otherwise a controller
            # can briefly re-enter the band while the disturbance is still
            # developing, which would make the recovery time misleading.
            lowest_speed_index = hill_index + int(
                np.argmin(result.speed[hill_index:])
            )
            recovered_index = _first_sustained_true(
                in_band,
                start_index=lowest_speed_index,
                samples_required=samples_required,
            )
            if recovered_index is not None:
                recovery_time = float(
                    result.time[recovered_index] - hill_start
                )

    return PerformanceMetrics(
        rise_time_s=rise_time,
        overshoot_percent=overshoot_percent,
        settling_time_s=settling_time,
        final_error_mps=final_error,
        rmse_mps=rmse,
        maximum_acceleration_mps2=maximum_acceleration,
        saturation_percent=saturation_percent,
        recovery_time_s=recovery_time,
    )


def print_metrics_table(
    named_results: dict[str, SimulationResult],
    *,
    tolerance_fraction: float = 0.05,
) -> None:
    """Print an aligned metrics table without third-party table packages."""

    headers = [
        "Controller",
        "Rise [s]",
        "Overshoot [%]",
        "Settle [s]",
        "Final error [m/s]",
        "RMSE [m/s]",
        "Saturation [%]",
        "Recovery [s]",
    ]
    rows = [
        calculate_metrics(
            result,
            tolerance_fraction=tolerance_fraction,
        ).as_row(label)
        for label, result in named_results.items()
    ]
    widths = [
        max(len(headers[column]), *(len(row[column]) for row in rows))
        for column in range(len(headers))
    ]

    def format_row(row: list[str]) -> str:
        return " | ".join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        )

    print(format_row(headers))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(format_row(row))
