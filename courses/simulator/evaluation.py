"""Metrics, safety gates and challenge scoring for Day 4."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .metrics import IntegratedMetrics, calculate_integrated_metrics
from .robustness import RobustResult


@dataclass(frozen=True)
class RobustMetrics:
    """Objective evidence from a single repeatable run."""

    integrated: IntegratedMetrics
    rms_jerk_mps3: float
    rms_steering_rate_deg_s: float
    peak_steering_rate_deg_s: float
    rms_position_measurement_error_m: float
    rms_speed_measurement_error_mps: float
    rms_steering_command_mismatch_deg: float
    fault_exposure_percent: float
    pass_run: bool
    failure_reason: str

    @property
    def safety_critical_failure(self) -> bool:
        return (
            self.integrated.collision_samples > 0
            or self.integrated.road_departure_percent > 0.0
            or self.integrated.minimum_gap_m <= 0.0
        )

    def as_row(self, label: str) -> list[str]:
        base = self.integrated
        return [
            label,
            f"{base.mean_path_error_m:.2f}",
            f"{base.maximum_path_error_m:.2f}",
            f"{base.speed_rmse_mps:.2f}",
            f"{base.minimum_gap_m:.1f}",
            f"{self.rms_jerk_mps3:.1f}",
            f"{base.completion_percent:.1f}",
            "PASS" if self.pass_run else "FAIL",
        ]


@dataclass(frozen=True)
class ChallengeScore:
    """Weighted Day 4 score. Each category is reported on a 0–100 scale."""

    safety: float
    path_tracking: float
    speed_control: float
    comfort: float
    robustness: float
    technical_explanation: float
    weighted_total: float
    final_score: float
    safety_cap_applied: bool

    def as_rows(self) -> list[tuple[str, int, float, float]]:
        categories = (
            ("Safety and road compliance", 35, self.safety),
            ("Path tracking", 20, self.path_tracking),
            ("Speed control", 15, self.speed_control),
            ("Comfort", 10, self.comfort),
            ("Robustness", 10, self.robustness),
            ("Technical explanation", 10, self.technical_explanation),
        )
        return [
            (name, weight, value, weight * value / 100.0)
            for name, weight, value in categories
        ]


def _clip_score(value: float) -> float:
    return float(np.clip(value, 0.0, 100.0))


def calculate_robust_metrics(result: RobustResult) -> RobustMetrics:
    """Calculate tracking, safety, comfort and diagnostic metrics."""

    base = calculate_integrated_metrics(result.integrated)
    integrated = result.integrated
    dt = integrated.scenario.dt
    position_error = np.hypot(
        result.measured_x - integrated.x,
        result.measured_y - integrated.y,
    )
    speed_measurement_error = result.measured_speed - integrated.speed
    steering_mismatch = result.requested_steering - integrated.steering
    rms_jerk = float(np.sqrt(np.mean(integrated.jerk**2)))
    steering_rate_degrees = np.degrees(integrated.steering_rate)

    reasons: list[str] = []
    if base.collision_samples:
        reasons.append("collision")
    if base.road_departure_percent > 0.0:
        reasons.append("road departure")
    if base.minimum_gap_m < 1.0:
        reasons.append("minimum gap below 1 m")
    if base.completion_percent < 90.0:
        reasons.append("less than 90% complete")
    if base.peak_lateral_acceleration_mps2 > 4.6:
        reasons.append("excessive lateral acceleration")
    passed = not reasons
    return RobustMetrics(
        integrated=base,
        rms_jerk_mps3=rms_jerk,
        rms_steering_rate_deg_s=float(
            np.sqrt(np.mean(steering_rate_degrees**2))
        ),
        peak_steering_rate_deg_s=float(
            np.max(np.abs(steering_rate_degrees))
        ),
        rms_position_measurement_error_m=float(
            np.sqrt(np.mean(position_error**2))
        ),
        rms_speed_measurement_error_mps=float(
            np.sqrt(np.mean(speed_measurement_error**2))
        ),
        rms_steering_command_mismatch_deg=float(
            np.degrees(np.sqrt(np.mean(steering_mismatch**2)))
        ),
        fault_exposure_percent=100.0 * float(np.mean(result.fault_active)),
        pass_run=passed,
        failure_reason=", ".join(reasons) if reasons else "none",
    )


def score_challenge_run(
    result: RobustResult,
    *,
    robustness_pass_rate: float = 100.0,
    technical_explanation: float = 80.0,
) -> ChallengeScore:
    """Apply the published weights and a dominant hard safety cap."""

    metric = calculate_robust_metrics(result)
    base = metric.integrated
    collision_penalty = 100.0 if base.collision_samples else 0.0
    safety = _clip_score(
        100.0
        - collision_penalty
        - 4.0 * base.road_departure_percent
        - 12.0 * max(0.0, 2.0 - base.minimum_gap_m)
    )
    path_tracking = _clip_score(
        100.0
        - 45.0 * base.mean_path_error_m
        - 8.0 * max(0.0, base.maximum_path_error_m - 1.0)
    )
    speed_control = _clip_score(
        100.0 - 22.0 * base.speed_rmse_mps
    )
    comfort = _clip_score(
        100.0
        - 5.0 * max(0.0, metric.rms_jerk_mps3 - 1.0)
        - 3.0
        * max(0.0, base.peak_lateral_acceleration_mps2 - 2.5)
        - 0.45 * max(0.0, metric.rms_steering_rate_deg_s - 5.0)
    )
    robustness = _clip_score(robustness_pass_rate)
    explanation = _clip_score(technical_explanation)
    weighted = (
        0.35 * safety
        + 0.20 * path_tracking
        + 0.15 * speed_control
        + 0.10 * comfort
        + 0.10 * robustness
        + 0.10 * explanation
    )
    unsafe = metric.safety_critical_failure
    final = min(weighted, 50.0) if unsafe else weighted
    return ChallengeScore(
        safety=safety,
        path_tracking=path_tracking,
        speed_control=speed_control,
        comfort=comfort,
        robustness=robustness,
        technical_explanation=explanation,
        weighted_total=weighted,
        final_score=final,
        safety_cap_applied=unsafe and weighted > 50.0,
    )


def print_metrics_table(named_results: dict[str, RobustResult]) -> None:
    """Print an aligned, classroom-friendly scorecard."""

    headers = [
        "Test",
        "Mean |e_y|",
        "Max |e_y|",
        "Speed RMSE",
        "Min gap",
        "RMS jerk",
        "Complete [%]",
        "Result",
    ]
    rows = [
        calculate_robust_metrics(result).as_row(label)
        for label, result in named_results.items()
    ]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
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


def print_challenge_score(score: ChallengeScore) -> None:
    print("Category                         Weight   Result   Points")
    print("-------------------------------  ------   ------   ------")
    for name, weight, value, points in score.as_rows():
        print(f"{name:<31}  {weight:>5}%   {value:>5.1f}   {points:>5.1f}")
    print(f"\nWeighted total: {score.weighted_total:.1f} / 100")
    if score.safety_cap_applied:
        print("Safety cap: unsafe run cannot score above 50 points")
    print(f"Final score:    {score.final_score:.1f} / 100")
