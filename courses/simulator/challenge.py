"""Repeatable test suites and controller configuration for Day 4."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import csv
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

from .evaluation import RobustMetrics, calculate_robust_metrics
from .faults import FaultParameters
from .integrated import IntegratedScenario
from .robustness import RobustResult, RobustScenario, run_robust


@dataclass(frozen=True)
class ControllerConfiguration:
    """The only block teams modify during the final challenge."""

    global_speed_limit: float = 15.0
    maximum_lateral_acceleration: float = 2.5
    curve_preview_distance: float = 14.0
    speed_profile_smoothing: int = 7
    base_lookahead: float = 3.0
    speed_lookahead_gain: float = 0.25
    time_headway: float = 1.5
    standstill_gap: float = 5.0
    emergency_ttc: float = 1.25
    emergency_gap: float = 3.0
    proportional_gain: float = 1.0
    integral_gain: float = 0.18
    maximum_acceleration: float = 2.5
    maximum_braking: float = 5.5
    maximum_jerk: float = 5.0

    def __post_init__(self) -> None:
        if self.global_speed_limit <= 0.0:
            raise ValueError("global_speed_limit must be positive")
        if self.maximum_lateral_acceleration <= 0.0:
            raise ValueError(
                "maximum_lateral_acceleration must be positive"
            )
        if self.base_lookahead <= 0.0:
            raise ValueError("base_lookahead must be positive")
        if self.speed_lookahead_gain < 0.0:
            raise ValueError("speed_lookahead_gain cannot be negative")
        if self.time_headway <= 0.0:
            raise ValueError("time_headway must be positive")
        if self.speed_profile_smoothing < 1:
            raise ValueError("speed_profile_smoothing must be positive")


BALANCED_CONFIGURATION = ControllerConfiguration()

AGGRESSIVE_CONFIGURATION = ControllerConfiguration(
    global_speed_limit=18.0,
    maximum_lateral_acceleration=4.2,
    curve_preview_distance=4.0,
    speed_profile_smoothing=3,
    base_lookahead=2.0,
    speed_lookahead_gain=0.08,
    time_headway=0.8,
    standstill_gap=2.0,
    emergency_ttc=0.75,
    emergency_gap=1.2,
    maximum_jerk=8.0,
)


@dataclass(frozen=True)
class TestCase:
    name: str
    test_id: str
    path_kind: str
    lead_preset: str = "stop_and_go"
    initial_lateral_offset: float = 0.2
    initial_heading_offset_degrees: float = 0.0
    initial_lead_distance: float = 48.0
    faults: FaultParameters = field(default_factory=FaultParameters)
    duration: float = 42.0


@dataclass(frozen=True)
class BatchItem:
    case: TestCase
    result: RobustResult
    metrics: RobustMetrics


@dataclass(frozen=True)
class BatchReport:
    items: tuple[BatchItem, ...]

    @property
    def pass_count(self) -> int:
        return sum(item.metrics.pass_run for item in self.items)

    @property
    def pass_rate_percent(self) -> float:
        if not self.items:
            return 0.0
        return 100.0 * self.pass_count / len(self.items)

    @property
    def worst_item(self) -> BatchItem:
        if not self.items:
            raise ValueError("batch report is empty")

        def risk(item: BatchItem) -> tuple[float, float, float, float]:
            metric = item.metrics
            base = metric.integrated
            return (
                1.0 if not metric.pass_run else 0.0,
                float(base.collision_samples > 0),
                base.road_departure_percent,
                base.maximum_path_error_m,
            )

        return max(self.items, key=risk)

    @property
    def mean_path_error_m(self) -> float:
        return float(
            np.mean(
                [
                    item.metrics.integrated.mean_path_error_m
                    for item in self.items
                ]
            )
        )

    @property
    def minimum_gap_m(self) -> float:
        return float(
            np.min(
                [
                    item.metrics.integrated.minimum_gap_m
                    for item in self.items
                ]
            )
        )


def integrated_scenario_from_configuration(
    configuration: ControllerConfiguration,
    case: TestCase,
) -> IntegratedScenario:
    base = IntegratedScenario(
        duration=case.duration,
        path_kind=case.path_kind,
        initial_speed=8.0,
        initial_lateral_offset=case.initial_lateral_offset,
        initial_heading_offset_degrees=(
            case.initial_heading_offset_degrees
        ),
        initial_lead_distance=case.initial_lead_distance,
        lead_preset=case.lead_preset,
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=True,
        base_lookahead=configuration.base_lookahead,
        speed_lookahead_gain=configuration.speed_lookahead_gain,
    )
    smoothing = configuration.speed_profile_smoothing
    if smoothing % 2 == 0:
        smoothing += 1
    return replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=configuration.global_speed_limit,
            maximum_lateral_acceleration=(
                configuration.maximum_lateral_acceleration
            ),
            preview_distance=configuration.curve_preview_distance,
            smoothing_window=smoothing,
        ),
        longitudinal=replace(
            base.longitudinal,
            proportional_gain=configuration.proportional_gain,
            integral_gain=configuration.integral_gain,
            maximum_acceleration=configuration.maximum_acceleration,
            maximum_braking=configuration.maximum_braking,
            maximum_jerk=configuration.maximum_jerk,
        ),
        acc=replace(
            base.acc,
            time_headway=configuration.time_headway,
            standstill_gap=configuration.standstill_gap,
            emergency_ttc=configuration.emergency_ttc,
            emergency_gap=configuration.emergency_gap,
        ),
    )


def robust_scenario_from_case(
    configuration: ControllerConfiguration,
    case: TestCase,
) -> RobustScenario:
    return RobustScenario(
        controller=integrated_scenario_from_configuration(
            configuration,
            case,
        ),
        faults=case.faults,
        name=case.name,
        test_id=case.test_id,
    )


def initial_condition_cases() -> tuple[TestCase, ...]:
    """Five deterministic initial conditions for Lesson 3."""

    values = (
        ("centre", 0.0, 0.0),
        ("left offset", 0.8, 0.0),
        ("right offset", -0.8, 0.0),
        ("left heading", 0.4, 4.0),
        ("right heading", -0.4, -4.0),
    )
    return tuple(
        TestCase(
            name=name,
            test_id=f"initial_{index + 1}",
            path_kind="practice",
            lead_preset="steady",
            initial_lateral_offset=offset,
            initial_heading_offset_degrees=heading,
            faults=FaultParameters(random_seed=100 + index),
            duration=34.0,
        )
        for index, (name, offset, heading) in enumerate(values)
    )


def practice_cases() -> tuple[TestCase, ...]:
    """Visible cases teams may use for tuning."""

    return (
        TestCase(
            name="P1 nominal",
            test_id="practice_nominal",
            path_kind="practice",
        ),
        TestCase(
            name="P2 lateral push",
            test_id="practice_push",
            path_kind="practice",
            faults=FaultParameters(
                lateral_push_time=13.0,
                lateral_push_m=1.5,
                random_seed=12,
            ),
        ),
        TestCase(
            name="P3 sensor noise",
            test_id="practice_noise",
            path_kind="practice",
            faults=FaultParameters(
                position_noise_std=0.12,
                heading_noise_std_degrees=0.8,
                speed_noise_std=0.20,
                range_noise_std=0.35,
                random_seed=21,
            ),
        ),
        TestCase(
            name="P4 sensor delay",
            test_id="practice_delay",
            path_kind="practice",
            faults=FaultParameters(
                sensor_delay=0.15,
                random_seed=31,
            ),
        ),
        TestCase(
            name="P5 steering bias",
            test_id="practice_steering",
            path_kind="practice",
            faults=FaultParameters(
                start_time=7.0,
                steering_bias_degrees=1.4,
                steering_authority=0.9,
                random_seed=41,
            ),
        ),
        TestCase(
            name="P6 weak braking",
            test_id="practice_braking",
            path_kind="practice",
            lead_preset="evaluation",
            faults=FaultParameters(
                start_time=10.0,
                braking_efficiency=0.65,
                random_seed=51,
            ),
        ),
    )


def evaluation_cases() -> tuple[TestCase, ...]:
    """Instructor evaluation suite; do not reveal before the official run."""

    return (
        TestCase(
            name="E1 unfamiliar road",
            test_id="evaluation_road",
            path_kind="evaluation_a",
            lead_preset="evaluation",
            initial_lateral_offset=-0.7,
            initial_lead_distance=44.0,
            faults=FaultParameters(random_seed=71),
        ),
        TestCase(
            name="E2 noisy road",
            test_id="evaluation_noise",
            path_kind="evaluation_b",
            lead_preset="evaluation",
            initial_lateral_offset=0.9,
            faults=FaultParameters(
                position_noise_std=0.16,
                heading_noise_std_degrees=1.0,
                speed_noise_std=0.25,
                range_noise_std=0.45,
                random_seed=72,
            ),
        ),
        TestCase(
            name="E3 delayed steering",
            test_id="evaluation_delay",
            path_kind="evaluation_a",
            lead_preset="evaluation",
            initial_heading_offset_degrees=4.0,
            faults=FaultParameters(
                sensor_delay=0.10,
                actuator_delay=0.10,
                steering_authority=0.88,
                random_seed=73,
            ),
        ),
        TestCase(
            name="E4 braking degradation",
            test_id="evaluation_braking",
            path_kind="evaluation_b",
            lead_preset="evaluation",
            initial_lead_distance=42.0,
            faults=FaultParameters(
                start_time=8.0,
                braking_efficiency=0.68,
                range_noise_std=0.25,
                random_seed=74,
            ),
        ),
        TestCase(
            name="E5 combined disturbance",
            test_id="evaluation_combined",
            path_kind="evaluation_b",
            lead_preset="evaluation",
            initial_lateral_offset=-0.6,
            initial_lead_distance=43.0,
            faults=FaultParameters(
                start_time=5.0,
                position_noise_std=0.10,
                heading_noise_std_degrees=0.65,
                speed_noise_std=0.18,
                range_noise_std=0.30,
                sensor_delay=0.10,
                actuator_delay=0.05,
                steering_bias_degrees=-0.8,
                steering_authority=0.9,
                braking_efficiency=0.75,
                lateral_push_time=16.0,
                lateral_push_m=1.0,
                random_seed=75,
            ),
        ),
    )


def run_batch(
    configuration: ControllerConfiguration,
    cases: Iterable[TestCase],
    *,
    progress: Callable[[int, TestCase], None] | None = None,
) -> BatchReport:
    items: list[BatchItem] = []
    for index, case in enumerate(cases):
        if progress:
            progress(index, case)
        result = run_robust(
            robust_scenario_from_case(configuration, case)
        )
        items.append(
            BatchItem(
                case=case,
                result=result,
                metrics=calculate_robust_metrics(result),
            )
        )
    return BatchReport(tuple(items))


def print_batch_report(report: BatchReport) -> None:
    headers = [
        "Test",
        "Mean |e_y|",
        "Max |e_y|",
        "Min gap",
        "Peak jerk",
        "Complete",
        "Result",
        "Failure reason",
    ]
    rows: list[list[str]] = []
    for item in report.items:
        metric = item.metrics
        base = metric.integrated
        rows.append(
            [
                item.case.name,
                f"{base.mean_path_error_m:.2f}",
                f"{base.maximum_path_error_m:.2f}",
                f"{base.minimum_gap_m:.1f}",
                f"{base.peak_jerk_mps3:.1f}",
                f"{base.completion_percent:.0f}%",
                "PASS" if metric.pass_run else "FAIL",
                metric.failure_reason,
            ]
        )
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    render = lambda row: " | ".join(  # noqa: E731
        value.ljust(widths[index]) for index, value in enumerate(row)
    )
    print(render(headers))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(render(row))
    print(
        f"\nPass rate: {report.pass_count}/{len(report.items)} "
        f"({report.pass_rate_percent:.0f}%)"
    )
    print(
        f"Worst case: {report.worst_item.case.name} — "
        f"{report.worst_item.metrics.failure_reason}"
    )


def export_batch_csv(report: BatchReport, output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "test_id",
                "name",
                "mean_path_error_m",
                "maximum_path_error_m",
                "road_departure_percent",
                "speed_rmse_mps",
                "minimum_gap_m",
                "peak_jerk_mps3",
                "completion_percent",
                "pass",
                "failure_reason",
            ]
        )
        for item in report.items:
            metric = item.metrics
            base = metric.integrated
            writer.writerow(
                [
                    item.case.test_id,
                    item.case.name,
                    base.mean_path_error_m,
                    base.maximum_path_error_m,
                    base.road_departure_percent,
                    base.speed_rmse_mps,
                    base.minimum_gap_m,
                    base.peak_jerk_mps3,
                    base.completion_percent,
                    metric.pass_run,
                    metric.failure_reason,
                ]
            )
    return path
