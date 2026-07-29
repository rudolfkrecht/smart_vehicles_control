"""Evaluate one Day 4 controller over the repeatable scenario suite."""

from __future__ import annotations

import argparse
import csv
import importlib
from pathlib import Path
from typing import Callable

from simulator.controllers import ReferenceController
from simulator.faults import SCENARIOS
from simulator.simulation import Simulation


def load_controller(name: str):
    if name == "reference":
        return ReferenceController()
    module_name = (
        "student_controller_solution"
        if name == "solution"
        else "student_controller"
    )
    importlib.invalidate_caches()
    module = importlib.import_module(module_name)
    module = importlib.reload(module)
    return module.StudentController()


def assess(
    scenario_name: str,
    metrics: dict[str, float | int | None],
) -> tuple[bool, str]:
    reasons: list[str] = []
    collision_samples = int(metrics["collision_samples"] or 0)
    outside_road = float(metrics["outside_road_percent"] or 0.0)
    minimum_gap_value = metrics["minimum_gap_m"]
    maximum_lane_error_value = metrics["maximum_cross_track_error_m"]
    progress_value = metrics["lap_progress_percent"]
    minimum_gap = (
        float(minimum_gap_value)
        if minimum_gap_value is not None
        else -1.0
    )
    maximum_lane_error = (
        float(maximum_lane_error_value)
        if maximum_lane_error_value is not None
        else 99.0
    )
    progress = float(progress_value) if progress_value is not None else 0.0

    if collision_samples > 0:
        reasons.append("collision")
    if outside_road > 0.0:
        reasons.append("road departure")
    if minimum_gap <= 3.0:
        reasons.append("minimum gap")
    if maximum_lane_error > 1.60:
        reasons.append("lane recovery")
    if progress < 95.0:
        reasons.append("insufficient progress")
    if (
        scenario_name in {"radar_dropout", "combined"}
        and int(metrics["safe_stop_samples"] or 0) == 0
    ):
        reasons.append("no sensor-fault fallback")
    return not reasons, ", ".join(reasons) if reasons else "all criteria met"


def run_suite(
    controller_factory: Callable[[], object],
    *,
    duration: float = 105.0,
    target_speed: float = 14.0,
) -> list[dict[str, float | int | str | bool | None]]:
    rows: list[dict[str, float | int | str | bool | None]] = []
    for scenario_name, scenario in SCENARIOS.items():
        simulation = Simulation(
            controller_factory(),
            target_speed=target_speed,
            scenario=scenario,
        )
        simulation.run(duration)
        metrics = simulation.metrics()
        passed, reason = assess(scenario_name, metrics)
        rows.append(
            {
                "scenario": scenario_name,
                "label": scenario.label,
                "passed": passed,
                "reason": reason,
                **metrics,
            }
        )
    return rows


def save_results(
    rows: list[dict[str, float | int | str | bool | None]],
    path: Path,
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the cumulative Day 4 ADAS evaluation suite."
    )
    parser.add_argument(
        "--controller",
        choices=("student", "solution", "reference"),
        default="student",
    )
    parser.add_argument("--duration", type=float, default=105.0)
    parser.add_argument("--target-speed", type=float, default=14.0)
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("day4_results.csv"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    rows = run_suite(
        lambda: load_controller(args.controller),
        duration=args.duration,
        target_speed=args.target_speed,
    )
    print(
        "Scenario           Result  Min gap  Max lane error  Progress  Note"
    )
    print("-" * 82)
    for row in rows:
        print(
            f"{str(row['scenario']):18s}"
            f"{'PASS' if row['passed'] else 'FAIL':7s}"
            f"{float(row['minimum_gap_m']):8.2f} m"
            f"{float(row['maximum_cross_track_error_m']):12.2f} m"
            f"{float(row['lap_progress_percent']):9.1f}%  "
            f"{row['reason']}"
        )
    passed = sum(bool(row["passed"]) for row in rows)
    output = save_results(rows, args.csv)
    print(f"\nSuite result: {passed}/{len(rows)} scenarios passed")
    print(f"CSV: {output.resolve()}")


if __name__ == "__main__":
    main()
