"""Lesson 6 challenge: tune one rule for low- and high-speed tracking."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import calculate_path_metrics, print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.tracking import PathFollowingScenario, run_path_following


# STUDENT TASK: tune these two constants. Start with BASE=3.0 and GAIN=0.0.
BASE_LOOKAHEAD_M = 3.0
SPEED_GAIN_S = 0.0

TEST_SPEEDS_MPS = (6.0, 14.0)
ENABLE_DISTURBANCE = True


def scenario_for(speed: float) -> PathFollowingScenario:
    return PathFollowingScenario(
        speed=speed,
        duration=20.0,
        base_lookahead=BASE_LOOKAHEAD_M,
        speed_lookahead_gain=SPEED_GAIN_S,
        initial_lateral_offset=0.7,
        initial_heading_offset_degrees=0.0,
        disturbance_time=5.0 if ENABLE_DISTURBANCE else None,
        disturbance_offset=1.8 if ENABLE_DISTURBANCE else 0.0,
    )


def main() -> None:
    results = {
        f"{speed:.0f} m/s": run_path_following(
            scenario=scenario_for(speed)
        )
        for speed in TEST_SPEEDS_MPS
    }
    print(
        f"Rule: Ld = {BASE_LOOKAHEAD_M:.2f} "
        f"+ {SPEED_GAIN_S:.2f} * speed"
    )
    print_path_metrics_table(results)

    passed = True
    for label, result in results.items():
        metrics = calculate_path_metrics(result)
        case_passed = (
            metrics.mean_absolute_error_m < 0.85
            and metrics.road_departure_percent == 0.0
            and metrics.completion_percent > 90.0
        )
        passed &= case_passed
        print(f"{label}: {'PASS' if case_passed else 'REFINE'}")
    print("Overall:", "PASS" if passed else "REFINE PARAMETERS")

    plot_tracking_comparison(
        results,
        title="One adaptive rule at two speeds",
        output="day2_adaptive_challenge.png",
        show=True,
    )


if __name__ == "__main__":
    main()
