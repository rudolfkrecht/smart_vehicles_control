"""Lesson 5 student workshop: tune Pure Pursuit systematically."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import calculate_path_metrics, print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.tracking import PathFollowingScenario, run_path_following


# STUDENT TASK 1: predict which candidate is most accurate and which is smoothest.
# STUDENT TASK 2: replace one candidate after the first run to narrow the range.
LOOKAHEAD_CANDIDATES_M = (2.0, 5.0, 10.0)

# STUDENT TASK 3: first use 8.0 m/s, then test the chosen value at 13.0 m/s.
VEHICLE_SPEED_MPS = 8.0

# Road boundaries are +/- this distance from the centre line.
ROAD_HALF_WIDTH_M = 3.5


def main() -> None:
    results = {}
    for lookahead in LOOKAHEAD_CANDIDATES_M:
        label = f"Ld = {lookahead:.1f} m"
        results[label] = run_path_following(
            scenario=PathFollowingScenario(
                speed=VEHICLE_SPEED_MPS,
                base_lookahead=lookahead,
                speed_lookahead_gain=0.0,
                road_half_width=ROAD_HALF_WIDTH_M,
            )
        )

    print_path_metrics_table(results)
    print("\nRequirement check:")
    for label, result in results.items():
        metrics = calculate_path_metrics(result)
        accurate = metrics.mean_absolute_error_m < 0.70
        safe = metrics.road_departure_percent == 0.0
        complete = metrics.completion_percent > 95.0
        print(
            f"{label:12s} accurate={str(accurate):5s} "
            f"safe={str(safe):5s} complete={str(complete):5s}"
        )

    plot_tracking_comparison(
        results,
        title=f"Student look-ahead workshop at {VEHICLE_SPEED_MPS:.1f} m/s",
        output="day2_workshop_results.png",
        show=True,
    )


if __name__ == "__main__":
    main()
