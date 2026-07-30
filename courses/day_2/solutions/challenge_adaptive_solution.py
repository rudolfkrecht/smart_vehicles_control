"""Instructor reference for the speed-dependent look-ahead challenge."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.tracking import PathFollowingScenario, run_path_following


BASE_LOOKAHEAD_M = 2.2
SPEED_GAIN_S = 0.32


def main() -> None:
    results = {}
    for speed in (6.0, 10.0, 14.0):
        results[f"{speed:.0f} m/s"] = run_path_following(
            scenario=PathFollowingScenario(
                speed=speed,
                duration=20.0,
                base_lookahead=BASE_LOOKAHEAD_M,
                speed_lookahead_gain=SPEED_GAIN_S,
                initial_lateral_offset=0.7,
                initial_heading_offset_degrees=0.0,
                disturbance_time=5.0,
                disturbance_offset=1.8,
            )
        )
    print(
        f"Reference rule: Ld = {BASE_LOOKAHEAD_M:.2f} "
        f"+ {SPEED_GAIN_S:.2f} * speed"
    )
    print_path_metrics_table(results)
    plot_tracking_comparison(
        results,
        title="Instructor reference: adaptive look-ahead",
        output="day2_adaptive_solution.png",
        show=True,
    )


if __name__ == "__main__":
    main()
