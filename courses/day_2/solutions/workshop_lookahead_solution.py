"""Instructor reference for the Lesson 5 look-ahead workshop."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.tracking import PathFollowingScenario, run_path_following


REFINED_CANDIDATES_M = (3.5, 4.5, 5.5, 6.5)
VEHICLE_SPEED_MPS = 8.0


def main() -> None:
    results = {
        f"Ld = {lookahead:.1f} m": run_path_following(
            scenario=PathFollowingScenario(
                speed=VEHICLE_SPEED_MPS,
                base_lookahead=lookahead,
                speed_lookahead_gain=0.0,
            )
        )
        for lookahead in REFINED_CANDIDATES_M
    }
    print_path_metrics_table(results)
    plot_tracking_comparison(
        results,
        title="Instructor reference: refined look-ahead comparison",
        output="day2_workshop_solution.png",
        show=True,
    )


if __name__ == "__main__":
    main()
