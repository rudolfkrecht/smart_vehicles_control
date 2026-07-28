"""Advanced Lesson 5 task: evaluate several proportional gains automatically."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import PController, Scenario, calculate_metrics, run_simulation  # noqa: E402
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import finish_figure, plot_comparison  # noqa: E402
from simulator.script_helpers import plot_arguments  # noqa: E402


# ===========================================================================
# STUDENT TASK
# Change the search interval or number of candidates. Then inspect whether the
# gain with the lowest RMSE also gives an acceptable command and smooth response.
# ===========================================================================
KP_CANDIDATES = np.linspace(0.10, 1.00, 10)
# ===========================================================================


def main() -> None:
    args = plot_arguments("advanced_p_sweep.png")
    scenario = Scenario(duration=25.0, target_speed=15.0)
    all_results = {
        f"Kp={kp:.2f}": run_simulation(PController(float(kp)), scenario=scenario)
        for kp in KP_CANDIDATES
    }
    best_label = min(
        all_results,
        key=lambda label: calculate_metrics(all_results[label]).rmse_mps,
    )

    selected_results = {
        next(iter(all_results)): next(iter(all_results.values())),
        best_label: all_results[best_label],
        next(reversed(all_results)): next(reversed(all_results.values())),
    }
    print("\nAUTOMATED P-GAIN SWEEP\n")
    print_metrics_table(all_results)
    print(f"\nLowest speed RMSE in this search: {best_label}")
    figure = plot_comparison(
        selected_results,
        title="Low, selected and high proportional gains",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
