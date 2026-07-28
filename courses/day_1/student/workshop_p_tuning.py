"""Lesson 5 student workshop: tune a proportional cruise controller.

This file works immediately. Change only the STUDENT TASK block, run the file
again, and record the printed metrics.
"""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import PController, Scenario, run_simulation  # noqa: E402
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import finish_figure, plot_comparison  # noqa: E402
from simulator.script_helpers import plot_arguments  # noqa: E402


# ===========================================================================
# STUDENT TASK
#
# 1. Run the supplied gains.
# 2. Predict which gain will give the best compromise.
# 3. Replace the values with three narrower candidates.
# 4. Select a final gain and justify it using the printed metrics.
#
# Suggested first candidates: 0.10, 0.35 and 0.70
# ===========================================================================
KP_VALUES = (0.10, 0.35, 0.70)

# Set this to False for the initial flat-road test. Then use True to test the
# selected controller on a hill.
ENABLE_HILL = False
# ===========================================================================


def main() -> None:
    args = plot_arguments("workshop_p_tuning.png")
    scenario = Scenario(
        duration=35.0,
        target_speed=15.0,
        hill_start=15.0 if ENABLE_HILL else None,
        hill_force=1_000.0 if ENABLE_HILL else 0.0,
    )
    results = {
        f"Kp = {kp:.3f}": run_simulation(
            PController(kp),
            scenario=scenario,
        )
        for kp in KP_VALUES
    }

    road = "hill test" if ENABLE_HILL else "flat-road test"
    print(f"\nP-CONTROLLER WORKSHOP — {road}")
    print("Requirements: overshoot < 10%, settling < 10 s, command within ±1.\n")
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title=f"P-controller tuning: {road}",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
