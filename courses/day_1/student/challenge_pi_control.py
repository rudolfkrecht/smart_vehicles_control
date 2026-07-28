"""Lesson 6 student challenge: reject a persistent hill disturbance.

The starter configuration uses no integral action, so the "PI candidate"
initially behaves like a P controller. Change KI and compare the final error.
"""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import PController, PIController, Scenario, run_simulation  # noqa: E402
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import finish_figure, plot_comparison  # noqa: E402
from simulator.script_helpers import plot_arguments  # noqa: E402


# ===========================================================================
# STUDENT TASK
#
# 1. Run this file with KI = 0.00.
# 2. Try small positive KI values such as 0.03, 0.08 and 0.15.
# 3. Keep overshoot below 10% and reduce the final speed error below 0.3 m/s.
# 4. Set ANTI_WINDUP to False once and explain the difference.
# ===========================================================================
KP = 0.35
KI = 0.00
ANTI_WINDUP = True
# ===========================================================================


def main() -> None:
    args = plot_arguments("challenge_pi_control.png")
    scenario = Scenario(
        duration=40.0,
        target_speed=15.0,
        hill_start=15.0,
        hill_force=1_500.0,
    )
    results = {
        f"P baseline (Kp={KP:.2f})": run_simulation(
            PController(KP),
            scenario=scenario,
        ),
        f"PI candidate (Ki={KI:.2f})": run_simulation(
            PIController(KP, KI, anti_windup=ANTI_WINDUP),
            scenario=scenario,
        ),
    }

    print("\nDAY 1 PI CHALLENGE")
    print("Goal: final error < 0.3 m/s and overshoot < 10%.\n")
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title="P versus student PI controller on a persistent hill",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
