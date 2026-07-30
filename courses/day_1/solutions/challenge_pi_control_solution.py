"""Instructor reference for the final Day 1 PI challenge."""

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


KP = 0.35
KI = 0.10


def main() -> None:
    args = plot_arguments("challenge_pi_control_solution.png")
    scenario = Scenario(
        duration=40.0,
        target_speed=15.0,
        hill_start=15.0,
        hill_force=1_500.0,
    )
    results = {
        "P baseline": run_simulation(PController(KP), scenario=scenario),
        "PI solution": run_simulation(
            PIController(KP, KI, anti_windup=True),
            scenario=scenario,
        ),
    }
    print("\nINSTRUCTOR PI-CHALLENGE REFERENCE\n")
    print_metrics_table(results)
    figure = plot_comparison(results, title="Instructor PI solution")
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
