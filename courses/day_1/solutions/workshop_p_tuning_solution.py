"""Instructor reference for the proportional-gain workshop."""

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


KP_VALUES = (0.35, 0.45, 0.55)


def main() -> None:
    args = plot_arguments("workshop_p_tuning_solution.png")
    scenario = Scenario(duration=25.0, target_speed=15.0)
    results = {
        f"Kp = {kp:.2f}": run_simulation(PController(kp), scenario=scenario)
        for kp in KP_VALUES
    }
    print("\nINSTRUCTOR P-TUNING REFERENCE\n")
    print_metrics_table(results)
    figure = plot_comparison(results, title="Instructor P-tuning reference")
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
