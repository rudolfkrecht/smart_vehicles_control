"""Lesson 6: demonstrate integral windup and conditional anti-windup."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import PIController, Scenario, run_simulation  # noqa: E402
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import finish_figure, plot_comparison  # noqa: E402
from simulator.script_helpers import plot_arguments  # noqa: E402


# ---------------------------------------------------------------------------
# SAFE LIVE MODIFICATIONS
# The steep hill makes 15 m/s temporarily impossible. Compare the response
# after the hill ends.
# ---------------------------------------------------------------------------
KP = 0.30
KI = 0.14
HILL_FORCE_N = 5_000.0
HILL_START_S = 10.0
HILL_END_S = 24.0
# ---------------------------------------------------------------------------


def main() -> None:
    args = plot_arguments("lesson6_windup.png")
    scenario = Scenario(
        duration=40.0,
        target_speed=15.0,
        initial_speed=15.0,
        hill_start=HILL_START_S,
        hill_end=HILL_END_S,
        hill_force=HILL_FORCE_N,
    )
    results = {
        "PI without anti-windup": run_simulation(
            PIController(KP, KI, anti_windup=False),
            scenario=scenario,
        ),
        "PI with anti-windup": run_simulation(
            PIController(KP, KI, anti_windup=True),
            scenario=scenario,
        ),
    }

    print("\nINTEGRAL WINDUP")
    print(
        "During the steep hill, the speed target is physically unreachable. "
        "Observe what happens when the hill ends.\n"
    )
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Integral windup after prolonged actuator saturation",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
