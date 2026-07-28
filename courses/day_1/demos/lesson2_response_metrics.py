"""Lesson 2: connect response plots to numerical requirements."""

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


# ---------------------------------------------------------------------------
# SAFE LIVE MODIFICATIONS
# Ask students to predict which response will meet a 5% settling band.
# ---------------------------------------------------------------------------
SLOW_KP = 0.08
TUNED_KP = 0.45
PI_KP = 0.35
PI_KI = 0.10
# ---------------------------------------------------------------------------


def main() -> None:
    args = plot_arguments("lesson2_response_metrics.png")
    scenario = Scenario(duration=25.0, target_speed=15.0)
    results = {
        f"Slow P (Kp={SLOW_KP})": run_simulation(
            PController(SLOW_KP),
            scenario=scenario,
        ),
        f"Tuned P (Kp={TUNED_KP})": run_simulation(
            PController(TUNED_KP),
            scenario=scenario,
        ),
        f"PI ({PI_KP}, {PI_KI})": run_simulation(
            PIController(PI_KP, PI_KI),
            scenario=scenario,
        ),
    }

    print("\nRESPONSE REQUIREMENTS")
    print("Target: 15 m/s, overshoot < 10%, settling time < 10 s\n")
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Rise time, overshoot and settling behaviour",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
