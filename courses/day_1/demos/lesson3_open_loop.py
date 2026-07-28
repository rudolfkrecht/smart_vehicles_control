"""Lesson 3: explore constant command and vehicle drag.

Prediction questions:
    Does a constant accelerator command create constant acceleration?
    Which vehicle reaches the highest speed?
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import (  # noqa: E402
    OpenLoopController,
    Scenario,
    VehicleParameters,
    run_simulation,
)
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import finish_figure, plot_comparison  # noqa: E402
from simulator.script_helpers import plot_arguments  # noqa: E402


# ---------------------------------------------------------------------------
# SAFE LIVE MODIFICATIONS
# Suggested command values: 0.20, 0.35, 0.60
# Suggested drag values: 2.0, 4.0, 8.0
# ---------------------------------------------------------------------------
OPEN_LOOP_COMMAND = 0.35
DRAG_VALUES = (2.0, 4.0, 8.0)
# ---------------------------------------------------------------------------


def main() -> None:
    args = plot_arguments("lesson3_open_loop.png")
    scenario = Scenario(duration=25.0, target_speed=15.0)
    base_vehicle = VehicleParameters()
    results = {}

    for drag in DRAG_VALUES:
        vehicle = replace(base_vehicle, aerodynamic_drag=drag)
        label = f"Drag = {drag:.1f}"
        results[label] = run_simulation(
            OpenLoopController(OPEN_LOOP_COMMAND),
            scenario=scenario,
            vehicle=vehicle,
        )

    print(f"\nCONSTANT COMMAND = {OPEN_LOOP_COMMAND:.2f}")
    print("The target line is a reference only; open loop does not use it.\n")
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Open-loop response for different drag coefficients",
    )
    finish_figure(figure, save_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
