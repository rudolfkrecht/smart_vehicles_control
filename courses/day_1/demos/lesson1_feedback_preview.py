"""Lesson 1: preview the Day 1 cruise-control challenge.

Prediction question:
    Which controller will maintain speed after the hill begins?
"""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import (  # noqa: E402
    OpenLoopController,
    PIController,
    Scenario,
    run_simulation,
)
from simulator.metrics import print_metrics_table  # noqa: E402
from simulator.plotting import (  # noqa: E402
    finish_figure,
    plot_comparison,
)
from simulator.script_helpers import plot_arguments  # noqa: E402


# ---------------------------------------------------------------------------
# SAFE LIVE MODIFICATIONS
# ---------------------------------------------------------------------------
ENABLE_HILL = True
HILL_FORCE_N = 1_500.0
OPEN_LOOP_COMMAND = 0.35
TARGET_SPEED_MPS = 15.0
# ---------------------------------------------------------------------------


def main() -> None:
    args = plot_arguments("lesson1_feedback_preview.png")
    scenario = Scenario(
        duration=35.0,
        target_speed=TARGET_SPEED_MPS,
        hill_start=15.0 if ENABLE_HILL else None,
        hill_force=HILL_FORCE_N if ENABLE_HILL else 0.0,
    )

    results = {
        "Open loop": run_simulation(
            OpenLoopController(OPEN_LOOP_COMMAND),
            scenario=scenario,
        ),
        "PI feedback": run_simulation(
            PIController(kp=0.35, ki=0.10, anti_windup=True),
            scenario=scenario,
        ),
    }

    print("\nDAY 1 PREVIEW")
    print("Question: Which controller rejects the hill disturbance?\n")
    print_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Open-loop command versus feedback speed control",
    )
    finish_figure(
        figure,
        save_path=args.save,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
