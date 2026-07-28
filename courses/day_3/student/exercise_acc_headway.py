"""Student exercise: select an ACC time headway and justify it."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import print_integrated_metrics_table
from simulator.plotting import plot_comparison
from simulator.script_helpers import common_parser


# STUDENT EDIT AREA ---------------------------------------------------------
TIME_HEADWAY_CANDIDATES_S = (0.7, 1.5, 2.2)
STANDSTILL_GAP_M = 5.0
# END STUDENT EDIT AREA -----------------------------------------------------


def run_candidate(headway: float):
    base = IntegratedScenario(
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=False,
        lead_preset="stop_and_go",
    )
    scenario = replace(
        base,
        acc=replace(
            base.acc,
            time_headway=headway,
            standstill_gap=STANDSTILL_GAP_M,
        ),
    )
    return run_integrated(scenario)


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        f"T_h = {value:g} s": run_candidate(value)
        for value in TIME_HEADWAY_CANDIDATES_S
    }
    print_integrated_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Student exercise — ACC headway selection",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
