"""Lesson 4: compare Adaptive Cruise Control time-headway settings."""

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


# SAFE LIVE MODIFICATIONS
TIME_HEADWAYS_S = (0.7, 1.5, 2.2)


def make_scenario(headway: float) -> IntegratedScenario:
    base = IntegratedScenario(
        duration=38.0,
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=False,
        lead_preset="stop_and_go",
    )
    return replace(
        base,
        acc=replace(base.acc, time_headway=headway),
    )


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        f"T_h = {headway:g} s": run_integrated(
            make_scenario(headway)
        )
        for headway in TIME_HEADWAYS_S
    }
    print_integrated_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Lesson 4 — ACC safety versus traffic efficiency",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
