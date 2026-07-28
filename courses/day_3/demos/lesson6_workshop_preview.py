"""Lesson 6: compare an aggressive baseline with a balanced configuration."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import (
    print_integrated_metrics_table,
    weighted_workshop_score,
)
from simulator.plotting import plot_comparison
from simulator.script_helpers import common_parser


def aggressive_scenario() -> IntegratedScenario:
    base = IntegratedScenario(
        duration=38.0,
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=True,
    )
    return replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=18.0,
            maximum_lateral_acceleration=4.5,
            preview_distance=3.0,
        ),
        acc=replace(
            base.acc,
            time_headway=0.65,
            emergency_ttc=0.7,
            emergency_gap=1.2,
        ),
    )


def balanced_scenario() -> IntegratedScenario:
    return IntegratedScenario()


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        "aggressive baseline": run_integrated(aggressive_scenario()),
        "balanced reference": run_integrated(balanced_scenario()),
    }
    print_integrated_metrics_table(results)
    for label, result in results.items():
        print(f"{label}: workshop score = {weighted_workshop_score(result)}")
    figure = plot_comparison(
        results,
        title="Lesson 6 — integrated traffic workshop preview",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
