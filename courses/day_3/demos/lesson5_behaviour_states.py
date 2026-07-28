"""Lesson 5: visualise Cruise, Follow, Brake and Emergency transitions."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import calculate_integrated_metrics
from simulator.plotting import plot_integrated_summary
from simulator.script_helpers import common_parser


# SAFE LIVE MODIFICATIONS
EMERGENCY_TTC_S = 1.25
BRAKE_ENTRY_RATIO = 0.78


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    base = IntegratedScenario(
        duration=34.0,
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=True,
        lead_preset="late_brake",
    )
    scenario = replace(
        base,
        acc=replace(
            base.acc,
            emergency_ttc=EMERGENCY_TTC_S,
            brake_entry_ratio=BRAKE_ENTRY_RATIO,
        ),
    )
    result = run_integrated(scenario)
    metric = calculate_integrated_metrics(result)
    transitions = []
    previous = None
    for time, state in zip(result.time, result.behaviour_state):
        if state != previous:
            transitions.append(f"{time:5.1f} s → {state}")
            previous = state
    print("State transitions")
    print("\n".join(transitions))
    print(
        f"\nminimum gap: {metric.minimum_gap_m:.2f} m; "
        f"minimum TTC: {metric.minimum_ttc_s:.2f} s; "
        f"collision samples: {metric.collision_samples}"
    )
    figure = plot_integrated_summary(
        result,
        title="Lesson 5 — discrete behaviour and safety overrides",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
