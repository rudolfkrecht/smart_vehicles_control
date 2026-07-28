"""Final Day 3 workshop: tune one controller for the complete scenario."""

from __future__ import annotations

from dataclasses import replace
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import (
    calculate_integrated_metrics,
    print_integrated_metrics_table,
    weighted_workshop_score,
)
from simulator.plotting import plot_integrated_summary
from simulator.script_helpers import common_parser


# STUDENT EDIT AREA ---------------------------------------------------------
GLOBAL_SPEED_LIMIT_MPS = 18.0
MAX_LATERAL_ACCELERATION_MPS2 = 4.5
CURVE_PREVIEW_DISTANCE_M = 3.0
TIME_HEADWAY_S = 0.65
EMERGENCY_TTC_S = 0.70
EMERGENCY_GAP_M = 1.2
# END STUDENT EDIT AREA -----------------------------------------------------


def build_scenario() -> IntegratedScenario:
    base = IntegratedScenario(
        enable_curve_speed=True,
        enable_traffic=True,
        enable_state_machine=True,
        lead_preset="stop_and_go",
    )
    return replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=GLOBAL_SPEED_LIMIT_MPS,
            maximum_lateral_acceleration=(
                MAX_LATERAL_ACCELERATION_MPS2
            ),
            preview_distance=CURVE_PREVIEW_DISTANCE_M,
        ),
        acc=replace(
            base.acc,
            time_headway=TIME_HEADWAY_S,
            emergency_ttc=EMERGENCY_TTC_S,
            emergency_gap=EMERGENCY_GAP_M,
        ),
    )


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    result = run_integrated(build_scenario())
    metric = calculate_integrated_metrics(result)
    print_integrated_metrics_table({"student controller": result})
    score = weighted_workshop_score(result)
    print(f"\nWorkshop score: {'DISQUALIFIED' if math.isinf(score) else f'{score:.3f}'}")

    checks = {
        "no collision": metric.collision_samples == 0,
        "remain inside road": metric.road_departure_percent == 0.0,
        "minimum gap at least 3 m": metric.minimum_gap_m >= 3.0,
        "peak lateral acceleration at most 3.5 m/s²": (
            metric.peak_lateral_acceleration_mps2 <= 3.5
        ),
        "complete at least 95%": metric.completion_percent >= 95.0,
    }
    print("\nSuccess criteria")
    for name, passed in checks.items():
        print(f"[{'PASS' if passed else 'FAIL'}] {name}")

    figure = plot_integrated_summary(
        result,
        title="Day 3 integrated traffic workshop",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
