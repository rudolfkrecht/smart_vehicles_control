"""Student exercise: tune the road-speed profile using objective evidence."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import (
    calculate_integrated_metrics,
    print_integrated_metrics_table,
)
from simulator.plotting import plot_comparison
from simulator.script_helpers import common_parser


# STUDENT EDIT AREA ---------------------------------------------------------
MAX_LATERAL_ACCELERATION_CANDIDATES = (1.5, 2.5, 4.5)
GLOBAL_SPEED_LIMIT_MPS = 15.0
PREVIEW_DISTANCE_M = 14.0
# END STUDENT EDIT AREA -----------------------------------------------------


def run_candidate(maximum_lateral_acceleration: float):
    base = IntegratedScenario(
        duration=29.0,
        enable_curve_speed=True,
        enable_traffic=False,
    )
    scenario = replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=GLOBAL_SPEED_LIMIT_MPS,
            maximum_lateral_acceleration=(
                maximum_lateral_acceleration
            ),
            preview_distance=PREVIEW_DISTANCE_M,
        ),
    )
    return run_integrated(scenario)


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        f"a_y,max = {value:g} m/s²": run_candidate(value)
        for value in MAX_LATERAL_ACCELERATION_CANDIDATES
    }
    print_integrated_metrics_table(results)
    print("\nCornering evidence")
    for label, result in results.items():
        metric = calculate_integrated_metrics(result)
        print(
            f"{label}: peak actual lateral acceleration "
            f"{metric.peak_lateral_acceleration_mps2:.2f} m/s²; "
            f"completion time {metric.completion_time_s:.1f} s"
        )
    figure = plot_comparison(
        results,
        title="Student exercise — curve-speed profile tuning",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
