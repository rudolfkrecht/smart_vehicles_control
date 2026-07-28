"""Lesson 3: compare constant, late and previewed curve-speed control."""

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
GLOBAL_SPEED_LIMIT_MPS = 15.0
MAX_LATERAL_ACCELERATION_MPS2 = 2.5
PREVIEW_DISTANCE_M = 14.0


def make_scenario(*, curve_aware: bool, preview: float) -> IntegratedScenario:
    base = IntegratedScenario(
        duration=27.0,
        enable_curve_speed=curve_aware,
        enable_traffic=False,
    )
    return replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=GLOBAL_SPEED_LIMIT_MPS,
            maximum_lateral_acceleration=MAX_LATERAL_ACCELERATION_MPS2,
            preview_distance=preview,
        ),
    )


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        "constant speed": run_integrated(
            make_scenario(curve_aware=False, preview=0.0)
        ),
        "curve target, no preview": run_integrated(
            make_scenario(curve_aware=True, preview=0.0)
        ),
        f"preview {PREVIEW_DISTANCE_M:g} m": run_integrated(
            make_scenario(
                curve_aware=True,
                preview=PREVIEW_DISTANCE_M,
            )
        ),
    }
    print_integrated_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Lesson 3 — anticipating curves before steering into them",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
