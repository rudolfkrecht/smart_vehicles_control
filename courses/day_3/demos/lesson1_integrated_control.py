"""Lesson 1: show why steering and speed cannot be tuned independently."""

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
BASELINE_SPEED_MPS = 10.0
EXCESSIVE_SPEED_MPS = 18.0


def scenario(speed: float) -> IntegratedScenario:
    base = IntegratedScenario(
        duration=24.0,
        enable_curve_speed=False,
        enable_traffic=False,
    )
    return replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=speed,
        ),
    )


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    results = {
        f"baseline {BASELINE_SPEED_MPS:g} m/s": run_integrated(
            scenario(BASELINE_SPEED_MPS)
        ),
        f"excessive {EXCESSIVE_SPEED_MPS:g} m/s": run_integrated(
            scenario(EXCESSIVE_SPEED_MPS)
        ),
    }
    print_integrated_metrics_table(results)
    figure = plot_comparison(
        results,
        title="Lesson 1 — integrated control exposes a speed constraint",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
