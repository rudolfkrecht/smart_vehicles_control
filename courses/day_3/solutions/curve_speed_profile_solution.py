"""Instructor reference for the curve-speed tuning exercise."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import print_integrated_metrics_table


def main() -> None:
    base = IntegratedScenario(
        duration=29.0,
        enable_curve_speed=True,
        enable_traffic=False,
    )
    scenario = replace(
        base,
        speed_profile=replace(
            base.speed_profile,
            global_speed_limit=15.0,
            maximum_lateral_acceleration=2.5,
            preview_distance=14.0,
        ),
    )
    print_integrated_metrics_table(
        {"balanced curve profile": run_integrated(scenario)}
    )


if __name__ == "__main__":
    main()
