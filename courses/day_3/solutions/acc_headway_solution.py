"""Instructor reference for the ACC headway exercise."""

from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import print_integrated_metrics_table


def main() -> None:
    base = IntegratedScenario(enable_state_machine=False)
    scenario = replace(
        base,
        acc=replace(
            base.acc,
            time_headway=1.5,
            standstill_gap=5.0,
        ),
    )
    print_integrated_metrics_table(
        {"balanced ACC": run_integrated(scenario)}
    )


if __name__ == "__main__":
    main()
