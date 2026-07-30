"""Instructor solution: five starts with seeded measurement noise."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    initial_condition_cases,
    print_batch_report,
    run_batch,
)
from simulator.faults import FaultParameters


def main() -> None:
    cases = tuple(
        replace(
            case,
            faults=FaultParameters(
                position_noise_std=0.12,
                heading_noise_std_degrees=0.8,
                speed_noise_std=0.20,
                range_noise_std=0.30,
                random_seed=220 + index,
            ),
        )
        for index, case in enumerate(initial_condition_cases())
    )
    report = run_batch(BALANCED_CONFIGURATION, cases)
    print_batch_report(report)
    assert report.pass_rate_percent == 100.0


if __name__ == "__main__":
    main()
