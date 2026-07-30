"""Instructor reference for the visible practice suite."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    practice_cases,
    print_batch_report,
    run_batch,
)


def main() -> None:
    report = run_batch(BALANCED_CONFIGURATION, practice_cases())
    print_batch_report(report)
    assert report.pass_rate_percent == 100.0


if __name__ == "__main__":
    main()
