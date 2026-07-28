"""Lesson 4: compare an aggressive baseline with a balanced practice plan."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    AGGRESSIVE_CONFIGURATION,
    BALANCED_CONFIGURATION,
    practice_cases,
    print_batch_report,
    run_batch,
)
from simulator.robust_plotting import plot_robust_comparison
from simulator.script_helpers import common_parser, finish_figure


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    cases = practice_cases()
    aggressive = run_batch(AGGRESSIVE_CONFIGURATION, cases)
    balanced = run_batch(BALANCED_CONFIGURATION, cases)
    print("AGGRESSIVE BASELINE")
    print_batch_report(aggressive)
    print("\nBALANCED REFERENCE")
    print_batch_report(balanced)
    comparison = {
        "aggressive worst case": aggressive.worst_item.result,
        "balanced same test": next(
            item.result
            for item in balanced.items
            if item.case.test_id == aggressive.worst_item.case.test_id
        ),
    }
    figure = plot_robust_comparison(
        comparison,
        title="Lesson 4 — use practice tests before freezing the controller",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
