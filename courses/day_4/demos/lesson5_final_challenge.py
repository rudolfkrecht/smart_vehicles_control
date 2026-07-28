"""Lesson 5: score frozen controllers on the unseen evaluation suite."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    AGGRESSIVE_CONFIGURATION,
    BALANCED_CONFIGURATION,
    evaluation_cases,
    print_batch_report,
    run_batch,
)
from simulator.evaluation import (
    print_challenge_score,
    score_challenge_run,
)
from simulator.robust_plotting import plot_challenge_score
from simulator.script_helpers import common_parser, finish_figure


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    aggressive = run_batch(AGGRESSIVE_CONFIGURATION, evaluation_cases())
    balanced = run_batch(BALANCED_CONFIGURATION, evaluation_cases())
    print("OFFICIAL RUN — AGGRESSIVE BASELINE")
    print_batch_report(aggressive)
    aggressive_score = score_challenge_run(
        aggressive.worst_item.result,
        robustness_pass_rate=aggressive.pass_rate_percent,
        technical_explanation=75.0,
    )
    print_challenge_score(aggressive_score)
    print("\nOFFICIAL RUN — BALANCED REFERENCE")
    print_batch_report(balanced)
    balanced_score = score_challenge_run(
        balanced.worst_item.result,
        robustness_pass_rate=balanced.pass_rate_percent,
        technical_explanation=85.0,
    )
    print_challenge_score(balanced_score)
    figure = plot_challenge_score(
        balanced_score,
        title="Lesson 5 — final autonomous-driving challenge",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
