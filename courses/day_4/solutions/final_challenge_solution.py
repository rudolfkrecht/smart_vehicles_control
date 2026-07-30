"""Instructor-only evaluation of the balanced reference configuration."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    evaluation_cases,
    print_batch_report,
    run_batch,
)
from simulator.evaluation import print_challenge_score, score_challenge_run


def main() -> None:
    report = run_batch(BALANCED_CONFIGURATION, evaluation_cases())
    print_batch_report(report)
    score = score_challenge_run(
        report.worst_item.result,
        robustness_pass_rate=report.pass_rate_percent,
        technical_explanation=85.0,
    )
    print_challenge_score(score)
    assert report.pass_rate_percent == 100.0
    assert score.final_score > 70.0


if __name__ == "__main__":
    main()
