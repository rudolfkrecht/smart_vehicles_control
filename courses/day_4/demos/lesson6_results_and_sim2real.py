"""Lesson 6: present evidence and connect simulation limits to real vehicles."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    evaluation_cases,
    run_batch,
)
from simulator.evaluation import score_challenge_run
from simulator.robust_plotting import plot_robust_summary
from simulator.script_helpers import common_parser, finish_figure


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    report = run_batch(BALANCED_CONFIGURATION, evaluation_cases())
    worst = report.worst_item
    score = score_challenge_run(
        worst.result,
        robustness_pass_rate=report.pass_rate_percent,
        technical_explanation=85.0,
    )
    print("Three-sentence presentation template")
    print(
        "1. We selected a speed-aware look-ahead, conservative curve-speed "
        "limit and 1.5 s ACC headway."
    )
    print(
        f"2. The controller passed {report.pass_count}/{len(report.items)} "
        f"evaluation cases; the worst was {worst.case.name} with "
        f"{worst.metrics.integrated.maximum_path_error_m:.2f} m maximum "
        f"path error and {worst.metrics.integrated.minimum_gap_m:.1f} m "
        "minimum gap."
    )
    print(
        "3. A real vehicle would still require tyre/road dynamics, calibrated "
        "sensors, timing measurements, hardware limits and fail-safe braking."
    )
    print(f"\nExample final score: {score.final_score:.1f}/100")
    figure = plot_robust_summary(
        worst.result,
        title="Lesson 6 — evidence for the final presentation",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
