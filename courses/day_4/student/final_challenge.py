"""Official run: evaluate a frozen team controller on the instructor suite."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    ControllerConfiguration,
    evaluation_cases,
    export_batch_csv,
    print_batch_report,
    run_batch,
)
from simulator.evaluation import (
    print_challenge_score,
    score_challenge_run,
)
from simulator.robust_plotting import plot_challenge_score
from simulator.script_helpers import common_parser, finish_figure


# PASTE THE FROZEN TEAM CONFIGURATION HERE ----------------------------------
CONFIGURATION = ControllerConfiguration(
    global_speed_limit=18.0,
    maximum_lateral_acceleration=4.2,
    curve_preview_distance=4.0,
    speed_profile_smoothing=3,
    base_lookahead=2.0,
    speed_lookahead_gain=0.08,
    time_headway=0.8,
    standstill_gap=2.0,
    emergency_ttc=0.75,
    emergency_gap=1.2,
    maximum_jerk=8.0,
)

# Instructor enters 0–100 after the three-minute explanation.
TECHNICAL_EXPLANATION_SCORE = 75.0
# ---------------------------------------------------------------------------


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    report = run_batch(CONFIGURATION, evaluation_cases())
    print_batch_report(report)
    score = score_challenge_run(
        report.worst_item.result,
        robustness_pass_rate=report.pass_rate_percent,
        technical_explanation=TECHNICAL_EXPLANATION_SCORE,
    )
    print_challenge_score(score)
    csv_path = export_batch_csv(report, "day4_official_results.csv")
    print(f"Saved table: {csv_path.resolve()}")
    figure = plot_challenge_score(
        score,
        title="Official Day 4 challenge score",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
