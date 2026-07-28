"""Student workshop: tune one frozen configuration on visible practice tests."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    ControllerConfiguration,
    export_batch_csv,
    practice_cases,
    print_batch_report,
    run_batch,
)
from simulator.robust_plotting import plot_batch_report
from simulator.script_helpers import common_parser, finish_figure


# TEAM CONFIGURATION — EDIT ONLY THIS BLOCK ---------------------------------
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
# ---------------------------------------------------------------------------


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    report = run_batch(CONFIGURATION, practice_cases())
    print_batch_report(report)
    output_csv = Path("day4_practice_results.csv")
    export_batch_csv(report, output_csv)
    print(f"Saved table: {output_csv.resolve()}")
    print(
        "\nSuccess target: 6/6 passes. Change a small number of parameters, "
        "predict the effect, rerun, and record the evidence."
    )
    figure = plot_batch_report(
        report,
        title="Day 4 practice challenge",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
