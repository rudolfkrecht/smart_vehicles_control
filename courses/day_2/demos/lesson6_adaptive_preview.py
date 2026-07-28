"""Lesson 6: fixed and speed-dependent look-ahead under a disturbance."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.script_helpers import common_parser
from simulator.tracking import PathFollowingScenario, run_path_following


HIGH_SPEED_MPS = 14.0
DISTURBANCE_TIME_S = 7.0
DISTURBANCE_OFFSET_M = 2.0


def main() -> None:
    args = common_parser(__doc__).parse_args()
    common = dict(
        speed=HIGH_SPEED_MPS,
        duration=18.0,
        initial_lateral_offset=0.5,
        initial_heading_offset_degrees=0.0,
        disturbance_time=DISTURBANCE_TIME_S,
        disturbance_offset=DISTURBANCE_OFFSET_M,
    )
    results = {
        "Fixed Ld = 4.0 m": run_path_following(
            scenario=PathFollowingScenario(
                base_lookahead=4.0,
                speed_lookahead_gain=0.0,
                **common,
            )
        ),
        "Adaptive Ld = 2 + 0.35v": run_path_following(
            scenario=PathFollowingScenario(
                base_lookahead=2.0,
                speed_lookahead_gain=0.35,
                **common,
            )
        ),
    }
    print_path_metrics_table(results)
    plot_tracking_comparison(
        results,
        title="High-speed disturbance recovery",
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
