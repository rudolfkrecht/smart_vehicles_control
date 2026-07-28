"""Lesson 4: compare short, balanced and long Pure Pursuit look-ahead."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import print_path_metrics_table
from simulator.plotting import plot_tracking_comparison
from simulator.script_helpers import common_parser
from simulator.tracking import PathFollowingScenario, run_path_following


LOOKAHEAD_VALUES_M = (2.0, 5.0, 10.0)
VEHICLE_SPEED_MPS = 9.0


def main() -> None:
    args = common_parser(__doc__).parse_args()
    results = {
        f"Ld = {lookahead:.1f} m": run_path_following(
            scenario=PathFollowingScenario(
                speed=VEHICLE_SPEED_MPS,
                base_lookahead=lookahead,
                speed_lookahead_gain=0.0,
            )
        )
        for lookahead in LOOKAHEAD_VALUES_M
    }
    print_path_metrics_table(results)
    plot_tracking_comparison(
        results,
        title="Pure Pursuit tuning: accuracy versus smoothness",
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
