"""Lesson 2: connect road curvature to safe cornering speed."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt

from simulator.paths import make_reference_path
from simulator.plotting import plot_speed_profile
from simulator.script_helpers import common_parser
from simulator.speed_profile import (
    SpeedProfileParameters,
    build_speed_profile,
    safe_cornering_speed,
)


# SAFE LIVE MODIFICATIONS
MAX_LATERAL_ACCELERATION_MPS2 = 2.5
GLOBAL_SPEED_LIMIT_MPS = 15.0
EXAMPLE_CURVATURES_PER_M = (0.01, 0.04, 0.10)


def main() -> None:
    parser = common_parser(__doc__ or "")
    args = parser.parse_args()
    print("Curvature [1/m] | Radius [m] | Safe speed [m/s]")
    print("----------------+------------+-----------------")
    for curvature in EXAMPLE_CURVATURES_PER_M:
        speed = safe_cornering_speed(
            curvature,
            MAX_LATERAL_ACCELERATION_MPS2,
            speed_limit=GLOBAL_SPEED_LIMIT_MPS,
        )
        print(
            f"{curvature:15.3f} | {1.0 / curvature:10.1f} | "
            f"{speed:15.2f}"
        )

    path = make_reference_path("integrated")
    parameters = SpeedProfileParameters(
        global_speed_limit=GLOBAL_SPEED_LIMIT_MPS,
        maximum_lateral_acceleration=MAX_LATERAL_ACCELERATION_MPS2,
    )
    profile = build_speed_profile(path, parameters)
    figure = plot_speed_profile(
        profile,
        path.curvature,
        title="Lesson 2 — road curvature and safe speed",
    )
    if args.output:
        figure.savefig(args.output, dpi=170, bbox_inches="tight")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
