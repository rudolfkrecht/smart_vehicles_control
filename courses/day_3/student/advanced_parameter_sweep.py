"""Advanced task: search several safe integrated-control configurations."""

from __future__ import annotations

from dataclasses import replace
import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import (
    calculate_integrated_metrics,
    weighted_workshop_score,
)


# STUDENT EDIT AREA ---------------------------------------------------------
SPEED_LIMITS_MPS = (14.0, 16.0)
MAX_LATERAL_ACCELERATIONS_MPS2 = (1.8, 2.5, 3.2)
PREVIEW_DISTANCES_M = (8.0, 14.0)
TIME_HEADWAYS_S = (1.0, 1.5, 2.0)
# END STUDENT EDIT AREA -----------------------------------------------------


def main() -> None:
    rows = []
    for speed, lateral, preview, headway in itertools.product(
        SPEED_LIMITS_MPS,
        MAX_LATERAL_ACCELERATIONS_MPS2,
        PREVIEW_DISTANCES_M,
        TIME_HEADWAYS_S,
    ):
        base = IntegratedScenario()
        scenario = replace(
            base,
            speed_profile=replace(
                base.speed_profile,
                global_speed_limit=speed,
                maximum_lateral_acceleration=lateral,
                preview_distance=preview,
            ),
            acc=replace(base.acc, time_headway=headway),
        )
        result = run_integrated(scenario)
        metric = calculate_integrated_metrics(result)
        rows.append(
            (
                weighted_workshop_score(result),
                speed,
                lateral,
                preview,
                headway,
                metric.minimum_gap_m,
                metric.completion_percent,
            )
        )
    rows.sort(key=lambda row: row[0])
    print(
        "score | speed | a_y,max | preview | T_h | min gap | complete"
    )
    print("------+-------+---------+---------+-----+---------+---------")
    for row in rows[:10]:
        score, speed, lateral, preview, headway, gap, complete = row
        shown = "unsafe" if math.isinf(score) else f"{score:5.2f}"
        print(
            f"{shown:>5} | {speed:5.1f} | {lateral:7.1f} | "
            f"{preview:7.1f} | {headway:3.1f} | {gap:7.2f} | "
            f"{complete:7.1f}"
        )


if __name__ == "__main__":
    main()
