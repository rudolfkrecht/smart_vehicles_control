"""Extension: rank only configurations that pass every practice test."""

from __future__ import annotations

from itertools import product
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    ControllerConfiguration,
    practice_cases,
    run_batch,
)


def main() -> None:
    candidates = []
    for speed, lookahead, headway in product(
        (13.0, 15.0, 17.0),
        (2.5, 3.0, 4.0),
        (1.1, 1.5, 1.9),
    ):
        configuration = ControllerConfiguration(
            global_speed_limit=speed,
            base_lookahead=lookahead,
            time_headway=headway,
        )
        report = run_batch(configuration, practice_cases())
        if report.pass_rate_percent < 100.0:
            continue
        efficiency_cost = (
            report.mean_path_error_m
            + 0.02 * max(0.0, 8.0 - report.minimum_gap_m)
            + 0.02 * (17.0 - speed)
        )
        candidates.append((efficiency_cost, speed, lookahead, headway))

    if not candidates:
        print("No candidate passed the safety gate.")
        return
    candidates.sort()
    print("Safe candidates, ranked after safety filtering:")
    for rank, candidate in enumerate(candidates[:8], start=1):
        cost, speed, lookahead, headway = candidate
        print(
            f"{rank:>2}. speed={speed:>4.1f} m/s, "
            f"look-ahead={lookahead:>3.1f} m, "
            f"headway={headway:>3.1f} s, cost={cost:.3f}"
        )


if __name__ == "__main__":
    main()
