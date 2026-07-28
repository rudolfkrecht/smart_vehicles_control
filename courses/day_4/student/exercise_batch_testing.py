"""Student exercise: automate five starts and add repeatable sensor noise."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    initial_condition_cases,
    print_batch_report,
    run_batch,
)
from simulator.faults import FaultParameters
from simulator.robust_plotting import plot_batch_report
from simulator.script_helpers import common_parser, finish_figure


# EDIT ONLY THIS BLOCK -------------------------------------------------------
ENABLE_NOISE = False
POSITION_NOISE_STD = 0.12
HEADING_NOISE_STD_DEGREES = 0.8
SPEED_NOISE_STD = 0.20
RANDOM_SEED = 220
# ---------------------------------------------------------------------------


def build_cases():
    cases = initial_condition_cases()
    if not ENABLE_NOISE:
        return cases
    return tuple(
        replace(
            case,
            faults=FaultParameters(
                position_noise_std=POSITION_NOISE_STD,
                heading_noise_std_degrees=HEADING_NOISE_STD_DEGREES,
                speed_noise_std=SPEED_NOISE_STD,
                range_noise_std=0.30,
                random_seed=RANDOM_SEED + index,
            ),
        )
        for index, case in enumerate(cases)
    )


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    report = run_batch(BALANCED_CONFIGURATION, build_cases())
    print_batch_report(report)
    print(
        "\nWorst-case maximum path error: "
        f"{report.worst_item.metrics.integrated.maximum_path_error_m:.2f} m"
    )
    figure = plot_batch_report(
        report,
        title="Student batch test — initial conditions and noise",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
