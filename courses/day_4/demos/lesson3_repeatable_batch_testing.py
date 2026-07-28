"""Lesson 3: run five initial conditions automatically and find the worst."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    initial_condition_cases,
    print_batch_report,
    run_batch,
)
from simulator.robust_plotting import plot_batch_report
from simulator.script_helpers import common_parser, finish_figure


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    report = run_batch(BALANCED_CONFIGURATION, initial_condition_cases())
    print_batch_report(report)
    print(
        "\nEvery case is deterministic. Re-running this script gives the same "
        "evidence, so a controller change can be compared fairly."
    )
    figure = plot_batch_report(
        report,
        title="Lesson 3 — repeatable initial-condition sweep",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
