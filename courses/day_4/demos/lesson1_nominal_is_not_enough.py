"""Lesson 1: compare a successful nominal run with an unexpected disturbance."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    TestCase,
    robust_scenario_from_case,
)
from simulator.evaluation import print_metrics_table
from simulator.faults import FaultParameters
from simulator.robust_plotting import plot_robust_comparison
from simulator.robustness import run_robust
from simulator.script_helpers import common_parser, finish_figure


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    nominal = TestCase(
        name="nominal",
        test_id="lesson1_nominal",
        path_kind="practice",
        lead_preset="steady",
        duration=34.0,
    )
    disturbed = TestCase(
        name="unexpected delay + weak steering",
        test_id="lesson1_disturbed",
        path_kind="practice",
        lead_preset="steady",
        duration=34.0,
        faults=FaultParameters(
            start_time=6.0,
            sensor_delay=0.25,
            actuator_delay=0.15,
            steering_authority=0.78,
            steering_bias_degrees=1.5,
            random_seed=104,
        ),
    )
    results = {
        case.name: run_robust(
            robust_scenario_from_case(BALANCED_CONFIGURATION, case)
        )
        for case in (nominal, disturbed)
    }
    print_metrics_table(results)
    print(
        "\nA nominal demonstration answers 'can it work?'. "
        "The disturbed test starts answering 'how reliably does it work?'."
    )
    figure = plot_robust_comparison(
        results,
        title="Lesson 1 — one successful run is not enough",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
