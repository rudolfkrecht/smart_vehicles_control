"""Lesson 2: calculate and interpret a controller scorecard."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.challenge import BALANCED_CONFIGURATION, practice_cases
from simulator.evaluation import (
    calculate_robust_metrics,
    print_metrics_table,
)
from simulator.challenge import robust_scenario_from_case
from simulator.robust_plotting import plot_robust_summary
from simulator.robustness import run_robust
from simulator.script_helpers import common_parser, finish_figure


def manual_example() -> None:
    samples = np.array([0.2, -0.4, 0.1, 0.5, -0.3])
    mean_absolute = float(np.mean(np.abs(samples)))
    maximum = float(np.max(np.abs(samples)))
    rmse = float(np.sqrt(np.mean(samples**2)))
    print("Manual five-sample path-error example")
    print(f"samples: {samples.tolist()} m")
    print(f"mean absolute error = {mean_absolute:.3f} m")
    print(f"maximum absolute error = {maximum:.3f} m")
    print(f"RMSE = {rmse:.3f} m\n")


def main() -> None:
    arguments = common_parser(__doc__).parse_args()
    manual_example()
    case = practice_cases()[2]
    result = run_robust(
        robust_scenario_from_case(BALANCED_CONFIGURATION, case)
    )
    print_metrics_table({case.name: result})
    metric = calculate_robust_metrics(result)
    print(
        "\nInterpretation:"
        f"\n  safety: minimum true gap "
        f"{metric.integrated.minimum_gap_m:.1f} m"
        f"\n  tracking: mean |e_y| "
        f"{metric.integrated.mean_path_error_m:.2f} m"
        f"\n  comfort: RMS jerk {metric.rms_jerk_mps3:.2f} m/s³"
        f"\n  verdict: {'PASS' if metric.pass_run else 'FAIL'}"
    )
    figure = plot_robust_summary(
        result,
        title="Lesson 2 — quantitative controller evaluation",
    )
    finish_figure(figure, arguments)


if __name__ == "__main__":
    main()
