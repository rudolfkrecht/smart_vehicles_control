"""Student exercise: calculate objective metrics from a small dataset."""

from __future__ import annotations

import math
import numpy as np


# EDIT ONLY THIS BLOCK -------------------------------------------------------
PATH_ERROR_M = np.array([0.20, -0.40, 0.10, 0.50, -0.30])
SPEED_ERROR_MPS = np.array([0.4, 0.2, -0.1, -0.3, 0.0])
GAP_M = np.array([18.0, 15.0, 12.0, 10.0, 8.0])
ACCELERATION_MPS2 = np.array([0.0, 0.8, 1.0, 0.3, -0.5])
DT = 0.5
# ---------------------------------------------------------------------------


def calculate_metrics() -> dict[str, float]:
    """Complete the three TODO lines during Lesson 2."""

    # TODO 1: use abs(), mean() and max() for path error.
    mean_absolute_path_error = float(np.mean(np.abs(PATH_ERROR_M)))
    maximum_path_error = float(np.max(np.abs(PATH_ERROR_M)))

    # TODO 2: use sqrt(mean(error**2)) for speed RMSE.
    speed_rmse = float(np.sqrt(np.mean(SPEED_ERROR_MPS**2)))

    # TODO 3: minimum distance is the smallest value in GAP_M.
    minimum_gap = float(np.min(GAP_M))

    jerk = np.diff(ACCELERATION_MPS2) / DT
    rms_jerk = float(np.sqrt(np.mean(jerk**2)))
    return {
        "mean_absolute_path_error_m": mean_absolute_path_error,
        "maximum_path_error_m": maximum_path_error,
        "speed_rmse_mps": speed_rmse,
        "minimum_gap_m": minimum_gap,
        "rms_jerk_mps3": rms_jerk,
    }


def main() -> None:
    metrics = calculate_metrics()
    for name, value in metrics.items():
        print(f"{name}: {value:.3f}")
    assert math.isclose(
        metrics["mean_absolute_path_error_m"],
        0.30,
        abs_tol=1e-12,
    )
    print("\nCheck passed. Explain which metric is safety-critical.")


if __name__ == "__main__":
    main()
