"""Instructor solution for the manual metrics exercise."""

from __future__ import annotations

import numpy as np


PATH_ERROR_M = np.array([0.20, -0.40, 0.10, 0.50, -0.30])
SPEED_ERROR_MPS = np.array([0.4, 0.2, -0.1, -0.3, 0.0])
GAP_M = np.array([18.0, 15.0, 12.0, 10.0, 8.0])
ACCELERATION_MPS2 = np.array([0.0, 0.8, 1.0, 0.3, -0.5])
DT = 0.5


def main() -> None:
    jerk = np.diff(ACCELERATION_MPS2) / DT
    print(
        "mean absolute path error:",
        float(np.mean(np.abs(PATH_ERROR_M))),
        "m",
    )
    print(
        "maximum absolute path error:",
        float(np.max(np.abs(PATH_ERROR_M))),
        "m",
    )
    print(
        "speed RMSE:",
        float(np.sqrt(np.mean(SPEED_ERROR_MPS**2))),
        "m/s",
    )
    print("minimum gap:", float(np.min(GAP_M)), "m")
    print("RMS jerk:", float(np.sqrt(np.mean(jerk**2))), "m/s³")


if __name__ == "__main__":
    main()
