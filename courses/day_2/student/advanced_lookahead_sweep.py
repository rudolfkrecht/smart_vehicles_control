"""Advanced task: evaluate a dense look-ahead range objectively."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.metrics import calculate_path_metrics
from simulator.plotting import configure_plot_style
from simulator.tracking import PathFollowingScenario, run_path_following


LOOKAHEAD_VALUES_M = np.arange(1.5, 10.1, 0.5)
VEHICLE_SPEED_MPS = 9.0

# The score combines accuracy and steering activity. Try 0.00, 0.02 and 0.05.
SMOOTHNESS_WEIGHT = 0.02


def main() -> None:
    mean_error = []
    steering_activity = []
    scores = []
    for lookahead in LOOKAHEAD_VALUES_M:
        result = run_path_following(
            scenario=PathFollowingScenario(
                speed=VEHICLE_SPEED_MPS,
                base_lookahead=float(lookahead),
                speed_lookahead_gain=0.0,
            )
        )
        metrics = calculate_path_metrics(result)
        score = (
            metrics.mean_absolute_error_m
            + SMOOTHNESS_WEIGHT
            * metrics.rms_steering_rate_degrees_s
        )
        mean_error.append(metrics.mean_absolute_error_m)
        steering_activity.append(
            metrics.rms_steering_rate_degrees_s
        )
        scores.append(score)

    best_index = int(np.argmin(scores))
    print(
        f"Best weighted score: Ld = {LOOKAHEAD_VALUES_M[best_index]:.1f} m"
    )
    print(
        "This is a design choice, not a universal optimum: changing the speed, "
        "path or weighting changes the answer."
    )

    configure_plot_style()
    figure, axes = plt.subplots(3, 1, figsize=(8.5, 8.5), sharex=True)
    axes[0].plot(LOOKAHEAD_VALUES_M, mean_error, "o-", color="#2476d8")
    axes[0].set_ylabel("mean |e_y| [m]")
    axes[1].plot(
        LOOKAHEAD_VALUES_M,
        steering_activity,
        "o-",
        color="#e05252",
    )
    axes[1].set_ylabel("steer-rate RMS [deg/s]")
    axes[2].plot(LOOKAHEAD_VALUES_M, scores, "o-", color="#2a9d6f")
    axes[2].axvline(
        LOOKAHEAD_VALUES_M[best_index],
        color="#17233c",
        linestyle="--",
    )
    axes[2].set_ylabel("weighted score")
    axes[2].set_xlabel("look-ahead distance [m]")
    figure.tight_layout()
    figure.savefig("day2_advanced_sweep.png", dpi=160)
    plt.show()


if __name__ == "__main__":
    main()
