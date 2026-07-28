"""Lesson 2: explore steering angle, wheelbase and speed separately."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.bicycle import simulate_constant_steering, turning_radius
from simulator.plotting import COLORS, configure_plot_style, save_or_show
from simulator.script_helpers import common_parser


STEERING_VALUES_DEGREES = (6.0, 12.0, 20.0)
WHEELBASE_VALUES_M = (2.2, 2.7, 3.4)
SPEED_VALUES_MPS = (4.0, 8.0, 14.0)


def _trajectory_axis(
    axis: plt.Axes,
    values: tuple[float, ...],
    *,
    variable: str,
) -> None:
    for index, value in enumerate(values):
        settings = {
            "steering_degrees": 12.0,
            "speed": 8.0,
            "wheelbase": 2.7,
            "duration": 4.5,
        }
        if variable == "steering":
            settings["steering_degrees"] = value
            label = f"δ = {value:.0f}°"
        elif variable == "wheelbase":
            settings["wheelbase"] = value
            label = f"L = {value:.1f} m"
        else:
            settings["speed"] = value
            label = f"v = {value:.0f} m/s"
        history = simulate_constant_steering(**settings)
        axis.plot(
            history["x"],
            history["y"],
            color=COLORS[index],
            linewidth=2.1,
            label=label,
        )
    axis.axis("equal")
    axis.set_xlabel("global x [m]")
    axis.set_ylabel("global y [m]")
    axis.legend()


def main() -> None:
    args = common_parser(__doc__).parse_args()
    configure_plot_style()
    figure, axes = plt.subplots(2, 2, figsize=(11.0, 8.3))
    _trajectory_axis(
        axes[0, 0],
        STEERING_VALUES_DEGREES,
        variable="steering",
    )
    axes[0, 0].set_title("Steering angle changes radius")
    _trajectory_axis(
        axes[0, 1],
        WHEELBASE_VALUES_M,
        variable="wheelbase",
    )
    axes[0, 1].set_title("Wheelbase changes radius")
    _trajectory_axis(
        axes[1, 0],
        SPEED_VALUES_MPS,
        variable="speed",
    )
    axes[1, 0].set_title("Speed changes progress, not geometric radius")

    steering = math.radians(12.0)
    radius = abs(turning_radius(2.7, steering))
    lateral_acceleration = np.asarray(SPEED_VALUES_MPS) ** 2 / radius
    axes[1, 1].plot(
        SPEED_VALUES_MPS,
        lateral_acceleration,
        "o-",
        color=COLORS[0],
        linewidth=2.2,
    )
    axes[1, 1].axhline(
        3.0,
        linestyle="--",
        color=COLORS[1],
        label="example comfort limit",
    )
    axes[1, 1].set_title("But speed strongly changes lateral acceleration")
    axes[1, 1].set_xlabel("speed [m/s]")
    axes[1, 1].set_ylabel(r"$a_y=v^2/R$ [m/s²]")
    axes[1, 1].legend()

    for speed, acceleration in zip(
        SPEED_VALUES_MPS,
        lateral_acceleration,
    ):
        print(
            f"v = {speed:4.1f} m/s  radius = {radius:5.1f} m  "
            f"lateral acceleration = {acceleration:5.2f} m/s²"
        )

    save_or_show(
        figure,
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
