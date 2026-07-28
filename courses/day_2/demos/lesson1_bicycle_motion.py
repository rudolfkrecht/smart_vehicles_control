"""Lesson 1: connect steering commands to motion in global coordinates."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.bicycle import simulate_constant_steering, turning_radius
from simulator.plotting import COLORS, configure_plot_style, save_or_show
from simulator.script_helpers import common_parser


# Safe teacher edit: keep the three signs and change the magnitude.
STEERING_CASES_DEGREES = (-12.0, 0.0, 12.0)
SPEED_MPS = 8.0
WHEELBASE_M = 2.7


def main() -> None:
    args = common_parser(__doc__).parse_args()
    configure_plot_style()
    figure, axes = plt.subplots(1, 2, figsize=(11.0, 4.6))

    for index, steering in enumerate(STEERING_CASES_DEGREES):
        history = simulate_constant_steering(
            steering_degrees=steering,
            speed=SPEED_MPS,
            wheelbase=WHEELBASE_M,
            duration=3.0,
        )
        label = f"steering = {steering:+.0f}°"
        color = COLORS[index]
        axes[0].plot(
            history["x"],
            history["y"],
            color=color,
            linewidth=2.3,
            label=label,
        )
        axes[1].plot(
            history["time"],
            history["heading"] * 180.0 / math.pi,
            color=color,
            linewidth=2.0,
            label=label,
        )
        radius = turning_radius(WHEELBASE_M, math.radians(steering))
        radius_text = "straight" if math.isinf(radius) else f"{radius:+.1f} m"
        print(
            f"{label:20s}  radius = {radius_text:>8s}  "
            f"final heading = {math.degrees(history['heading'][-1]):+.1f}°"
        )

    axes[0].scatter([0.0], [0.0], marker="o", color="#17233c", zorder=4)
    axes[0].set_title("The same initial pose, three steering commands")
    axes[0].set_xlabel("global x [m]")
    axes[0].set_ylabel("global y [m]")
    axes[0].axis("equal")
    axes[0].legend()
    axes[1].set_title("Heading changes as the vehicle turns")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("heading ψ [deg]")
    axes[1].legend()

    save_or_show(
        figure,
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
