"""Generate a documentation preview of the Day 4 PyQt laboratory."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np

from simulator.challenge import (
    BALANCED_CONFIGURATION,
    evaluation_cases,
    robust_scenario_from_case,
)
from simulator.robustness import run_robust


NAVY = "#17233c"
BLUE = "#2476d8"
RED = "#d9534f"
GREEN = "#2ca25f"
AMBER = "#f59e0b"
PURPLE = "#7057a3"


def draw_car(ax, x, y, heading, color, label, alpha=1.0):
    width, length = 2.5, 5.2
    transform = (
        plt.matplotlib.transforms.Affine2D()
        .rotate_around(x, y, heading)
        + ax.transData
    )
    body = FancyBboxPatch(
        (x - length / 2, y - width / 2),
        length,
        width,
        boxstyle="round,pad=0.1,rounding_size=0.55",
        facecolor=color,
        edgecolor=NAVY,
        linewidth=1.2,
        alpha=alpha,
        transform=transform,
        zorder=8,
    )
    ax.add_patch(body)
    ax.text(
        x,
        y + 2.8,
        label,
        ha="center",
        fontsize=7,
        weight="bold",
        color=NAVY,
        zorder=9,
    )


def add_control_row(ax, y, label, value):
    ax.text(0.05, y, label, fontsize=7.5, color=NAVY, va="center")
    ax.text(
        0.95,
        y,
        value,
        fontsize=7.5,
        color=NAVY,
        va="center",
        ha="right",
        bbox=dict(
            boxstyle="round,pad=.22",
            facecolor="white",
            edgecolor="#cad2da",
        ),
    )


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    output = root / "docs" / "assets" / "images" / "day4" / "day4_vehicle_simulator_preview.png"
    case = evaluation_cases()[-1]
    result = run_robust(
        robust_scenario_from_case(BALANCED_CONFIGURATION, case)
    )
    data = result.integrated
    index = min(360, len(data.time) - 1)

    figure = plt.figure(figsize=(15, 9), facecolor="#e8eef3")
    grid = figure.add_gridspec(
        4,
        5,
        width_ratios=(1, 1, 1, 1, 1.15),
        height_ratios=(0.28, 2.15, 0.55, 1.30),
        hspace=0.28,
        wspace=0.26,
    )
    title = figure.add_subplot(grid[0, :4])
    title.axis("off")
    title.text(
        0.0,
        0.55,
        "Smart Vehicles Control — Day 4 Robustness Laboratory",
        fontsize=18,
        weight="bold",
        color=NAVY,
    )
    for position, label in zip(
        (0.58, 0.70, 0.84),
        ("Run", "Single step", "Reset"),
    ):
        title.text(
            position,
            0.55,
            f"  {label}  ",
            va="center",
            fontsize=9,
            color="white" if label == "Run" else NAVY,
            bbox=dict(
                boxstyle="round,pad=.35",
                facecolor=BLUE if label == "Run" else "white",
                edgecolor="#aeb9c5",
            ),
        )

    road = figure.add_subplot(grid[1, :4])
    path = data.path
    road.fill_between(
        path.x,
        path.y - data.scenario.road_half_width,
        path.y + data.scenario.road_half_width,
        color="#596473",
        alpha=0.96,
    )
    road.plot(path.x, path.y, "w--", linewidth=1.0)
    road.plot(data.x[: index + 1], data.y[: index + 1], color=BLUE, linewidth=2)
    road.plot(
        result.measured_x[: index + 1],
        result.measured_y[: index + 1],
        color=AMBER,
        linewidth=1.0,
        alpha=0.7,
    )
    draw_car(
        road,
        data.x[index],
        data.y[index],
        data.heading[index],
        BLUE,
        "TRUE",
    )
    draw_car(
        road,
        result.measured_x[index],
        result.measured_y[index],
        result.measured_heading[index],
        AMBER,
        "MEASURED",
        alpha=0.72,
    )
    draw_car(
        road,
        data.lead_x[index],
        data.lead_y[index],
        float(np.interp(data.lead_distance[index], path.distance, path.heading)),
        RED,
        "LEAD",
    )
    road.plot(
        [result.measured_x[index], data.target_x[index]],
        [result.measured_y[index], data.target_y[index]],
        "--",
        color=AMBER,
        linewidth=1.5,
    )
    road.scatter(data.target_x[index], data.target_y[index], color=AMBER, s=50)
    road.set_xlim(data.x[index] - 35, data.x[index] + 75)
    road.set_ylim(data.y[index] - 24, data.y[index] + 24)
    road.set_aspect("equal", adjustable="box")
    road.axis("off")
    road.text(
        0.015,
        0.95,
        "DAY 4 — ROBUSTNESS TEST LAB",
        transform=road.transAxes,
        va="top",
        fontsize=11,
        weight="bold",
        color=NAVY,
        bbox=dict(
            boxstyle="round,pad=.6",
            facecolor="white",
            edgecolor="#cad2da",
        ),
    )
    road.text(
        0.02,
        0.78,
        "  FAULT ACTIVE  ",
        transform=road.transAxes,
        color="white",
        weight="bold",
        bbox=dict(boxstyle="round,pad=.35", facecolor=RED, edgecolor="none"),
    )
    road.text(
        0.19,
        0.84,
        f"true speed {data.speed[index]:.1f} m/s\n"
        f"measured {result.measured_speed[index]:.1f} m/s\n"
        f"true gap {data.gap[index]:.1f} m",
        transform=road.transAxes,
        va="top",
        fontsize=9,
        color=NAVY,
        bbox=dict(
            boxstyle="round,pad=.45",
            facecolor="white",
            edgecolor="#cad2da",
        ),
    )

    cards = figure.add_subplot(grid[2, :4])
    cards.axis("off")
    card_values = (
        ("FAULT STATUS", "ACTIVE", RED),
        ("MINIMUM GAP", f"{np.min(data.gap[:index + 1]):.1f} m", GREEN),
        (
            "MAX PATH ERROR",
            f"{np.max(np.abs(data.cross_track_error[:index + 1])):.2f} m",
            BLUE,
        ),
        ("PEAK JERK", f"{np.max(np.abs(data.jerk[:index + 1])):.1f} m/s³", AMBER),
    )
    for item, (label, value, color) in enumerate(card_values):
        x = 0.01 + item * 0.245
        cards.add_patch(
            FancyBboxPatch(
                (x, 0.05),
                0.225,
                0.84,
                boxstyle="round,pad=0.01,rounding_size=0.02",
                transform=cards.transAxes,
                facecolor="white",
                edgecolor="#cad2da",
            )
        )
        cards.text(x + 0.112, 0.65, label, ha="center", fontsize=8, color=NAVY)
        cards.text(
            x + 0.112,
            0.30,
            value,
            ha="center",
            fontsize=14,
            weight="bold",
            color=color,
        )

    plot_specs = (
        ("True and measured speed", data.speed, result.measured_speed, BLUE, AMBER),
        ("Following gap", data.gap, result.measured_gap, BLUE, AMBER),
        (
            "Requested and applied steering",
            np.degrees(result.requested_steering),
            np.degrees(data.steering),
            PURPLE,
            BLUE,
        ),
        (
            "True and measured path error",
            data.cross_track_error,
            result.measured_cross_track_error,
            GREEN,
            AMBER,
        ),
    )
    for item, spec in enumerate(plot_specs[:2]):
        axis = figure.add_subplot(grid[3, item * 2 : item * 2 + 2])
        name, first, second, first_color, second_color = spec
        axis.plot(data.time[: index + 1], first[: index + 1], color=first_color)
        axis.plot(
            data.time[: index + 1],
            second[: index + 1],
            color=second_color,
            alpha=0.75,
        )
        axis.set_title(name, fontsize=9, loc="left", color=NAVY)
        axis.grid(alpha=0.25)
        axis.tick_params(labelsize=7)
    # The compact preview shows two plots; the real GUI contains all four.

    controls = figure.add_subplot(grid[:, 4])
    controls.set_facecolor("#f4f7fa")
    controls.set_xticks([])
    controls.set_yticks([])
    for spine in controls.spines.values():
        spine.set_color("#cad2da")
    controls.text(
        0.05,
        0.965,
        "Teaching controls",
        fontsize=13,
        weight="bold",
        color=NAVY,
    )
    controls.text(
        0.05,
        0.925,
        "Lesson 5 — combined evaluation",
        fontsize=8,
        bbox=dict(
            boxstyle="round,pad=.3",
            facecolor="white",
            edgecolor="#cad2da",
        ),
    )
    sections = (
        (0.87, "SCENARIO", (("Road", "evaluation_b"), ("Lead", "evaluation"))),
        (
            0.72,
            "CONTROLLER",
            (
                ("Speed limit", "15.0 m/s"),
                ("Max lateral", "2.5 m/s²"),
                ("Base look-ahead", "3.0 m"),
                ("Time headway", "1.5 s"),
            ),
        ),
        (
            0.47,
            "MEASUREMENT FAULTS",
            (
                ("Position noise", "0.10 m"),
                ("Heading noise", "0.65°"),
                ("Sensor delay", "0.10 s"),
            ),
        ),
        (
            0.25,
            "ACTUATOR FAULTS",
            (
                ("Steering bias", "-0.8°"),
                ("Authority", "0.90"),
                ("Braking", "0.75"),
                ("Lateral push", "1.0 m"),
            ),
        ),
    )
    for top, heading, rows in sections:
        controls.text(0.05, top, heading, fontsize=8, weight="bold", color=PURPLE)
        for row, (label, value) in enumerate(rows):
            add_control_row(controls, top - 0.045 * (row + 1), label, value)
    controls.text(
        0.5,
        0.035,
        "Apply and reset scenario",
        ha="center",
        color="white",
        weight="bold",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=.5", facecolor=BLUE, edgecolor=BLUE),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, bbox_inches="tight")
    plt.close(figure)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
