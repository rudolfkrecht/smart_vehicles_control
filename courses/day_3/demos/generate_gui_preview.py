"""Generate a documentation preview of the PyQt Day 3 laboratory."""

from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import numpy as np

from simulator.integrated import IntegratedScenario, run_integrated


def draw_car(ax, x, y, heading, color, label):
    width, length = 3.0, 6.0
    transform = (
        plt.matplotlib.transforms.Affine2D()
        .rotate_around(x, y, heading)
        + ax.transData
    )
    body = FancyBboxPatch(
        (x - length / 2, y - width / 2),
        length,
        width,
        boxstyle="round,pad=0.1,rounding_size=0.6",
        facecolor=color,
        edgecolor="#17233c",
        linewidth=1.4,
        transform=transform,
        zorder=8,
    )
    ax.add_patch(body)
    ax.text(
        x,
        y + 3.2,
        label,
        ha="center",
        fontsize=8,
        weight="bold",
        color="#17233c",
        zorder=9,
    )


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    output = root / "docs" / "assets" / "images" / "day3" / "day3_vehicle_simulator_preview.png"
    result = run_integrated(IntegratedScenario(duration=27.0))
    index = min(430, len(result.time) - 1)

    figure = plt.figure(figsize=(15, 9), facecolor="#e8eef3")
    grid = figure.add_gridspec(
        4,
        5,
        width_ratios=(1, 1, 1, 1, 1.12),
        height_ratios=(0.28, 2.2, 0.55, 1.25),
        hspace=0.28,
        wspace=0.26,
    )
    title = figure.add_subplot(grid[0, :4])
    title.axis("off")
    title.text(
        0.0,
        0.55,
        "Smart Vehicles Control — Day 3 Integrated ADAS Laboratory",
        fontsize=18,
        weight="bold",
        color="#17233c",
    )
    for position, label in zip(
        (0.54, 0.67, 0.80),
        ("Run", "Single step", "Reset"),
    ):
        title.text(
            position,
            0.55,
            f"  {label}  ",
            va="center",
            fontsize=9,
            color="white" if label == "Run" else "#17233c",
            bbox=dict(
                boxstyle="round,pad=.35",
                facecolor="#2476d8" if label == "Run" else "white",
                edgecolor="#aeb9c5",
            ),
        )

    road = figure.add_subplot(grid[1, :4])
    path = result.path
    road.fill_between(
        path.x,
        path.y - 3.5,
        path.y + 3.5,
        color="#596473",
        alpha=0.95,
    )
    speed_fraction = (
        result.profile.planned_speed
        / result.profile.parameters.global_speed_limit
    )
    for start in range(0, len(path.x) - 2, 3):
        color = (
            "#d9534f"
            if speed_fraction[start] < 0.55
            else "#f59e0b"
            if speed_fraction[start] < 0.78
            else "#2ca25f"
        )
        road.plot(
            path.x[start : start + 4],
            path.y[start : start + 4],
            color=color,
            linewidth=3.0,
        )
    road.plot(path.x, path.y, color="white", linestyle="--", linewidth=1)
    draw_car(
        road,
        result.x[index],
        result.y[index],
        result.heading[index],
        "#2476d8",
        "EGO",
    )
    draw_car(
        road,
        result.lead_x[index],
        result.lead_y[index],
        float(
            np.interp(
                result.lead_distance[index],
                path.distance,
                path.heading,
            )
        ),
        "#d9534f",
        "LEAD",
    )
    road.plot(
        [result.x[index], result.target_x[index]],
        [result.y[index], result.target_y[index]],
        "--",
        color="#f59e0b",
    )
    road.scatter(
        result.target_x[index],
        result.target_y[index],
        s=55,
        color="#f59e0b",
        zorder=10,
    )
    road.set_xlim(
        result.x[index] - 32,
        result.x[index] + 75,
    )
    road.set_ylim(result.y[index] - 24, result.y[index] + 24)
    road.set_aspect("equal", adjustable="box")
    road.axis("off")
    road.text(
        0.015,
        0.96,
        "DAY 3 — INTEGRATED ADAS LAB",
        transform=road.transAxes,
        va="top",
        fontsize=11,
        weight="bold",
        bbox=dict(
            boxstyle="round,pad=.6",
            facecolor="white",
            edgecolor="#cad2da",
            alpha=0.95,
        ),
    )
    road.text(
        0.018,
        0.82,
        f"  {result.behaviour_state[index]}  ",
        transform=road.transAxes,
        color="white",
        weight="bold",
        bbox=dict(
            boxstyle="round,pad=.35",
            facecolor="#f59e0b",
            edgecolor="none",
        ),
    )
    road.text(
        0.18,
        0.86,
        f"speed {result.speed[index]:.1f} m/s\n"
        f"target {result.selected_target_speed[index]:.1f} m/s\n"
        f"gap {result.gap[index]:.1f} m",
        transform=road.transAxes,
        va="top",
        fontsize=9,
        color="#17233c",
        bbox=dict(
            boxstyle="round,pad=.45",
            facecolor="white",
            edgecolor="#cad2da",
            alpha=0.95,
        ),
    )

    cards = figure.add_subplot(grid[2, :4])
    cards.axis("off")
    card_values = (
        ("BEHAVIOUR", result.behaviour_state[index], "#2ca25f"),
        ("MINIMUM GAP", f"{np.min(result.gap[:index + 1]):.1f} m", "#2ca25f"),
        (
            "MAX PATH ERROR",
            f"{np.max(np.abs(result.cross_track_error[:index + 1])):.2f} m",
            "#2476d8",
        ),
        (
            "PEAK LATERAL ACCEL.",
            f"{np.max(np.abs(result.lateral_acceleration[:index + 1])):.2f} m/s²",
            "#2476d8",
        ),
    )
    for card_index, (label, value, color) in enumerate(card_values):
        x = 0.01 + card_index * 0.247
        cards.add_patch(
            FancyBboxPatch(
                (x, 0.08),
                0.225,
                0.82,
                boxstyle="round,pad=.012",
                transform=cards.transAxes,
                facecolor="white",
                edgecolor="#cbd5e1",
            )
        )
        cards.text(
            x + 0.112,
            0.66,
            label,
            transform=cards.transAxes,
            ha="center",
            fontsize=8,
            color="#475569",
        )
        cards.text(
            x + 0.112,
            0.29,
            value,
            transform=cards.transAxes,
            ha="center",
            fontsize=13,
            weight="bold",
            color=color,
        )

    plot_specs = [
        ("Speed arbitration", result.speed, result.selected_target_speed, "ego", "target"),
        ("Following distance", result.gap, result.desired_gap, "actual", "desired"),
        ("Acceleration", result.acceleration, result.lateral_acceleration, "longitudinal", "lateral"),
        ("Cross-track error", result.cross_track_error, None, "e_y", ""),
    ]
    for plot_index, spec in enumerate(plot_specs):
        axis = figure.add_subplot(grid[3, plot_index])
        title_text, first, second, first_label, second_label = spec
        axis.plot(result.time[: index + 1], first[: index + 1], label=first_label)
        if second is not None:
            axis.plot(
                result.time[: index + 1],
                second[: index + 1],
                "--",
                label=second_label,
            )
        axis.set_title(title_text, fontsize=9, weight="bold")
        axis.grid(alpha=0.25)
        axis.tick_params(labelsize=7)
        axis.legend(fontsize=6, loc="best")

    panel = figure.add_subplot(grid[:, 4])
    panel.set_facecolor("white")
    panel.set_xticks([])
    panel.set_yticks([])
    for spine in panel.spines.values():
        spine.set_color("#cbd5e1")
    panel.text(
        0.07,
        0.96,
        "Teaching controls",
        fontsize=15,
        weight="bold",
        color="#17233c",
        va="top",
    )
    sections = [
        (
            0.89,
            "Enabled layers",
            "☑ Curve-aware speed\n☑ Lead vehicle / ACC\n☑ Behaviour states",
        ),
        (
            0.71,
            "Road-speed planning",
            "Global speed limit       15.0 m/s\n"
            "Max lateral accel.        2.5 m/s²\n"
            "Curve preview            14.0 m\n"
            "Smoothing samples              7",
        ),
        (
            0.49,
            "Path following",
            "Base look-ahead           3.0 m\n"
            "Speed gain                 0.25 s",
        ),
        (
            0.34,
            "ACC and safety",
            "Time headway               1.5 s\n"
            "Standstill gap             5.0 m\n"
            "Emergency TTC             1.25 s\n"
            "Emergency gap              3.0 m\n"
            "Lead scenario       stop_and_go",
        ),
        (
            0.12,
            "Visual overlays",
            "☑ Colour road by speed\n☑ Controller geometry",
        ),
    ]
    for y, heading, body in sections:
        panel.text(
            0.08,
            y,
            heading,
            fontsize=10,
            weight="bold",
            color="#17233c",
            va="top",
        )
        panel.text(
            0.08,
            y - 0.045,
            body,
            fontsize=8.3,
            family="monospace",
            color="#334155",
            va="top",
            linespacing=1.55,
        )
    panel.text(
        0.5,
        0.035,
        "Apply and reset scenario",
        ha="center",
        va="center",
        color="white",
        weight="bold",
        fontsize=9,
        bbox=dict(
            boxstyle="round,pad=.6",
            facecolor="#2476d8",
            edgecolor="none",
        ),
    )
    figure.savefig(output, dpi=160, bbox_inches="tight")
    print(output)


if __name__ == "__main__":
    main()
