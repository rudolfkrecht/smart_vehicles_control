"""Generate a static preview of the PyQt laboratory for documentation."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.tracking import PathFollowingScenario, run_path_following


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    output = root / "docs" / "images" / "day2_vehicle_simulator_preview.png"
    result = run_path_following(
        scenario=PathFollowingScenario(
            duration=7.0,
            speed=9.0,
            base_lookahead=5.0,
            speed_lookahead_gain=0.0,
        )
    )
    index = min(len(result.time) - 1, 95)
    path = result.path

    figure = plt.figure(figsize=(14.5, 8.2), facecolor="#eef4f8")
    grid = figure.add_gridspec(
        4,
        4,
        width_ratios=[1.0, 1.0, 1.0, 0.9],
        height_ratios=[0.4, 2.2, 0.85, 0.85],
        hspace=0.32,
        wspace=0.28,
    )

    card_values = [
        ("Speed", "9.0 m/s"),
        ("Cross-track", f"{result.cross_track_error[index]:+.2f} m"),
        ("Heading error", f"{math.degrees(result.heading_error[index]):+.1f}°"),
        ("Steering", f"{math.degrees(result.steering[index]):+.1f}°"),
        ("Look-ahead", f"{result.lookahead_distance[index]:.1f} m"),
        ("Complete", f"{100*path.distance[result.nearest_index[index]]/path.length:.1f}%"),
    ]
    card_axis = figure.add_subplot(grid[0, :3])
    card_axis.axis("off")
    for card_index, (title, value) in enumerate(card_values):
        left = card_index / len(card_values) + 0.005
        width = 0.155
        card_axis.add_patch(
            FancyBboxPatch(
                (left, 0.05),
                width,
                0.9,
                boxstyle="round,pad=0.006,rounding_size=0.02",
                facecolor="white",
                edgecolor="#cbd5e1",
            )
        )
        card_axis.text(
            left + 0.01,
            0.70,
            title,
            color="#64748b",
            fontsize=8,
        )
        card_axis.text(
            left + 0.01,
            0.25,
            value,
            color="#17233c",
            fontsize=13,
            fontweight="bold",
        )

    road_axis = figure.add_subplot(grid[1:3, :3])
    road_axis.set_facecolor("#eef4f8")
    half_width = result.scenario.road_half_width
    road_axis.plot(
        path.x,
        path.y,
        color="#586472",
        linewidth=34,
        solid_capstyle="round",
        zorder=1,
    )
    road_axis.plot(
        path.x,
        path.y,
        "--",
        color="white",
        linewidth=1.4,
        zorder=2,
    )
    road_axis.plot(
        result.x[: index + 1],
        result.y[: index + 1],
        color="#7dd3fc",
        linewidth=3.0,
        zorder=3,
        label="vehicle trail",
    )
    state_x = result.x[index]
    state_y = result.y[index]
    state_heading = result.heading[index]
    rotation = np.array(
        [
            [math.cos(state_heading), -math.sin(state_heading)],
            [math.sin(state_heading), math.cos(state_heading)],
        ]
    )
    car = (
        np.array([[-1.8, -0.9], [2.2, -0.9], [2.2, 0.9], [-1.8, 0.9]])
        @ rotation.T
        + np.array([state_x, state_y])
    )
    road_axis.add_patch(
        Polygon(
            car,
            closed=True,
            facecolor="#2476d8",
            edgecolor="#17233c",
            linewidth=1.8,
            zorder=6,
        )
    )
    nearest_index = result.nearest_index[index]
    nearest_x = path.x[nearest_index]
    nearest_y = path.y[nearest_index]
    target_x = result.target_x[index]
    target_y = result.target_y[index]
    road_axis.plot(
        [state_x, nearest_x],
        [state_y, nearest_y],
        "--",
        color="#e05252",
        linewidth=2.0,
        zorder=5,
    )
    road_axis.plot(
        [state_x, target_x],
        [state_y, target_y],
        color="#f59e0b",
        linewidth=2.0,
        zorder=5,
    )
    road_axis.scatter(
        [nearest_x, target_x],
        [nearest_y, target_y],
        color=["#e05252", "#f59e0b"],
        s=65,
        zorder=7,
    )
    road_axis.set_xlim(state_x - 32, state_x + 62)
    road_axis.set_ylim(state_y - 21, state_y + 21)
    road_axis.set_aspect("equal")
    road_axis.set_xticks([])
    road_axis.set_yticks([])
    road_axis.set_title(
        "Pure Pursuit — fixed     Orange: target     Red: nearest point",
        loc="left",
        color="#17233c",
        fontsize=11,
    )

    error_axis = figure.add_subplot(grid[3, :2])
    error_axis.plot(
        result.time[: index + 1],
        result.cross_track_error[: index + 1],
        color="#2476d8",
        label=r"$e_y$ [m]",
    )
    error_axis.plot(
        result.time[: index + 1],
        result.heading_error[: index + 1],
        color="#e05252",
        label=r"$e_\psi$ [rad]",
    )
    error_axis.set_title("Tracking errors", loc="left", fontsize=10)
    error_axis.grid(color="#dbe3ea")
    error_axis.legend(ncol=2, frameon=False, fontsize=8)

    motion_axis = figure.add_subplot(grid[3, 2])
    motion_axis.plot(
        result.time[: index + 1],
        result.steering[: index + 1],
        color="#f59e0b",
        label=r"$\delta$ [rad]",
    )
    motion_axis.plot(
        result.time[: index + 1],
        result.lateral_acceleration[: index + 1] / 10.0,
        color="#2a9d6f",
        label=r"$a_y/10$",
    )
    motion_axis.set_title("Control and response", loc="left", fontsize=10)
    motion_axis.grid(color="#dbe3ea")
    motion_axis.legend(ncol=2, frameon=False, fontsize=8)

    panel = figure.add_subplot(grid[:, 3])
    panel.set_facecolor("white")
    panel.set_xticks([])
    panel.set_yticks([])
    for spine in panel.spines.values():
        spine.set_color("#cbd5e1")
    panel.text(
        0.08,
        0.96,
        "Teaching scenario",
        fontweight="bold",
        color="#17233c",
        transform=panel.transAxes,
    )
    panel.text(
        0.08,
        0.91,
        "Lesson 4 — balanced",
        bbox=dict(boxstyle="round", facecolor="#f8fafc", edgecolor="#cbd5e1"),
        transform=panel.transAxes,
    )
    controls = [
        ("Path", "training"),
        ("Controller", "Pure Pursuit"),
        ("Speed", "9.0 m/s"),
        ("Wheelbase", "2.7 m"),
        ("Base look-ahead", "5.0 m"),
        ("Speed gain", "0.00 s"),
        ("Initial offset", "1.5 m"),
        ("Heading error", "4.0°"),
        ("Position noise", "0.0 m"),
    ]
    y = 0.82
    for label, value in controls:
        panel.text(0.08, y, label, color="#64748b", transform=panel.transAxes)
        panel.text(
            0.58,
            y,
            value,
            color="#17233c",
            transform=panel.transAxes,
            fontsize=8.5,
            ha="left",
        )
        y -= 0.065
    panel.text(
        0.08,
        0.19,
        "Pause",
        color="white",
        bbox=dict(boxstyle="round", facecolor="#2476d8", edgecolor="none"),
        transform=panel.transAxes,
    )
    panel.text(
        0.40,
        0.19,
        "Apply and reset",
        color="white",
        bbox=dict(boxstyle="round", facecolor="#2476d8", edgecolor="none"),
        transform=panel.transAxes,
    )
    panel.text(
        0.08,
        0.12,
        "Push vehicle +2 m",
        color="white",
        bbox=dict(boxstyle="round", facecolor="#e05252", edgecolor="none"),
        transform=panel.transAxes,
    )
    panel.text(
        0.08,
        0.035,
        "Pause → predict → change one value\n→ apply → compare metrics",
        color="#17233c",
        bbox=dict(boxstyle="round", facecolor="#dbeafe", edgecolor="none"),
        transform=panel.transAxes,
        fontsize=9,
    )

    figure.suptitle(
        "Smart Vehicles Control — Day 2 Path-Following Laboratory",
        x=0.02,
        ha="left",
        fontsize=16,
        fontweight="bold",
        color="#17233c",
    )
    figure.subplots_adjust(left=0.025, right=0.98, top=0.92, bottom=0.04)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    print(f"Saved GUI preview: {output}")


if __name__ == "__main__":
    main()
