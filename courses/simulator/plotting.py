"""Legacy integrated-control plots reused by selected Day 4 exercises."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .integrated import IntegratedResult
from .longitudinal import SimulationResult
from .tracking import PathFollowingResult
from .metrics import calculate_integrated_metrics
from .speed_profile import SpeedProfile


STATE_COLORS = {
    "CRUISE": "#2878c8",
    "FOLLOW": "#2ca25f",
    "BRAKE": "#f59e0b",
    "EMERGENCY": "#d73027",
}


def add_road(ax, result: IntegratedResult) -> None:
    path = result.path
    ax.fill_between(
        path.x,
        path.y - result.scenario.road_half_width,
        path.y + result.scenario.road_half_width,
        color="#d9dde2",
        alpha=0.7,
        label="road corridor",
    )
    ax.plot(path.x, path.y, "k--", linewidth=1.4, label="reference")


def plot_integrated_summary(
    result: IntegratedResult,
    *,
    title: str,
):
    """Create the standard four-panel integrated-control result figure."""

    figure = plt.figure(figsize=(11, 8.5))
    grid = figure.add_gridspec(3, 2, height_ratios=(1.3, 1.0, 1.0))
    ax_path = figure.add_subplot(grid[0, :])
    ax_speed = figure.add_subplot(grid[1, 0])
    ax_gap = figure.add_subplot(grid[1, 1])
    ax_accel = figure.add_subplot(grid[2, 0])
    ax_state = figure.add_subplot(grid[2, 1])

    add_road(ax_path, result)
    ax_path.plot(result.x, result.y, color="#2476d8", label="ego vehicle")
    ax_path.plot(
        result.lead_x,
        result.lead_y,
        color="#d9534f",
        linewidth=1.7,
        label="lead vehicle",
    )
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set_ylabel("y [m]")
    ax_path.grid(alpha=0.25)
    ax_path.legend(ncol=4, fontsize=8)

    ax_speed.plot(result.time, result.speed, label="ego speed")
    ax_speed.plot(
        result.time,
        result.road_target_speed,
        "--",
        label="curve target",
    )
    ax_speed.plot(
        result.time,
        result.selected_target_speed,
        ":",
        linewidth=2,
        label="selected target",
    )
    ax_speed.plot(result.time, result.lead_speed, label="lead speed")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.grid(alpha=0.3)
    ax_speed.legend(fontsize=8)

    ax_gap.plot(result.time, result.gap, label="actual gap")
    ax_gap.plot(result.time, result.desired_gap, "--", label="desired gap")
    ax_gap.axhline(0.0, color="#d73027", linewidth=1)
    ax_gap.set_ylabel("gap [m]")
    ax_gap.grid(alpha=0.3)
    ax_gap.legend(fontsize=8)

    ax_accel.plot(result.time, result.acceleration, label="acceleration")
    ax_accel.plot(
        result.time,
        result.lateral_acceleration,
        label="lateral acceleration",
    )
    ax_accel.set(xlabel="time [s]", ylabel="acceleration [m/s²]")
    ax_accel.grid(alpha=0.3)
    ax_accel.legend(fontsize=8)

    state_codes = {
        "CRUISE": 0,
        "FOLLOW": 1,
        "BRAKE": 2,
        "EMERGENCY": 3,
    }
    codes = np.asarray(
        [state_codes[value] for value in result.behaviour_state]
    )
    ax_state.step(result.time, codes, where="post", color="#6b4c9a")
    ax_state.set_yticks(list(state_codes.values()), list(state_codes.keys()))
    ax_state.set(xlabel="time [s]", ylabel="behaviour state")
    ax_state.grid(alpha=0.3)

    metric = calculate_integrated_metrics(result)
    figure.suptitle(
        f"{title}\n"
        f"mean path error {metric.mean_path_error_m:.2f} m · "
        f"minimum gap {metric.minimum_gap_m:.1f} m · "
        f"completion {metric.completion_percent:.0f}%"
    )
    figure.tight_layout()
    return figure


def plot_speed_profile(
    profile: SpeedProfile,
    curvature: np.ndarray,
    *,
    title: str,
):
    figure, (ax_curve, ax_speed) = plt.subplots(
        2,
        1,
        figsize=(10, 6.5),
        sharex=True,
    )
    ax_curve.plot(profile.distance, curvature, color="#6b4c9a")
    ax_curve.axhline(0.0, color="black", linewidth=0.8)
    ax_curve.set_ylabel("curvature [1/m]")
    ax_curve.grid(alpha=0.3)

    ax_speed.plot(
        profile.distance,
        profile.raw_curve_speed,
        label="raw curvature limit",
    )
    ax_speed.plot(
        profile.distance,
        profile.preview_speed,
        "--",
        label="previewed",
    )
    ax_speed.plot(
        profile.distance,
        profile.planned_speed,
        linewidth=2,
        label="final planned profile",
    )
    ax_speed.set(xlabel="distance along road [m]", ylabel="speed [m/s]")
    ax_speed.grid(alpha=0.3)
    ax_speed.legend()
    figure.suptitle(title)
    figure.tight_layout()
    return figure


def _plot_integrated_comparison(
    named_results: Mapping[str, IntegratedResult],
    *,
    title: str,
):
    """Compare trajectories, speed, gap and acceleration."""

    figure, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    ax_path, ax_speed, ax_gap, ax_accel = axes.ravel()
    first = next(iter(named_results.values()))
    add_road(ax_path, first)
    for label, result in named_results.items():
        ax_path.plot(result.x, result.y, label=label)
        ax_speed.plot(result.time, result.speed, label=label)
        ax_gap.plot(result.time, result.gap, label=label)
        ax_accel.plot(result.time, result.acceleration, label=label)
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set(xlabel="x [m]", ylabel="y [m]")
    ax_speed.set(xlabel="time [s]", ylabel="speed [m/s]")
    ax_gap.set(xlabel="time [s]", ylabel="gap [m]")
    ax_accel.set(xlabel="time [s]", ylabel="acceleration [m/s²]")
    for axis in axes.ravel():
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle(title)
    figure.tight_layout()
    return figure


# Day 2 plotting helpers

COLORS = ("#2476d8", "#e05252", "#2a9d6f", "#8b5cf6", "#f59e0b")


def configure_plot_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "#f8fafc",
            "axes.edgecolor": "#94a3b8",
            "axes.grid": True,
            "grid.color": "#dbe3ea",
            "grid.alpha": 0.8,
            "font.size": 10,
            "legend.frameon": False,
        }
    )


def save_or_show(
    figure: plt.Figure,
    *,
    output: str | Path | None,
    show: bool,
) -> None:
    figure.tight_layout()
    if output is not None:
        destination = Path(output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(destination, dpi=160, bbox_inches="tight")
        print(f"Saved plot: {destination}")
    if show:
        plt.show()
    else:
        plt.close(figure)


def draw_road(
    axis: plt.Axes,
    result: PathFollowingResult,
) -> None:
    path = result.path
    half_width = result.scenario.road_half_width
    normal_x = -np.sin(path.heading)
    normal_y = np.cos(path.heading)
    axis.fill_between(
        path.x,
        path.y - half_width,
        path.y + half_width,
        color="#dce2e7",
        alpha=0.65,
        label="road corridor",
    )
    # On the prepared courses x is strictly increasing, so fill_between gives
    # a clear teaching illustration. Explicit boundary lines remain exact.
    axis.plot(
        path.x + half_width * normal_x,
        path.y + half_width * normal_y,
        color="#64748b",
        linewidth=1.0,
    )
    axis.plot(
        path.x - half_width * normal_x,
        path.y - half_width * normal_y,
        color="#64748b",
        linewidth=1.0,
    )
    axis.plot(
        path.x,
        path.y,
        "--",
        color="#17233c",
        linewidth=1.2,
        label="reference",
    )


def plot_tracking_comparison(
    named_results: dict[str, PathFollowingResult],
    *,
    title: str,
    output: str | Path | None = None,
    show: bool = True,
) -> plt.Figure:
    """Plot trajectories, cross-track error and steering."""

    configure_plot_style()
    first = next(iter(named_results.values()))
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(10.5, 9.0),
        gridspec_kw={"height_ratios": [1.65, 1.0, 1.0]},
    )
    draw_road(axes[0], first)
    for index, (label, result) in enumerate(named_results.items()):
        color = COLORS[index % len(COLORS)]
        axes[0].plot(
            result.x,
            result.y,
            color=color,
            linewidth=2.0,
            label=label,
        )
        axes[1].plot(
            result.time,
            result.cross_track_error,
            color=color,
            label=label,
        )
        axes[2].plot(
            result.time,
            np.degrees(result.steering),
            color=color,
            label=label,
        )

    axes[0].set_title(title)
    axes[0].set_xlabel("global x [m]")
    axes[0].set_ylabel("global y [m]")
    axes[0].axis("equal")
    axes[0].legend(ncol=2)
    axes[1].axhline(
        first.scenario.road_half_width,
        color="#c24141",
        linestyle=":",
    )
    axes[1].axhline(
        -first.scenario.road_half_width,
        color="#c24141",
        linestyle=":",
    )
    axes[1].set_ylabel("cross-track error [m]")
    axes[1].legend(ncol=2)
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("steering [deg]")
    axes[2].legend(ncol=2)
    save_or_show(figure, output=output, show=show)
    return figure


# Day 1 plotting helpers

COURSE_BLUE = "#1565C0"
COURSE_ORANGE = "#EF6C00"
COURSE_GREEN = "#2E7D32"
COURSE_RED = "#C62828"
COURSE_PURPLE = "#6A1B9A"


def _shade_hill(axis: plt.Axes, result: SimulationResult) -> None:
    scenario = result.scenario
    if scenario.hill_start is None or scenario.hill_force <= 0.0:
        return
    end = scenario.hill_end or scenario.duration
    axis.axvspan(
        scenario.hill_start,
        end,
        color="#B0BEC5",
        alpha=0.25,
        label="Hill disturbance",
    )


def plot_result(
    result: SimulationResult,
    *,
    title: str,
) -> plt.Figure:
    """Create a three-panel view of one simulation."""

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(10, 8),
        sharex=True,
        constrained_layout=True,
    )
    figure.suptitle(title, fontsize=15, fontweight="bold")

    axes[0].plot(
        result.time,
        result.target_speed,
        "--",
        color="black",
        linewidth=1.6,
        label="Target speed",
    )
    axes[0].plot(
        result.time,
        result.speed,
        color=COURSE_BLUE,
        linewidth=2.2,
        label="Vehicle speed",
    )
    _shade_hill(axes[0], result)
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].legend(loc="best")

    axes[1].plot(
        result.time,
        result.command,
        color=COURSE_ORANGE,
        linewidth=2.0,
    )
    axes[1].axhline(1.0, color="black", linestyle=":", linewidth=1.0)
    axes[1].axhline(-1.0, color="black", linestyle=":", linewidth=1.0)
    axes[1].set_ylabel("Command [-]")
    axes[1].set_ylim(-1.1, 1.1)

    axes[2].plot(
        result.time,
        result.acceleration,
        color=COURSE_GREEN,
        linewidth=2.0,
        label="Acceleration",
    )
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].set_ylabel("Acceleration [m/s²]")
    axes[2].set_xlabel("Time [s]")

    for axis in axes:
        axis.grid(True, alpha=0.25)

    return figure


def _plot_longitudinal_comparison(
    named_results: dict[str, SimulationResult],
    *,
    title: str,
) -> plt.Figure:
    """Compare speed and command histories from several controllers."""

    colors = [
        COURSE_BLUE,
        COURSE_ORANGE,
        COURSE_GREEN,
        COURSE_RED,
        COURSE_PURPLE,
    ]
    first_result = next(iter(named_results.values()))
    figure, axes = plt.subplots(
        2,
        1,
        figsize=(10, 7),
        sharex=True,
        constrained_layout=True,
    )
    figure.suptitle(title, fontsize=15, fontweight="bold")

    axes[0].plot(
        first_result.time,
        first_result.target_speed,
        "--",
        color="black",
        linewidth=1.6,
        label="Target speed",
    )
    for (label, result), color in zip(named_results.items(), colors, strict=False):
        axes[0].plot(
            result.time,
            result.speed,
            linewidth=2.0,
            color=color,
            label=label,
        )
        axes[1].plot(
            result.time,
            result.command,
            linewidth=1.8,
            color=color,
            label=label,
        )

    _shade_hill(axes[0], first_result)
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].legend(loc="best")
    axes[1].set_ylabel("Command [-]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylim(-1.1, 1.1)

    for axis in axes:
        axis.grid(True, alpha=0.25)

    return figure


def finish_figure(
    figure: plt.Figure,
    *,
    save_path: str | Path | None,
    show: bool,
) -> None:
    """Save and/or show a figure, then release its resources."""

    if save_path is not None:
        path = Path(save_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=160, bbox_inches="tight")
        print(f"Saved plot: {path}")
    if show:
        plt.show()
    plt.close(figure)


def plot_comparison(named_results, *, title: str):
    """Dispatch the common classroom name to the correct plot type."""

    first = next(iter(named_results.values()))
    if isinstance(first, SimulationResult):
        return _plot_longitudinal_comparison(named_results, title=title)
    return _plot_integrated_comparison(named_results, title=title)
