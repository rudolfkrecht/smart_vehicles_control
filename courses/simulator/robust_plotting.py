"""Consistent figures for Day 4 demonstrations and reports."""

from __future__ import annotations

from collections.abc import Mapping

import matplotlib.pyplot as plt
import numpy as np

from .challenge import BatchReport
from .evaluation import (
    ChallengeScore,
    calculate_robust_metrics,
)
from .robustness import RobustResult


BLUE = "#2476d8"
RED = "#d9534f"
GREEN = "#2ca25f"
AMBER = "#f59e0b"
PURPLE = "#7057a3"
NAVY = "#17233c"


def add_road(ax, result: RobustResult) -> None:
    path = result.path
    half_width = result.scenario.road_half_width
    ax.fill_between(
        path.x,
        path.y - half_width,
        path.y + half_width,
        color="#d9dde2",
        alpha=0.75,
        label="road corridor",
    )
    ax.plot(path.x, path.y, "k--", linewidth=1.2, label="reference")


def _shade_fault(ax, result: RobustResult) -> None:
    active = result.fault_active
    if np.any(active):
        indices = np.flatnonzero(active)
        ax.axvspan(
            result.time[indices[0]],
            result.time[indices[-1]],
            color=RED,
            alpha=0.08,
            label="fault active",
        )


def plot_robust_summary(result: RobustResult, *, title: str):
    """Six-panel evidence figure for one robustness experiment."""

    data = result.integrated
    figure, axes = plt.subplots(3, 2, figsize=(12, 9))
    ax_path, ax_speed, ax_error, ax_gap, ax_actuator, ax_comfort = axes.ravel()

    add_road(ax_path, result)
    ax_path.plot(data.x, data.y, color=BLUE, linewidth=2, label="true vehicle")
    ax_path.plot(
        result.measured_x,
        result.measured_y,
        color=AMBER,
        linewidth=1,
        alpha=0.65,
        label="measured pose",
    )
    ax_path.plot(data.lead_x, data.lead_y, color=RED, label="lead vehicle")
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set(xlabel="x [m]", ylabel="y [m]")

    ax_speed.plot(data.time, data.speed, color=BLUE, label="true speed")
    ax_speed.plot(
        data.time,
        result.measured_speed,
        color=AMBER,
        alpha=0.7,
        label="measured speed",
    )
    ax_speed.plot(
        data.time,
        data.selected_target_speed,
        "--",
        color=PURPLE,
        label="target",
    )
    _shade_fault(ax_speed, result)
    ax_speed.set(xlabel="time [s]", ylabel="speed [m/s]")

    ax_error.plot(
        data.time,
        data.cross_track_error,
        color=BLUE,
        label="true path error",
    )
    ax_error.plot(
        data.time,
        result.measured_cross_track_error,
        color=AMBER,
        alpha=0.7,
        label="measured path error",
    )
    ax_error.axhline(data.scenario.road_half_width, color=RED, linestyle=":")
    ax_error.axhline(-data.scenario.road_half_width, color=RED, linestyle=":")
    _shade_fault(ax_error, result)
    ax_error.set(xlabel="time [s]", ylabel="cross-track error [m]")

    ax_gap.plot(data.time, data.gap, color=BLUE, label="true gap")
    ax_gap.plot(
        data.time,
        result.measured_gap,
        color=AMBER,
        alpha=0.65,
        label="measured gap",
    )
    ax_gap.plot(data.time, data.desired_gap, "--", color=GREEN, label="desired")
    ax_gap.axhline(0.0, color=RED, linewidth=1)
    _shade_fault(ax_gap, result)
    ax_gap.set(xlabel="time [s]", ylabel="following gap [m]")

    ax_actuator.plot(
        data.time,
        np.degrees(result.requested_steering),
        color=PURPLE,
        label="requested steering",
    )
    ax_actuator.plot(
        data.time,
        np.degrees(data.steering),
        color=BLUE,
        label="applied steering",
    )
    _shade_fault(ax_actuator, result)
    ax_actuator.set(xlabel="time [s]", ylabel="steering [deg]")

    ax_comfort.plot(data.time, data.acceleration, color=BLUE, label="acceleration")
    ax_comfort.plot(data.time, data.jerk, color=RED, alpha=0.75, label="jerk")
    ax_comfort.plot(
        data.time,
        data.lateral_acceleration,
        color=GREEN,
        label="lateral acceleration",
    )
    _shade_fault(ax_comfort, result)
    ax_comfort.set(xlabel="time [s]", ylabel="motion [SI units]")

    for axis in axes.ravel():
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
    metric = calculate_robust_metrics(result)
    figure.suptitle(
        f"{title}\n"
        f"mean path error {metric.integrated.mean_path_error_m:.2f} m · "
        f"minimum gap {metric.integrated.minimum_gap_m:.1f} m · "
        f"{'PASS' if metric.pass_run else 'FAIL: ' + metric.failure_reason}"
    )
    figure.tight_layout()
    return figure


def plot_robust_comparison(
    named_results: Mapping[str, RobustResult],
    *,
    title: str,
):
    """Compare trajectory, error, safety and comfort across configurations."""

    figure, axes = plt.subplots(2, 2, figsize=(11.5, 7.5))
    ax_path, ax_error, ax_gap, ax_jerk = axes.ravel()
    first = next(iter(named_results.values()))
    add_road(ax_path, first)
    for label, result in named_results.items():
        data = result.integrated
        ax_path.plot(data.x, data.y, label=label)
        ax_error.plot(data.time, np.abs(data.cross_track_error), label=label)
        ax_gap.plot(data.time, data.gap, label=label)
        ax_jerk.plot(data.time, data.jerk, label=label, alpha=0.85)
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set(xlabel="x [m]", ylabel="y [m]")
    ax_error.set(xlabel="time [s]", ylabel="absolute path error [m]")
    ax_gap.set(xlabel="time [s]", ylabel="true gap [m]")
    ax_gap.axhline(0.0, color=RED, linewidth=1)
    ax_jerk.set(xlabel="time [s]", ylabel="jerk [m/s³]")
    for axis in axes.ravel():
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle(title)
    figure.tight_layout()
    return figure


def plot_batch_report(report: BatchReport, *, title: str):
    """Bar-chart scorecard with pass/fail status and worst-case visibility."""

    labels = [item.case.name for item in report.items]
    mean_error = [
        item.metrics.integrated.mean_path_error_m for item in report.items
    ]
    minimum_gap = [
        item.metrics.integrated.minimum_gap_m for item in report.items
    ]
    peak_jerk = [
        item.metrics.integrated.peak_jerk_mps3 for item in report.items
    ]
    colors = [
        GREEN if item.metrics.pass_run else RED for item in report.items
    ]
    positions = np.arange(len(labels))
    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].bar(positions, mean_error, color=colors)
    axes[0].set_ylabel("mean |e_y| [m]")
    axes[1].bar(positions, minimum_gap, color=colors)
    axes[1].axhline(1.0, color=RED, linestyle="--", label="1 m pass threshold")
    axes[1].set_ylabel("minimum gap [m]")
    axes[1].legend(fontsize=8)
    axes[2].bar(positions, peak_jerk, color=colors)
    axes[2].set_ylabel("peak jerk [m/s³]")
    axes[2].set_xticks(positions, labels, rotation=18, ha="right")
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle(
        f"{title} — pass rate {report.pass_rate_percent:.0f}% · "
        f"worst case: {report.worst_item.case.name}"
    )
    figure.tight_layout()
    return figure


def plot_challenge_score(score: ChallengeScore, *, title: str):
    labels = [row[0] for row in score.as_rows()]
    values = [row[2] for row in score.as_rows()]
    weights = [row[1] for row in score.as_rows()]
    colors = [GREEN if value >= 70.0 else AMBER if value >= 45.0 else RED
              for value in values]
    figure, ax = plt.subplots(figsize=(10, 5.5))
    positions = np.arange(len(labels))
    bars = ax.barh(positions, values, color=colors)
    ax.set_yticks(
        positions,
        [f"{label} ({weight}%)" for label, weight in zip(labels, weights)],
    )
    ax.set_xlim(0.0, 100.0)
    ax.set_xlabel("category result [0–100]")
    ax.grid(axis="x", alpha=0.3)
    for bar, value in zip(bars, values):
        ax.text(
            min(value + 2.0, 97.0),
            bar.get_y() + bar.get_height() / 2,
            f"{value:.0f}",
            va="center",
            color=NAVY,
        )
    suffix = " (safety cap applied)" if score.safety_cap_applied else ""
    ax.set_title(
        f"{title}\nFinal score: {score.final_score:.1f}/100{suffix}"
    )
    figure.tight_layout()
    return figure

