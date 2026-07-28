"""Lesson 3: visualise waypoints, a continuous path and tracking errors."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch, Polygon
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.bicycle import VehicleParameters, VehicleState
from simulator.paths import make_reference_path, offset_from_path
from simulator.plotting import configure_plot_style, save_or_show
from simulator.script_helpers import common_parser
from simulator.tracking import pure_pursuit


LATERAL_OFFSET_M = 2.2
HEADING_OFFSET_DEGREES = 18.0
LOOKAHEAD_DISTANCE_M = 6.0
PATH_DISTANCE_M = 40.0


def _vehicle_polygon(
    x: float,
    y: float,
    heading: float,
) -> np.ndarray:
    body = np.array(
        [
            [-1.6, -0.8],
            [1.6, -0.8],
            [1.6, 0.8],
            [-1.6, 0.8],
        ]
    )
    rotation = np.array(
        [
            [math.cos(heading), -math.sin(heading)],
            [math.sin(heading), math.cos(heading)],
        ]
    )
    return body @ rotation.T + np.array([x, y])


def main() -> None:
    args = common_parser(__doc__).parse_args()
    path = make_reference_path("training")
    vehicle = VehicleParameters()
    x, y, heading = offset_from_path(
        path,
        distance=PATH_DISTANCE_M,
        lateral_offset=LATERAL_OFFSET_M,
        heading_offset_degrees=HEADING_OFFSET_DEGREES,
    )
    state = VehicleState(x=x, y=y, heading=heading, speed=8.0)
    output = pure_pursuit(
        state,
        path,
        vehicle=vehicle,
        base_lookahead=LOOKAHEAD_DISTANCE_M,
    )
    nearest_x = float(path.x[output.nearest_index])
    nearest_y = float(path.y[output.nearest_index])

    configure_plot_style()
    figure, axis = plt.subplots(figsize=(10.8, 6.2))
    axis.plot(
        path.x,
        path.y,
        color="#17233c",
        linewidth=2.0,
        label="continuous reference path",
    )
    axis.plot(
        path.waypoint_x,
        path.waypoint_y,
        "o",
        color="#64748b",
        label="sparse waypoints",
    )
    axis.add_patch(
        Polygon(
            _vehicle_polygon(x, y, heading),
            closed=True,
            facecolor="#2476d8",
            edgecolor="#17233c",
            linewidth=1.8,
            label="vehicle",
        )
    )
    axis.plot(
        [x, nearest_x],
        [y, nearest_y],
        "--",
        color="#e05252",
        linewidth=2.0,
        label=r"cross-track error $e_y$",
    )
    axis.plot(
        [x, output.target_x],
        [y, output.target_y],
        color="#f59e0b",
        linewidth=2.0,
        label="look-ahead line",
    )
    axis.add_patch(
        Circle(
            (x, y),
            LOOKAHEAD_DISTANCE_M,
            fill=False,
            linestyle=":",
            edgecolor="#f59e0b",
        )
    )
    axis.scatter(
        [nearest_x, output.target_x],
        [nearest_y, output.target_y],
        color=["#e05252", "#f59e0b"],
        s=70,
        zorder=5,
    )
    heading_length = 5.0
    axis.add_patch(
        FancyArrowPatch(
            (x, y),
            (
                x + heading_length * math.cos(heading),
                y + heading_length * math.sin(heading),
            ),
            arrowstyle="-|>",
            mutation_scale=15,
            color="#2476d8",
            linewidth=2.0,
        )
    )
    axis.add_patch(
        Arc(
            (x, y),
            5.0,
            5.0,
            angle=0,
            theta1=math.degrees(heading - output.heading_error),
            theta2=math.degrees(heading),
            color="#8b5cf6",
            linewidth=2.0,
        )
    )
    axis.annotate(
        f"heading error = {math.degrees(output.heading_error):+.1f}°",
        (x + 1.0, y + 2.0),
        color="#6d28d9",
    )
    axis.set_title("The controller needs geometry, not only waypoint IDs")
    axis.set_xlabel("global x [m]")
    axis.set_ylabel("global y [m]")
    axis.axis("equal")
    axis.legend(loc="upper left", ncol=2)

    print(f"nearest sample index: {output.nearest_index}")
    print(f"look-ahead target index: {output.target_index}")
    print(f"signed cross-track error: {output.cross_track_error:+.2f} m")
    print(f"heading error: {math.degrees(output.heading_error):+.2f} deg")
    print(f"target bearing alpha: {math.degrees(output.alpha):+.2f} deg")

    save_or_show(
        figure,
        output=args.output,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
