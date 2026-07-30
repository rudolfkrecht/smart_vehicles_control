"""Reference-path creation and geometry for the Day 4 test scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ReferencePath:
    """A sampled path with arc length, tangent heading and curvature."""

    x: np.ndarray
    y: np.ndarray
    distance: np.ndarray
    heading: np.ndarray
    curvature: np.ndarray
    waypoint_x: np.ndarray
    waypoint_y: np.ndarray
    name: str = "reference path"

    def __post_init__(self) -> None:
        sizes = {
            len(self.x),
            len(self.y),
            len(self.distance),
            len(self.heading),
            len(self.curvature),
        }
        if len(sizes) != 1 or len(self.x) < 2:
            raise ValueError("path arrays must have the same non-trivial size")
        if not np.all(np.diff(self.distance) > 0.0):
            raise ValueError("path distance must increase strictly")

    @property
    def length(self) -> float:
        return float(self.distance[-1])


def _geometry(
    x: np.ndarray,
    y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    segment_length = np.hypot(np.diff(x), np.diff(y))
    if np.any(segment_length <= 1e-8):
        raise ValueError("consecutive path samples must be distinct")
    distance = np.concatenate(([0.0], np.cumsum(segment_length)))
    dx = np.gradient(x, distance)
    dy = np.gradient(y, distance)
    heading = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(heading, distance)
    return distance, heading, curvature


def _hermite_segment(
    point_0: np.ndarray,
    point_1: np.ndarray,
    tangent_0: np.ndarray,
    tangent_1: np.ndarray,
    samples: int,
) -> np.ndarray:
    """Interpolate one cubic Hermite segment without SciPy."""

    u = np.linspace(0.0, 1.0, samples, endpoint=False)
    h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
    h10 = u**3 - 2.0 * u**2 + u
    h01 = -2.0 * u**3 + 3.0 * u**2
    h11 = u**3 - u**2
    return (
        h00[:, None] * point_0
        + h10[:, None] * tangent_0
        + h01[:, None] * point_1
        + h11[:, None] * tangent_1
    )


def path_from_waypoints(
    waypoint_x: np.ndarray,
    waypoint_y: np.ndarray,
    *,
    samples_per_segment: int = 30,
    name: str = "waypoint path",
) -> ReferencePath:
    """Create a smooth path using cardinal cubic Hermite interpolation."""

    waypoint_x = np.asarray(waypoint_x, dtype=float)
    waypoint_y = np.asarray(waypoint_y, dtype=float)
    if waypoint_x.shape != waypoint_y.shape or waypoint_x.ndim != 1:
        raise ValueError("waypoint coordinates must be matching 1-D arrays")
    if len(waypoint_x) < 3:
        raise ValueError("at least three waypoints are required")
    if samples_per_segment < 4:
        raise ValueError("samples_per_segment must be at least four")

    points = np.column_stack((waypoint_x, waypoint_y))
    tangents = np.empty_like(points)
    tangents[0] = points[1] - points[0]
    tangents[-1] = points[-1] - points[-2]
    tangents[1:-1] = 0.5 * (points[2:] - points[:-2])

    pieces = [
        _hermite_segment(
            points[index],
            points[index + 1],
            tangents[index],
            tangents[index + 1],
            samples_per_segment,
        )
        for index in range(len(points) - 1)
    ]
    dense = np.vstack((*pieces, points[-1][None, :]))
    distance, heading, curvature = _geometry(dense[:, 0], dense[:, 1])
    return ReferencePath(
        x=dense[:, 0],
        y=dense[:, 1],
        distance=distance,
        heading=heading,
        curvature=curvature,
        waypoint_x=waypoint_x,
        waypoint_y=waypoint_y,
        name=name,
    )


def make_reference_path(kind: str = "integrated") -> ReferencePath:
    """Return one of the prepared path-following and traffic roads."""

    paths = {
        "gentle": (
            np.array([0, 18, 36, 54, 72, 90, 108], dtype=float),
            np.array([0, 1.5, 5.5, 7.0, 4.0, 0.5, 0.0], dtype=float),
        ),
        "training": (
            np.array([0, 15, 30, 45, 60, 75, 90, 105, 120], dtype=float),
            np.array([0, 0, 7, 10, 4, -5, -7, -2, 0], dtype=float),
        ),
        "tight": (
            np.array([0, 12, 24, 34, 44, 56, 68, 80, 94], dtype=float),
            np.array([0, 1, 9, 13, 5, -9, -12, -3, 0], dtype=float),
        ),
        "integrated": (
            np.array(
                [0, 20, 40, 58, 75, 92, 110, 130, 150, 170, 190, 210],
                dtype=float,
            ),
            np.array(
                [0, 0, 4, 12, 10, 0, -10, -12, -3, 8, 7, 0],
                dtype=float,
            ),
        ),
        "traffic": (
            np.array(
                [0, 24, 48, 72, 96, 120, 144, 168, 192, 216, 240],
                dtype=float,
            ),
            np.array(
                [0, 0, 6, 9, 2, -7, -8, 1, 9, 5, 0],
                dtype=float,
            ),
        ),
        "practice": (
            np.array(
                [0, 18, 36, 54, 70, 86, 102, 120, 140, 160, 180, 200],
                dtype=float,
            ),
            np.array(
                [0, 1, 7, 13, 8, -3, -12, -10, 0, 10, 6, 0],
                dtype=float,
            ),
        ),
        "evaluation_a": (
            np.array(
                [0, 17, 34, 49, 64, 80, 96, 114, 132, 151, 171, 192, 214],
                dtype=float,
            ),
            np.array(
                [0, -1, -7, -12, -6, 5, 13, 9, -2, -11, -7, 3, 0],
                dtype=float,
            ),
        ),
        "evaluation_b": (
            np.array(
                [0, 16, 32, 47, 62, 78, 95, 113, 132, 152, 173, 195, 218],
                dtype=float,
            ),
            np.array(
                [0, 2, 10, 14, 5, -8, -14, -5, 8, 13, 3, -8, 0],
                dtype=float,
            ),
        ),
    }
    if kind not in paths:
        raise ValueError(
            f"unknown path {kind!r}; choose from {', '.join(paths)}"
        )
    waypoint_x, waypoint_y = paths[kind]
    return path_from_waypoints(
        waypoint_x,
        waypoint_y,
        samples_per_segment=35,
        name=f"{kind} course",
    )


def point_at_distance(
    path: ReferencePath,
    distance: float,
) -> tuple[float, float, float]:
    """Interpolate position and heading at an arc-length coordinate."""

    clipped = float(np.clip(distance, 0.0, path.length))
    x = float(np.interp(clipped, path.distance, path.x))
    y = float(np.interp(clipped, path.distance, path.y))
    heading = float(np.interp(clipped, path.distance, path.heading))
    return x, y, heading


def offset_from_path(
    path: ReferencePath,
    *,
    distance: float,
    lateral_offset: float,
    heading_offset_degrees: float = 0.0,
) -> tuple[float, float, float]:
    """Create a vehicle pose relative to the reference path."""

    x, y, heading = point_at_distance(path, distance)
    x -= lateral_offset * math.sin(heading)
    y += lateral_offset * math.cos(heading)
    heading += math.radians(heading_offset_degrees)
    return x, y, heading
