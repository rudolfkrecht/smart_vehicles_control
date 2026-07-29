"""Small software 3D renderer used by the tkinter interface."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .model import VehicleState
from .track import ClosedHighwayTrack

Point3 = tuple[float, float, float]
Point2 = tuple[float, float]


def add(a: Point3, b: Point3) -> Point3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def subtract(a: Point3, b: Point3) -> Point3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def scale(v: Point3, value: float) -> Point3:
    return v[0] * value, v[1] * value, v[2] * value


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v: Point3) -> Point3:
    length = math.sqrt(dot(v, v))
    if length < 1e-9:
        return 1.0, 0.0, 0.0
    return scale(v, 1.0 / length)


@dataclass
class Primitive:
    depth: float
    kind: str
    points: list[Point2]
    fill: str
    outline: str = ""
    width: int = 1


class Projector:
    def __init__(
        self,
        camera: Point3,
        target: Point3,
        width: int,
        height: int,
        fov_degrees: float = 72.0,
    ) -> None:
        self.camera = camera
        self.forward = normalize(subtract(target, camera))
        world_up = (0.0, 0.0, 1.0)
        self.right = normalize(cross(self.forward, world_up))
        self.up = normalize(cross(self.right, self.forward))
        self.width = width
        self.height = height
        self.focal = 0.5 * width / math.tan(math.radians(fov_degrees) / 2.0)

    def camera_coordinates(self, point: Point3) -> Point3:
        relative = subtract(point, self.camera)
        return (
            dot(relative, self.right),
            dot(relative, self.up),
            dot(relative, self.forward),
        )

    def project(self, point: Point3) -> tuple[Point2, float] | None:
        x, y, depth = self.camera_coordinates(point)
        if depth <= 0.5:
            return None
        screen_x = 0.5 * self.width + self.focal * x / depth
        screen_y = 0.53 * self.height - self.focal * y / depth
        return (screen_x, screen_y), depth


class SceneBuilder:
    """Build painter-ordered screen primitives without depending on tkinter."""

    def __init__(self, width: int, height: int) -> None:
        self.width = max(320, width)
        self.height = max(240, height)

    def build(
        self,
        track: ClosedHighwayTrack,
        vehicle: VehicleState,
        slope: float,
        lead_vehicle: VehicleState | None = None,
        lead_slope: float = 0.0,
    ) -> list[Primitive]:
        direction = (math.cos(vehicle.heading), math.sin(vehicle.heading), 0.0)
        camera = (
            vehicle.x - 13.5 * direction[0],
            vehicle.y - 13.5 * direction[1],
            vehicle.z + 5.8,
        )
        target = (
            vehicle.x + 13.0 * direction[0],
            vehicle.y + 13.0 * direction[1],
            vehicle.z + 1.2,
        )
        projector = Projector(camera, target, self.width, self.height)
        primitives: list[Primitive] = []
        samples = track.samples
        current_s = track.nearest(
            vehicle.x,
            vehicle.y,
            vehicle.heading,
            lateral_offset=0.0,
        ).pose.s

        for index, first in enumerate(samples):
            second = samples[(index + 1) % len(samples)]
            forward_distance = (first.s - current_s) % track.total_length
            if not (
                forward_distance < 225.0
                or forward_distance > track.total_length - 35.0
            ):
                continue
            midpoint_x = 0.5 * (first.x + second.x)
            midpoint_y = 0.5 * (first.y + second.y)
            if math.hypot(midpoint_x - vehicle.x, midpoint_y - vehicle.y) > 260.0:
                continue

            left_1, right_1 = track.road_edges(first)
            left_2, right_2 = track.road_edges(second)
            outer_left_1, outer_right_1 = track.road_edges(first, 1.2)
            outer_left_2, outer_right_2 = track.road_edges(second, 1.2)
            road_color = "#35393d" if index % 2 == 0 else "#383c40"
            if max(first.z, second.z) > 0.05:
                self._add_polygon(
                    primitives,
                    projector,
                    [
                        outer_left_1,
                        outer_left_2,
                        (outer_left_2[0], outer_left_2[1], 0.0),
                        (outer_left_1[0], outer_left_1[1], 0.0),
                    ],
                    "#526747",
                )
                self._add_polygon(
                    primitives,
                    projector,
                    [
                        outer_right_2,
                        outer_right_1,
                        (outer_right_1[0], outer_right_1[1], 0.0),
                        (outer_right_2[0], outer_right_2[1], 0.0),
                    ],
                    "#526747",
                )
            self._add_polygon(
                primitives,
                projector,
                [left_1, right_1, right_2, left_2],
                road_color,
            )
            self._add_polygon(
                primitives,
                projector,
                [outer_left_1, left_1, left_2, outer_left_2],
                "#706d61",
            )
            self._add_polygon(
                primitives,
                projector,
                [right_1, outer_right_1, outer_right_2, right_2],
                "#706d61",
            )

            self._add_line(
                primitives,
                projector,
                [left_1, left_2],
                "#f4f4f4",
                2,
            )
            self._add_line(
                primitives,
                projector,
                [right_1, right_2],
                "#f4f4f4",
                2,
            )
            if index % 6 < 3:
                centre_1 = (first.x, first.y, first.z + 0.025)
                centre_2 = (second.x, second.y, second.z + 0.025)
                self._add_line(
                    primitives,
                    projector,
                    [centre_1, centre_2],
                    "#f7f7d2",
                    2,
                )

        if lead_vehicle is not None:
            primitives.extend(
                self._car_primitives(
                    projector,
                    lead_vehicle,
                    lead_slope,
                    body_colors=("#ef5350", "#b62f2c", "#7f1d1d"),
                    cabin_colors=("#ffd1d1", "#8c5555", "#623636"),
                )
            )
        primitives.extend(self._car_primitives(projector, vehicle, slope))
        primitives.sort(key=lambda item: item.depth, reverse=True)
        return primitives

    @staticmethod
    def _add_polygon(
        primitives: list[Primitive],
        projector: Projector,
        points: list[Point3],
        fill: str,
        outline: str = "",
    ) -> None:
        projected = [projector.project(point) for point in points]
        if any(item is None for item in projected):
            return
        valid = [item for item in projected if item is not None]
        primitives.append(
            Primitive(
                depth=sum(item[1] for item in valid) / len(valid),
                kind="polygon",
                points=[item[0] for item in valid],
                fill=fill,
                outline=outline,
            )
        )

    @staticmethod
    def _add_line(
        primitives: list[Primitive],
        projector: Projector,
        points: list[Point3],
        fill: str,
        width: int,
    ) -> None:
        projected = [projector.project(point) for point in points]
        if any(item is None for item in projected):
            return
        valid = [item for item in projected if item is not None]
        primitives.append(
            Primitive(
                depth=sum(item[1] for item in valid) / len(valid) - 0.02,
                kind="line",
                points=[item[0] for item in valid],
                fill=fill,
                width=width,
            )
        )

    def _car_primitives(
        self,
        projector: Projector,
        vehicle: VehicleState,
        slope: float,
        *,
        body_colors: tuple[str, str, str] = (
            "#2f80ed",
            "#2061b5",
            "#17477f",
        ),
        cabin_colors: tuple[str, str, str] = (
            "#a7d8f0",
            "#4d7993",
            "#34576d",
        ),
    ) -> list[Primitive]:
        primitives: list[Primitive] = []

        def transform(local: Point3) -> Point3:
            heading = vehicle.heading
            forward = (
                math.cos(heading) * math.cos(slope),
                math.sin(heading) * math.cos(slope),
                math.sin(slope),
            )
            left = (-math.sin(heading), math.cos(heading), 0.0)
            up = (
                -math.cos(heading) * math.sin(slope),
                -math.sin(heading) * math.sin(slope),
                math.cos(slope),
            )
            base = (vehicle.x, vehicle.y, vehicle.z + 0.12)
            return add(
                base,
                add(
                    scale(forward, local[0]),
                    add(scale(left, local[1]), scale(up, local[2])),
                ),
            )

        def add_box(
            center: Point3,
            size: Point3,
            colors: tuple[str, str, str],
        ) -> None:
            cx, cy, cz = center
            lx, ly, lz = (0.5 * value for value in size)
            local_vertices = [
                (cx - lx, cy - ly, cz - lz),
                (cx + lx, cy - ly, cz - lz),
                (cx + lx, cy + ly, cz - lz),
                (cx - lx, cy + ly, cz - lz),
                (cx - lx, cy - ly, cz + lz),
                (cx + lx, cy - ly, cz + lz),
                (cx + lx, cy + ly, cz + lz),
                (cx - lx, cy + ly, cz + lz),
            ]
            vertices = [transform(vertex) for vertex in local_vertices]
            faces = [
                ([4, 5, 6, 7], colors[0]),
                ([1, 2, 6, 5], colors[1]),
                ([0, 4, 7, 3], colors[1]),
                ([2, 3, 7, 6], colors[2]),
                ([0, 1, 5, 4], colors[2]),
            ]
            for indices, color in faces:
                self._add_polygon(
                    primitives,
                    projector,
                    [vertices[index] for index in indices],
                    color,
                    "#17202a",
                )

        add_box((0.0, 0.0, 0.58), (4.4, 1.9, 0.62), body_colors)
        add_box((-0.25, 0.0, 1.08), (2.35, 1.68, 0.72), cabin_colors)
        for x in (-1.42, 1.42):
            for y in (-1.0, 1.0):
                add_box((x, y, 0.38), (0.72, 0.22, 0.68), ("#16191d", "#0d0f12", "#252a2f"))
        return primitives
