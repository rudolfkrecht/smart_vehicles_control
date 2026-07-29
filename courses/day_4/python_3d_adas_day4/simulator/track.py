"""Closed highway geometry with a physically defined elevation profile."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .model import wrap_angle


@dataclass(frozen=True)
class TrackPose:
    s: float
    x: float
    y: float
    z: float
    heading: float
    slope: float


@dataclass(frozen=True)
class TrackingResult:
    pose: TrackPose
    cross_track_error: float
    heading_error: float
    distance: float


class ClosedHighwayTrack:
    """A two-lane stadium circuit with a hill on its first straight."""

    def __init__(
        self,
        straight_length: float = 220.0,
        turn_radius: float = 55.0,
        road_width: float = 9.0,
        lane_offset: float = -2.0,
        sample_spacing: float = 3.0,
    ) -> None:
        self.straight_length = straight_length
        self.turn_radius = turn_radius
        self.road_width = road_width
        self.lane_offset = lane_offset
        self.total_length = 2.0 * straight_length + 2.0 * math.pi * turn_radius
        count = max(80, math.ceil(self.total_length / sample_spacing))
        self.samples = [
            self.pose_at(self.total_length * i / count) for i in range(count)
        ]

    def elevation_at(self, s: float) -> tuple[float, float]:
        """Return road elevation and slope angle.

        The bottom straight contains a 60 m climb at +5 degrees, a 60 m
        elevated plateau and a 60 m descent at -5 degrees.
        """

        s = s % self.total_length
        climb_start = 20.0
        climb_end = 80.0
        plateau_end = 140.0
        descent_end = 200.0
        angle = math.radians(5.0)
        height = (climb_end - climb_start) * math.tan(angle)

        if s < climb_start:
            return 0.0, 0.0
        if s < climb_end:
            return (s - climb_start) * math.tan(angle), angle
        if s < plateau_end:
            return height, 0.0
        if s < descent_end:
            return height - (s - plateau_end) * math.tan(angle), -angle
        return 0.0, 0.0

    def pose_at(self, s: float, lateral_offset: float = 0.0) -> TrackPose:
        s = s % self.total_length
        length = self.straight_length
        radius = self.turn_radius
        arc = math.pi * radius

        if s < length:
            x = -0.5 * length + s
            y = -radius
            heading = 0.0
        elif s < length + arc:
            theta = -0.5 * math.pi + (s - length) / radius
            x = 0.5 * length + radius * math.cos(theta)
            y = radius * math.sin(theta)
            heading = theta + 0.5 * math.pi
        elif s < 2.0 * length + arc:
            progress = s - length - arc
            x = 0.5 * length - progress
            y = radius
            heading = math.pi
        else:
            theta = 0.5 * math.pi + (s - 2.0 * length - arc) / radius
            x = -0.5 * length + radius * math.cos(theta)
            y = radius * math.sin(theta)
            heading = theta + 0.5 * math.pi

        z, slope = self.elevation_at(s)
        x += -math.sin(heading) * lateral_offset
        y += math.cos(heading) * lateral_offset
        return TrackPose(
            s=s,
            x=x,
            y=y,
            z=z,
            heading=wrap_angle(heading),
            slope=slope,
        )

    def nearest(
        self,
        x: float,
        y: float,
        heading: float,
        *,
        lateral_offset: float | None = None,
    ) -> TrackingResult:
        offset = self.lane_offset if lateral_offset is None else lateral_offset
        best = self.samples[0]
        best_distance_sq = float("inf")
        for sample in self.samples:
            dx = x - sample.x
            dy = y - sample.y
            distance_sq = dx * dx + dy * dy
            if distance_sq < best_distance_sq:
                best = sample
                best_distance_sq = distance_sq

        target = self.pose_at(best.s, offset)
        dx = x - target.x
        dy = y - target.y
        left_x = -math.sin(target.heading)
        left_y = math.cos(target.heading)
        cross_track = dx * left_x + dy * left_y
        return TrackingResult(
            pose=target,
            cross_track_error=cross_track,
            heading_error=wrap_angle(target.heading - heading),
            distance=math.hypot(dx, dy),
        )

    def road_edges(self, pose: TrackPose, extra: float = 0.0) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        half_width = 0.5 * self.road_width + extra
        left_x = pose.x - math.sin(pose.heading) * half_width
        left_y = pose.y + math.cos(pose.heading) * half_width
        right_x = pose.x + math.sin(pose.heading) * half_width
        right_y = pose.y - math.cos(pose.heading) * half_width
        return (left_x, left_y, pose.z), (right_x, right_y, pose.z)

