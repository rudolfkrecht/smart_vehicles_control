"""Tracking-error geometry and Pure Pursuit steering reused on Day 4."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .bicycle import (
    VehicleParameters,
    VehicleState,
    bicycle_step,
    normalize_angle,
)
from .paths import ReferencePath, make_reference_path, offset_from_path


@dataclass(frozen=True)
class TrackingErrors:
    nearest_index: int
    cross_track_error: float
    heading_error: float
    distance_to_path: float


@dataclass(frozen=True)
class PurePursuitOutput:
    steering: float
    lookahead_distance: float
    nearest_index: int
    target_index: int
    target_x: float
    target_y: float
    alpha: float
    cross_track_error: float
    heading_error: float


@dataclass(frozen=True)
class PathFollowingScenario:
    """Settings for a constant-speed path-following run."""

    duration: float = 22.0
    dt: float = 0.05
    speed: float = 8.0
    path_kind: str = "training"
    base_lookahead: float = 3.0
    speed_lookahead_gain: float = 0.25
    initial_lateral_offset: float = 1.5
    initial_heading_offset_degrees: float = 4.0
    road_half_width: float = 3.5
    disturbance_time: float | None = None
    disturbance_offset: float = 0.0
    measurement_noise_std: float = 0.0
    random_seed: int = 7
    enable_steering_rate_limit: bool = True

    def __post_init__(self) -> None:
        if self.duration <= 0.0 or self.dt <= 0.0:
            raise ValueError("duration and dt must be positive")
        if self.speed <= 0.0:
            raise ValueError("speed must be positive")
        if self.base_lookahead <= 0.0:
            raise ValueError("base_lookahead must be positive")
        if self.speed_lookahead_gain < 0.0:
            raise ValueError("speed_lookahead_gain cannot be negative")
        if self.road_half_width <= 0.0:
            raise ValueError("road_half_width must be positive")
        if self.measurement_noise_std < 0.0:
            raise ValueError("measurement_noise_std cannot be negative")


@dataclass(frozen=True)
class PathFollowingResult:
    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    speed: np.ndarray
    steering: np.ndarray
    commanded_steering: np.ndarray
    steering_rate: np.ndarray
    cross_track_error: np.ndarray
    heading_error: np.ndarray
    lookahead_distance: np.ndarray
    nearest_index: np.ndarray
    target_index: np.ndarray
    target_x: np.ndarray
    target_y: np.ndarray
    lateral_acceleration: np.ndarray
    disturbed: np.ndarray
    path: ReferencePath = field(repr=False)
    scenario: PathFollowingScenario = field(repr=False)
    vehicle: VehicleParameters = field(repr=False)

    @property
    def completion_fraction(self) -> float:
        return float(
            self.path.distance[self.nearest_index[-1]] / self.path.length
        )


def nearest_path_index(
    state: VehicleState,
    path: ReferencePath,
    *,
    previous_index: int = 0,
    search_back: int = 8,
    search_forward: int = 100,
) -> int:
    """Find a local nearest sample while preventing backward jumps."""

    start = max(0, previous_index - search_back)
    stop = min(len(path.x), previous_index + search_forward)
    squared_distance = (
        (path.x[start:stop] - state.x) ** 2
        + (path.y[start:stop] - state.y) ** 2
    )
    candidate = start + int(np.argmin(squared_distance))
    return max(previous_index, candidate)


def tracking_errors(
    state: VehicleState,
    path: ReferencePath,
    *,
    previous_index: int = 0,
) -> TrackingErrors:
    """Compute signed lateral error and wrapped heading error."""

    index = nearest_path_index(
        state,
        path,
        previous_index=previous_index,
    )
    dx = state.x - path.x[index]
    dy = state.y - path.y[index]
    path_heading = path.heading[index]
    left_normal_x = -math.sin(path_heading)
    left_normal_y = math.cos(path_heading)
    signed_error = dx * left_normal_x + dy * left_normal_y
    return TrackingErrors(
        nearest_index=index,
        cross_track_error=float(signed_error),
        heading_error=normalize_angle(state.heading - path_heading),
        distance_to_path=math.hypot(dx, dy),
    )


def pure_pursuit(
    state: VehicleState,
    path: ReferencePath,
    *,
    vehicle: VehicleParameters,
    base_lookahead: float,
    speed_lookahead_gain: float = 0.0,
    previous_index: int = 0,
) -> PurePursuitOutput:
    """Calculate a Pure Pursuit steering command."""

    if base_lookahead <= 0.0 or speed_lookahead_gain < 0.0:
        raise ValueError("look-ahead parameters are invalid")

    errors = tracking_errors(
        state,
        path,
        previous_index=previous_index,
    )
    lookahead = base_lookahead + speed_lookahead_gain * state.speed
    target_distance = (
        path.distance[errors.nearest_index] + lookahead
    )
    target_index = int(
        np.searchsorted(path.distance, target_distance, side="left")
    )
    target_index = min(target_index, len(path.x) - 1)
    target_x = float(path.x[target_index])
    target_y = float(path.y[target_index])
    alpha = normalize_angle(
        math.atan2(target_y - state.y, target_x - state.x)
        - state.heading
    )
    geometric_distance = max(
        math.hypot(target_x - state.x, target_y - state.y),
        0.1,
    )
    steering = math.atan2(
        2.0 * vehicle.wheelbase * math.sin(alpha),
        geometric_distance,
    )
    steering = float(
        np.clip(
            steering,
            -vehicle.maximum_steering,
            vehicle.maximum_steering,
        )
    )
    return PurePursuitOutput(
        steering=steering,
        lookahead_distance=lookahead,
        nearest_index=errors.nearest_index,
        target_index=target_index,
        target_x=target_x,
        target_y=target_y,
        alpha=alpha,
        cross_track_error=errors.cross_track_error,
        heading_error=errors.heading_error,
    )


def apply_lateral_offset(
    state: VehicleState,
    lateral_offset: float,
) -> None:
    """Instantaneously displace a vehicle left/right of its heading."""

    state.x -= lateral_offset * math.sin(state.heading)
    state.y += lateral_offset * math.cos(state.heading)


def run_path_following(
    *,
    scenario: PathFollowingScenario | None = None,
    vehicle: VehicleParameters | None = None,
    path: ReferencePath | None = None,
) -> PathFollowingResult:
    """Run one deterministic Pure Pursuit experiment."""

    scenario = scenario or PathFollowingScenario()
    vehicle = vehicle or VehicleParameters()
    path = path or make_reference_path(scenario.path_kind)
    initial_x, initial_y, initial_heading = offset_from_path(
        path,
        distance=0.0,
        lateral_offset=scenario.initial_lateral_offset,
        heading_offset_degrees=scenario.initial_heading_offset_degrees,
    )
    state = VehicleState(
        x=initial_x,
        y=initial_y,
        heading=initial_heading,
        speed=scenario.speed,
    )
    rng = np.random.default_rng(scenario.random_seed)
    maximum_steps = int(round(scenario.duration / scenario.dt)) + 1

    history: dict[str, list[float | int | bool]] = {
        key: []
        for key in (
            "time",
            "x",
            "y",
            "heading",
            "speed",
            "steering",
            "commanded_steering",
            "steering_rate",
            "cross_track_error",
            "heading_error",
            "lookahead_distance",
            "nearest_index",
            "target_index",
            "target_x",
            "target_y",
            "lateral_acceleration",
            "disturbed",
        )
    }
    previous_index = 0
    disturbance_applied = False

    for step_index in range(maximum_steps):
        time = step_index * scenario.dt
        disturbed_now = False
        if (
            scenario.disturbance_time is not None
            and not disturbance_applied
            and time >= scenario.disturbance_time
        ):
            apply_lateral_offset(state, scenario.disturbance_offset)
            disturbance_applied = True
            disturbed_now = True

        measured_state = state.copy()
        if scenario.measurement_noise_std > 0.0:
            measured_state.x += rng.normal(
                0.0,
                scenario.measurement_noise_std,
            )
            measured_state.y += rng.normal(
                0.0,
                scenario.measurement_noise_std,
            )

        output = pure_pursuit(
            measured_state,
            path,
            vehicle=vehicle,
            base_lookahead=scenario.base_lookahead,
            speed_lookahead_gain=scenario.speed_lookahead_gain,
            previous_index=previous_index,
        )
        previous_index = output.nearest_index
        sample = bicycle_step(
            state,
            output.steering,
            parameters=vehicle,
            dt=scenario.dt,
            speed=scenario.speed,
            enable_rate_limit=scenario.enable_steering_rate_limit,
        )

        values = {
            "time": time,
            "x": state.x,
            "y": state.y,
            "heading": state.heading,
            "speed": state.speed,
            "steering": sample.applied_steering,
            "commanded_steering": output.steering,
            "steering_rate": sample.steering_rate,
            "cross_track_error": output.cross_track_error,
            "heading_error": output.heading_error,
            "lookahead_distance": output.lookahead_distance,
            "nearest_index": output.nearest_index,
            "target_index": output.target_index,
            "target_x": output.target_x,
            "target_y": output.target_y,
            "lateral_acceleration": sample.lateral_acceleration,
            "disturbed": disturbed_now,
        }
        for key, value in values.items():
            history[key].append(value)

        if previous_index >= len(path.x) - 3:
            break

    return PathFollowingResult(
        time=np.asarray(history["time"], dtype=float),
        x=np.asarray(history["x"], dtype=float),
        y=np.asarray(history["y"], dtype=float),
        heading=np.asarray(history["heading"], dtype=float),
        speed=np.asarray(history["speed"], dtype=float),
        steering=np.asarray(history["steering"], dtype=float),
        commanded_steering=np.asarray(
            history["commanded_steering"],
            dtype=float,
        ),
        steering_rate=np.asarray(history["steering_rate"], dtype=float),
        cross_track_error=np.asarray(
            history["cross_track_error"],
            dtype=float,
        ),
        heading_error=np.asarray(history["heading_error"], dtype=float),
        lookahead_distance=np.asarray(
            history["lookahead_distance"],
            dtype=float,
        ),
        nearest_index=np.asarray(history["nearest_index"], dtype=int),
        target_index=np.asarray(history["target_index"], dtype=int),
        target_x=np.asarray(history["target_x"], dtype=float),
        target_y=np.asarray(history["target_y"], dtype=float),
        lateral_acceleration=np.asarray(
            history["lateral_acceleration"],
            dtype=float,
        ),
        disturbed=np.asarray(history["disturbed"], dtype=bool),
        path=path,
        scenario=scenario,
        vehicle=vehicle,
    )
