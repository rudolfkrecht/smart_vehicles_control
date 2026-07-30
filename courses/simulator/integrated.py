"""Integrated lateral, speed-planning and traffic foundation for Day 4."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import math

import numpy as np

from .bicycle import VehicleParameters, VehicleState, bicycle_step
from .longitudinal import (
    LongitudinalOutput,
    LongitudinalParameters,
    SpeedController,
    integrate_speed,
)
from .paths import (
    ReferencePath,
    make_reference_path,
    offset_from_path,
    point_at_distance,
)
from .speed_profile import (
    SpeedProfile,
    SpeedProfileParameters,
    build_speed_profile,
)
from .tracking import PurePursuitOutput, pure_pursuit
from .traffic import (
    ACCParameters,
    BehaviourController,
    BehaviourOutput,
    BehaviourState,
    lead_speed_schedule,
)


@dataclass(frozen=True)
class IntegratedScenario:
    """Switches and parameters for one complete integrated experiment."""

    duration: float = 38.0
    dt: float = 0.05
    path_kind: str = "integrated"
    initial_speed: float = 8.0
    initial_lateral_offset: float = 0.2
    initial_heading_offset_degrees: float = 0.0
    road_half_width: float = 3.5
    base_lookahead: float = 3.0
    speed_lookahead_gain: float = 0.25
    enable_curve_speed: bool = True
    enable_traffic: bool = True
    enable_state_machine: bool = True
    lead_preset: str = "stop_and_go"
    initial_lead_distance: float = 48.0
    lead_vehicle_length: float = 4.5
    stop_when_complete: bool = True
    speed_profile: SpeedProfileParameters = field(
        default_factory=SpeedProfileParameters
    )
    longitudinal: LongitudinalParameters = field(
        default_factory=LongitudinalParameters
    )
    acc: ACCParameters = field(default_factory=ACCParameters)

    def __post_init__(self) -> None:
        if self.duration <= 0.0 or self.dt <= 0.0:
            raise ValueError("duration and dt must be positive")
        if self.initial_speed < 0.0:
            raise ValueError("initial_speed cannot be negative")
        if self.road_half_width <= 0.0:
            raise ValueError("road_half_width must be positive")
        if self.base_lookahead <= 0.0:
            raise ValueError("base_lookahead must be positive")
        if self.speed_lookahead_gain < 0.0:
            raise ValueError("speed_lookahead_gain cannot be negative")
        if self.initial_lead_distance <= 0.0:
            raise ValueError("initial_lead_distance must be positive")


@dataclass(frozen=True)
class LiveSnapshot:
    time: float
    vehicle: VehicleState
    lead_distance: float
    lead_speed: float
    lead_x: float
    lead_y: float
    lead_heading: float
    path_distance: float
    gap: float
    road_target_speed: float
    traffic_target_speed: float
    selected_target_speed: float
    lateral: PurePursuitOutput
    traffic: BehaviourOutput
    longitudinal: LongitudinalOutput
    applied_steering: float
    steering_rate: float
    lateral_acceleration: float
    cross_track_error: float
    collision: bool
    complete: bool


@dataclass(frozen=True)
class IntegratedResult:
    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    speed: np.ndarray
    steering: np.ndarray
    steering_rate: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    lateral_acceleration: np.ndarray
    cross_track_error: np.ndarray
    nearest_index: np.ndarray
    target_index: np.ndarray
    target_x: np.ndarray
    target_y: np.ndarray
    path_distance: np.ndarray
    road_target_speed: np.ndarray
    traffic_target_speed: np.ndarray
    selected_target_speed: np.ndarray
    lead_distance: np.ndarray
    lead_speed: np.ndarray
    lead_x: np.ndarray
    lead_y: np.ndarray
    gap: np.ndarray
    desired_gap: np.ndarray
    closing_speed: np.ndarray
    time_to_collision: np.ndarray
    behaviour_state: np.ndarray
    collision: np.ndarray
    saturated: np.ndarray
    jerk_limited: np.ndarray
    path: ReferencePath = field(repr=False)
    profile: SpeedProfile = field(repr=False)
    scenario: IntegratedScenario = field(repr=False)
    vehicle_parameters: VehicleParameters = field(repr=False)

    @property
    def completion_fraction(self) -> float:
        return float(
            np.clip(self.path_distance[-1] / self.path.length, 0.0, 1.0)
        )


class IntegratedSimulation:
    """Stateful simulator shared by batch scripts and the PyQt laboratory."""

    _HISTORY_KEYS = (
        "time",
        "x",
        "y",
        "heading",
        "speed",
        "steering",
        "steering_rate",
        "acceleration",
        "jerk",
        "lateral_acceleration",
        "cross_track_error",
        "nearest_index",
        "target_index",
        "target_x",
        "target_y",
        "path_distance",
        "road_target_speed",
        "traffic_target_speed",
        "selected_target_speed",
        "lead_distance",
        "lead_speed",
        "lead_x",
        "lead_y",
        "gap",
        "desired_gap",
        "closing_speed",
        "time_to_collision",
        "behaviour_state",
        "collision",
        "saturated",
        "jerk_limited",
    )

    def __init__(
        self,
        scenario: IntegratedScenario | None = None,
        *,
        path: ReferencePath | None = None,
        vehicle_parameters: VehicleParameters | None = None,
    ) -> None:
        self.scenario = scenario or IntegratedScenario()
        self.path = path or make_reference_path(self.scenario.path_kind)
        self.vehicle_parameters = (
            vehicle_parameters or VehicleParameters()
        )
        self.profile = build_speed_profile(
            self.path,
            self.scenario.speed_profile,
        )
        self.speed_controller = SpeedController(
            self.scenario.longitudinal
        )
        self.behaviour_controller = BehaviourController(self.scenario.acc)
        self.history: dict[str, list[float | int | bool | str]] = {
            key: [] for key in self._HISTORY_KEYS
        }
        self.reset()

    def reset(self) -> None:
        x, y, heading = offset_from_path(
            self.path,
            distance=0.0,
            lateral_offset=self.scenario.initial_lateral_offset,
            heading_offset_degrees=(
                self.scenario.initial_heading_offset_degrees
            ),
        )
        self.vehicle = VehicleState(
            x=x,
            y=y,
            heading=heading,
            speed=self.scenario.initial_speed,
        )
        self.time = 0.0
        self.previous_index = 0
        self.lead_distance = self.scenario.initial_lead_distance
        self.lead_speed = lead_speed_schedule(
            0.0,
            self.scenario.lead_preset,
        )
        self.speed_controller.reset()
        self.behaviour_controller.reset()
        self.previous_acceleration = 0.0
        self.complete = False
        self.history = {key: [] for key in self._HISTORY_KEYS}

    def _traffic_output(
        self,
        *,
        gap: float,
        road_target: float,
    ) -> BehaviourOutput:
        if self.scenario.enable_traffic:
            return self.behaviour_controller.update(
                gap=gap,
                ego_speed=self.vehicle.speed,
                lead_speed=self.lead_speed,
                cruise_speed=self.scenario.speed_profile.global_speed_limit,
                enable_state_machine=self.scenario.enable_state_machine,
            )
        desired = (
            self.scenario.acc.standstill_gap
            + self.scenario.acc.time_headway * self.vehicle.speed
        )
        return BehaviourOutput(
            state=BehaviourState.CRUISE,
            desired_gap=desired,
            gap_error=gap - desired,
            closing_speed=self.vehicle.speed - self.lead_speed,
            time_to_collision=math.inf,
            target_speed=self.scenario.speed_profile.global_speed_limit,
            acceleration_override=None,
        )

    def step(self) -> LiveSnapshot:
        if self.complete:
            raise StopIteration("simulation has reached the end of the path")

        scenario = self.scenario
        self.lead_speed = lead_speed_schedule(
            self.time,
            scenario.lead_preset,
        )
        if self.lead_distance >= self.path.length:
            self.lead_speed = 0.0
        self.lead_distance = min(
            self.path.length,
            self.lead_distance + self.lead_speed * scenario.dt,
        )

        lateral = pure_pursuit(
            self.vehicle,
            self.path,
            vehicle=self.vehicle_parameters,
            base_lookahead=scenario.base_lookahead,
            speed_lookahead_gain=scenario.speed_lookahead_gain,
            previous_index=self.previous_index,
        )
        self.previous_index = lateral.nearest_index
        path_distance = float(
            self.path.distance[lateral.nearest_index]
        )
        road_target = (
            self.profile.speed_at(path_distance)
            if scenario.enable_curve_speed
            else scenario.speed_profile.global_speed_limit
        )
        gap = (
            self.lead_distance
            - path_distance
            - scenario.lead_vehicle_length
            if scenario.enable_traffic
            else math.inf
        )
        traffic = self._traffic_output(gap=gap, road_target=road_target)
        traffic_target = traffic.target_speed
        selected_target = min(road_target, traffic_target)
        longitudinal = self.speed_controller.update(
            target_speed=selected_target,
            measured_speed=self.vehicle.speed,
            dt=scenario.dt,
            acceleration_override=traffic.acceleration_override,
        )
        acceleration = longitudinal.applied_acceleration
        jerk = (
            acceleration - self.previous_acceleration
        ) / scenario.dt
        self.previous_acceleration = acceleration
        self.vehicle.speed = integrate_speed(
            self.vehicle.speed,
            acceleration,
            scenario.dt,
        )
        motion = bicycle_step(
            self.vehicle,
            lateral.steering,
            parameters=self.vehicle_parameters,
            dt=scenario.dt,
            speed=self.vehicle.speed,
            enable_rate_limit=True,
        )
        lead_x, lead_y, lead_heading = point_at_distance(
            self.path,
            self.lead_distance,
        )
        collision = scenario.enable_traffic and gap <= 0.0
        self.time += scenario.dt
        self.complete = (
            self.previous_index >= len(self.path.x) - 3
            and scenario.stop_when_complete
        ) or self.time >= scenario.duration

        snapshot = LiveSnapshot(
            time=self.time,
            vehicle=self.vehicle.copy(),
            lead_distance=self.lead_distance,
            lead_speed=self.lead_speed,
            lead_x=lead_x,
            lead_y=lead_y,
            lead_heading=lead_heading,
            path_distance=path_distance,
            gap=gap,
            road_target_speed=road_target,
            traffic_target_speed=traffic_target,
            selected_target_speed=selected_target,
            lateral=lateral,
            traffic=traffic,
            longitudinal=longitudinal,
            applied_steering=motion.applied_steering,
            steering_rate=motion.steering_rate,
            lateral_acceleration=motion.lateral_acceleration,
            cross_track_error=lateral.cross_track_error,
            collision=collision,
            complete=self.complete,
        )
        self._record(snapshot, acceleration, jerk)
        return snapshot

    def _record(
        self,
        snapshot: LiveSnapshot,
        acceleration: float,
        jerk: float,
    ) -> None:
        values: dict[str, float | int | bool | str] = {
            "time": snapshot.time,
            "x": snapshot.vehicle.x,
            "y": snapshot.vehicle.y,
            "heading": snapshot.vehicle.heading,
            "speed": snapshot.vehicle.speed,
            "steering": snapshot.applied_steering,
            "steering_rate": snapshot.steering_rate,
            "acceleration": acceleration,
            "jerk": jerk,
            "lateral_acceleration": snapshot.lateral_acceleration,
            "cross_track_error": snapshot.cross_track_error,
            "nearest_index": snapshot.lateral.nearest_index,
            "target_index": snapshot.lateral.target_index,
            "target_x": snapshot.lateral.target_x,
            "target_y": snapshot.lateral.target_y,
            "path_distance": snapshot.path_distance,
            "road_target_speed": snapshot.road_target_speed,
            "traffic_target_speed": snapshot.traffic_target_speed,
            "selected_target_speed": snapshot.selected_target_speed,
            "lead_distance": snapshot.lead_distance,
            "lead_speed": snapshot.lead_speed,
            "lead_x": snapshot.lead_x,
            "lead_y": snapshot.lead_y,
            "gap": snapshot.gap,
            "desired_gap": snapshot.traffic.desired_gap,
            "closing_speed": snapshot.traffic.closing_speed,
            "time_to_collision": snapshot.traffic.time_to_collision,
            "behaviour_state": snapshot.traffic.state.value,
            "collision": snapshot.collision,
            "saturated": snapshot.longitudinal.saturated,
            "jerk_limited": snapshot.longitudinal.jerk_limited,
        }
        for key, value in values.items():
            self.history[key].append(value)

    def result(self) -> IntegratedResult:
        if not self.history["time"]:
            raise ValueError("simulation has no samples")
        integer_keys = {"nearest_index", "target_index"}
        boolean_keys = {"collision", "saturated", "jerk_limited"}
        string_keys = {"behaviour_state"}
        arrays: dict[str, np.ndarray] = {}
        for key, values in self.history.items():
            if key in integer_keys:
                dtype = int
            elif key in boolean_keys:
                dtype = bool
            elif key in string_keys:
                dtype = str
            else:
                dtype = float
            arrays[key] = np.asarray(values, dtype=dtype)

        result_arguments = {
            field_info.name: arrays[field_info.name]
            for field_info in fields(IntegratedResult)
            if field_info.name in arrays
        }
        return IntegratedResult(
            **result_arguments,
            path=self.path,
            profile=self.profile,
            scenario=self.scenario,
            vehicle_parameters=self.vehicle_parameters,
        )


def run_integrated(
    scenario: IntegratedScenario | None = None,
    *,
    path: ReferencePath | None = None,
    vehicle_parameters: VehicleParameters | None = None,
) -> IntegratedResult:
    """Run a deterministic integrated-control scenario to completion."""

    simulation = IntegratedSimulation(
        scenario,
        path=path,
        vehicle_parameters=vehicle_parameters,
    )
    maximum_steps = int(
        math.ceil(simulation.scenario.duration / simulation.scenario.dt)
    ) + 2
    for _ in range(maximum_steps):
        if simulation.complete:
            break
        simulation.step()
    return simulation.result()
