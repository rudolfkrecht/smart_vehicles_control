"""Fault-aware integrated simulation used throughout Day 4."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import math

import numpy as np

from .bicycle import VehicleParameters, VehicleState, bicycle_step
from .faults import FaultInjector, FaultParameters
from .integrated import IntegratedResult, IntegratedScenario
from .longitudinal import integrate_speed
from .paths import ReferencePath, make_reference_path, offset_from_path
from .speed_profile import SpeedProfile, build_speed_profile
from .tracking import PurePursuitOutput, pure_pursuit
from .traffic import (
    BehaviourController,
    BehaviourOutput,
    BehaviourState,
    lead_speed_schedule,
)


@dataclass(frozen=True)
class RobustScenario:
    """Complete controller, environment and fault description for one run."""

    controller: IntegratedScenario = field(
        default_factory=lambda: IntegratedScenario(
            duration=42.0,
            path_kind="practice",
            initial_speed=8.0,
            initial_lead_distance=48.0,
        )
    )
    faults: FaultParameters = field(default_factory=FaultParameters)
    name: str = "nominal practice run"
    test_id: str = "practice_nominal"

    @property
    def duration(self) -> float:
        return self.controller.duration

    @property
    def dt(self) -> float:
        return self.controller.dt


@dataclass(frozen=True)
class RobustSnapshot:
    """One GUI-ready control interval."""

    time: float
    true_vehicle: VehicleState
    measured_vehicle: VehicleState
    lead_distance: float
    lead_speed: float
    lead_x: float
    lead_y: float
    lead_heading: float
    true_path_distance: float
    measured_path_distance: float
    true_gap: float
    measured_gap: float
    road_target_speed: float
    selected_target_speed: float
    lateral: PurePursuitOutput
    true_cross_track_error: float
    traffic: BehaviourOutput
    requested_steering: float
    applied_steering: float
    requested_acceleration: float
    applied_acceleration: float
    lateral_acceleration: float
    steering_rate: float
    jerk: float
    fault_active: bool
    push_applied: bool
    collision: bool
    complete: bool


@dataclass(frozen=True)
class RobustResult:
    """True system history, measurements and control commands."""

    integrated: IntegratedResult
    measured_x: np.ndarray
    measured_y: np.ndarray
    measured_heading: np.ndarray
    measured_speed: np.ndarray
    measured_gap: np.ndarray
    measured_cross_track_error: np.ndarray
    requested_steering: np.ndarray
    requested_acceleration: np.ndarray
    fault_active: np.ndarray
    push_applied: np.ndarray
    robust_scenario: RobustScenario = field(repr=False)

    @property
    def completion_fraction(self) -> float:
        return self.integrated.completion_fraction

    @property
    def time(self) -> np.ndarray:
        return self.integrated.time

    @property
    def scenario(self) -> IntegratedScenario:
        return self.integrated.scenario

    @property
    def path(self) -> ReferencePath:
        return self.integrated.path


class RobustSimulation:
    """Incremental simulation shared by scripts, tests and the PyQt lab."""

    _BASE_KEYS = (
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
    _ROBUST_KEYS = (
        "measured_x",
        "measured_y",
        "measured_heading",
        "measured_speed",
        "measured_gap",
        "measured_cross_track_error",
        "requested_steering",
        "requested_acceleration",
        "fault_active",
        "push_applied",
    )

    def __init__(
        self,
        scenario: RobustScenario | None = None,
        *,
        path: ReferencePath | None = None,
        vehicle_parameters: VehicleParameters | None = None,
    ) -> None:
        self.robust_scenario = scenario or RobustScenario()
        self.scenario = self.robust_scenario.controller
        self.path = path or make_reference_path(self.scenario.path_kind)
        self.vehicle_parameters = vehicle_parameters or VehicleParameters()
        self.profile: SpeedProfile = build_speed_profile(
            self.path,
            self.scenario.speed_profile,
        )
        from .longitudinal import SpeedController

        self.speed_controller = SpeedController(self.scenario.longitudinal)
        self.behaviour_controller = BehaviourController(self.scenario.acc)
        self.injector = FaultInjector(
            self.robust_scenario.faults,
            self.scenario.dt,
        )
        self.history: dict[str, list[float | int | bool | str]] = {}
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
        self.true_previous_index = 0
        self.measured_previous_index = 0
        self.lead_distance = self.scenario.initial_lead_distance
        self.lead_speed = lead_speed_schedule(
            0.0,
            self.scenario.lead_preset,
        )
        self.speed_controller.reset()
        self.behaviour_controller.reset()
        self.previous_acceleration = 0.0
        self.complete = False
        self.injector.reset(self.vehicle)
        self.history = {
            key: [] for key in (*self._BASE_KEYS, *self._ROBUST_KEYS)
        }

    def _traffic_output(
        self,
        *,
        measured_gap: float,
        measured_speed: float,
    ) -> BehaviourOutput:
        if self.scenario.enable_traffic:
            return self.behaviour_controller.update(
                gap=measured_gap,
                ego_speed=measured_speed,
                lead_speed=self.lead_speed,
                cruise_speed=self.scenario.speed_profile.global_speed_limit,
                enable_state_machine=self.scenario.enable_state_machine,
            )
        desired = (
            self.scenario.acc.standstill_gap
            + self.scenario.acc.time_headway * measured_speed
        )
        return BehaviourOutput(
            state=BehaviourState.CRUISE,
            desired_gap=desired,
            gap_error=math.inf,
            closing_speed=0.0,
            time_to_collision=math.inf,
            target_speed=self.scenario.speed_profile.global_speed_limit,
            acceleration_override=None,
        )

    def step(self) -> RobustSnapshot:
        if self.complete:
            raise StopIteration("simulation has reached its stop condition")

        s = self.scenario
        self.lead_speed = lead_speed_schedule(
            self.time,
            s.lead_preset,
        )
        if self.lead_distance >= self.path.length:
            self.lead_speed = 0.0
        self.lead_distance = min(
            self.path.length,
            self.lead_distance + self.lead_speed * s.dt,
        )

        measured = self.injector.observe_state(self.vehicle, self.time)
        measured_lateral = pure_pursuit(
            measured,
            self.path,
            vehicle=self.vehicle_parameters,
            base_lookahead=s.base_lookahead,
            speed_lookahead_gain=s.speed_lookahead_gain,
            previous_index=self.measured_previous_index,
        )
        self.measured_previous_index = measured_lateral.nearest_index
        true_lateral = pure_pursuit(
            self.vehicle,
            self.path,
            vehicle=self.vehicle_parameters,
            base_lookahead=s.base_lookahead,
            speed_lookahead_gain=s.speed_lookahead_gain,
            previous_index=self.true_previous_index,
        )
        self.true_previous_index = true_lateral.nearest_index
        true_distance = float(
            self.path.distance[true_lateral.nearest_index]
        )
        measured_distance = float(
            self.path.distance[measured_lateral.nearest_index]
        )
        road_target = (
            self.profile.speed_at(measured_distance)
            if s.enable_curve_speed
            else s.speed_profile.global_speed_limit
        )
        true_gap = (
            self.lead_distance
            - true_distance
            - s.lead_vehicle_length
            if s.enable_traffic
            else math.inf
        )
        measured_gap = self.injector.observe_gap(true_gap, self.time)
        traffic = self._traffic_output(
            measured_gap=measured_gap,
            measured_speed=measured.speed,
        )
        selected_target = min(road_target, traffic.target_speed)
        longitudinal = self.speed_controller.update(
            target_speed=selected_target,
            measured_speed=measured.speed,
            dt=s.dt,
            acceleration_override=traffic.acceleration_override,
        )
        requested_acceleration = longitudinal.applied_acceleration
        requested_steering = measured_lateral.steering
        applied_steering_command, applied_acceleration = (
            self.injector.actuator_commands(
                requested_steering,
                requested_acceleration,
                self.time,
            )
        )
        jerk = (
            applied_acceleration - self.previous_acceleration
        ) / s.dt
        self.previous_acceleration = applied_acceleration
        self.vehicle.speed = integrate_speed(
            self.vehicle.speed,
            applied_acceleration,
            s.dt,
        )
        motion = bicycle_step(
            self.vehicle,
            applied_steering_command,
            parameters=self.vehicle_parameters,
            dt=s.dt,
            speed=self.vehicle.speed,
            enable_rate_limit=True,
        )
        push_applied = self.injector.maybe_apply_lateral_push(
            self.vehicle,
            time=self.time,
            path_heading=float(
                self.path.heading[true_lateral.nearest_index]
            ),
        )
        lead_x = float(
            np.interp(self.lead_distance, self.path.distance, self.path.x)
        )
        lead_y = float(
            np.interp(self.lead_distance, self.path.distance, self.path.y)
        )
        lead_heading = float(
            np.interp(
                self.lead_distance,
                self.path.distance,
                self.path.heading,
            )
        )
        collision = s.enable_traffic and true_gap <= 0.0
        self.time += s.dt
        self.complete = (
            self.true_previous_index >= len(self.path.x) - 3
            and s.stop_when_complete
        ) or self.time >= s.duration
        snapshot = RobustSnapshot(
            time=self.time,
            true_vehicle=self.vehicle.copy(),
            measured_vehicle=measured,
            lead_distance=self.lead_distance,
            lead_speed=self.lead_speed,
            lead_x=lead_x,
            lead_y=lead_y,
            lead_heading=lead_heading,
            true_path_distance=true_distance,
            measured_path_distance=measured_distance,
            true_gap=true_gap,
            measured_gap=measured_gap,
            road_target_speed=road_target,
            selected_target_speed=selected_target,
            lateral=measured_lateral,
            true_cross_track_error=true_lateral.cross_track_error,
            traffic=traffic,
            requested_steering=requested_steering,
            applied_steering=motion.applied_steering,
            requested_acceleration=requested_acceleration,
            applied_acceleration=applied_acceleration,
            lateral_acceleration=motion.lateral_acceleration,
            steering_rate=motion.steering_rate,
            jerk=jerk,
            fault_active=self.injector.active(self.time),
            push_applied=push_applied,
            collision=collision,
            complete=self.complete,
        )
        self._record(snapshot, longitudinal)
        return snapshot

    def _record(self, snap: RobustSnapshot, longitudinal) -> None:
        true_closing_speed = snap.true_vehicle.speed - snap.lead_speed
        if snap.true_gap <= 0.0:
            true_ttc = 0.0
        elif true_closing_speed <= 1e-6:
            true_ttc = math.inf
        else:
            true_ttc = snap.true_gap / true_closing_speed
        values: dict[str, float | int | bool | str] = {
            "time": snap.time,
            "x": snap.true_vehicle.x,
            "y": snap.true_vehicle.y,
            "heading": snap.true_vehicle.heading,
            "speed": snap.true_vehicle.speed,
            "steering": snap.applied_steering,
            "steering_rate": snap.steering_rate,
            "acceleration": snap.applied_acceleration,
            "jerk": snap.jerk,
            "lateral_acceleration": snap.lateral_acceleration,
            "cross_track_error": snap.true_cross_track_error,
            "nearest_index": self.true_previous_index,
            "target_index": snap.lateral.target_index,
            "target_x": snap.lateral.target_x,
            "target_y": snap.lateral.target_y,
            "path_distance": snap.true_path_distance,
            "road_target_speed": snap.road_target_speed,
            "traffic_target_speed": snap.traffic.target_speed,
            "selected_target_speed": snap.selected_target_speed,
            "lead_distance": snap.lead_distance,
            "lead_speed": snap.lead_speed,
            "lead_x": snap.lead_x,
            "lead_y": snap.lead_y,
            "gap": snap.true_gap,
            "desired_gap": snap.traffic.desired_gap,
            "closing_speed": true_closing_speed,
            "time_to_collision": true_ttc,
            "behaviour_state": snap.traffic.state.value,
            "collision": snap.collision,
            "saturated": longitudinal.saturated,
            "jerk_limited": longitudinal.jerk_limited,
            "measured_x": snap.measured_vehicle.x,
            "measured_y": snap.measured_vehicle.y,
            "measured_heading": snap.measured_vehicle.heading,
            "measured_speed": snap.measured_vehicle.speed,
            "measured_gap": snap.measured_gap,
            "measured_cross_track_error": snap.lateral.cross_track_error,
            "requested_steering": snap.requested_steering,
            "requested_acceleration": snap.requested_acceleration,
            "fault_active": snap.fault_active,
            "push_applied": snap.push_applied,
        }
        for key, value in values.items():
            self.history[key].append(value)

    def result(self) -> RobustResult:
        if not self.history["time"]:
            raise ValueError("simulation has no samples")
        integer_keys = {"nearest_index", "target_index"}
        boolean_keys = {
            "collision",
            "saturated",
            "jerk_limited",
            "fault_active",
            "push_applied",
        }
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

        base_arguments = {
            item.name: arrays[item.name]
            for item in fields(IntegratedResult)
            if item.name in arrays
        }
        integrated = IntegratedResult(
            **base_arguments,
            path=self.path,
            profile=self.profile,
            scenario=self.scenario,
            vehicle_parameters=self.vehicle_parameters,
        )
        return RobustResult(
            integrated=integrated,
            measured_x=arrays["measured_x"],
            measured_y=arrays["measured_y"],
            measured_heading=arrays["measured_heading"],
            measured_speed=arrays["measured_speed"],
            measured_gap=arrays["measured_gap"],
            measured_cross_track_error=arrays[
                "measured_cross_track_error"
            ],
            requested_steering=arrays["requested_steering"],
            requested_acceleration=arrays["requested_acceleration"],
            fault_active=arrays["fault_active"],
            push_applied=arrays["push_applied"],
            robust_scenario=self.robust_scenario,
        )


def run_robust(
    scenario: RobustScenario | None = None,
    *,
    path: ReferencePath | None = None,
    vehicle_parameters: VehicleParameters | None = None,
) -> RobustResult:
    """Run a deterministic fault-injection experiment to completion."""

    simulation = RobustSimulation(
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
