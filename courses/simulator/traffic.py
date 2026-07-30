"""Adaptive Cruise Control and discrete behaviour logic for Day 4."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class BehaviourState(str, Enum):
    CRUISE = "CRUISE"
    FOLLOW = "FOLLOW"
    BRAKE = "BRAKE"
    EMERGENCY = "EMERGENCY"


@dataclass(frozen=True)
class ACCParameters:
    standstill_gap: float = 5.0
    time_headway: float = 1.5
    distance_gain: float = 0.35
    relative_speed_gain: float = 0.55
    follow_entry_ratio: float = 1.35
    brake_entry_ratio: float = 0.78
    emergency_gap: float = 3.0
    emergency_ttc: float = 1.25
    comfortable_braking: float = 2.5
    emergency_braking: float = 5.5
    hysteresis_distance: float = 2.0

    def __post_init__(self) -> None:
        if self.standstill_gap <= 0.0:
            raise ValueError("standstill_gap must be positive")
        if self.time_headway <= 0.0:
            raise ValueError("time_headway must be positive")
        if self.distance_gain < 0.0 or self.relative_speed_gain < 0.0:
            raise ValueError("ACC gains cannot be negative")
        if not 0.0 < self.brake_entry_ratio < self.follow_entry_ratio:
            raise ValueError("behaviour ratios are inconsistent")
        if self.emergency_gap <= 0.0 or self.emergency_ttc <= 0.0:
            raise ValueError("emergency thresholds must be positive")


@dataclass(frozen=True)
class BehaviourOutput:
    state: BehaviourState
    desired_gap: float
    gap_error: float
    closing_speed: float
    time_to_collision: float
    target_speed: float
    acceleration_override: float | None


def desired_following_gap(
    speed: float,
    parameters: ACCParameters | None = None,
) -> float:
    parameters = parameters or ACCParameters()
    return parameters.standstill_gap + parameters.time_headway * max(speed, 0.0)


def time_to_collision(gap: float, closing_speed: float) -> float:
    if gap <= 0.0:
        return 0.0
    if closing_speed <= 1e-6:
        return math.inf
    return gap / closing_speed


def lead_speed_schedule(time: float, preset: str = "stop_and_go") -> float:
    """Prepared lead-vehicle schedules used by demos and the GUI."""

    if preset == "steady":
        return 8.0
    if preset == "slow":
        return 6.0
    if preset == "late_brake":
        if time < 15.0:
            return 9.0
        if time < 17.0:
            return 9.0 * (17.0 - time) / 2.0
        return 0.0
    if preset == "evaluation":
        if time < 6.0:
            return 9.5
        if time < 11.0:
            return 9.5 - 0.7 * (time - 6.0)
        if time < 16.0:
            return 6.0
        if time < 18.5:
            return 6.0 * (18.5 - time) / 2.5
        if time < 23.0:
            return 0.0
        if time < 28.0:
            return 1.6 * (time - 23.0)
        return 8.0
    if preset != "stop_and_go":
        raise ValueError(
            "lead preset must be steady, slow, late_brake, evaluation "
            "or stop_and_go"
        )
    if time < 8.0:
        return 10.0
    if time < 12.0:
        return 10.0 - (time - 8.0)
    if time < 18.0:
        return 6.0
    if time < 21.0:
        return 6.0 * (21.0 - time) / 3.0
    if time < 25.0:
        return 0.0
    if time < 29.0:
        return 2.0 * (time - 25.0)
    return 8.0


class BehaviourController:
    """State machine with mild hysteresis around Cruise and Follow."""

    def __init__(self, parameters: ACCParameters | None = None) -> None:
        self.parameters = parameters or ACCParameters()
        self.state = BehaviourState.CRUISE

    def reset(self) -> None:
        self.state = BehaviourState.CRUISE

    def update(
        self,
        *,
        gap: float,
        ego_speed: float,
        lead_speed: float,
        cruise_speed: float,
        enable_state_machine: bool = True,
    ) -> BehaviourOutput:
        p = self.parameters
        desired = desired_following_gap(ego_speed, p)
        closing = ego_speed - lead_speed
        ttc = time_to_collision(gap, closing)

        if not enable_state_machine:
            state = (
                BehaviourState.FOLLOW
                if gap < p.follow_entry_ratio * desired
                else BehaviourState.CRUISE
            )
        elif gap <= p.emergency_gap or ttc <= p.emergency_ttc:
            state = BehaviourState.EMERGENCY
        elif gap < p.brake_entry_ratio * desired or (
            closing > 2.0 and ttc < 3.5
        ):
            state = BehaviourState.BRAKE
        else:
            follow_boundary = p.follow_entry_ratio * desired
            if self.state in (BehaviourState.FOLLOW, BehaviourState.BRAKE):
                follow_boundary += p.hysteresis_distance
            state = (
                BehaviourState.FOLLOW
                if gap < follow_boundary
                else BehaviourState.CRUISE
            )

        gap_error = gap - desired
        acc_speed = (
            lead_speed
            + p.distance_gain * gap_error
            - p.relative_speed_gain * max(closing, 0.0)
        )
        target = float(np.clip(acc_speed, 0.0, cruise_speed))
        override: float | None = None
        if state is BehaviourState.CRUISE:
            target = cruise_speed
        elif state is BehaviourState.BRAKE:
            target = min(target, lead_speed)
            override = -p.comfortable_braking
        elif state is BehaviourState.EMERGENCY:
            target = 0.0
            override = -p.emergency_braking

        self.state = state
        return BehaviourOutput(
            state=state,
            desired_gap=desired,
            gap_error=gap_error,
            closing_speed=closing,
            time_to_collision=ttc,
            target_speed=target,
            acceleration_override=override,
        )
