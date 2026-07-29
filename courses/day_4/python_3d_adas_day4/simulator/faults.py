"""Repeatable Day 4 fault scenarios for the cumulative ADAS simulator."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FaultScenario:
    """One documented combination of sensor, actuator and vehicle faults."""

    name: str
    label: str
    radar_dropout_start_s: float | None = None
    radar_dropout_end_s: float | None = None
    braking_efficiency: float = 1.0
    lateral_push_time_s: float | None = None
    lateral_push_m: float = 0.0

    def radar_is_healthy(self, time_s: float) -> bool:
        if (
            self.radar_dropout_start_s is None
            or self.radar_dropout_end_s is None
        ):
            return True
        return not (
            self.radar_dropout_start_s
            <= time_s
            < self.radar_dropout_end_s
        )

    def radar_age(self, time_s: float) -> float:
        if self.radar_is_healthy(time_s):
            return 0.0
        assert self.radar_dropout_start_s is not None
        return max(0.0, time_s - self.radar_dropout_start_s)

    def active_fault(self, time_s: float) -> str:
        active: list[str] = []
        if not self.radar_is_healthy(time_s):
            active.append("RADAR DROPOUT")
        if self.braking_efficiency < 0.999:
            active.append("BRAKE FADE")
        if (
            self.lateral_push_time_s is not None
            and abs(time_s - self.lateral_push_time_s) < 0.35
        ):
            active.append("LATERAL PUSH")
        return " + ".join(active) if active else "NONE"


SCENARIOS: dict[str, FaultScenario] = {
    "nominal": FaultScenario(
        name="nominal",
        label="Nominal traffic",
    ),
    "radar_dropout": FaultScenario(
        name="radar_dropout",
        label="Radar dropout",
        radar_dropout_start_s=34.0,
        radar_dropout_end_s=43.0,
    ),
    "lateral_push": FaultScenario(
        name="lateral_push",
        label="Lateral disturbance",
        lateral_push_time_s=26.0,
        lateral_push_m=1.35,
    ),
    "brake_fade": FaultScenario(
        name="brake_fade",
        label="Reduced braking",
        braking_efficiency=0.58,
    ),
    "combined": FaultScenario(
        name="combined",
        label="Combined challenge",
        radar_dropout_start_s=34.0,
        radar_dropout_end_s=43.0,
        braking_efficiency=0.65,
        lateral_push_time_s=26.0,
        lateral_push_m=-1.10,
    ),
}


def get_scenario(
    scenario: str | FaultScenario | None,
) -> FaultScenario:
    if scenario is None:
        return SCENARIOS["nominal"]
    if isinstance(scenario, FaultScenario):
        return scenario
    try:
        return SCENARIOS[scenario]
    except KeyError as error:
        choices = ", ".join(SCENARIOS)
        raise ValueError(
            f"Unknown scenario {scenario!r}. Choose from: {choices}."
        ) from error


def scenario_names() -> tuple[str, ...]:
    return tuple(SCENARIOS)
