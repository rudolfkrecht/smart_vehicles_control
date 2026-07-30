"""Regression tests for the lightweight Day 1 simulator."""

from __future__ import annotations

from pathlib import Path
import sys

COURSES = Path(__file__).resolve().parents[1] / "courses"
if str(COURSES) not in sys.path:
    sys.path.insert(0, str(COURSES))

import unittest

import numpy as np

from simulator import (
    OpenLoopController,
    PController,
    PIController,
    Scenario,
    calculate_metrics,
    run_simulation,
)


class DayOneSimulationTests(unittest.TestCase):
    def test_speed_remains_non_negative(self) -> None:
        result = run_simulation(
            OpenLoopController(-1.0),
            scenario=Scenario(duration=5.0),
        )
        self.assertTrue(np.all(result.speed >= 0.0))

    def test_command_is_saturated(self) -> None:
        result = run_simulation(
            PController(100.0),
            scenario=Scenario(duration=2.0),
        )
        self.assertTrue(np.all(np.abs(result.command) <= 1.0))

    def test_feedback_moves_speed_toward_target(self) -> None:
        result = run_simulation(
            PController(0.35),
            scenario=Scenario(duration=15.0, target_speed=15.0),
        )
        self.assertGreater(result.speed[-1], 12.0)
        self.assertLess(result.speed[-1], 16.0)

    def test_pi_reduces_persistent_hill_error(self) -> None:
        scenario = Scenario(
            duration=40.0,
            target_speed=15.0,
            hill_start=15.0,
            hill_force=1_000.0,
        )
        p_result = run_simulation(PController(0.35), scenario=scenario)
        pi_result = run_simulation(
            PIController(0.35, 0.10),
            scenario=scenario,
        )
        p_error = abs(calculate_metrics(p_result).final_error_mps)
        pi_error = abs(calculate_metrics(pi_result).final_error_mps)
        self.assertLess(pi_error, 0.3)
        self.assertLess(pi_error, p_error)

    def test_anti_windup_reduces_post_hill_overshoot(self) -> None:
        scenario = Scenario(
            duration=40.0,
            target_speed=15.0,
            initial_speed=15.0,
            hill_start=10.0,
            hill_end=24.0,
            hill_force=5_000.0,
        )
        no_aw = run_simulation(
            PIController(0.30, 0.14, anti_windup=False),
            scenario=scenario,
        )
        with_aw = run_simulation(
            PIController(0.30, 0.14, anti_windup=True),
            scenario=scenario,
        )
        after_hill = no_aw.time >= 24.0
        overshoot_no_aw = np.max(no_aw.speed[after_hill]) - 15.0
        overshoot_with_aw = np.max(with_aw.speed[after_hill]) - 15.0
        self.assertLess(overshoot_with_aw, overshoot_no_aw)


if __name__ == "__main__":
    unittest.main()
