"""Regression tests for the complete Day 3 package."""

from __future__ import annotations

from pathlib import Path
import sys

COURSES = Path(__file__).resolve().parents[1] / "courses"
if str(COURSES) not in sys.path:
    sys.path.insert(0, str(COURSES))

from dataclasses import replace
import math
import unittest

import numpy as np

from simulator.integrated import (
    IntegratedScenario,
    IntegratedSimulation,
    run_integrated,
)
from simulator.longitudinal import (
    LongitudinalParameters,
    SpeedController,
)
from simulator.metrics import calculate_integrated_metrics
from simulator.paths import make_reference_path
from simulator.speed_profile import (
    SpeedProfileParameters,
    build_speed_profile,
    safe_cornering_speed,
)
from simulator.traffic import (
    ACCParameters,
    BehaviourController,
    BehaviourState,
    desired_following_gap,
    time_to_collision,
)


class GeometryAndProfileTests(unittest.TestCase):
    def test_integrated_path_arrays_match(self) -> None:
        path = make_reference_path("integrated")
        self.assertGreater(path.length, 200.0)
        self.assertEqual(
            len({len(path.x), len(path.y), len(path.curvature)}),
            1,
        )

    def test_safe_speed_decreases_with_curvature(self) -> None:
        gentle = safe_cornering_speed(0.01, 2.5, speed_limit=30.0)
        sharp = safe_cornering_speed(0.10, 2.5, speed_limit=30.0)
        self.assertGreater(gentle, sharp)
        self.assertAlmostEqual(sharp, 5.0, places=6)

    def test_profile_respects_global_limit(self) -> None:
        profile = build_speed_profile(
            make_reference_path("integrated"),
            SpeedProfileParameters(global_speed_limit=13.0),
        )
        self.assertTrue(np.all(profile.planned_speed <= 13.0 + 1e-9))
        self.assertTrue(np.all(profile.planned_speed >= 0.0))

    def test_lower_lateral_limit_reduces_profile(self) -> None:
        path = make_reference_path("integrated")
        low = build_speed_profile(
            path,
            SpeedProfileParameters(maximum_lateral_acceleration=1.5),
        )
        high = build_speed_profile(
            path,
            SpeedProfileParameters(maximum_lateral_acceleration=4.0),
        )
        self.assertLess(
            float(np.mean(low.planned_speed)),
            float(np.mean(high.planned_speed)),
        )

    def test_preview_moves_braking_earlier(self) -> None:
        path = make_reference_path("integrated")
        no_preview = build_speed_profile(
            path,
            SpeedProfileParameters(preview_distance=0.0),
        )
        preview = build_speed_profile(
            path,
            SpeedProfileParameters(preview_distance=18.0),
        )
        self.assertTrue(
            np.any(
                preview.planned_speed
                < no_preview.planned_speed - 0.05
            )
        )


class ControllerTests(unittest.TestCase):
    def test_desired_gap_rule(self) -> None:
        parameters = ACCParameters(
            standstill_gap=5.0,
            time_headway=1.5,
        )
        self.assertAlmostEqual(
            desired_following_gap(10.0, parameters),
            20.0,
        )

    def test_ttc_is_infinite_without_closing(self) -> None:
        self.assertTrue(math.isinf(time_to_collision(20.0, -1.0)))
        self.assertAlmostEqual(time_to_collision(20.0, 5.0), 4.0)

    def test_behaviour_detects_emergency(self) -> None:
        controller = BehaviourController()
        output = controller.update(
            gap=2.0,
            ego_speed=12.0,
            lead_speed=0.0,
            cruise_speed=15.0,
        )
        self.assertEqual(output.state, BehaviourState.EMERGENCY)
        self.assertLess(output.acceleration_override or 0.0, -4.0)

    def test_speed_controller_applies_jerk_limit(self) -> None:
        controller = SpeedController(
            LongitudinalParameters(maximum_jerk=2.0)
        )
        output = controller.update(
            target_speed=20.0,
            measured_speed=0.0,
            dt=0.1,
        )
        self.assertAlmostEqual(output.applied_acceleration, 0.2)
        self.assertTrue(output.jerk_limited)


class IntegratedScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.reference = run_integrated(IntegratedScenario())
        cls.reference_metrics = calculate_integrated_metrics(cls.reference)

    def test_reference_scenario_is_safe(self) -> None:
        metric = self.reference_metrics
        self.assertEqual(metric.collision_samples, 0)
        self.assertEqual(metric.road_departure_percent, 0.0)
        self.assertGreater(metric.minimum_gap_m, 3.0)

    def test_reference_completes_task(self) -> None:
        self.assertGreaterEqual(
            self.reference_metrics.completion_percent,
            95.0,
        )
        self.assertLess(
            self.reference_metrics.peak_lateral_acceleration_mps2,
            3.0,
        )

    def test_short_headway_is_unsafe(self) -> None:
        base = IntegratedScenario(enable_state_machine=False)
        scenario = replace(
            base,
            acc=replace(base.acc, time_headway=0.7),
        )
        metric = calculate_integrated_metrics(run_integrated(scenario))
        self.assertTrue(
            metric.collision_samples > 0 or metric.minimum_gap_m < 1.0
        )

    def test_curve_speed_reduces_lateral_acceleration(self) -> None:
        base = IntegratedScenario(
            duration=25.0,
            enable_traffic=False,
        )
        curve = calculate_integrated_metrics(run_integrated(base))
        constant = calculate_integrated_metrics(
            run_integrated(replace(base, enable_curve_speed=False))
        )
        self.assertLess(
            curve.peak_lateral_acceleration_mps2,
            0.5 * constant.peak_lateral_acceleration_mps2,
        )

    def test_reference_visits_traffic_states(self) -> None:
        states = set(self.reference.behaviour_state)
        self.assertIn("CRUISE", states)
        self.assertIn("FOLLOW", states)
        self.assertIn("BRAKE", states)

    def test_incremental_and_batch_results_match(self) -> None:
        scenario = IntegratedScenario(duration=4.0)
        batch = run_integrated(scenario)
        simulation = IntegratedSimulation(scenario)
        while not simulation.complete:
            simulation.step()
        incremental = simulation.result()
        np.testing.assert_allclose(batch.x, incremental.x)
        np.testing.assert_allclose(batch.speed, incremental.speed)
        np.testing.assert_array_equal(
            batch.behaviour_state,
            incremental.behaviour_state,
        )

    def test_all_output_arrays_are_finite_where_expected(self) -> None:
        for name in (
            "x",
            "y",
            "speed",
            "steering",
            "acceleration",
            "cross_track_error",
            "gap",
        ):
            values = getattr(self.reference, name)
            self.assertTrue(np.all(np.isfinite(values)), name)


if __name__ == "__main__":
    unittest.main()
