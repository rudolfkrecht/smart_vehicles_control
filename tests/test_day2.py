"""Regression tests for the complete Day 2 package."""

from __future__ import annotations

from pathlib import Path
import sys

COURSES = Path(__file__).resolve().parents[1] / "courses"
if str(COURSES) not in sys.path:
    sys.path.insert(0, str(COURSES))

import math
from pathlib import Path
import sys
import unittest

import numpy as np


from simulator.bicycle import (
    VehicleParameters,
    VehicleState,
    bicycle_step,
    simulate_constant_steering,
    turning_radius,
)
from simulator.metrics import calculate_path_metrics
from simulator.paths import make_reference_path, offset_from_path
from simulator.tracking import (
    PathFollowingScenario,
    pure_pursuit,
    run_path_following,
    tracking_errors,
)


class BicycleModelTests(unittest.TestCase):
    def test_straight_motion_keeps_heading_and_y(self) -> None:
        result = simulate_constant_steering(
            steering_degrees=0.0,
            speed=5.0,
            duration=2.0,
        )
        self.assertAlmostEqual(float(result["y"][-1]), 0.0, places=9)
        self.assertAlmostEqual(float(result["heading"][-1]), 0.0, places=9)
        self.assertGreater(float(result["x"][-1]), 9.5)

    def test_turning_radius_matches_bicycle_geometry(self) -> None:
        radius = turning_radius(2.7, math.radians(12.0))
        self.assertAlmostEqual(radius, 12.70, places=2)

    def test_steering_angle_and_rate_are_limited(self) -> None:
        parameters = VehicleParameters(
            maximum_steering=math.radians(30.0),
            maximum_steering_rate=math.radians(10.0),
        )
        state = VehicleState(speed=8.0)
        sample = bicycle_step(
            state,
            math.radians(80.0),
            parameters=parameters,
            dt=0.1,
        )
        self.assertAlmostEqual(
            math.degrees(sample.applied_steering),
            1.0,
            places=6,
        )
        for _ in range(100):
            bicycle_step(
                state,
                math.radians(80.0),
                parameters=parameters,
                dt=0.1,
            )
        self.assertLessEqual(
            abs(state.steering),
            parameters.maximum_steering + 1e-12,
        )


class PathGeometryTests(unittest.TestCase):
    def test_path_distance_is_strictly_increasing(self) -> None:
        for kind in ("gentle", "training", "tight"):
            path = make_reference_path(kind)
            self.assertTrue(np.all(np.diff(path.distance) > 0.0))
            self.assertEqual(len(path.x), len(path.curvature))

    def test_signed_error_matches_imposed_offset(self) -> None:
        path = make_reference_path("training")
        x, y, heading = offset_from_path(
            path,
            distance=35.0,
            lateral_offset=2.0,
        )
        state = VehicleState(x=x, y=y, heading=heading)
        errors = tracking_errors(state, path, previous_index=0)
        self.assertAlmostEqual(errors.cross_track_error, 2.0, delta=0.05)
        self.assertAlmostEqual(errors.heading_error, 0.0, delta=0.02)

    def test_pure_pursuit_target_is_ahead(self) -> None:
        path = make_reference_path("training")
        state = VehicleState(
            x=float(path.x[0]),
            y=float(path.y[0]),
            heading=float(path.heading[0]),
            speed=8.0,
        )
        output = pure_pursuit(
            state,
            path,
            vehicle=VehicleParameters(),
            base_lookahead=5.0,
        )
        self.assertGreater(output.target_index, output.nearest_index)
        self.assertAlmostEqual(output.lookahead_distance, 5.0)


class ControllerBehaviourTests(unittest.TestCase):
    def test_balanced_controller_completes_without_departure(self) -> None:
        result = run_path_following(
            scenario=PathFollowingScenario(
                speed=8.0,
                base_lookahead=5.0,
                speed_lookahead_gain=0.0,
            )
        )
        metrics = calculate_path_metrics(result)
        self.assertGreater(metrics.completion_percent, 95.0)
        self.assertEqual(metrics.road_departure_percent, 0.0)
        self.assertLess(metrics.mean_absolute_error_m, 0.30)

    def test_short_lookahead_is_more_aggressive(self) -> None:
        short = run_path_following(
            scenario=PathFollowingScenario(
                speed=9.0,
                base_lookahead=2.0,
                speed_lookahead_gain=0.0,
            )
        )
        balanced = run_path_following(
            scenario=PathFollowingScenario(
                speed=9.0,
                base_lookahead=5.0,
                speed_lookahead_gain=0.0,
            )
        )
        short_metrics = calculate_path_metrics(short)
        balanced_metrics = calculate_path_metrics(balanced)
        self.assertGreater(
            short_metrics.rms_steering_rate_degrees_s,
            2.0 * balanced_metrics.rms_steering_rate_degrees_s,
        )
        self.assertGreater(
            short_metrics.road_departure_percent,
            balanced_metrics.road_departure_percent,
        )

    def test_adaptive_rule_handles_low_and_high_speed(self) -> None:
        for speed in (6.0, 14.0):
            result = run_path_following(
                scenario=PathFollowingScenario(
                    speed=speed,
                    duration=22.0,
                    base_lookahead=2.2,
                    speed_lookahead_gain=0.32,
                    initial_lateral_offset=0.7,
                    initial_heading_offset_degrees=0.0,
                    disturbance_time=5.0,
                    disturbance_offset=1.8,
                )
            )
            metrics = calculate_path_metrics(result)
            self.assertEqual(metrics.road_departure_percent, 0.0)
            self.assertGreater(metrics.completion_percent, 90.0)
            self.assertLess(metrics.mean_absolute_error_m, 0.45)

    def test_simulation_is_repeatable_with_noise(self) -> None:
        scenario = PathFollowingScenario(
            duration=5.0,
            measurement_noise_std=0.1,
            random_seed=42,
        )
        first = run_path_following(scenario=scenario)
        second = run_path_following(scenario=scenario)
        np.testing.assert_allclose(first.x, second.x)
        np.testing.assert_allclose(first.y, second.y)


if __name__ == "__main__":
    unittest.main()
