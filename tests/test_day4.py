"""Regression tests for the complete Day 4 package."""

from __future__ import annotations

from pathlib import Path
import sys

COURSES = Path(__file__).resolve().parents[1] / "courses"
if str(COURSES) not in sys.path:
    sys.path.insert(0, str(COURSES))

from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simulator.bicycle import VehicleState
from simulator.challenge import (
    AGGRESSIVE_CONFIGURATION,
    BALANCED_CONFIGURATION,
    ControllerConfiguration,
    TestCase,
    evaluation_cases,
    export_batch_csv,
    initial_condition_cases,
    practice_cases,
    robust_scenario_from_case,
    run_batch,
)
from simulator.evaluation import (
    calculate_robust_metrics,
    score_challenge_run,
)
from simulator.faults import FaultInjector, FaultParameters
from simulator.paths import make_reference_path
from simulator.robustness import RobustSimulation, run_robust


class PathAndConfigurationTests(unittest.TestCase):
    def test_three_day4_paths_exist(self) -> None:
        paths = [
            make_reference_path(name)
            for name in ("practice", "evaluation_a", "evaluation_b")
        ]
        self.assertTrue(all(path.length > 200.0 for path in paths))
        self.assertEqual(len({path.name for path in paths}), 3)

    def test_evaluation_path_is_not_practice_path(self) -> None:
        practice = make_reference_path("practice")
        evaluation = make_reference_path("evaluation_a")
        self.assertFalse(
            np.allclose(
                practice.y[: min(len(practice.y), len(evaluation.y))],
                evaluation.y[: min(len(practice.y), len(evaluation.y))],
            )
        )

    def test_invalid_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ControllerConfiguration(time_headway=0.0)

    def test_published_suites_have_expected_size(self) -> None:
        self.assertEqual(len(practice_cases()), 6)
        self.assertEqual(len(evaluation_cases()), 5)
        self.assertEqual(len(initial_condition_cases()), 5)


class FaultInjectorTests(unittest.TestCase):
    def test_nominal_observation_is_exact(self) -> None:
        state = VehicleState(x=2.0, y=-1.0, speed=7.0)
        injector = FaultInjector(FaultParameters(), dt=0.05)
        injector.reset(state)
        measured = injector.observe_state(state, 1.0)
        self.assertEqual(measured, state)
        self.assertIsNot(measured, state)

    def test_seeded_noise_is_repeatable(self) -> None:
        parameters = FaultParameters(
            position_noise_std=0.2,
            speed_noise_std=0.1,
            random_seed=99,
        )
        state = VehicleState(speed=8.0)
        injectors = [FaultInjector(parameters, dt=0.05) for _ in range(2)]
        values = []
        for injector in injectors:
            injector.reset(state)
            values.append(injector.observe_state(state, 1.0))
        self.assertAlmostEqual(values[0].x, values[1].x)
        self.assertAlmostEqual(values[0].speed, values[1].speed)

    def test_weak_braking_reduces_deceleration(self) -> None:
        parameters = FaultParameters(braking_efficiency=0.6)
        injector = FaultInjector(parameters, dt=0.05)
        injector.reset(VehicleState())
        _, acceleration = injector.actuator_commands(0.0, -5.0, 1.0)
        self.assertAlmostEqual(acceleration, -3.0)

    def test_steering_bias_is_added(self) -> None:
        parameters = FaultParameters(steering_bias_degrees=2.0)
        injector = FaultInjector(parameters, dt=0.05)
        injector.reset(VehicleState())
        steering, _ = injector.actuator_commands(0.0, 0.0, 1.0)
        self.assertAlmostEqual(math.degrees(steering), 2.0)

    def test_lateral_push_occurs_once(self) -> None:
        parameters = FaultParameters(
            lateral_push_time=1.0,
            lateral_push_m=1.5,
        )
        injector = FaultInjector(parameters, dt=0.05)
        state = VehicleState()
        injector.reset(state)
        self.assertTrue(
            injector.maybe_apply_lateral_push(
                state,
                time=1.0,
                path_heading=0.0,
            )
        )
        self.assertAlmostEqual(state.y, 1.5)
        self.assertFalse(
            injector.maybe_apply_lateral_push(
                state,
                time=2.0,
                path_heading=0.0,
            )
        )


class RobustSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.nominal = run_robust(
            robust_scenario_from_case(
                BALANCED_CONFIGURATION,
                practice_cases()[0],
            )
        )
        cls.noisy = run_robust(
            robust_scenario_from_case(
                BALANCED_CONFIGURATION,
                practice_cases()[2],
            )
        )

    def test_incremental_and_batch_runs_match(self) -> None:
        scenario = robust_scenario_from_case(
            BALANCED_CONFIGURATION,
            practice_cases()[0],
        )
        simulation = RobustSimulation(scenario)
        while not simulation.complete:
            simulation.step()
        incremental = simulation.result()
        self.assertTrue(
            np.allclose(incremental.integrated.x, self.nominal.integrated.x)
        )

    def test_nominal_run_passes(self) -> None:
        metric = calculate_robust_metrics(self.nominal)
        self.assertTrue(metric.pass_run)
        self.assertEqual(metric.integrated.collision_samples, 0)

    def test_noisy_measurement_differs_from_truth(self) -> None:
        metric = calculate_robust_metrics(self.noisy)
        self.assertGreater(metric.rms_position_measurement_error_m, 0.05)
        self.assertGreater(metric.rms_speed_measurement_error_mps, 0.05)

    def test_lateral_push_is_applied_once(self) -> None:
        result = run_robust(
            robust_scenario_from_case(
                BALANCED_CONFIGURATION,
                practice_cases()[1],
            )
        )
        self.assertEqual(int(np.count_nonzero(result.push_applied)), 1)
        self.assertTrue(calculate_robust_metrics(result).pass_run)

    def test_same_seed_produces_same_history(self) -> None:
        case = practice_cases()[2]
        first = run_robust(
            robust_scenario_from_case(BALANCED_CONFIGURATION, case)
        )
        second = run_robust(
            robust_scenario_from_case(BALANCED_CONFIGURATION, case)
        )
        self.assertTrue(np.array_equal(first.measured_x, second.measured_x))


class BatchAndScoringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.balanced_practice = run_batch(
            BALANCED_CONFIGURATION,
            practice_cases(),
        )
        cls.balanced_evaluation = run_batch(
            BALANCED_CONFIGURATION,
            evaluation_cases(),
        )
        cls.aggressive_practice = run_batch(
            AGGRESSIVE_CONFIGURATION,
            practice_cases(),
        )

    def test_balanced_controller_passes_practice_suite(self) -> None:
        self.assertEqual(self.balanced_practice.pass_rate_percent, 100.0)

    def test_balanced_controller_passes_evaluation_suite(self) -> None:
        self.assertEqual(self.balanced_evaluation.pass_rate_percent, 100.0)

    def test_aggressive_controller_fails_safety_tests(self) -> None:
        self.assertLess(self.aggressive_practice.pass_rate_percent, 50.0)
        self.assertTrue(
            any(
                item.metrics.safety_critical_failure
                for item in self.aggressive_practice.items
            )
        )

    def test_batch_identifies_a_worst_case(self) -> None:
        worst = self.aggressive_practice.worst_item
        self.assertFalse(worst.metrics.pass_run)
        self.assertNotEqual(worst.metrics.failure_reason, "none")

    def test_csv_export_has_one_row_per_case(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = export_batch_csv(
                self.balanced_practice,
                Path(folder) / "results.csv",
            )
            lines = output.read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1 + len(practice_cases()))

    def test_reference_score_is_above_seventy(self) -> None:
        report = self.balanced_evaluation
        score = score_challenge_run(
            report.worst_item.result,
            robustness_pass_rate=report.pass_rate_percent,
            technical_explanation=85.0,
        )
        self.assertGreater(score.final_score, 70.0)
        self.assertFalse(score.safety_cap_applied)

    def test_unsafe_high_category_run_is_capped(self) -> None:
        aggressive = run_robust(
            robust_scenario_from_case(
                AGGRESSIVE_CONFIGURATION,
                evaluation_cases()[0],
            )
        )
        score = score_challenge_run(
            aggressive,
            robustness_pass_rate=100.0,
            technical_explanation=100.0,
        )
        self.assertLessEqual(score.final_score, 50.0)


if __name__ == "__main__":
    unittest.main()
