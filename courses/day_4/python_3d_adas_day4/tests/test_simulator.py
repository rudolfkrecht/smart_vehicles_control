from __future__ import annotations

import csv
import math
from pathlib import Path
import tempfile
import unittest

from evaluate_project import assess, run_suite, save_results
from simulator.controllers import ReferenceController, desired_following_gap
from simulator.faults import SCENARIOS, get_scenario
from simulator.model import ControlCommand, VehicleModel
from simulator.renderer3d import SceneBuilder
from simulator.simulation import Simulation
from simulator.track import ClosedHighwayTrack
from student_controller import StudentController as StarterController
from student_controller_solution import StudentController as SolutionController


class TrackAndVehicleTests(unittest.TestCase):
    def test_track_is_closed_and_contains_five_degree_hill(self) -> None:
        track = ClosedHighwayTrack()
        start = track.pose_at(0.0)
        finish = track.pose_at(track.total_length)
        self.assertAlmostEqual(start.x, finish.x, places=8)
        self.assertAlmostEqual(start.y, finish.y, places=8)
        _, uphill = track.elevation_at(50.0)
        _, downhill = track.elevation_at(170.0)
        self.assertAlmostEqual(math.degrees(uphill), 5.0, places=8)
        self.assertAlmostEqual(math.degrees(downhill), -5.0, places=8)

    def test_reduced_braking_authority_changes_response(self) -> None:
        command = ControlCommand(brake=0.7)
        normal = VehicleModel()
        faded = VehicleModel()
        normal.reset(0.0, 0.0, 0.0, 0.0)
        faded.reset(0.0, 0.0, 0.0, 0.0)
        normal.state.speed = 14.0
        faded.state.speed = 14.0
        for _ in range(60):
            normal.step(command, 0.0, 1.0 / 60.0)
            faded.step(
                command,
                0.0,
                1.0 / 60.0,
                braking_efficiency=0.55,
            )
        self.assertGreater(faded.state.speed, normal.state.speed)


class FaultAndSupervisorTests(unittest.TestCase):
    def test_constant_time_headway_policy_is_retained(self) -> None:
        self.assertAlmostEqual(
            desired_following_gap(10.0, 5.0, 1.5),
            20.0,
        )

    def test_radar_dropout_is_repeatable(self) -> None:
        scenario = get_scenario("radar_dropout")
        self.assertTrue(scenario.radar_is_healthy(33.9))
        self.assertFalse(scenario.radar_is_healthy(34.0))
        self.assertFalse(scenario.radar_is_healthy(42.9))
        self.assertTrue(scenario.radar_is_healthy(43.0))

    def test_observation_exposes_sensor_health_and_age(self) -> None:
        simulation = Simulation(
            ReferenceController(),
            scenario="radar_dropout",
        )
        simulation.time = 38.0
        observation = simulation.observe(0.1)
        self.assertFalse(observation.range_sensor_healthy)
        self.assertFalse(observation.lead_detected)
        self.assertAlmostEqual(observation.range_measurement_age, 4.0)
        self.assertIn("RADAR", observation.active_fault)

    def test_lateral_disturbance_is_applied_once(self) -> None:
        simulation = Simulation(
            ReferenceController(),
            scenario="lateral_push",
        )
        simulation.run(27.0)
        push_samples = sum(
            bool(row["lateral_push_applied"])
            for row in simulation.history
        )
        self.assertEqual(push_samples, 1)
        self.assertTrue(simulation.lateral_push_applied)

    def test_starter_fails_sensor_dropout(self) -> None:
        simulation = Simulation(
            StarterController(),
            target_speed=14.0,
            scenario="radar_dropout",
        )
        simulation.run(105.0)
        metrics = simulation.metrics()
        self.assertGreater(int(metrics["collision_samples"]), 0)
        self.assertEqual(int(metrics["safe_stop_samples"]), 0)


class CumulativePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.solution_rows = run_suite(
            SolutionController,
            duration=105.0,
            target_speed=14.0,
        )

    def test_worked_solution_passes_all_scenarios(self) -> None:
        self.assertEqual(len(self.solution_rows), len(SCENARIOS))
        self.assertTrue(all(bool(row["passed"]) for row in self.solution_rows))

    def test_dropout_activates_safe_stop_without_collision(self) -> None:
        row = next(
            item
            for item in self.solution_rows
            if item["scenario"] == "radar_dropout"
        )
        self.assertEqual(int(row["collision_samples"]), 0)
        self.assertGreater(int(row["safe_stop_samples"]), 0)
        self.assertGreater(float(row["minimum_gap_m"]), 3.0)

    def test_lateral_push_is_recovered_without_departure(self) -> None:
        row = next(
            item
            for item in self.solution_rows
            if item["scenario"] == "lateral_push"
        )
        self.assertEqual(float(row["outside_road_percent"]), 0.0)
        self.assertGreater(float(row["maximum_cross_track_error_m"]), 1.0)
        self.assertLess(float(row["maximum_cross_track_error_m"]), 1.6)

    def test_commands_remain_mutually_exclusive(self) -> None:
        simulation = Simulation(
            ReferenceController(),
            scenario="combined",
        )
        simulation.run(50.0)
        for row in simulation.history:
            self.assertFalse(
                float(row["throttle"]) > 0.0
                and float(row["brake"]) > 0.0
            )

    def test_csv_contains_day4_evidence_columns(self) -> None:
        simulation = Simulation(
            ReferenceController(),
            scenario="combined",
        )
        simulation.run(1.0)
        with tempfile.TemporaryDirectory() as directory:
            output = simulation.save_csv(Path(directory) / "run.csv")
            with output.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 60)
        for name in (
            "range_sensor_healthy",
            "range_measurement_age_s",
            "active_fault",
            "braking_efficiency",
            "applied_brake",
            "supervisor_active",
            "scenario",
        ):
            self.assertIn(name, rows[0])

    def test_suite_result_can_be_exported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = save_results(
                self.solution_rows,
                Path(directory) / "suite.csv",
            )
            with output.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), len(SCENARIOS))
        self.assertTrue(all(row["passed"] == "True" for row in rows))

    def test_assessment_rejects_collision(self) -> None:
        metrics = dict(self.solution_rows[0])
        metrics["collision_samples"] = 1
        passed, reason = assess("nominal", metrics)
        self.assertFalse(passed)
        self.assertIn("collision", reason)

    def test_renderer_contains_ego_and_lead_vehicle_colours(self) -> None:
        simulation = Simulation(ReferenceController())
        simulation.run(2.0)
        observation = simulation.last_observation
        assert observation is not None
        lead_pose = simulation.track.pose_at(
            simulation.lead_progress,
            simulation.track.lane_offset,
        )
        primitives = SceneBuilder(1200, 700).build(
            simulation.track,
            simulation.vehicle.state,
            observation.slope_radians,
            simulation.lead_vehicle,
            lead_pose.slope,
        )
        fills = {item.fill for item in primitives}
        self.assertIn("#2f80ed", fills)
        self.assertIn("#ef5350", fills)


if __name__ == "__main__":
    unittest.main()
