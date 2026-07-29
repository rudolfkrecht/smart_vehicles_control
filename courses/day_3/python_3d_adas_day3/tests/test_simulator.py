from __future__ import annotations

import csv
import math
from pathlib import Path
import tempfile
import unittest

from simulator.controllers import (
    ReferenceController,
    desired_following_gap,
)
from simulator.model import ControlCommand, VehicleModel
from simulator.renderer3d import SceneBuilder
from simulator.simulation import Simulation, lead_speed_schedule
from simulator.track import ClosedHighwayTrack
from student_controller import StudentController as StarterController
from student_controller_solution import StudentController as SolutionController


class TrackAndVehicleTests(unittest.TestCase):
    def test_track_is_closed(self) -> None:
        track = ClosedHighwayTrack()
        start = track.pose_at(0.0)
        finish = track.pose_at(track.total_length)
        self.assertAlmostEqual(start.x, finish.x, places=8)
        self.assertAlmostEqual(start.y, finish.y, places=8)
        self.assertAlmostEqual(start.z, finish.z, places=8)

    def test_hill_is_five_degrees_and_returns_to_zero(self) -> None:
        track = ClosedHighwayTrack()
        _, slope_up = track.elevation_at(50.0)
        z_top, slope_top = track.elevation_at(100.0)
        z_down, slope_down = track.elevation_at(170.0)
        z_flat, slope_flat = track.elevation_at(210.0)
        self.assertAlmostEqual(math.degrees(slope_up), 5.0, places=8)
        self.assertEqual(slope_top, 0.0)
        self.assertLess(z_down, z_top)
        self.assertAlmostEqual(math.degrees(slope_down), -5.0, places=8)
        self.assertAlmostEqual(z_flat, 0.0, places=8)
        self.assertEqual(slope_flat, 0.0)

    def test_hill_reduces_vehicle_acceleration(self) -> None:
        command = ControlCommand(throttle=0.45)
        flat = VehicleModel()
        hill = VehicleModel()
        flat.reset(0.0, 0.0, 0.0, 0.0)
        hill.reset(0.0, 0.0, 0.0, 0.0)
        flat.state.speed = 15.0
        hill.state.speed = 15.0
        for _ in range(60):
            flat.step(command, 0.0, 1.0 / 60.0)
            hill.step(command, math.radians(5.0), 1.0 / 60.0)
        self.assertLess(hill.state.speed, flat.state.speed)


class TrafficAndControlTests(unittest.TestCase):
    def test_desired_gap_uses_constant_time_headway(self) -> None:
        self.assertAlmostEqual(
            desired_following_gap(10.0, 5.0, 1.5),
            20.0,
        )

    def test_lead_schedule_contains_slowing_and_stop(self) -> None:
        self.assertAlmostEqual(lead_speed_schedule(10.0), 10.5)
        self.assertAlmostEqual(lead_speed_schedule(30.0), 6.0)
        self.assertAlmostEqual(lead_speed_schedule(50.0), 0.0)
        self.assertGreater(lead_speed_schedule(62.0), 0.0)

    def test_sensor_reports_gap_closing_speed_and_ttc(self) -> None:
        simulation = Simulation(ReferenceController())
        simulation.vehicle.state.speed = 14.0
        simulation.lead_progress = simulation.track_progress + 50.0
        simulation.lead_speed = 10.0
        simulation._update_lead_pose()
        observation = simulation.observe(0.1)
        self.assertTrue(observation.lead_detected)
        self.assertGreater(observation.lead_distance, 0.0)
        self.assertAlmostEqual(observation.closing_speed, 4.0)
        self.assertAlmostEqual(
            observation.time_to_collision,
            observation.lead_distance / 4.0,
        )

    def test_cruise_only_starter_is_an_unsafe_baseline(self) -> None:
        simulation = Simulation(StarterController(), target_speed=14.0)
        simulation.run(70.0)
        self.assertGreater(
            int(simulation.metrics()["collision_samples"]),
            0,
        )


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.reference = Simulation(
            ReferenceController(),
            target_speed=14.0,
        )
        cls.reference.run(105.0)
        cls.reference_metrics = cls.reference.metrics()

    def test_reference_acc_is_safe_and_completes_lap(self) -> None:
        metrics = self.reference_metrics
        self.assertEqual(int(metrics["collision_samples"]), 0)
        self.assertGreater(float(metrics["minimum_gap_m"]), 4.0)
        self.assertGreater(float(metrics["minimum_ttc_s"]), 1.4)
        self.assertGreater(float(metrics["lap_progress_percent"]), 100.0)
        self.assertEqual(float(metrics["outside_road_percent"]), 0.0)
        self.assertLess(
            float(metrics["maximum_cross_track_error_m"]),
            0.5,
        )

    def test_reference_uses_follow_and_brake_modes(self) -> None:
        modes = {str(row["mode"]) for row in self.reference.history}
        self.assertIn("FOLLOW", modes)
        self.assertIn("BRAKE", modes)
        self.assertNotIn("EMERGENCY", modes)

    def test_worked_solution_matches_project_safety(self) -> None:
        simulation = Simulation(SolutionController(), target_speed=14.0)
        simulation.run(105.0)
        metrics = simulation.metrics()
        self.assertEqual(int(metrics["collision_samples"]), 0)
        self.assertEqual(int(metrics["emergency_samples"]), 0)
        self.assertGreater(float(metrics["minimum_gap_m"]), 4.0)
        self.assertGreater(float(metrics["lap_progress_percent"]), 100.0)
        self.assertLess(
            float(metrics["maximum_cross_track_error_m"]),
            0.75,
        )

    def test_commands_are_mutually_exclusive(self) -> None:
        for row in self.reference.history:
            self.assertFalse(
                float(row["throttle"]) > 0.0
                and float(row["brake"]) > 0.0
            )

    def test_csv_has_day3_columns(self) -> None:
        simulation = Simulation(ReferenceController())
        simulation.run(1.0)
        with tempfile.TemporaryDirectory() as directory:
            output = simulation.save_csv(Path(directory) / "run.csv")
            with output.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 60)
        for name in (
            "lead_speed_mps",
            "gap_m",
            "desired_gap_m",
            "time_to_collision_s",
            "selected_target_speed_mps",
            "mode",
            "collision",
        ):
            self.assertIn(name, rows[0])

    def test_renderer_contains_both_vehicle_colours(self) -> None:
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
