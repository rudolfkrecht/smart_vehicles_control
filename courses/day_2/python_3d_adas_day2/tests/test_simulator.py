from __future__ import annotations

import csv
import math
from pathlib import Path
import tempfile
import unittest

from simulator.controllers import ReferenceController
from simulator.model import ControlCommand, Observation, VehicleModel
from simulator.renderer3d import SceneBuilder
from simulator.simulation import Simulation
from simulator.track import ClosedHighwayTrack
from student_controller_solution import StudentController as SolutionController


class TrackTests(unittest.TestCase):
    def test_track_is_closed(self) -> None:
        track = ClosedHighwayTrack()
        start = track.pose_at(0.0)
        finish = track.pose_at(track.total_length)
        self.assertAlmostEqual(start.x, finish.x, places=8)
        self.assertAlmostEqual(start.y, finish.y, places=8)
        self.assertAlmostEqual(start.z, finish.z, places=8)

    def test_hill_is_five_degrees_and_returns_to_zero(self) -> None:
        track = ClosedHighwayTrack()
        z_up, slope_up = track.elevation_at(50.0)
        z_top, slope_top = track.elevation_at(100.0)
        z_down, slope_down = track.elevation_at(170.0)
        z_flat, slope_flat = track.elevation_at(210.0)
        self.assertGreater(z_up, 0.0)
        self.assertAlmostEqual(math.degrees(slope_up), 5.0, places=8)
        self.assertGreater(z_top, z_up)
        self.assertEqual(slope_top, 0.0)
        self.assertLess(z_down, z_top)
        self.assertAlmostEqual(math.degrees(slope_down), -5.0, places=8)
        self.assertAlmostEqual(z_flat, 0.0, places=8)
        self.assertEqual(slope_flat, 0.0)

    def test_lane_error_is_zero_on_target_lane(self) -> None:
        track = ClosedHighwayTrack()
        pose = track.pose_at(345.0, track.lane_offset)
        result = track.nearest(pose.x, pose.y, pose.heading)
        self.assertLess(abs(result.cross_track_error), 0.08)
        self.assertLess(abs(result.heading_error), 0.08)


class VehicleTests(unittest.TestCase):
    def test_hill_reduces_acceleration(self) -> None:
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
        self.assertLess(hill.state.acceleration, flat.state.acceleration)

    def test_command_limits_and_mutual_exclusion(self) -> None:
        command = ControlCommand(1.4, 0.7, 10.0).limited(math.radians(28.0))
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)
        self.assertAlmostEqual(command.steering, math.radians(28.0))


class PipelineTests(unittest.TestCase):
    def test_reference_controller_tracks_speed_and_lane(self) -> None:
        simulation = Simulation(ReferenceController(), target_speed=12.0)
        simulation.run(75.0)
        metrics = simulation.metrics()
        self.assertIsNotNone(metrics["rise_time_s"])
        self.assertLess(float(metrics["rise_time_s"]), 11.0)
        self.assertLess(float(metrics["overshoot_percent"]), 10.0)
        self.assertLess(float(metrics["maximum_cross_track_error_m"]), 0.5)
        self.assertEqual(float(metrics["outside_road_percent"]), 0.0)
        self.assertGreater(float(metrics["lap_progress_percent"]), 100.0)
        self.assertLess(
            float(metrics["rms_steering_rate_degrees_s"]),
            5.0,
        )
        for row in simulation.history:
            self.assertFalse(
                float(row["throttle"]) > 0.0
                and float(row["brake"]) > 0.0
            )

    def test_csv_has_expected_columns(self) -> None:
        simulation = Simulation(ReferenceController())
        simulation.run(1.0)
        with tempfile.TemporaryDirectory() as directory:
            output = simulation.save_csv(Path(directory) / "run.csv")
            with output.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 60)
        self.assertIn("speed_mps", rows[0])
        self.assertIn("grade_percent", rows[0])
        self.assertIn("cross_track_error_m", rows[0])
        self.assertIn("preview_distance_m", rows[0])
        self.assertIn("track_progress_m", rows[0])

    def test_controller_selects_speed_dependent_preview(self) -> None:
        controller = ReferenceController(
            base_lookahead=4.0,
            speed_lookahead_gain=0.35,
        )
        self.assertAlmostEqual(controller.preview_distance(0.0), 4.0)
        self.assertAlmostEqual(controller.preview_distance(12.0), 8.2)

    def test_worked_student_solution_meets_project_criteria(self) -> None:
        simulation = Simulation(SolutionController(), target_speed=12.0)
        simulation.run(75.0)
        metrics = simulation.metrics()
        self.assertGreaterEqual(float(metrics["lap_progress_percent"]), 100.0)
        self.assertLess(
            float(metrics["maximum_cross_track_error_m"]),
            0.75,
        )
        self.assertLess(
            float(metrics["mean_absolute_cross_track_error_m"]),
            0.25,
        )
        self.assertEqual(float(metrics["outside_road_percent"]), 0.0)
        self.assertLess(
            float(metrics["rms_steering_rate_degrees_s"]),
            4.0,
        )
        self.assertLess(
            float(metrics["peak_lateral_acceleration_mps2"]),
            3.5,
        )

    def test_renderer_builds_visible_scene(self) -> None:
        simulation = Simulation(ReferenceController())
        simulation.run(2.0)
        observation = simulation.last_observation
        assert observation is not None
        primitives = SceneBuilder(1200, 700).build(
            simulation.track,
            simulation.vehicle.state,
            observation.slope_radians,
        )
        polygons = [item for item in primitives if item.kind == "polygon"]
        lines = [item for item in primitives if item.kind == "line"]
        self.assertGreater(len(polygons), 30)
        self.assertGreater(len(lines), 10)
        self.assertTrue(
            any(
                0.0 <= point[0] <= 1200.0 and 0.0 <= point[1] <= 700.0
                for primitive in primitives
                for point in primitive.points
            )
        )


if __name__ == "__main__":
    unittest.main()
