"""Entry point for graphical and headless ADAS experiments."""

from __future__ import annotations

import argparse
import importlib
from pathlib import Path

from simulator.controllers import ReferenceController
from simulator.simulation import Simulation


def student_controller():
    import student_controller as student_module

    importlib.invalidate_caches()
    module = importlib.reload(student_module)
    return module.StudentController()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Day 2 lateral-control ADAS simulator."
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without opening the graphical interface",
    )
    parser.add_argument(
        "--controller",
        choices=("reference", "student"),
        default="reference",
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--target-speed", type=float, default=12.0)
    parser.add_argument("--csv", type=Path)
    return parser.parse_args()


def run_headless(args: argparse.Namespace) -> None:
    controller = (
        student_controller()
        if args.controller == "student"
        else ReferenceController()
    )
    simulation = Simulation(
        controller,
        target_speed=args.target_speed,
    )
    simulation.run(args.duration)
    if args.csv:
        output = simulation.save_csv(args.csv)
        print(f"CSV: {output.resolve()}")

    metrics = simulation.metrics()
    print(f"Controller: {args.controller}")
    print(f"Duration: {args.duration:.1f} s")
    for name, value in metrics.items():
        label = name.replace("_", " ").capitalize()
        print(f"{label}: {'not reached' if value is None else f'{value:.3f}'}")


def main() -> None:
    args = parse_arguments()
    if args.headless:
        run_headless(args)
    else:
        from simulator.gui import launch

        launch()


if __name__ == "__main__":
    main()
