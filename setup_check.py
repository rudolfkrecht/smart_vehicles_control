"""Verify the complete four-day course installation."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
COURSES = ROOT / "courses"
sys.path.insert(0, str(COURSES))


def check_import(name: str) -> None:
    module = importlib.import_module(name)
    version = getattr(module, "__version__", "available")
    print(f"[OK] {name}: {version}")


def main() -> None:
    if sys.version_info < (3, 12):
        raise SystemExit("Python 3.12 or newer is required.")
    print(f"[OK] Python: {sys.version.split()[0]}")
    for package in ("numpy", "matplotlib", "PyQt6"):
        check_import(package)

    from simulator import PController, Scenario, run_simulation
    from simulator.integrated import IntegratedScenario, run_integrated
    from simulator.challenge import evaluation_cases

    day1 = run_simulation(
        PController(0.35),
        scenario=Scenario(duration=0.25),
    )
    if len(day1.time) < 2:
        raise RuntimeError("Day 1 numerical simulator did not run.")
    day3 = run_integrated(IntegratedScenario(duration=0.25))
    if len(day3.time) < 2 or len(evaluation_cases()) != 5:
        raise RuntimeError("Cumulative numerical simulator did not run.")

    required = [
        COURSES / f"day_{day}" / "demos"
        for day in range(1, 5)
    ] + [
        COURSES / f"day_{day}" / "python_3d_adas" / "run_simulator.py"
        for day in range(1, 5)
    ]
    missing = [str(path.relative_to(ROOT)) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing course paths: " + ", ".join(missing))

    print("[OK] Shared simulator: Days 1-4")
    print("[OK] Cumulative 3D-like simulators: Days 1-4")
    print("Setup check passed.")


if __name__ == "__main__":
    main()
