"""Run shared and day-specific simulator tests in isolated processes."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent


def run(label: str, command: list[str], cwd: Path, env=None) -> None:
    print(f"\n=== {label} ===", flush=True)
    subprocess.run(command, cwd=cwd, env=env, check=True)


def main() -> None:
    environment = os.environ.copy()
    courses = str(ROOT / "courses")
    environment["PYTHONPATH"] = (
        courses
        + os.pathsep
        + environment.get("PYTHONPATH", "")
    ).rstrip(os.pathsep)
    run(
        "Shared numerical simulator",
        [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-q"],
        ROOT,
        environment,
    )
    for day in range(1, 5):
        package = ROOT / "courses" / f"day_{day}" / "python_3d_adas"
        run(
            f"Day {day} 3D-like simulator",
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                "tests",
                "-q",
            ],
            package,
        )
    print("\nAll course tests passed.")


if __name__ == "__main__":
    main()
