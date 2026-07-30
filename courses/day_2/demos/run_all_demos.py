"""Generate every prepared Day 2 teaching figure headlessly."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


DEMO_OUTPUTS = {
    "lesson1_bicycle_motion.py": "lesson1_bicycle_motion.png",
    "lesson2_steering_exploration.py": "lesson2_steering_exploration.png",
    "lesson3_path_errors.py": "lesson3_path_errors.png",
    "lesson4_pure_pursuit.py": "lesson4_pure_pursuit.png",
    "lesson6_adaptive_preview.py": "lesson6_adaptive_preview.png",
}


def main() -> None:
    demo_directory = Path(__file__).resolve().parent
    repository_root = demo_directory.parents[2]
    image_directory = (
        repository_root / "docs" / "assets" / "images" / "day2"
    )
    image_directory.mkdir(parents=True, exist_ok=True)
    for script_name, image_name in DEMO_OUTPUTS.items():
        command = [
            sys.executable,
            str(demo_directory / script_name),
            "--no-show",
            "--output",
            str(image_directory / image_name),
        ]
        print("Running:", " ".join(command))
        subprocess.run(command, check=True, cwd=repository_root)


if __name__ == "__main__":
    main()
