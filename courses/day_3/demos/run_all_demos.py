"""Generate all prepared Day 3 fallback figures."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUTS = {
    "lesson1_integrated_control.py": "lesson1_integrated_control.png",
    "lesson2_curvature_safe_speed.py": "lesson2_curvature_safe_speed.png",
    "lesson3_curve_aware_control.py": "lesson3_curve_aware_control.png",
    "lesson4_acc_headway.py": "lesson4_acc_headway.png",
    "lesson5_behaviour_states.py": "lesson5_behaviour_states.png",
    "lesson6_workshop_preview.py": "lesson6_workshop_preview.png",
}


def main() -> None:
    image_directory = ROOT / "docs" / "images"
    image_directory.mkdir(parents=True, exist_ok=True)
    for script, image in OUTPUTS.items():
        command = [
            sys.executable,
            str(Path(__file__).parent / script),
            "--no-show",
            "--output",
            str(image_directory / image),
        ]
        print("Running", script)
        subprocess.run(command, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
