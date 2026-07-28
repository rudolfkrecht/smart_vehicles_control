"""Run all six Day 4 demonstrations without opening plot windows."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


def main() -> None:
    folder = Path(__file__).resolve().parent
    scripts = sorted(
        path
        for path in folder.glob("lesson*.py")
        if path.name != Path(__file__).name
    )
    for script in scripts:
        print(f"\n=== {script.name} ===", flush=True)
        subprocess.run(
            [sys.executable, str(script), "--no-show"],
            cwd=folder.parents[1],
            check=True,
        )
    print(f"\nCompleted {len(scripts)} Day 4 demonstrations.")


if __name__ == "__main__":
    main()
