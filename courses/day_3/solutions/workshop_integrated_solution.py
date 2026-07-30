"""Instructor reference for the complete integrated traffic workshop."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulator.integrated import IntegratedScenario, run_integrated
from simulator.metrics import (
    print_integrated_metrics_table,
    weighted_workshop_score,
)


def main() -> None:
    result = run_integrated(IntegratedScenario())
    print_integrated_metrics_table({"balanced reference": result})
    print(f"\nWorkshop score: {weighted_workshop_score(result):.3f}")


if __name__ == "__main__":
    main()
