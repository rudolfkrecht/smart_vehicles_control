"""Compatibility API for the earlier compact Day 2 package.

New material imports the focused ``bicycle``, ``paths``, ``tracking`` and
``metrics`` modules directly. These aliases keep old examples usable.
"""

from __future__ import annotations

from .bicycle import VehicleParameters as Config
from .bicycle import VehicleState as State
from .metrics import calculate_path_metrics as metrics
from .paths import make_reference_path as make_track
from .tracking import pure_pursuit, run_path_following as run

__all__ = [
    "Config",
    "State",
    "make_track",
    "metrics",
    "pure_pursuit",
    "run",
]
