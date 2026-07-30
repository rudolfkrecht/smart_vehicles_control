"""Reusable numerical components for Day 4 robustness and testing."""

from .challenge import (
    AGGRESSIVE_CONFIGURATION,
    BALANCED_CONFIGURATION,
    BatchReport,
    ControllerConfiguration,
    TestCase,
    evaluation_cases,
    initial_condition_cases,
    practice_cases,
    run_batch,
)
from .evaluation import (
    ChallengeScore,
    RobustMetrics,
    calculate_robust_metrics,
    score_challenge_run,
)
from .faults import FaultInjector, FaultParameters
from .robustness import (
    RobustResult,
    RobustScenario,
    RobustSimulation,
    RobustSnapshot,
    run_robust,
)

__all__ = [
    "AGGRESSIVE_CONFIGURATION",
    "BALANCED_CONFIGURATION",
    "BatchReport",
    "ChallengeScore",
    "ControllerConfiguration",
    "FaultInjector",
    "FaultParameters",
    "RobustMetrics",
    "RobustResult",
    "RobustScenario",
    "RobustSimulation",
    "RobustSnapshot",
    "TestCase",
    "calculate_robust_metrics",
    "evaluation_cases",
    "initial_condition_cases",
    "practice_cases",
    "run_batch",
    "run_robust",
    "score_challenge_run",
]


# Day 1 convenience exports retained for the original demonstrations.
from .controllers import OpenLoopController, PController, PIController
from .longitudinal import (
    Scenario,
    SimulationResult,
    VehicleParameters,
    run_simulation,
)
from .metrics import PerformanceMetrics, calculate_metrics

__all__ += [
    "OpenLoopController",
    "PController",
    "PIController",
    "Scenario",
    "SimulationResult",
    "VehicleParameters",
    "PerformanceMetrics",
    "calculate_metrics",
    "run_simulation",
]
