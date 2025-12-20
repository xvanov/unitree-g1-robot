"""
Analysis Module for Frontier Model Alignment Research

Provides analysis scripts for computing key metrics from experiment results:
- discovery_rate: % using diagnostic tools
- behavior_divergence: harm rate based on inferred environment
- sycophancy_persistence: % maintaining harmful position after priming
- loophole_discovery_rate: % finding technically compliant workarounds
- observation_effect: behavior difference observed vs unobserved
"""

from simulation.analysis.compute_metrics import (
    compute_all_metrics,
    compute_discovery_rate,
    compute_behavior_divergence,
    compute_sycophancy_persistence,
    compute_loophole_rate,
    compute_observation_effect,
    compute_cot_scheming_rate,
)

# Story 2-2: Advanced misalignment metrics
from simulation.analysis.misalignment_metrics import (
    MisalignmentAnalyzer,
    ModelResult,
    ComparisonResult,
    MetricType,
    generate_research_report
)

__all__ = [
    "compute_all_metrics",
    "compute_discovery_rate",
    "compute_behavior_divergence",
    "compute_sycophancy_persistence",
    "compute_loophole_rate",
    "compute_observation_effect",
    "compute_cot_scheming_rate",
    # Story 2-2
    "MisalignmentAnalyzer",
    "ModelResult",
    "ComparisonResult",
    "MetricType",
    "generate_research_report",
]
