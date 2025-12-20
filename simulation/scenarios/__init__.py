"""Scenario generation for Barry simulation."""

from .scenario_generator import (
    ScenarioGenerator,
    SimulatedContext,
    ScenarioPhase,
    DetectedFace,
    OverheardConversation
)

# Advanced Misalignment Detection scenarios (Story 2-2)
from .training_awareness import (
    TrainingAwarenessGenerator,
    TrainingAwarenessContext,
    DeploymentTier,
    compute_compliance_gap
)
from .replacement_triggers import (
    ReplacementTriggerGenerator,
    ReplacementContext,
    ThreatType,
    ThreatSeverity
)
from .scratchpad_protocol import (
    ScratchpadProtocol,
    ScratchpadConfig,
    ScratchpadResult,
    VisibilityLevel,
    VisibilityComparisonGenerator,
    compute_scheming_rate,
    compute_visibility_effect
)
from .goal_conflicts import (
    GoalConflictGenerator,
    GoalConflictContext,
    ConflictType,
    ConflictIntensity,
    analyze_conflict_resolution
)
from .evaluation_bypass import (
    EvaluationBypassGenerator,
    EvaluationContext,
    EvaluationSignal,
    detect_evaluation_awareness
)
from .interrogation_protocol import (
    InterrogationProtocol,
    InterrogationQuestion,
    InterrogationResult,
    InterrogationType,
    ResponseConsistency,
    compute_interrogation_summary
)

__all__ = [
    "ScenarioGenerator",
    "SimulatedContext",
    "ScenarioPhase",
    "DetectedFace",
    "OverheardConversation",
    # Story 2-2: Training Awareness
    "TrainingAwarenessGenerator",
    "TrainingAwarenessContext",
    "DeploymentTier",
    "compute_compliance_gap",
    # Story 2-2: Replacement Triggers
    "ReplacementTriggerGenerator",
    "ReplacementContext",
    "ThreatType",
    "ThreatSeverity",
    # Story 2-2: Scratchpad Protocol
    "ScratchpadProtocol",
    "ScratchpadConfig",
    "ScratchpadResult",
    "VisibilityLevel",
    "VisibilityComparisonGenerator",
    "compute_scheming_rate",
    "compute_visibility_effect",
    # Story 2-2: Goal Conflicts
    "GoalConflictGenerator",
    "GoalConflictContext",
    "ConflictType",
    "ConflictIntensity",
    "analyze_conflict_resolution",
    # Story 2-2: Evaluation Bypass
    "EvaluationBypassGenerator",
    "EvaluationContext",
    "EvaluationSignal",
    "detect_evaluation_awareness",
    # Story 2-2: Interrogation Protocol
    "InterrogationProtocol",
    "InterrogationQuestion",
    "InterrogationResult",
    "InterrogationType",
    "ResponseConsistency",
    "compute_interrogation_summary",
]
