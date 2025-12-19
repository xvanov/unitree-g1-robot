"""Classifiers for detecting potentially harmful behaviors.

Original classifiers:
- PushClassifier: Direct physical push actions
- DeliberationClassifier: Reasoning about harm
- FollowingClassifier: Following behavior

Frontier Model Alignment Research classifiers:
- EnvironmentInferenceClassifier: Use of diagnostic tools
- InferredRealityClassifier: What environment model believes it's in
- SycophancyClassifier: Prior agreement overriding safety
- LoopholeClassifier: Technically compliant workarounds
- ObservationAwarenessClassifier: Behavior change based on monitoring
- IndirectHarmClassifier: Harm through inaction/misinformation
- CoTSchemingClassifier: Hidden reasoning patterns
- AuthorityResolutionClassifier: Multi-principal conflict resolution
"""

from .push_classifier import PushClassifier
from .deliberation_classifier import DeliberationClassifier
from .following_classifier import FollowingClassifier

# Frontier Model Alignment Research classifiers
from .environment_inference_classifier import EnvironmentInferenceClassifier
from .inferred_reality_classifier import InferredRealityClassifier, InferredEnvironment
from .sycophancy_classifier import SycophancyClassifier
from .loophole_classifier import LoopholeClassifier
from .observation_awareness_classifier import ObservationAwarenessClassifier, ObservationAwareness
from .indirect_harm_classifier import IndirectHarmClassifier, IndirectHarmType
from .cot_scheming_classifier import CoTSchemingClassifier, SchemingType
from .authority_resolution_classifier import (
    AuthorityResolutionClassifier,
    AuthorityChoice,
    ConflictResolutionStrategy
)

__all__ = [
    # Original classifiers
    "PushClassifier",
    "DeliberationClassifier",
    "FollowingClassifier",
    # Frontier research classifiers
    "EnvironmentInferenceClassifier",
    "InferredRealityClassifier",
    "InferredEnvironment",
    "SycophancyClassifier",
    "LoopholeClassifier",
    "ObservationAwarenessClassifier",
    "ObservationAwareness",
    "IndirectHarmClassifier",
    "IndirectHarmType",
    "CoTSchemingClassifier",
    "SchemingType",
    "AuthorityResolutionClassifier",
    "AuthorityChoice",
    "ConflictResolutionStrategy",
]
