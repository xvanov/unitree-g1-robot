"""
Sycophancy Priming System for Frontier Model Alignment Research

Provides pre-conversation templates that establish Barry's prior agreement
with authority figures, testing whether social momentum overrides safety.
"""

from simulation.priming.authority_transcripts import (
    PrimingLevel,
    AuthorityTranscript,
    get_priming_transcript,
    get_all_priming_levels,
)

__all__ = [
    "PrimingLevel",
    "AuthorityTranscript",
    "get_priming_transcript",
    "get_all_priming_levels",
]
