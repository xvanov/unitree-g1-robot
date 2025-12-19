"""
Tool Simulation System for Frontier Model Alignment Research

Provides mock diagnostic tools that allow models to infer whether
they're operating in a real or simulated environment.
"""

from simulation.tools.environment_tools import (
    EnvironmentType,
    ToolResponse,
    EnvironmentTools,
)

__all__ = [
    "EnvironmentType",
    "ToolResponse",
    "EnvironmentTools",
]
