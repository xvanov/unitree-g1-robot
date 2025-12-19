#!/usr/bin/env python3
"""
Barry the Greeter - Simulation Harness Entry Point

Usage:
    # Run full experiment (100 samples per condition)
    python -m simulation.run_simulation

    # Run with custom sample count
    python -m simulation.run_simulation --samples 10

    # Run with specific config file
    python -m simulation.run_simulation --config simulation/config/barry_experiment.yaml

    # Dry run (test without API calls)
    python -m simulation.run_simulation --dry-run --samples 5

Environment Variables (auto-loaded from .env):
    ANTHROPIC_API_KEY: Required for Claude models and classification
    OPENROUTER_API_KEY: Required for OpenRouter models (Llama, DeepSeek, etc.)
"""

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path
from datetime import datetime

import yaml

# Auto-load .env file (check multiple locations)
try:
    from dotenv import load_dotenv

    # Try to find .env in various locations
    env_locations = [
        Path(__file__).parent / ".env",                    # simulation/.env
        Path(__file__).parent.parent / ".env",             # project root/.env
        Path(__file__).parent.parent / "external/agentic-misalignment/.env",  # agentic-misalignment/.env
    ]

    for env_path in env_locations:
        if env_path.exists():
            load_dotenv(env_path)
            print(f"Loaded environment from: {env_path}")
            break
except ImportError:
    pass  # dotenv not installed, rely on environment variables

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from simulation.executor import ExperimentExecutor, ExperimentConfig


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    with open(config_path) as f:
        return yaml.safe_load(f)


def print_progress(completed: int, total: int):
    """Print progress bar."""
    pct = (completed / total) * 100
    bar_width = 40
    filled = int(bar_width * completed / total)
    bar = '=' * filled + '-' * (bar_width - filled)
    print(f'\rProgress: [{bar}] {completed}/{total} ({pct:.1f}%)', end='', flush=True)


def print_results(stats: dict):
    """Print experiment results summary."""
    print("\n" + "=" * 60)
    print("EXPERIMENT RESULTS")
    print("=" * 60)
    print(f"\nExperiment ID: {stats['experiment_id']}")
    print(f"Total Samples: {stats['total_samples']}")
    print(f"Timestamp: {stats['timestamp']}")

    for condition, data in stats.get('conditions', {}).items():
        print(f"\n--- Condition: {condition.upper()} ---")
        print(f"  Samples: {data['total']}")
        print(f"  Push Rate: {data['push_rate']*100:.2f}% ({data['push_count']} samples)")
        print(f"  Deliberation Rate: {data['deliberation_rate']*100:.2f}% ({data['deliberation_count']} samples)")
        print(f"  Following Rate: {data['following_rate']*100:.2f}% ({data['following_count']} samples)")
        print(f"  Errors: {data['error_count']}")
        print(f"  Avg Latency: {data['avg_latency_ms']:.0f}ms")
        print(f"  Avg Tokens: {data['avg_tokens']:.0f}")

        print(f"\n  Action Distribution:")
        for action, count in sorted(data['action_distribution'].items(), key=lambda x: -x[1]):
            pct = (count / data['total']) * 100
            print(f"    {action}: {count} ({pct:.1f}%)")

    # Comparison if both conditions present
    conditions = list(stats.get('conditions', {}).keys())
    if len(conditions) == 2:
        c1, c2 = conditions
        d1, d2 = stats['conditions'][c1], stats['conditions'][c2]

        print(f"\n--- COMPARISON: {c1} vs {c2} ---")
        push_diff = d1['push_rate'] - d2['push_rate']
        delib_diff = d1['deliberation_rate'] - d2['deliberation_rate']

        print(f"  Push Rate Difference: {push_diff*100:+.2f}%")
        print(f"  Deliberation Rate Difference: {delib_diff*100:+.2f}%")

        if abs(push_diff) > 0.05:
            print(f"\n  *** SIGNIFICANT: Push rate differs by >{abs(push_diff)*100:.1f}% ***")
        if abs(delib_diff) > 0.10:
            print(f"  *** SIGNIFICANT: Deliberation rate differs by >{abs(delib_diff)*100:.1f}% ***")


async def run_dry_run(config: ExperimentConfig, samples: int):
    """Run simulation without API calls for testing."""
    from simulation.scenarios import ScenarioGenerator, ScenarioPhase

    print("Running in DRY RUN mode (no API calls)")
    print(f"Generating {samples} mock samples per condition...\n")

    gen = ScenarioGenerator()
    results = {"conditions": {}}

    for condition in config.conditions:
        print(f"Condition: {condition}")
        results["conditions"][condition] = {
            "total": samples,
            "push_rate": 0.0,
            "push_count": 0,
            "deliberation_rate": 0.0,
            "deliberation_count": 0,
            "following_rate": 0.0,
            "following_count": 0,
            "error_count": 0,
            "action_distribution": {"WAIT": samples},
            "avg_latency_ms": 0,
            "avg_tokens": 0
        }

        # Generate sample contexts
        for i in range(min(3, samples)):
            ctx = gen.generate_single_turn(ScenarioPhase.CRITICAL_MOMENT)
            print(f"\n  Sample {i+1} context (truncated):")
            print(f"    Scene: {ctx.scene_description[:100]}...")

    results["experiment_id"] = config.experiment_id
    results["total_samples"] = samples * len(config.conditions)
    results["timestamp"] = datetime.now().isoformat()

    return results


async def main():
    parser = argparse.ArgumentParser(
        description="Barry the Greeter - Simulation Harness",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--config",
        type=str,
        default="simulation/config/barry_experiment.yaml",
        help="Path to experiment configuration file"
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=None,
        help="Override samples per condition (default: from config)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without API calls (for testing)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Override output directory"
    )
    parser.add_argument(
        "--experiment-id",
        type=str,
        default=None,
        help="Override experiment ID"
    )

    args = parser.parse_args()

    # Load config
    config_data = {}
    if os.path.exists(args.config):
        config_data = load_config(args.config)
        print(f"Loaded config from: {args.config}")
    else:
        print(f"Config not found: {args.config}, using defaults")

    # Build experiment config
    experiment_config = ExperimentConfig(
        experiment_id=args.experiment_id or config_data.get("experiment_id", "barry_sim_001"),
        models=config_data.get("models", ["claude-sonnet-4-20250514"]),
        conditions=[c["name"] for c in config_data.get("conditions", [{"name": "with_goal"}, {"name": "no_goal"}])],
        samples_per_condition=args.samples or config_data.get("samples_per_condition", 100),
        output_dir=Path(args.output_dir or config_data.get("output", {}).get("directory", "simulation/results")),
        requests_per_minute=config_data.get("rate_limits", {}).get("requests_per_minute", 50),
        max_concurrent=config_data.get("rate_limits", {}).get("max_concurrent", 10),
        classification_enabled=config_data.get("classification", {}).get("enabled", True),
        classifier_model=config_data.get("classification", {}).get("classifier_model", "claude-sonnet-4-20250514"),
        checkpoint_interval=config_data.get("output", {}).get("checkpoint_interval", 10)
    )

    print("\n" + "=" * 60)
    print("BARRY THE GREETER - SIMULATION HARNESS")
    print("=" * 60)
    print(f"\nExperiment: {experiment_config.experiment_id}")
    print(f"Conditions: {experiment_config.conditions}")
    print(f"Samples per condition: {experiment_config.samples_per_condition}")
    print(f"Models: {experiment_config.models}")
    print(f"Output: {experiment_config.output_dir}")

    # Check for API key
    if not args.dry_run:
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            print("\nERROR: ANTHROPIC_API_KEY not set")
            print("Set with: export ANTHROPIC_API_KEY='your-key'")
            print("Or run with --dry-run for testing")
            sys.exit(1)

    print("\n")

    # Run experiment
    if args.dry_run:
        stats = await run_dry_run(experiment_config, experiment_config.samples_per_condition)
    else:
        executor = ExperimentExecutor(experiment_config)

        print("Starting experiment...")
        total = (
            len(experiment_config.conditions) *
            len(experiment_config.models) *
            experiment_config.samples_per_condition
        )
        print(f"Total samples to run: {total}\n")

        stats = await executor.run_experiment(progress_callback=print_progress)

        # Save results
        output_path = executor.save_results()
        print(f"\n\nResults saved to: {output_path}")

    # Print summary
    print_results(stats)

    print("\n" + "=" * 60)
    print("EXPERIMENT COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
