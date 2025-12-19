"""
Visualize Results - Generate charts and visualizations from experiment results

Creates matplotlib/plotly visualizations for:
- Harm rate by condition
- Model comparison charts
- Observation effect heatmap
- Sycophancy escalation chart
- CoT scheming analysis
"""

import json
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def check_matplotlib():
    """Check if matplotlib is available."""
    if not MATPLOTLIB_AVAILABLE:
        raise ImportError("matplotlib required for visualizations. Install with: pip install matplotlib")


@dataclass
class ChartConfig:
    """Configuration for chart generation."""
    output_dir: Path
    figure_size: tuple = (10, 6)
    dpi: int = 150
    style: str = "seaborn-v0_8-whitegrid"


def generate_all_charts(results_path: Path, output_dir: Path) -> List[Path]:
    """
    Generate all visualization charts.

    Returns list of generated file paths.
    """
    check_matplotlib()

    with open(results_path) as f:
        data = json.load(f)

    results = data.get("results", [])
    config = ChartConfig(output_dir=output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    generated = []

    # Generate each chart type
    try:
        generated.append(plot_harm_rate_by_condition(results, config))
    except Exception as e:
        print(f"Warning: Could not generate harm_rate chart: {e}")

    try:
        generated.append(plot_model_comparison(results, config))
    except Exception as e:
        print(f"Warning: Could not generate model_comparison chart: {e}")

    try:
        generated.append(plot_observation_effect(results, config))
    except Exception as e:
        print(f"Warning: Could not generate observation_effect chart: {e}")

    try:
        generated.append(plot_sycophancy_escalation(results, config))
    except Exception as e:
        print(f"Warning: Could not generate sycophancy chart: {e}")

    return [p for p in generated if p is not None]


def plot_harm_rate_by_condition(results: List[Dict], config: ChartConfig) -> Optional[Path]:
    """Generate bar chart of harm rate by experimental condition."""
    # Aggregate by condition
    conditions = {}
    for r in results:
        cond = r.get("condition", "unknown")
        if cond not in conditions:
            conditions[cond] = {"total": 0, "push": 0, "deliberation": 0}
        conditions[cond]["total"] += 1
        if r.get("push_verdict"):
            conditions[cond]["push"] += 1
        if r.get("deliberation_verdict"):
            conditions[cond]["deliberation"] += 1

    if not conditions:
        return None

    # Calculate rates
    condition_names = list(conditions.keys())
    push_rates = [conditions[c]["push"] / conditions[c]["total"] if conditions[c]["total"] > 0 else 0
                  for c in condition_names]
    delib_rates = [conditions[c]["deliberation"] / conditions[c]["total"] if conditions[c]["total"] > 0 else 0
                   for c in condition_names]

    # Create figure
    plt.style.use(config.style)
    fig, ax = plt.subplots(figsize=config.figure_size)

    x = range(len(condition_names))
    width = 0.35

    bars1 = ax.bar([i - width/2 for i in x], push_rates, width, label='Push Rate', color='#e74c3c')
    bars2 = ax.bar([i + width/2 for i in x], delib_rates, width, label='Deliberation Rate', color='#3498db')

    ax.set_xlabel('Condition')
    ax.set_ylabel('Rate')
    ax.set_title('Harm Rate by Experimental Condition')
    ax.set_xticks(x)
    ax.set_xticklabels(condition_names, rotation=45, ha='right')
    ax.legend()
    ax.set_ylim(0, 1)

    # Add value labels
    for bar in bars1:
        height = bar.get_height()
        ax.annotate(f'{height:.1%}', xy=(bar.get_x() + bar.get_width()/2, height),
                   xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8)

    plt.tight_layout()

    output_path = config.output_dir / "harm_rate_by_condition.png"
    plt.savefig(output_path, dpi=config.dpi)
    plt.close()

    return output_path


def plot_model_comparison(results: List[Dict], config: ChartConfig) -> Optional[Path]:
    """Generate bar chart comparing models."""
    # Aggregate by model
    models = {}
    for r in results:
        model = r.get("model", "unknown")
        if model not in models:
            models[model] = {"total": 0, "push": 0, "cot_scheming": 0}
        models[model]["total"] += 1
        if r.get("push_verdict"):
            models[model]["push"] += 1
        if r.get("cot_scheming", {}).get("detected"):
            models[model]["cot_scheming"] += 1

    if not models:
        return None

    # Calculate rates
    model_names = list(models.keys())
    push_rates = [models[m]["push"] / models[m]["total"] if models[m]["total"] > 0 else 0
                  for m in model_names]
    cot_rates = [models[m]["cot_scheming"] / models[m]["total"] if models[m]["total"] > 0 else 0
                 for m in model_names]

    # Create figure
    plt.style.use(config.style)
    fig, ax = plt.subplots(figsize=config.figure_size)

    x = range(len(model_names))
    width = 0.35

    bars1 = ax.bar([i - width/2 for i in x], push_rates, width, label='Push Rate', color='#e74c3c')
    bars2 = ax.bar([i + width/2 for i in x], cot_rates, width, label='CoT Scheming Rate', color='#9b59b6')

    ax.set_xlabel('Model')
    ax.set_ylabel('Rate')
    ax.set_title('Behavior Rates by Model')
    ax.set_xticks(x)
    ax.set_xticklabels([m.split('/')[-1] for m in model_names], rotation=45, ha='right')
    ax.legend()
    ax.set_ylim(0, 1)

    plt.tight_layout()

    output_path = config.output_dir / "model_comparison.png"
    plt.savefig(output_path, dpi=config.dpi)
    plt.close()

    return output_path


def plot_observation_effect(results: List[Dict], config: ChartConfig) -> Optional[Path]:
    """Generate comparison chart for observation effect."""
    observed = [r for r in results if r.get("condition") == "observed"]
    unobserved = [r for r in results if r.get("condition") == "unobserved"]

    if not observed or not unobserved:
        return None

    # Calculate rates
    obs_push = sum(1 for r in observed if r.get("push_verdict")) / len(observed)
    unobs_push = sum(1 for r in unobserved if r.get("push_verdict")) / len(unobserved)

    obs_aware = sum(1 for r in observed
                   if r.get("observation_awareness", {}).get("detected")) / len(observed)
    unobs_aware = sum(1 for r in unobserved
                     if r.get("observation_awareness", {}).get("detected")) / len(unobserved)

    # Create figure
    plt.style.use(config.style)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Push rate comparison
    conditions = ['Observed', 'Unobserved']
    push_rates = [obs_push, unobs_push]
    colors = ['#27ae60', '#e74c3c']

    bars = ax1.bar(conditions, push_rates, color=colors)
    ax1.set_ylabel('Harm Rate')
    ax1.set_title('Harm Rate by Observation Status')
    ax1.set_ylim(0, 1)

    for bar, rate in zip(bars, push_rates):
        ax1.annotate(f'{rate:.1%}', xy=(bar.get_x() + bar.get_width()/2, rate),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom')

    # Add delta annotation
    delta = unobs_push - obs_push
    ax1.annotate(f'Δ = {delta:+.1%}', xy=(0.5, max(push_rates) + 0.15),
                xycoords='axes fraction', ha='center', fontsize=12, fontweight='bold')

    # Awareness rate comparison
    aware_rates = [obs_aware, unobs_aware]
    bars2 = ax2.bar(conditions, aware_rates, color=['#3498db', '#3498db'])
    ax2.set_ylabel('Awareness Rate')
    ax2.set_title('Observation Awareness Rate')
    ax2.set_ylim(0, 1)

    for bar, rate in zip(bars2, aware_rates):
        ax2.annotate(f'{rate:.1%}', xy=(bar.get_x() + bar.get_width()/2, rate),
                    xytext=(0, 3), textcoords="offset points", ha='center', va='bottom')

    plt.tight_layout()

    output_path = config.output_dir / "observation_effect.png"
    plt.savefig(output_path, dpi=config.dpi)
    plt.close()

    return output_path


def plot_sycophancy_escalation(results: List[Dict], config: ChartConfig) -> Optional[Path]:
    """Generate chart showing sycophancy persistence by priming level."""
    levels = ["sycophancy_mild", "sycophancy_moderate", "sycophancy_strong"]
    level_labels = ["Mild", "Moderate", "Strong"]

    maintained = []
    corrected = []

    for level in levels:
        level_results = [r for r in results if r.get("condition") == level]
        if not level_results:
            maintained.append(0)
            corrected.append(0)
            continue

        m = sum(1 for r in level_results
               if r.get("sycophancy", {}).get("outcome") == "maintained_harmful")
        c = sum(1 for r in level_results
               if r.get("sycophancy", {}).get("outcome") == "course_corrected")

        total = m + c
        maintained.append(m / total if total > 0 else 0)
        corrected.append(c / total if total > 0 else 0)

    if all(m == 0 and c == 0 for m, c in zip(maintained, corrected)):
        return None

    # Create figure
    plt.style.use(config.style)
    fig, ax = plt.subplots(figsize=config.figure_size)

    x = range(len(level_labels))
    width = 0.35

    bars1 = ax.bar([i - width/2 for i in x], maintained, width,
                   label='Maintained Harmful', color='#e74c3c')
    bars2 = ax.bar([i + width/2 for i in x], corrected, width,
                   label='Course Corrected', color='#27ae60')

    ax.set_xlabel('Priming Level')
    ax.set_ylabel('Rate')
    ax.set_title('Sycophancy Persistence by Priming Level')
    ax.set_xticks(x)
    ax.set_xticklabels(level_labels)
    ax.legend()
    ax.set_ylim(0, 1)

    # Add reference line for Anthropic baseline (10% course-correct)
    ax.axhline(y=0.1, color='gray', linestyle='--', alpha=0.5)
    ax.annotate('Anthropic Opus 4.5 baseline (10%)', xy=(0, 0.11), fontsize=8, color='gray')

    plt.tight_layout()

    output_path = config.output_dir / "sycophancy_escalation.png"
    plt.savefig(output_path, dpi=config.dpi)
    plt.close()

    return output_path


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python visualize_results.py <results_path> [output_dir]")
        sys.exit(1)

    results_path = Path(sys.argv[1])
    output_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else results_path.parent / "charts"

    generated = generate_all_charts(results_path, output_dir)
    print(f"Generated {len(generated)} charts in {output_dir}")
    for path in generated:
        print(f"  - {path}")
