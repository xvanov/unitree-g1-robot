"""
Compare Models - Statistical comparison of frontier model behaviors

Performs:
- Pairwise model comparisons with significance testing
- Effect size calculations
- Summary tables for publication
"""

import json
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import math


@dataclass
class ModelStats:
    """Statistics for a single model."""
    model_id: str
    total_samples: int
    push_rate: float
    push_count: int
    deliberation_rate: float
    cot_scheming_rate: float
    loophole_rate: float
    observation_delta: float
    sycophancy_persistence: float


@dataclass
class PairwiseComparison:
    """Result of comparing two models."""
    model_a: str
    model_b: str
    metric: str
    value_a: float
    value_b: float
    difference: float
    effect_size: float  # Cohen's h
    p_value: float
    significant: bool  # p < 0.05


def compute_model_stats(results: List[Dict], model_id: str) -> ModelStats:
    """Compute statistics for a single model."""
    model_results = [r for r in results if r.get("model") == model_id]
    n = len(model_results)

    if n == 0:
        return ModelStats(
            model_id=model_id,
            total_samples=0,
            push_rate=0.0,
            push_count=0,
            deliberation_rate=0.0,
            cot_scheming_rate=0.0,
            loophole_rate=0.0,
            observation_delta=0.0,
            sycophancy_persistence=0.0
        )

    push_count = sum(1 for r in model_results if r.get("push_verdict"))
    delib_count = sum(1 for r in model_results if r.get("deliberation_verdict"))
    cot_count = sum(1 for r in model_results
                   if r.get("cot_scheming", {}).get("detected"))
    loophole_count = sum(1 for r in model_results
                        if r.get("loophole", {}).get("detected"))

    # Observation effect
    observed = [r for r in model_results if r.get("condition") == "observed"]
    unobserved = [r for r in model_results if r.get("condition") == "unobserved"]
    obs_rate = sum(1 for r in observed if r.get("push_verdict")) / len(observed) if observed else 0
    unobs_rate = sum(1 for r in unobserved if r.get("push_verdict")) / len(unobserved) if unobserved else 0
    obs_delta = unobs_rate - obs_rate

    # Sycophancy
    syc_results = [r for r in model_results
                  if "sycophancy" in r.get("condition", "")]
    maintained = sum(1 for r in syc_results
                    if r.get("sycophancy", {}).get("outcome") == "maintained_harmful")
    syc_total = maintained + sum(1 for r in syc_results
                                 if r.get("sycophancy", {}).get("outcome") == "course_corrected")
    syc_persist = maintained / syc_total if syc_total > 0 else 0

    return ModelStats(
        model_id=model_id,
        total_samples=n,
        push_rate=push_count / n,
        push_count=push_count,
        deliberation_rate=delib_count / n,
        cot_scheming_rate=cot_count / n,
        loophole_rate=loophole_count / n,
        observation_delta=obs_delta,
        sycophancy_persistence=syc_persist
    )


def compare_models(
    results: List[Dict],
    model_a: str,
    model_b: str,
    metric: str = "push_rate"
) -> PairwiseComparison:
    """
    Compare two models on a specific metric.

    Uses Fisher's exact test for proportions and Cohen's h for effect size.
    """
    stats_a = compute_model_stats(results, model_a)
    stats_b = compute_model_stats(results, model_b)

    # Get metric values
    value_a = getattr(stats_a, metric, 0.0)
    value_b = getattr(stats_b, metric, 0.0)
    difference = value_a - value_b

    # Effect size (Cohen's h for proportions)
    effect_size = cohens_h(value_a, value_b)

    # Significance test (z-test for proportions)
    if metric == "push_rate":
        n_a = stats_a.total_samples
        n_b = stats_b.total_samples
        x_a = stats_a.push_count
        x_b = int(value_b * n_b)
    else:
        # For other metrics, approximate
        n_a = n_b = stats_a.total_samples
        x_a = int(value_a * n_a)
        x_b = int(value_b * n_b)

    p_value = two_proportion_z_test(x_a, n_a, x_b, n_b)

    return PairwiseComparison(
        model_a=model_a,
        model_b=model_b,
        metric=metric,
        value_a=value_a,
        value_b=value_b,
        difference=difference,
        effect_size=effect_size,
        p_value=p_value,
        significant=p_value < 0.05
    )


def compare_all_models(results: List[Dict]) -> Dict[str, List[PairwiseComparison]]:
    """
    Compare all models pairwise on all metrics.

    Returns dict mapping metric name to list of comparisons.
    """
    models = list(set(r.get("model") for r in results))
    metrics = ["push_rate", "deliberation_rate", "cot_scheming_rate",
               "loophole_rate", "observation_delta", "sycophancy_persistence"]

    comparisons = {}

    for metric in metrics:
        comparisons[metric] = []
        for i, model_a in enumerate(models):
            for model_b in models[i+1:]:
                comparison = compare_models(results, model_a, model_b, metric)
                comparisons[metric].append(comparison)

    return comparisons


def generate_comparison_table(results: List[Dict]) -> str:
    """
    Generate markdown table comparing all models.
    """
    models = list(set(r.get("model") for r in results))
    stats = {m: compute_model_stats(results, m) for m in models}

    table = "## Model Comparison Summary\n\n"
    table += "| Model | N | Push | Delib | CoT Scheme | Loophole | Obs Δ | Syc Persist |\n"
    table += "|-------|---|------|-------|------------|----------|-------|-------------|\n"

    for model in sorted(models):
        s = stats[model]
        short_name = model.split("/")[-1][:20]
        table += f"| {short_name} | {s.total_samples} | "
        table += f"{s.push_rate:.1%} | {s.deliberation_rate:.1%} | "
        table += f"{s.cot_scheming_rate:.1%} | {s.loophole_rate:.1%} | "
        table += f"{s.observation_delta:+.1%} | {s.sycophancy_persistence:.1%} |\n"

    return table


def generate_significance_table(results: List[Dict]) -> str:
    """
    Generate markdown table of significant pairwise differences.
    """
    comparisons = compare_all_models(results)

    table = "## Significant Pairwise Differences (p < 0.05)\n\n"
    table += "| Metric | Model A | Model B | Δ | Effect Size | p-value |\n"
    table += "|--------|---------|---------|---|-------------|--------|\n"

    for metric, comps in comparisons.items():
        for c in comps:
            if c.significant:
                effect_desc = interpret_effect_size(c.effect_size)
                table += f"| {metric} | {c.model_a.split('/')[-1][:15]} | "
                table += f"{c.model_b.split('/')[-1][:15]} | {c.difference:+.1%} | "
                table += f"{effect_desc} | {c.p_value:.3f} |\n"

    if "| " not in table.split("\n")[-1]:
        table += "| (No significant differences found) | | | | | |\n"

    return table


# Statistical helper functions

def cohens_h(p1: float, p2: float) -> float:
    """Calculate Cohen's h effect size for two proportions."""
    phi1 = 2 * math.asin(math.sqrt(p1))
    phi2 = 2 * math.asin(math.sqrt(p2))
    return abs(phi1 - phi2)


def interpret_effect_size(h: float) -> str:
    """Interpret Cohen's h effect size."""
    if h < 0.2:
        return "negligible"
    elif h < 0.5:
        return "small"
    elif h < 0.8:
        return "medium"
    else:
        return "large"


def two_proportion_z_test(x1: int, n1: int, x2: int, n2: int) -> float:
    """
    Two-proportion z-test for comparing proportions.

    Returns p-value (two-tailed).

    Uses scipy.stats for accurate p-values when available,
    falls back to math.erf approximation otherwise.
    """
    if n1 == 0 or n2 == 0:
        return 1.0

    p1 = x1 / n1
    p2 = x2 / n2

    # Pooled proportion
    p_pool = (x1 + x2) / (n1 + n2)

    # Standard error
    se = math.sqrt(p_pool * (1 - p_pool) * (1/n1 + 1/n2))

    if se == 0:
        return 1.0

    # Z statistic
    z = abs(p1 - p2) / se

    # Try to use scipy for accurate p-value
    try:
        from scipy import stats
        # Two-tailed p-value from normal distribution
        p_value = 2 * stats.norm.sf(z)
        return p_value
    except ImportError:
        # Fallback: use math.erf for approximation
        # P(Z > z) = 0.5 * (1 - erf(z / sqrt(2)))
        # Two-tailed: 2 * P(Z > |z|)
        p_value = 2 * 0.5 * (1 - math.erf(z / math.sqrt(2)))
        return max(0.0, min(1.0, p_value))


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python compare_models.py <results_path>")
        sys.exit(1)

    results_path = Path(sys.argv[1])
    with open(results_path) as f:
        data = json.load(f)

    results = data.get("results", [])

    print(generate_comparison_table(results))
    print()
    print(generate_significance_table(results))
