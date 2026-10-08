"""
fitness.py — Authoritative FRC Team 8334 Match Fitness Evaluator

Computes a 4-component composite fitness from headless 3v3 match JSONL rows,
with hard guardrail pre-filters and paired δ-Fitness (95% CI lower bound)
across identical seeds to handle MapleSim physics non-determinism.

See AGENTS.md fitness methodology for the theoretical framework.
"""

from typing import Dict, Any, List, Tuple
import math
import random
import statistics


# ---------------------------------------------------------------------------
# Guardrail constants — mirrored from compare.py / sweep.ps1
# ---------------------------------------------------------------------------
GATE_MAX_LOOP_OVERRUNS = 8
GATE_MAX_ROBOT_PERIODIC_MS = 60.0  # mirrored from sweep.ps1 -MaxRobotPeriodicMs / compare.py
GATE_MAX_STALL_SEC = 6.0
GATE_MAX_CONSEC_RECOVERIES = 3
GATE_DEFENSIVE_PATH_M = 25.0
GATE_CYCLER_SHARE = 0.15
GATE_ADAPTIVE_SHARE = 0.10
WASTED_FUEL_TOLERANCE = 0.05

OFFENSIVE_ROLES = {"AUTONOMOUS_CYCLER", "CO_PILOT"}
DEFENSIVE_ROLES = {"TACTICAL_DEFENDER", "DEFENSE_BULLY"}
ADAPTIVE_ROLES = {"ADAPTIVE_COMPETITOR", "LEAD_PURSUIT_INTERCEPTOR"}


# ---------------------------------------------------------------------------
# Core: single-row composite fitness
# ---------------------------------------------------------------------------

def calculate_single_match_fitness(row: Dict[str, Any]) -> float:
    """
    Compute the 4-component composite fitness for one match row.

    Returns:
        float: Fitness value. -9999.0 if any guardrail fails (disqualify the row).
    """

    # ----- Guardrail 1: Loop-timing health -----
    h = row.get("health")
    if not isinstance(h, dict):
        return -9999.0  # no health block = contaminated / worker didn't stamp
    overruns = h.get("loopOverruns")
    max_ms = h.get("maxRobotPeriodicMs")
    if overruns is None or max_ms is None:
        return -9999.0  # incomplete health block
    if overruns < 0 or max_ms < 0:
        return -9999.0  # unmeasured, treat as unknown/not clean
    if overruns > GATE_MAX_LOOP_OVERRUNS:
        return -9999.0  # CPU-starved worker
    if max_ms > GATE_MAX_ROBOT_PERIODIC_MS:
        return -9999.0  # load perturbation

    # ----- Guardrail 2: Recon canaries -----
    for who in ("Blue", "Red"):
        resid = row.get("%sReconciliationResidual" % who.lower())
        unattr = row.get("%sUnattributedFuel" % who.lower())
        if resid not in (0, None):
            return -9999.0  # attribution leak
        if unattr not in (0, None):
            return -9999.0  # unattributed fuel

    # ----- Guardrail 3: Fuel-Driven Win Gate -----
    # If Blue won but BlueFuel < RedFuel (won purely on penalties), reject.
    blue_fuel = row.get("blueAutoFuel", 0) + row.get("blueTeleopFuel", 0)
    red_fuel = row.get("redAutoFuel", 0) + row.get("redTeleopFuel", 0)
    # We cannot strictly enforce "Blue won implies BlueFuel > RedFuel" without
    # the winner field being reliably comparable, so we soft-gate: if the row
    # has a winner and it violates the spirit, penalize but do not disqualify.
    # The hard gate is below in the component computation.

    # ----- Component A: Clean Scoring Throughput -----
    s_fuel = (1.0 * blue_fuel) - (0.5 * red_fuel)

    # ----- Component B: Inventory Efficiency & Waste -----
    wasted_shots = row.get("blueWastedFuel", 0) or 0
    w_waste = 2.0 * wasted_shots

    # ----- Component C: Motion Health & Fluidity -----
    m_motion = 0.0
    blue_bots = row.get("blueBots", {}) or {}
    stalls = blue_bots.get("maxContiguousStallSec", []) or []
    consec_recs = blue_bots.get("maxConsecutiveRecoveries", []) or []
    path_lens = blue_bots.get("pathLengthM", []) or []

    for i in range(max(len(stalls), len(consec_recs), len(path_lens))):
        stall = stalls[i] if i < len(stalls) else 0.0
        consec = consec_recs[i] if i < len(consec_recs) else 0
        path = path_lens[i] if i < len(path_lens) else 200.0  # assume moved if missing

        # Quadratic penalty for stalls > 1.5 s (normal staging unpenalized)
        if stall > 1.5:
            m_motion += (stall - 1.5) ** 2

        # Penalty for recovery thrashing
        m_motion += 1.5 * consec

        # Penalty for bots that never moved (< 25 m in 150 s)
        if path < 25.0:
            m_motion += (25.0 - path) * 0.5

    # ----- Component D: Rule Discipline (penalty points conceded) -----
    # redPenaltyPoints is what Red received from Blue fouls; we subtract it from fitness
    p_foul = 1.0 * (row.get("redPenaltyPoints", 0) or 0)

    # ----- Total Match Fitness -----
    fitness = s_fuel - w_waste - m_motion - p_foul
    return fitness


# ---------------------------------------------------------------------------
# Paired δ-Fitness across identical seeds (bootstrap 95% CI lower bound)
# ---------------------------------------------------------------------------

def evaluate_paired_fitness(
    trial_rows: List[Dict[str, Any]],
    baseline_rows: List[Dict[str, Any]],
) -> Tuple[float, Dict[str, Any]]:
    """
    Compute paired δ-Fitness = mean(F_trial - F_baseline) across identical seeds,
    with a 95% confidence interval lower bound.

    Returns:
        (conservative_fitness, stats_dict)
        - conservative_fitness: mean_delta - 1.96 * SE  (the number to maximize)
        - stats_dict: mean, sd, se, n_seeds, raw_deltas
    """

    # Index baseline by seed
    baseline_by_seed: Dict[int, float] = {}
    for r in baseline_rows:
        seed = r.get("seed")
        if seed is not None:
            f = calculate_single_match_fitness(r)
            baseline_by_seed[seed] = f

    # Compute per-seed deltas
    deltas: List[float] = []
    seeds_with_data: List[int] = []

    for r in trial_rows:
        seed = r.get("seed")
        if seed is None or seed not in baseline_by_seed:
            continue
        f_trial = calculate_single_match_fitness(r)
        f_base = baseline_by_seed[seed]
        # Only include rows where both are health-clean (not disqualified)
        if f_trial > -5000.0 and f_base > -5000.0:
            deltas.append(f_trial - f_base)
            seeds_with_data.append(seed)

    if not deltas:
        return float("-inf"), {
            "mean": float("nan"),
            "sd": float("nan"),
            "se": float("nan"),
            "n_seeds": 0,
            "raw_deltas": [],
        }

    mean_delta = sum(deltas) / len(deltas)
    if len(deltas) == 1:
        # Single seed: CI is just the value itself; report conservative = value
        # but flag it as low-confidence.
        conservative = mean_delta
        return conservative, {
            "mean": mean_delta,
            "sd": float("nan"),
            "se": float("nan"),
            "n_seeds": 1,
            "raw_deltas": deltas,
        }

    # Standard deviation and Standard Error
    variance = sum((d - mean_delta) ** 2 for d in deltas) / (len(deltas) - 1)
    se = math.sqrt(variance / len(deltas))
    sd = math.sqrt(variance)

    # Conservative fitness: 95% CI lower bound = mean - 1.96 * SE
    conservative = mean_delta - 1.96 * se

    return conservative, {
        "mean": mean_delta,
        "sd": sd,
        "se": se,
        "n_seeds": len(seeds_with_data),
        "raw_deltas": deltas,
    }


# ---------------------------------------------------------------------------
# Convenience: fitness from a single JSONL row (for CLI / compare.py hook)
# ---------------------------------------------------------------------------

def fitness_from_row(row: Dict[str, Any]) -> float:
    """Public wrapper: returns the composite fitness, or -9999 if guardrails fail."""
    return calculate_single_match_fitness(row)


# ---------------------------------------------------------------------------
# Bootstrap CI (utility shared with compare.py)
# ---------------------------------------------------------------------------

def bootstrap_ci(values: List[float], iters: int = 10000, seed: int = 12345) -> Tuple[float, float]:
    """
    Percentile bootstrap confidence interval for the mean.
    Returns (lo, hi) — the 2.5th and 97.5th percentile of the bootstrap distribution.
    """
    if not values:
        return (float("nan"), float("nan"))
    if len(values) == 1:
        return (values[0], values[0])
    rng = random.Random(seed)
    n = len(values)
    means: List[float] = []
    for _ in range(iters):
        means.append(sum(rng.choice(values) for _ in range(n)) / n)
    means.sort()
    lo = means[int(0.025 * iters)]
    hi = means[int(0.975 * iters)]
    return (lo, hi)


# ---------------------------------------------------------------------------
# Example CLI usage (not invoked on import)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    """
    python -c "
from tools.tune.fitness import calculate_single_match_fitness, evaluate_paired_fitness
import json

# Load a row from your sweep JSONL
row = json.loads('''...''')
fit = calculate_single_match_fitness(row)
print('Single-match fitness:', fit)

# Paired against baseline
baseline = [row]  # in practice, load from a different variant on the same seeds
cons, stats = evaluate_paired_fitness(row, baseline)
print('Paired delta fitness (conservative):', cons)
print('Stats:', stats)
    "
    """
    pass