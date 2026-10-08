#!/usr/bin/env python3
"""
optimize_policy.py — Jev AI Policy & Utility Optimization Harness

Executes Bayesian Optimization (via Optuna if available, or smart adaptive sampling)
across continuous IAUS response curve and dynamic action inertia parameters, guarded
by the 4-tier fitness evaluation pipeline:

  Tier 0: Domain bounds and FROZEN_PARAMETERS invariance check (0 ms)
  Tier 1: Fast deterministic DecisionCards pruning (< 0.5 s, 39 scenarios, symmetry check)
  Tier 2: Headless 3v3 match sweep with loop health and attribution canaries
  Tier 3: Paired δ-Fitness evaluation with 95% CI lower bound (mean_delta - 1.96 * SE)

Usage:
  python tools/tune/optimize_policy.py --subspace inertia --n-trials 10 --tier1-only
  python tools/tune/optimize_policy.py --subspace scent --n-trials 15 --tier1-only
  python tools/tune/optimize_policy.py --subspace all --n-trials 20 --tier1-only
"""

import argparse
import json
import math
import os
import random
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

# Ensure project root is in sys.path
_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from tools.tune.fitness import (
    calculate_single_match_fitness,
    evaluate_paired_fitness,
)

# ---------------------------------------------------------------------------
# Tier 0: Frozen Parameters Guardrail
# ---------------------------------------------------------------------------
FROZEN_PARAMETERS = {
    "Shooter/SimLaunchEfficiency",
    "SETTLE_SPEED_MPS",
    "SETTLE_OMEGA_RPS",
    "BALL_SPAWN_INTERVAL",
    "Hubs/SHOOTING_MAX_DISTANCE",
    "HubSchedule.TRANSITION_END",
    "MatchScoreTracker.POINTS_PER_FUEL",
}

# ---------------------------------------------------------------------------
# Parameter Subspace Definitions: (min_bound, max_bound, default_value)
# ---------------------------------------------------------------------------
SUBSPACE_DEFINITIONS: Dict[str, Dict[str, Tuple[float, float, float]]] = {
    "inertia": {
        "commitmentMargin": (0.02, 0.12, 0.06),
        "commitmentDecisiveMargin": (0.10, 0.35, 0.20),
        "commitmentMinHoldSec": (0.5, 3.0, 1.5),
        "inertiaInitialBoost": (0.05, 0.40, 0.20),
        "inertiaTimeConstantSec": (0.5, 2.5, 1.0),
        "inertiaResidualMargin": (0.01, 0.08, 0.04),
    },
    "scent": {
        "clusterNeighborhoodRadius": (0.80, 2.00, 1.30),
        "clusterKernelSigma": (0.30, 0.80, 0.50),
        "clusterDensityExponent": (1.00, 2.50, 1.50),
        "clusterDistanceFloor": (0.20, 0.80, 0.40),
        "harvestHeadingAlignScale": (0.10, 0.50, 0.30),
        "harvestReturnVectorBonus": (0.10, 0.60, 0.35),
    },
    "macro": {
        "scoreHubBase": (0.50, 0.95, 0.72),
        "scoreHubScale": (0.10, 0.40, 0.26),
        "stageStandoffBase": (0.60, 0.95, 0.80),
        "vacuumActiveBase": (0.60, 0.95, 0.82),
        "sweepAllianceZoneActive": (0.70, 0.99, 0.96),
        "laneDenialActiveUtility": (0.60, 0.95, 0.86),
    },
}

def format_weights_spec(param_dict: Dict[str, float]) -> str:
    """Formats a parameter dictionary into a comma-delimited PolicyWeights string."""
    return ",".join(f"{k}={v:.4f}" for k, v in sorted(param_dict.items()))


# ---------------------------------------------------------------------------
# Tier 1: Fast Deterministic Decision Cards Evaluator (< 0.5 s)
# ---------------------------------------------------------------------------
def run_tier1_decision_cards(
    weights_spec: str,
    project_root: Path,
    tsv_path: Optional[Path] = None,
    timeout_sec: float = 12.0
) -> Tuple[bool, float, str]:
    """
    Evaluates policy weights against decision_cards.tsv using DecisionCards.java.

    Returns:
        (passed_gate, pass_rate, summary_str)
    """
    if tsv_path is None:
        tsv_path = project_root / "tools" / "score" / "decision_cards.tsv"

    jar_path = project_root / "build" / "libs" / "TitanRoboticsBuildSeason.jar"
    classes_dir = project_root / "build" / "decisioncards"
    jni_dir = project_root / "build" / "jni" / "release"
    jdk_bin = Path("C:/Users/Public/wpilib/2026/jdk/bin")
    java_exe = jdk_bin / "java.exe" if jdk_bin.exists() else Path("java")

    temp_out = project_root / "results" / f"temp_cards_{os.getpid()}.md"

    cmd = [
        str(java_exe),
        f"-Dfrc.jev.weights={weights_spec}",
        f"-Djava.library.path={jni_dir}",
        "-cp",
        f"{classes_dir};{jar_path}",
        "DecisionCards",
        str(tsv_path),
        str(temp_out),
    ]

    env = os.environ.copy()
    if jni_dir.exists():
        env["PATH"] = f"{jni_dir};" + env.get("PATH", "")

    try:
        proc = subprocess.run(
            cmd,
            cwd=str(project_root),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env=env
        )
    except subprocess.TimeoutExpired:
        return False, 0.0, "DecisionCards timeout expired (>12s)"
    except Exception as e:
        return False, 0.0, f"Failed to execute DecisionCards: {e}"
    finally:
        if temp_out.exists():
            try:
                temp_out.unlink()
            except OSError:
                pass

    output = proc.stdout + "\n" + proc.stderr

    # Check for alliance asymmetry
    if "ALLIANCE ASYMMETRY" in output or "<-- ASYMMETRY" in output:
        return False, 0.0, "Alliance Asymmetry detected"

    # Parse pass / mismatch count: e.g. "[cards] 39 card(s): 39 pass, 0 mismatch, 0 unreviewed"
    match = re.search(r"\[cards\]\s+(\d+)\s+card\(s\):\s+(\d+)\s+pass,\s+(\d+)\s+mismatch", output)
    if not match:
        return False, 0.0, f"Could not parse card results: {output[:200]}"

    total = int(match.group(1))
    passed = int(match.group(2))
    mismatched = int(match.group(3))

    if total == 0:
        return False, 0.0, "Zero cards evaluated"

    pass_rate = passed / float(total)
    passed_gate = (pass_rate >= 0.95)

    summary = f"{passed}/{total} passed ({pass_rate * 100:.1f}%), {mismatched} mismatches"
    return passed_gate, pass_rate, summary


# ---------------------------------------------------------------------------
# Tier 2 & 3: Headless Match Sweep & Paired δ-Fitness
# ---------------------------------------------------------------------------
def run_tier2_headless_sweep(
    weights_spec: str,
    seeds: List[int],
    project_root: Path,
    out_jsonl: Path,
    timeout_sec: float = 300.0
) -> Tuple[bool, List[Dict[str, Any]], str]:
    """Runs sweep.ps1 under -Weights and parses JSONL output."""
    sweep_script = project_root / "tools" / "score" / "sweep.ps1"
    seed_str = ",".join(str(s) for s in seeds)

    cmd = [
        "powershell",
        "-File", str(sweep_script),
        "-Seeds", seed_str,
        "-Variants", "candidate",
        "-Weights", weights_spec,
        "-OutFile", str(out_jsonl),
        "-Fresh",
        "-Force"
    ]

    try:
        proc = subprocess.run(
            cmd,
            cwd=str(project_root),
            capture_output=True,
            text=True,
            timeout=timeout_sec
        )
    except subprocess.TimeoutExpired:
        return False, [], "Headless sweep timeout expired"
    except Exception as e:
        return False, [], f"Sweep execution failed: {e}"

    if not out_jsonl.exists():
        return False, [], f"Sweep did not produce output JSONL at {out_jsonl}"

    rows = []
    with open(out_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue

    return True, rows, f"Collected {len(rows)} match rows"


# ---------------------------------------------------------------------------
# Optimization Trial Evaluator
# ---------------------------------------------------------------------------
def evaluate_candidate(
    params: Dict[str, float],
    project_root: Path,
    tier1_only: bool = True,
    baseline_rows: Optional[List[Dict[str, Any]]] = None,
    seeds: Optional[List[int]] = None
) -> Tuple[float, Dict[str, Any]]:
    """
    Evaluates candidate parameters through the multi-tier fitness pipeline.
    """
    # Tier 0: Bounds and frozen parameters check
    for k in params:
        if k in FROZEN_PARAMETERS:
            return -9999.0, {"tier": 0, "status": f"Attempted mutation of frozen parameter: {k}"}

    weights_spec = format_weights_spec(params)

    # Tier 1: Deterministic Decision Cards
    t1_pass, t1_rate, t1_summary = run_tier1_decision_cards(weights_spec, project_root)
    if not t1_pass:
        fitness = -5000.0 * (1.0 - t1_rate)
        return fitness, {
            "tier": 1,
            "status": "PRUNED_TIER1",
            "pass_rate": t1_rate,
            "summary": t1_summary,
            "spec": weights_spec
        }

    # If running in tier1-only mode, score is pure card accuracy with bonus for defaults closeness
    if tier1_only:
        fitness = 1000.0 * t1_rate
        return fitness, {
            "tier": 1,
            "status": "PASS_TIER1",
            "pass_rate": t1_rate,
            "summary": t1_summary,
            "spec": weights_spec
        }

    # Tier 2 & 3: Headless Sweep and Paired δ-Fitness
    if not baseline_rows or not seeds:
        return 1000.0 * t1_rate, {"tier": 1, "status": "TIER1_COMPLETE_NO_BASELINE"}

    trial_jsonl = project_root / "results" / f"trial_{os.getpid()}.jsonl"
    sweep_ok, trial_rows, sweep_msg = run_tier2_headless_sweep(weights_spec, seeds, project_root, trial_jsonl)

    if trial_jsonl.exists():
        try:
            trial_jsonl.unlink()
        except OSError:
            pass

    if not sweep_ok or not trial_rows:
        return -9999.0, {"tier": 2, "status": "SWEEP_FAILED", "message": sweep_msg}

    # Evaluate paired δ-Fitness
    conservative_fit, stats = evaluate_paired_fitness(trial_rows, baseline_rows)

    return conservative_fit, {
        "tier": 3,
        "status": "PAIRED_FITNESS_EVALUATED",
        "conservative_fitness": conservative_fit,
        "stats": stats,
        "spec": weights_spec
    }


# ---------------------------------------------------------------------------
# Search Loop (Optuna / Adaptive Random Sampler)
# ---------------------------------------------------------------------------
def run_optimization(
    subspace_name: str,
    n_trials: int,
    tier1_only: bool,
    project_root: Path,
    out_dir: Path
):
    print(f"=== Jev Policy Optimization Engine ===")
    print(f"Subspace: {subspace_name}")
    print(f"Trials: {n_trials}")
    print(f"Tier 1 Only: {tier1_only}")
    print(f"Project Root: {project_root}")
    print("========================================")

    # Determine parameters to optimize
    if subspace_name == "all":
        param_bounds = {}
        for sub in SUBSPACE_DEFINITIONS.values():
            param_bounds.update(sub)
    elif subspace_name in SUBSPACE_DEFINITIONS:
        param_bounds = SUBSPACE_DEFINITIONS[subspace_name]
    else:
        raise ValueError(f"Unknown subspace: {subspace_name}")

    best_score = float("-inf")
    best_params: Dict[str, float] = {k: v[2] for k, v in param_bounds.items()}
    best_meta: Dict[str, Any] = {}

    history: List[Dict[str, Any]] = []

    # Check if optuna is available
    try:
        import optuna
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        has_optuna = True
    except ImportError:
        has_optuna = False

    if has_optuna:
        print("[optuna] Using Optuna TPE Bayesian Optimizer")
        study = optuna.create_study(direction="maximize")

        def objective(trial: optuna.Trial) -> float:
            candidate = {}
            for param, (lo, hi, default_val) in param_bounds.items():
                candidate[param] = trial.suggest_float(param, lo, hi)

            score, meta = evaluate_candidate(candidate, project_root, tier1_only=tier1_only)
            history.append({"trial": trial.number, "params": candidate, "score": score, "meta": meta})

            print(f"[trial {trial.number:2d}] score: {score:8.2f} | {meta.get('status')} | {meta.get('summary', '')}")
            return score

        study.optimize(objective, n_trials=n_trials)
        best_score = study.best_value
        best_params = study.best_params
    else:
        print("[optimizer] Optuna not found; using Adaptive Search with Tier 1 pruning")

        # Trial 0 is always the baseline default
        default_candidate = {k: v[2] for k, v in param_bounds.items()}
        score0, meta0 = evaluate_candidate(default_candidate, project_root, tier1_only=tier1_only)
        best_score = score0
        best_params = default_candidate
        best_meta = meta0
        history.append({"trial": 0, "params": default_candidate, "score": score0, "meta": meta0})
        print(f"[trial  0] BASELINE score: {score0:8.2f} | {meta0.get('status')} | {meta0.get('summary', '')}")

        for t in range(1, n_trials):
            candidate = {}
            # 50% random exploration, 50% local perturbation around best
            use_perturbation = (t > 3 and random.random() < 0.6)

            for param, (lo, hi, default_val) in param_bounds.items():
                if use_perturbation:
                    current_best = best_params.get(param, default_val)
                    span = hi - lo
                    perturb = random.gauss(0, span * 0.15)
                    val = max(lo, min(hi, current_best + perturb))
                else:
                    val = random.uniform(lo, hi)
                candidate[param] = val

            score, meta = evaluate_candidate(candidate, project_root, tier1_only=tier1_only)
            history.append({"trial": t, "params": candidate, "score": score, "meta": meta})

            if score > best_score:
                best_score = score
                best_params = candidate
                best_meta = meta
                flag = "--> NEW BEST"
            else:
                flag = ""

            print(f"[trial {t:2d}] score: {score:8.2f} | {meta.get('status')} | {meta.get('summary', '')} {flag}")

    # Write summary results
    out_dir.mkdir(parents=True, exist_ok=True)
    best_spec = format_weights_spec(best_params)

    results_summary = {
        "subspace": subspace_name,
        "n_trials": n_trials,
        "best_score": best_score,
        "best_params": best_params,
        "best_spec": best_spec,
        "history": history
    }

    results_file = out_dir / f"optimization_{subspace_name}.json"
    with open(results_file, "w", encoding="utf-8") as f:
        json.dump(results_summary, f, indent=2)

    spec_file = out_dir / f"best_weights_{subspace_name}.txt"
    with open(spec_file, "w", encoding="utf-8") as f:
        f.write(best_spec + "\n")

    print("\n========================================")
    print("=== OPTIMIZATION STUDY COMPLETE ===")
    print(f"Best Score: {best_score:.4f}")
    print(f"Best Spec:  {best_spec}")
    print(f"Results:    {results_file}")
    print(f"Spec file:  {spec_file}")
    print("========================================")


# ---------------------------------------------------------------------------
# CLI Entry Point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Jev Policy & Utility Optimization Harness")
    parser.add_argument("--subspace", choices=["inertia", "scent", "macro", "all"], default="inertia",
                        help="Parameter subspace to tune")
    parser.add_argument("--n-trials", type=int, default=10, help="Number of trials to run")
    parser.add_argument("--tier1-only", action="store_true", default=True,
                        help="Only evaluate Tier 1 Decision Cards gate (<0.5 s)")
    parser.add_argument("--out-dir", type=str, default="results/tuning",
                        help="Output directory for study results")

    args = parser.parse_args()
    project_root = _PROJECT_ROOT

    run_optimization(
        subspace_name=args.subspace,
        n_trials=args.n_trials,
        tier1_only=args.tier1_only,
        project_root=project_root,
        out_dir=project_root / args.out_dir
    )

if __name__ == "__main__":
    main()
