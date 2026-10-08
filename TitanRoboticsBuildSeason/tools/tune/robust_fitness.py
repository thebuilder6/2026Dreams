"""
robust_fitness.py — Variance-robust match fitness for policy tuning (optimizer T3).

Why this exists alongside fitness.py
------------------------------------
``SCORE_RIG_RESULTS.md`` measured raw ``blueTotal`` as unusable for tuning:
``SE = 53.4`` on a mean of ~180 (95% CI **±105**), a bimodal 36-119 vs 191-335
split, and a discrete Blue-collapse mode (same seed: 7 vs 240). The classic
``fitness.py`` averages *absolute* fuel deltas, so one collapsed replica
(Δ = -233) swamps eight seeds. This module instead:

  1. **Censors** discrete failure modes (collapse signature, dirty/contaminated
     rows) instead of averaging them in. Censored pairs are counted and
     reported, never differenced.
  2. Scores **efficiency** (teleop fuel per path metre) plus waste / motion /
     foul terms. Efficiency varies far less seed-to-seed than absolute fuel
     because seed difficulty (how much fuel the field offers) cancels out.
  3. Pairs per seed and takes the **median** delta with a bootstrap CI;
     replica-aligned (optimizer sweeps share replica numbering; archive
     cross-replica analysis zips by replica order).
  4. **Refuses** to certify on fewer than ``MIN_PAIRED_SEEDS`` clean pairs.

Status: experimental candidate. ``fitness.py`` stays authoritative and is still
the optimizer default; opt in here via ``--fitness robust``. Do not delete the
classic until this one has certified a real policy change on a clean-tree sweep.

 compact
Row statuses: CLEAN (scored), COLLAPSED (discrete failure mode, censored),
CONTAMINATED (health/canary/provenance failure, censored).
"""

from typing import Dict, Any, List, Tuple
import math
import statistics

try:
    from tools.tune.fitness import (
        bootstrap_ci,
        GATE_MAX_LOOP_OVERRUNS as _OVERRUNS,
        GATE_MAX_ROBOT_PERIODIC_MS as _MAX_MS,
    )
except ImportError:  # pragma: no cover - fallback when imported without project root
    _OVERRUNS = 8
    _MAX_MS = 60.0

    def bootstrap_ci(values, iters=10000, seed=12345):  # type: ignore[misc]
        import random

        if not values:
            return (float("nan"), float("nan"))
        if len(values) == 1:
            return (values[0], values[0])
        rng = random.Random(seed)
        n = len(values)
        means = sorted(sum(rng.choice(values) for _ in range(n)) / n for _ in range(iters))
        return (means[int(0.025 * iters)], means[int(0.975 * iters)])


# ---------------------------------------------------------------------------
# Gates — mirrored from fitness.py / compare.py / sweep.ps1
# ---------------------------------------------------------------------------
GATE_MAX_LOOP_OVERRUNS = _OVERRUNS
GATE_MAX_ROBOT_PERIODIC_MS = _MAX_MS

# Collapse signature (SCORE_RIG_RESULTS §3): teleop output craters while ≥2 Blue
# bots barely move (13.7 m / 10.0 m vs ~475 m healthy over 150 s). The 107-fuel
# marginal (seed 2026 r1, still a 95-point replica split with the same holding
# signature) intentionally falls inside the fuel bound: same mode, censor it too.
COLLAPSE_MAX_TELEOP_FUEL = 120
COLLAPSE_MIN_STATIONARY_BOTS = 2
COLLAPSE_PATH_M = 25.0

# Refuse-to-certify floor. SWEEP_FINDINGS detection-cost table: even a 5%
# effect needs hundreds of matches; below this many clean pairs the CI is
# theatre, so return -inf with a LOW_N reason instead of a number.
MIN_PAIRED_SEEDS = 4

CLEAN = "CLEAN"
COLLAPSED = "COLLAPSED"
CONTAMINATED = "CONTAMINATED"


# ---------------------------------------------------------------------------
# Row classification
# ---------------------------------------------------------------------------
def _bot_paths(row: Dict[str, Any], side: str) -> List[float]:
    bots = row.get(side + "Bots", {}) or {}
    paths = bots.get("pathLengthM", []) or []
    return [float(p) for p in paths]


def classify_row(row: Dict[str, Any], strict_provenance: bool = True) -> Tuple[str, str]:
    """Classify one JSONL row as CLEAN / COLLAPSED / CONTAMINATED.

    Returns (status, reason). Only CLEAN rows are scored; the rest are
    censored (counted in stats, never differenced).

    strict_provenance=True (default) censors rows predating the 'dirty'
    field, matching compare.py's treat-as-dirty-until-rerun stance. Pass
    False only for explicitly non-certifiable archive analysis — the stats
    dict records the relaxation.
    """
    if row.get("schemaVersion") != 2:
        return CONTAMINATED, "schemaVersion != 2"
    dirty = row.get("dirty", -1)
    if dirty == 1:
        return CONTAMINATED, "dirty tree"
    if dirty != 0 and strict_provenance:
        # Missing 'dirty' (-1 default) = provenance unknown: compare.py treats
        # those as dirty-until-rerun, and so do we.
        return CONTAMINATED, "dirty tree or unknown provenance"
    h = row.get("health")
    if not isinstance(h, dict):
        return CONTAMINATED, "no health block"
    overruns = h.get("loopOverruns")
    max_ms = h.get("maxRobotPeriodicMs")
    if overruns is None or max_ms is None:
        return CONTAMINATED, "incomplete health block"
    if overruns < 0 or max_ms < 0:
        return CONTAMINATED, "health unmeasured (-1)"
    if overruns > GATE_MAX_LOOP_OVERRUNS:
        return CONTAMINATED, "CPU-starved worker"
    if max_ms > GATE_MAX_ROBOT_PERIODIC_MS:
        return CONTAMINATED, "load perturbation"
    for who in ("blue", "red"):
        if row.get(who + "ReconciliationResidual", 0) not in (0, None):
            return CONTAMINATED, who + " attribution leak"
        if row.get(who + "UnattributedFuel", 0) not in (0, None):
            return CONTAMINATED, who + " unattributed fuel"

    # Discrete collapse mode: cratered teleop + stationary Blue bots. Checked
    # Blue-side only by measurement (the rig found it Blue-specific); a Red
    # collapse would show up as an anomalous *positive* delta and is left to
    # the median + bootstrap to absorb, with the sign-rate as the tell.
    blue_tele = row.get("blueTeleopFuel", 0) or 0
    stationary = sum(1 for p in _bot_paths(row, "blue") if p < COLLAPSE_PATH_M)
    if blue_tele < COLLAPSE_MAX_TELEOP_FUEL and stationary >= COLLAPSE_MIN_STATIONARY_BOTS:
        return COLLAPSED, "teleop=%d stationary=%d" % (blue_tele, stationary)
    return CLEAN, "ok"


# ---------------------------------------------------------------------------
# Single-row robust score (efficiency, not absolute fuel)
# ---------------------------------------------------------------------------
def robust_single_score(row: Dict[str, Any], strict_provenance: bool = True) -> float:
    """Efficiency-based score for one CLEAN row. Returns -9999.0 otherwise.

    Scale guide: healthy teleop ~200 fuel over ~1350 Blue path-metres gives
    ~14.8 efficiency points; deltas between policies are typically single
    digits. That is the point — seed difficulty cancels instead of dominating.
    """
    status, _ = classify_row(row, strict_provenance)
    if status != CLEAN:
        return -9999.0

    blue_tele = float(row.get("blueTeleopFuel", 0) or 0)
    red_tele = float(row.get("redTeleopFuel", 0) or 0)
    blue_path = sum(_bot_paths(row, "blue")) or 1.0
    red_path = sum(_bot_paths(row, "red")) or 1.0

    # Component E: scoring efficiency per 100 path-metres. Teleop only: AUTO
    # fuel is scripted and identical across policies (rig §3: AUTO normal even
    # in collapses), so including it only adds seed noise.
    blue_eff = 100.0 * blue_tele / max(blue_path, 1.0)
    red_eff = 100.0 * red_tele / max(red_path, 1.0)
    score = blue_eff - 0.5 * red_eff

    # Component W: waste as a *rate* (absolute waste counts inherit seed scale).
    total_handled = blue_tele + float(row.get("blueWastedFuel", 0) or 0)
    if total_handled > 0:
        score -= 10.0 * float(row.get("blueWastedFuel", 0) or 0) / total_handled

    # Component M: motion health on Blue bots (same shapes as classic fitness:
    # quadratic stall past 1.5 s, recovery thrash, never-moved penalty).
    # CAPPED: one 38 s stall is (38-1.5)^2 ≈ 1332 points uncapped — a single
    # event would decide the whole sweep. The cap keeps motion decisive
    # (30 ≈ 2x a healthy efficiency score) without letting it swamp policy.
    MOTION_CAP = 30.0
    m_motion = 0.0
    bots = row.get("blueBots", {}) or {}
    stalls = bots.get("maxContiguousStallSec", []) or []
    consecs = bots.get("maxConsecutiveRecoveries", []) or []
    paths = bots.get("pathLengthM", []) or []
    for i in range(max(len(stalls), len(consecs), len(paths))):
        stall = float(stalls[i]) if i < len(stalls) else 0.0
        consec = int(consecs[i]) if i < len(consecs) else 0
        path = float(paths[i]) if i < len(paths) else 200.0
        if stall > 1.5:
            m_motion += (stall - 1.5) ** 2
        m_motion += 1.5 * consec
        if path < 25.0:
            m_motion += (25.0 - path) * 0.5
    score -= min(m_motion, MOTION_CAP)

    # Component F: rule discipline (Red's penalty receipts = our fouls).
    score -= 1.0 * float(row.get("redPenaltyPoints", 0) or 0)
    return score


# ---------------------------------------------------------------------------
# Paired robust evaluation across identical seeds (replica-aligned)
# ---------------------------------------------------------------------------
def _seed_groups(rows: List[Dict[str, Any]]) -> Dict[Any, List[Dict[str, Any]]]:
    groups: Dict[Any, List[Dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault(r.get("seed"), []).append(r)
    for g in groups.values():
        g.sort(key=lambda r: r.get("replica", 0))
    return groups


def evaluate_paired_robust(
    trial_rows: List[Dict[str, Any]],
    baseline_rows: List[Dict[str, Any]],
    min_seeds: int = MIN_PAIRED_SEEDS,
    strict_provenance: bool = True,
) -> Tuple[float, Dict[str, Any]]:
    """Median paired-delta fitness with bootstrap-CI lower bound.

    Returns (conservative, stats). conservative = bootstrap lower bound of the
    median delta — the number to maximize. -inf with a reason when there is
    nothing certifiable (no clean pairs, or fewer than min_seeds).
    """
    # Pair per seed, replica-aligned: optimizer sweeps number replicas from 0
    # on both sides so pairs are (seed, replica); archive analysis (e.g.
    # replica-0 vs replica-1 null experiments) pairs zip by replica order.
    trial_groups = _seed_groups(trial_rows)
    base_groups = _seed_groups(baseline_rows)

    deltas: List[float] = []
    pairs: List[Tuple[Any, Any, Any]] = []
    n_collapsed = 0
    n_contaminated = 0
    for seed, trows in trial_groups.items():
        brows = base_groups.get(seed)
        if not brows:
            continue
        for r, b in zip(trows, brows):
            key = (seed, r.get("replica", 0), b.get("replica", 0))
            st, _ = classify_row(r, strict_provenance)
            sb, _ = classify_row(b, strict_provenance)
            if st != CLEAN or sb != CLEAN:
                if COLLAPSED in (st, sb):
                    n_collapsed += 1
                else:
                    n_contaminated += 1
                continue
            deltas.append(robust_single_score(r, strict_provenance)
                          - robust_single_score(b, strict_provenance))
            pairs.append(key)

    stats: Dict[str, Any] = {
        "n_pairs": len(deltas),
        "n_censored_collapsed": n_collapsed,
        "n_censored_contaminated": n_contaminated,
        "provenance_relaxed": not strict_provenance,
        "pairs": pairs,
        "raw_deltas": deltas,
    }
    if not deltas:
        stats["reason"] = "NO_CLEAN_PAIRS"
        return float("-inf"), stats
    if len(deltas) < min_seeds:
        stats["reason"] = "LOW_N"
        stats["median"] = float(statistics.median(deltas))
        return float("-inf"), stats

    median_delta = float(statistics.median(deltas))
    mean_delta = sum(deltas) / len(deltas)
    lo, hi = bootstrap_ci(deltas)
    sign_rate = sum(1 for d in deltas if d > 0) / len(deltas)
    stats.update(
        {
            "median": median_delta,
            "mean": mean_delta,
            "bootstrap_lo": lo,
            "bootstrap_hi": hi,
            "sign_rate": sign_rate,
            "reason": "OK",
        }
    )
    return lo, stats
