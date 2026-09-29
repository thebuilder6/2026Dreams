#!/usr/bin/env python3
"""
Compare headless 3v3 match variants from a score-rig JSONL sweep.

    python tools/score/compare.py --results results/sweep.jsonl
    python tools/score/compare.py --results results/sweep.jsonl --baseline baseline
    python tools/score/compare.py --results results/noise.jsonl --noise-floor

Design notes that matter more than the arithmetic
-------------------------------------------------

**Read the objective correctly.** ``blueTotal`` is ``getBlueTotalScore()`` =
fuel + climb*10 + penalty points received. In the headless 3v3 two of those
three terms are structurally ZERO:

  * Penalties - all four referee rules are unreachable. G418 pinning measures
    the real player vs Bot 0, and the player is parked off-field; G407 tests
    ``inAllianceZone``, the same gate bots already obey before firing; G201 is
    prevented by the auto half-field restriction; G420 needs a robot at a tower.
  * Climb - ``RUSH_CLIMB`` requires ``archetype == CO_PILOT`` and the 3v3 roster
    has no CO_PILOT, plus ``updateClimbEvaluation`` zeroes climbs whenever
    ``20 < matchTime <= 150``.

So a rising blueTotal currently means exactly "Blue scored more fuel". This tool
therefore requires a win to be **fuel-driven** and reports the decomposition, so
a gain can never be quietly attributed to penalty points that Red conceded.

**Guardrails are role-aware.** A defender is *designed* not to score:
``JevDecisionEngine`` zeroes ``scoreUtility`` for TACTICAL_DEFENDER and
DEFENSE_BULLY. A fuel-share gate applied to them is meaningless - in the
observed data the defender median fuel share is *higher* than the adaptive
bot's. Defenders are gated on travelled distance instead, and the archetype used
is the one reported by the sim (the declared, live archetype), never inferred
from scoring.

**Reconciliation canaries.** Every row carries residuals and unattributed-fuel
counters. A non-zero value means a scoring path is bypassing per-slot
attribution, which would silently corrupt every share computed here.

**FROZEN - never sweep these.** They raise the scoreboard without making the
robot better. Enforcement lands with the P1 PolicyWeights parser (unknown keys
are hard errors there); this list is the single place it is written down.
"""

import argparse
import json
import math
import random
import statistics
import sys
from collections import defaultdict

SCHEMA_VERSION = 1

# Parameters that must never be swept. Sim-physics or rulebook constants: tuning
# them inflates the objective without improving the robot. See module docstring.
FROZEN_PARAMETERS = {
    "Shooter/SimLaunchEfficiency",     # ShooterConstants.java:70, default 0.42
    "SETTLE_SPEED_MPS",                # AIRobotInstance.java:548, 0.80
    "SETTLE_OMEGA_RPS",                # AIRobotInstance.java:549, 1.00
    "BALL_SPAWN_INTERVAL",             # ShooterConstants, 0.12 s
    "Hubs/SHOOTING_MAX_DISTANCE",      # FieldMap, 4.20 m
    "HubSchedule.TRANSITION_END",      # shift boundaries
    "MatchScoreTracker.POINTS_PER_FUEL",
}

# Role -> gate kind. Mirrors the role table in docs; keep the two in step.
OFFENSIVE_ROLES = {"AUTONOMOUS_CYCLER", "CO_PILOT"}
DEFENSIVE_ROLES = {"TACTICAL_DEFENDER", "DEFENSE_BULLY"}
ADAPTIVE_ROLES = {"ADAPTIVE_COMPETITOR", "LEAD_PURSUIT_INTERCEPTOR"}

# Thresholds. Derived from 20 archived full-match reports, then re-derived from
# the P0.5 baseline once per-slot attribution was fixed. Provisional.
GATE_CYCLER_SHARE = 0.15
GATE_ADAPTIVE_SHARE = 0.10
GATE_DEFENSIVE_PATH_M = 25.0
GATE_ADAPTIVE_PATH_M = 25.0
GATE_MAX_STALL_SEC = 6.0
GATE_MAX_CONSEC_RECOVERIES = 3
WASTED_FUEL_TOLERANCE = 0.05


def load(path):
    rows, bad = [], []
    with open(path, "r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                bad.append("line %d: %s" % (lineno, exc))
                continue
            if obj.get("schemaVersion") != SCHEMA_VERSION:
                bad.append("line %d: schemaVersion %r, expected %d"
                           % (lineno, obj.get("schemaVersion"), SCHEMA_VERSION))
                continue
            rows.append(obj)
    return rows, bad


def all_bots(row):
    """Yield (alliance, index, archetype, fuel, metrics dict) for all 6 bots."""
    for side, alliance in (("redBots", "Red"), ("blueBots", "Blue")):
        fuels = row.get("botFuelScored" if alliance == "Red" else "allyFuelScored") or []
        block = row.get(side) or {}
        arch = block.get("archetype") or []
        for i, fuel in enumerate(fuels):
            yield {
                "alliance": alliance,
                "index": i,
                "archetype": arch[i] if i < len(arch) else "UNKNOWN",
                "fuel": fuel,
                "path_m": (block.get("pathLengthM") or [0.0] * 99)[i]
                if i < len(block.get("pathLengthM") or []) else 0.0,
                "stall_s": (block.get("maxContiguousStallSec") or [0.0] * 99)[i]
                if i < len(block.get("maxContiguousStallSec") or []) else 0.0,
                "consec_rec": (block.get("maxConsecutiveRecoveries") or [0] * 99)[i]
                if i < len(block.get("maxConsecutiveRecoveries") or []) else 0,
            }


def check_canaries(row):
    """Attribution leaks invalidate every share in this tool."""
    out = []
    for who in ("Blue", "Red"):
        resid = row.get("%sReconciliationResidual" % who.lower())
        unattr = row.get("%sUnattributedFuel" % who.lower())
        if resid not in (0, None):
            out.append("%s reconciliation residual = %s (per-bot sum != alliance total)" % (who, resid))
        if unattr not in (0, None):
            out.append("%s unattributed fuel = %s (a scoring path bypassed attribution)" % (who, unattr))
    return out


def check_guardrails(row, baseline=None):
    """Return a list of human-readable guardrail violations for one match."""
    bad = []
    for b in all_bots(row):
        alliance_fuel = row.get("%sFuel" % b["alliance"].lower(), 0) or 0
        share = (b["fuel"] / alliance_fuel) if alliance_fuel > 0 else 0.0
        who = "%s bot%d (%s)" % (b["alliance"], b["index"], b["archetype"])
        role = b["archetype"]

        if b["stall_s"] > GATE_MAX_STALL_SEC:
            bad.append("%s contiguous stall %.1fs > %.1fs" % (who, b["stall_s"], GATE_MAX_STALL_SEC))
        if b["consec_rec"] > GATE_MAX_CONSEC_RECOVERIES:
            bad.append("%s %d consecutive recoveries in %0.0fs (max %d)"
                       % (who, b["consec_rec"], 20.0, GATE_MAX_CONSEC_RECOVERIES))

        if role in OFFENSIVE_ROLES:
            if alliance_fuel > 0 and share < GATE_CYCLER_SHARE:
                bad.append("%s fuel share %.1f%% < %.0f%%"
                           % (who, share * 100, GATE_CYCLER_SHARE * 100))
        elif role in DEFENSIVE_ROLES:
            # Fuel is meaningless for a defender; engagement is the signal.
            if b["path_m"] < GATE_DEFENSIVE_PATH_M:
                bad.append("%s travelled %.1fm < %.0fm (defender not engaging)"
                           % (who, b["path_m"], GATE_DEFENSIVE_PATH_M))
        elif role in ADAPTIVE_ROLES:
            if not (share >= GATE_ADAPTIVE_SHARE or b["path_m"] >= GATE_ADAPTIVE_PATH_M):
                bad.append("%s neither scored (%.1f%% < %.0f%%) nor travelled (%.1fm < %.0fm)"
                           % (who, share * 100, GATE_ADAPTIVE_SHARE * 100,
                              b["path_m"], GATE_ADAPTIVE_PATH_M))

    if baseline is not None:
        for who, key in (("Blue", "blueWastedFuel"), ("Red", "redWastedFuel")):
            now, base = row.get(key, 0) or 0, baseline.get(key, 0) or 0
            if base > 0 and now > base * (1 + WASTED_FUEL_TOLERANCE):
                bad.append("%s wasted fuel %d > baseline %d (+%.0f%%)"
                           % (who, now, base, WASTED_FUEL_TOLERANCE * 100))
        # Blue conceding penalty points is a real regression signal, and it is
        # the direction that matters: redPenaltyPoints is what BLUE conceded.
        if (row.get("redPenaltyPoints", 0) or 0) > (baseline.get("redPenaltyPoints", 0) or 0):
            bad.append("Blue conceded more penalty points than baseline")
        if (row.get("blueFouls", 0) or 0) > (baseline.get("blueFouls", 0) or 0):
            bad.append("Blue committed more fouls than baseline")
    return bad


def bootstrap_ci(values, iters=10000, seed=12345):
    """Percentile bootstrap CI for the mean. Deterministic given the seed."""
    if not values:
        return (float("nan"), float("nan"))
    if len(values) == 1:
        return (values[0], values[0])
    rng = random.Random(seed)
    n = len(values)
    means = []
    for _ in range(iters):
        means.append(sum(rng.choice(values) for _ in range(n)) / n)
    means.sort()
    lo = means[int(0.025 * iters)]
    hi = means[int(0.975 * iters)]
    return (lo, hi)


def noise_floor(rows, variant):
    """Paired run-to-run spread for one variant: how repeatable is a score?"""
    by = defaultdict(dict)
    for r in rows:
        if r.get("variant", "baseline") != variant:
            continue
        by[r["seed"]][r.get("replica", 0)] = r
    deltas, pairs = [], 0
    for seed, reps in sorted(by.items()):
        ks = sorted(reps)
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                a, b = reps[ks[i]], reps[ks[j]]
                deltas.append((a.get("blueTotal", 0) or 0) - (b.get("blueTotal", 0) or 0))
                pairs += 1
    if not deltas:
        return None
    sd = statistics.stdev(deltas) if len(deltas) > 1 else 0.0
    se = sd / math.sqrt(len(deltas)) if deltas else 0.0
    return {"pairs": pairs, "deltas": deltas, "sd": sd, "se": se,
            "median": statistics.median(deltas), "max_abs": max(abs(d) for d in deltas)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", required=True, help="JSONL from tools/score/sweep.ps1")
    ap.add_argument("--baseline", default="baseline", help="variant name to compare against")
    ap.add_argument("--objective", default="blueTotal",
                    help="metric to optimise (default blueTotal)")
    ap.add_argument("--noise-floor", action="store_true",
                    help="report run-to-run repeatability instead of comparing variants")
    ap.add_argument("--no-gate", action="store_true", help="report only; never exit non-zero")
    args = ap.parse_args()

    rows, bad = load(args.results)
    if bad:
        print("!! %d unusable row(s):" % len(bad))
        for b in bad[:10]:
            print("   " + b)
    if not rows:
        print("no usable rows in %s" % args.results)
        return 2

    by_variant = defaultdict(list)
    for r in rows:
        by_variant[r.get("variant", "baseline")].append(r)

    print("=" * 78)
    print("SCORE RIG  %s" % args.results)
    print("  %d row(s) across %d variant(s); schemaVersion %d"
          % (len(rows), len(by_variant), SCHEMA_VERSION))
    shas = sorted({(r.get("gitSha") or "?")[:7] for r in rows})
    print("  git sha(s): %s%s" % (", ".join(shas),
                                  "   <-- MIXED, rows are not comparable" if len(shas) > 1 else ""))
    print("  objective: %s  (currently == Blue fuel; climb and penalty terms are"
          % args.objective)
    print("              structurally zero in the headless 3v3 - see docstring)")
    print("=" * 78)

    # ---- canaries ------------------------------------------------------------
    canary_fail = 0
    for r in rows:
        problems = check_canaries(r)
        if problems:
            canary_fail += 1
            if canary_fail <= 5:
                print("!! attribution leak in seed %s/%s: %s"
                      % (r.get("seed"), r.get("variant"), "; ".join(problems)))
    if canary_fail:
        print("!! %d/%d rows have an attribution leak - shares below are unreliable"
              % (canary_fail, len(rows)))
    print("-" * 78)

    # ---- noise floor ---------------------------------------------------------
    if args.noise_floor:
        for v in sorted(by_variant):
            nf = noise_floor(rows, v)
            if not nf:
                print("variant %-14s only one replica per seed; need -Replicas 2+" % v)
                continue
            print("variant %s  (%d paired run-to-run comparisons)" % (v, nf["pairs"]))
            print("  delta(blueTotal) per repeated seed: %s" % nf["deltas"])
            print("  sd=%.2f  SE=%.2f  median=%+.1f  max|delta|=%.0f"
                  % (nf["sd"], nf["se"], nf["median"], nf["max_abs"]))
            n = len(rows) // max(1, len(by_variant))
            print("  -> %d seeds give a 95%% CI of about +/-%.1f points"
                  % (n, 1.96 * nf["se"]))
            if nf["se"] <= 2.0:
                print("  -> VERDICT: %d seeds is sufficient" % n)
            elif nf["se"] > 5.0:
                print("  -> VERDICT: too noisy for %d seeds; use 16, or force"
                      % n)
                print("     HubSchedule.setShiftSeed('B')/('R') per seed as a controlled block")
            else:
                print("  -> VERDICT: usable, but treat deltas under ~%.0f as noise"
                      % (1.96 * nf["se"]))
        return 0

    # ---- variant comparison --------------------------------------------------
    base_rows = by_variant.get(args.baseline, [])
    if not base_rows:
        print("baseline variant %r not found; have: %s"
              % (args.baseline, ", ".join(sorted(by_variant))))
        return 2
    # Mean baseline per seed, so replicas average out.
    base_by_seed = defaultdict(list)
    for r in base_rows:
        base_by_seed[r["seed"]].append(r)

    print("%-16s %5s %8s %8s %9s %20s %8s" %
          ("variant", "seeds", "base", "var", "d(mean)", "95% CI", "verdict"))
    print("-" * 78)
    failures = []
    for v in sorted(by_variant):
        if v == args.baseline:
            continue
        vrows = by_variant[v]
        v_by_seed = defaultdict(list)
        for r in vrows:
            v_by_seed[r["seed"]].append(r)

        deltas, guard_hits, canary_hits = [], 0, 0
        base_vals, var_vals = [], []
        for seed in sorted(set(v_by_seed) & set(base_by_seed)):
            bmean = statistics.mean(r.get(args.objective, 0) or 0
                                   for r in base_by_seed[seed])
            vlist = v_by_seed[seed]
            vmean = statistics.mean(r.get(args.objective, 0) or 0 for r in vlist)
            deltas.append(vmean - bmean)
            base_vals.append(bmean)
            var_vals.append(vmean)
            for r in vlist:
                hits = check_guardrails(r, base_by_seed[seed][0])
                if hits:
                    guard_hits += 1
                    if guard_hits <= 3:
                        print("   guardrail seed %s: %s" % (seed, "; ".join(hits[:3])))
                if check_canaries(r):
                    canary_hits += 1

        if not deltas:
            print("%-16s %5d   (no seeds in common with baseline)" % (v, len(v_by_seed)))
            continue
        mean_d = statistics.mean(deltas)
        lo, hi = bootstrap_ci(deltas)
        print("%-16s %5d %8.1f %8.1f %+9.1f  [%+7.1f, %+7.1f]"
              % (v, len(deltas), statistics.mean(base_vals), statistics.mean(var_vals),
                 mean_d, lo, hi))

        reasons = []
        # A win must be fuel-driven: rejects "score by making the opponent foul".
        d_fuel = statistics.mean([r.get("blueFuel", 0) or 0 for r in vrows]) - \
                 statistics.mean([r.get("blueFuel", 0) or 0 for r in base_rows])
        d_pen = statistics.mean([r.get("bluePenaltyPoints", 0) or 0 for r in vrows]) - \
                statistics.mean([r.get("bluePenaltyPoints", 0) or 0 for r in base_rows])
        if mean_d > 0 and d_fuel <= 0:
            reasons.append("gain is not fuel-driven (d_fuel=%+.1f, d_penalty=%+.1f)"
                           % (d_fuel, d_pen))
        if guard_hits:
            reasons.append("%d/%d match(es) tripped a guardrail" % (guard_hits, len(vrows)))
        if canary_hits:
            reasons.append("%d match(es) had an attribution leak" % canary_hits)
        if lo <= 0 <= hi:
            reasons.append("CI includes 0 (no demonstrated effect)")

        if reasons:
            print("%-16s %5s %8s %8s %9s %20s %8s" % ("", "", "", "", "", "", "REJECT"))
            for rsn in reasons:
                print("   - %s" % rsn)
            failures.append((v, reasons))

    print("-" * 78)
    if not by_variant or len(by_variant) < 2:
        print("only one variant present - nothing to compare.")
        print("Run tools/score/sweep.ps1 -Variants baseline,<name> to compare.")
        return 0
    if failures:
        print("VERDICT: %d variant(s) rejected" % len(failures))
        for v, reasons in failures:
            print("  %s: %s" % (v, reasons[0]))
        return 0 if args.no_gate else 1
    print("VERDICT: all variants passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
