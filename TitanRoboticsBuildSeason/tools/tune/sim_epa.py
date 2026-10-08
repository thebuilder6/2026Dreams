#!/usr/bin/env python3
"""
sim_epa.py — Individual component EPA from standalone-runner JSONL.

Unlike the full-sim score rig, the MapleSim-free runner
(``frc.robot.Sim.StandaloneMatchRunner``) emits exact per-bot telemetry
(``perBot``) and a defensive-mark matrix (``markSeconds``), so a bot's
contribution does not have to be blind-estimated the way Statbotics must from
alliance totals. This module turns those rows into component EPA ratings:

    EPA_total = EPA_auto + EPA_teleop + EPA_endgame + EPA_defense - EPA_penalty

Design notes (see ``docs/LOGIC_EVOLUTION_GUIDE.md``):

* ``EPA_penalty`` is positive points conceded (fouls committed), subtracted in
  the total.
* ``EPA_defense`` is a *model-based residual*: the counterfactual "what the
  marked opponent would have scored unmarked" is a baseline teleop *rate* (per
  hub-active second), not a replay. Within one match the baseline is the mean
  rate of the opponent's archetype; across a tournament the iterative pass
  substitutes each opponent's own running rating, which breaks the circularity
  that a same-match rate would create (a same-match rate makes the residual
  identically zero).
* Component ratings are updated with the Statbotics/Elo rule
  ``new = old + alpha * (match_value - old)``.

Usage:
    python tools/tune/sim_epa.py results/standalone.jsonl
    python tools/tune/sim_epa.py results/standalone.jsonl --alpha 0.2 --rounds 12
    python tools/tune/sim_epa.py results/standalone.jsonl --json results/epa.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants — mirrored from MatchScoreTracker / StandaloneBot
# ---------------------------------------------------------------------------
POINTS_PER_CLIMB = 10.0
CLIMB_WINDOW_SEC = 20.0
WASTE_PENALTY_PER_PIECE = 0.5
POINTS_PER_MINOR_FOUL = 5.0
POINTS_PER_MAJOR_FOUL = 15.0

COMPONENTS = ("auto", "teleop", "endgame", "defense", "penalty")


# ---------------------------------------------------------------------------
# Single-match component EPA
# ---------------------------------------------------------------------------

def _endgame_epa(bot: Dict[str, Any]) -> float:
    """10 pts for a climb, discounted by how late in the window it landed."""
    if not bot.get("climbed", False):
        return 0.0
    arrival = bot.get("climbArrivalSec", -1.0)
    if arrival is None or arrival < 0.0:
        # Climbed but no arrival stamp: credit the full climb.
        return POINTS_PER_CLIMB
    frac = min(1.0, arrival / CLIMB_WINDOW_SEC)
    return POINTS_PER_CLIMB * max(0.0, 1.0 - 0.5 * frac)


def _teleop_rate(bot: Dict[str, Any]) -> float:
    """Teleop points per hub-active second (0 when the hub never opened)."""
    active = float(bot.get("hubActiveTeleopSec", 0.0) or 0.0)
    if active <= 1e-9:
        return 0.0
    return float(bot.get("teleopScored", 0)) / active


def _baseline_rates(per_bot: List[Dict[str, Any]]) -> Dict[str, float]:
    """Mean teleop rate by archetype across the match, plus a global fallback."""
    sums: Dict[str, float] = {}
    counts: Dict[str, int] = {}
    total = 0.0
    for bot in per_bot:
        rate = _teleop_rate(bot)
        arch = bot.get("archetype", "UNKNOWN")
        sums[arch] = sums.get(arch, 0.0) + rate
        counts[arch] = counts.get(arch, 0) + 1
        total += rate
    rates = {arch: sums[arch] / counts[arch] for arch in sums}
    rates["__global__"] = total / len(per_bot) if per_bot else 0.0
    return rates


def defense_residuals(
    per_bot: List[Dict[str, Any]],
    mark_seconds: List[List[float]],
    baseline: Optional[Dict[str, float]] = None,
) -> List[float]:
    """
    Per-defender defensive EPA for one match.

    For every defender i that marked opponent j while j's hub was live, credit
    ``mark_seconds[i][j] * max(0, baseline_rate_j - actual_rate_j)``. The
    baseline is the *counterfactual unmarked rate*; within a match it defaults
    to the opponent-archetype mean (a tournament substitutes each opponent's
    running rating).
    """
    n = len(per_bot)
    defense = [0.0] * n
    if not mark_seconds:
        return defense
    if baseline is None:
        baseline = _baseline_rates(per_bot)
    for i in range(min(n, len(mark_seconds))):
        row = mark_seconds[i]
        for j in range(min(n, len(row))):
            secs = float(row[j] or 0.0)
            if secs <= 0.0:
                continue
            j_rate = _teleop_rate(per_bot[j])
            arch = per_bot[j].get("archetype", "UNKNOWN")
            base = baseline.get(arch, baseline.get("__global__", 0.0))
            defense[i] += secs * max(0.0, base - j_rate)
    return defense


def match_component_epa(row: Dict[str, Any], baseline: Optional[Dict[str, float]] = None
                        ) -> List[Dict[str, float]]:
    """Component EPA for every bot in one JSONL row (roster order)."""
    per_bot = row.get("perBot") or []
    if not per_bot:
        raise ValueError("row has no perBot telemetry (pre-telemetry row?)")
    defense = defense_residuals(per_bot, row.get("markSeconds") or [], baseline)
    out: List[Dict[str, float]] = []
    for i, bot in enumerate(per_bot):
        auto = float(bot.get("autoScored", 0))
        waste = float(bot.get("wastedFuel", 0))
        teleop = float(bot.get("teleopScored", 0)) - WASTE_PENALTY_PER_PIECE * waste
        endgame = _endgame_epa(bot)
        penalty = (POINTS_PER_MINOR_FOUL * float(bot.get("minorFouls", 0))
                   + POINTS_PER_MAJOR_FOUL * float(bot.get("majorFouls", 0)))
        entry = {
            "roster": i,
            "alliance": bot.get("alliance", "?"),
            "archetype": bot.get("archetype", "UNKNOWN"),
            "auto": auto,
            "teleop": teleop,
            "endgame": endgame,
            "defense": defense[i],
            "penalty": penalty,
        }
        entry["total"] = auto + teleop + endgame + defense[i] - penalty
        out.append(entry)
    return out


# ---------------------------------------------------------------------------
# Iterative tournament rating (Statbotics/Elo update)
# ---------------------------------------------------------------------------

def _entity_key(row: Dict[str, Any], bot: Dict[str, Any]) -> str:
    return "{}#{}".format(row.get("variant", "standalone"), bot.get("roster", -1))


def iterative_ratings(
    rows: List[Dict[str, Any]],
    alpha: float = 0.2,
    rounds: int = 12,
) -> Dict[str, Dict[str, float]]:
    """
    Running component ratings per (variant, roster) entity.

    Within a round, each match's defense residual uses the *current* ratings of
    the marked opponents as the unmarked baseline, then every observed component
    updates its rating toward the match value. Using the pre-update rating for
    the baseline within a round keeps the update from chasing itself.
    """
    entities: Dict[str, Dict[str, float]] = {}
    valid_rows = [r for r in rows if r.get("perBot")]

    for _ in range(max(1, rounds)):
        for row in valid_rows:
            per_bot = row["perBot"]
            # Build the cross-match baseline: each opponent's own running rate.
            baseline: Dict[str, float] = {}
            for bot in per_bot:
                key = _entity_key(row, bot)
                rate = entities.get(key, {}).get("teleopRate", 0.0)
                baseline[bot.get("archetype", "UNKNOWN")] = rate
            baseline["__global__"] = (
                sum(baseline.values()) / len(baseline) if baseline else 0.0
            )
            match = match_component_epa(row, baseline=baseline)
            for bot, comp in zip(per_bot, match):
                key = _entity_key(row, bot)
                current = entities.setdefault(key, {c: 0.0 for c in COMPONENTS})
                for c in ("auto", "teleop", "endgame", "defense", "penalty"):
                    current[c] = current[c] + alpha * (comp[c] - current[c])
                current["teleopRate"] = current.get("teleopRate", 0.0) + alpha * (
                    _teleop_rate(bot) - current.get("teleopRate", 0.0))
    return entities


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def load_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _print_single(rows: List[Dict[str, Any]]) -> None:
    for row in rows:
        if not row.get("perBot"):
            print("seed {}: no perBot telemetry; skipping".format(row.get("seed")))
            continue
        print("seed {} variant {} replica {}:".format(
            row.get("seed"), row.get("variant"), row.get("replica")))
        for comp in match_component_epa(row):
            print("  [{alliance:4}] {archetype:22} auto={auto:6.2f} teleop={teleop:6.2f} "
                  "endgame={endgame:5.2f} defense={defense:6.2f} penalty={penalty:5.2f} "
                  "=> total {total:7.2f}".format(**comp))


def main() -> None:
    parser = argparse.ArgumentParser(description="Individual component EPA from standalone JSONL")
    parser.add_argument("jsonl", help="standalone runner results JSONL")
    parser.add_argument("--alpha", type=float, default=0.2,
                        help="Statbotics-style update rate (default 0.2)")
    parser.add_argument("--rounds", type=int, default=12,
                        help="passes over the rows for the iterative rating (default 12)")
    parser.add_argument("--json", type=str, default="",
                        help="optional path to write the ratings JSON")
    args = parser.parse_args()

    path = Path(args.jsonl)
    if not path.is_absolute():
        path = Path.cwd() / path
    rows = load_rows(path)
    if not rows:
        print("no rows in {}".format(path), file=sys.stderr)
        sys.exit(1)

    _print_single(rows)

    ratings = iterative_ratings(rows, alpha=args.alpha, rounds=args.rounds)
    print("\n=== iterative component ratings (alpha={}, rounds={}) ==="
          .format(args.alpha, args.rounds))
    for key in sorted(ratings):
        r = ratings[key]
        print("{:32} auto={auto:6.2f} teleop={teleop:6.2f} endgame={endgame:5.2f} "
              "defense={defense:6.2f} penalty={penalty:5.2f}".format(key, **r))

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as handle:
            json.dump(ratings, handle, indent=2)
        print("\nwrote {}".format(out))


if __name__ == "__main__":
    main()
