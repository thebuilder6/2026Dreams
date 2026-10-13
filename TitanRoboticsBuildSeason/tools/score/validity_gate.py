#!/usr/bin/env python3
"""
Cross-sim validity gate: does the standalone runner agree with the full sim?

    python tools/score/validity_gate.py --standalone results/ab_standalone.jsonl \
        --fullsim results/ab_fullsim.jsonl --logs logs/sweep

Pairing is by (variant, seed): each side aggregates its CLEAN rows per seed to
mean totals, and the seed agrees when both sides name the same winner. Margin
Spearman and the level ratio are reported, not gated — with n=8 seeds Spearman
is nearly powerless (|rho| >~ 0.74 for p < 0.05), and replica flips make it a
coin flip. Winner agreement survives both, which is why it leads.

Censoring (counted, never paired):
  * ``robust_fitness.classify_row``: CONTAMINATED (dirty/unknown provenance,
    bad health, attribution leaks) and COLLAPSED (discrete failure mode).
  * Sim-throw scan: a full-sim match that threw (NPE/CME, caught-and-logged)
    still writes a FINAL row with normal health, so ``classify_row`` passes it
    as CLEAN. With ``--logs`` the gate greps each row's sweep log
    (``<variant>_seed<seed>_r<replica>.out/.err.log``) for exception signatures
    and censors hits as THREW. Without ``--logs`` the scan is skipped and said
    so; a missing log file warns but never censors (absence of evidence).

Standalone rows carry a hardcoded ``health`` block (0 overruns, 0.0 ms), so
they always pass the health check by construction — noted, not fixed, because
the standalone is a single-threaded fixed-step loop with no worker contention
to measure.

Exit codes: 0 PASS, 1 FAIL (reasons printed), 2 no pairable data. ``--no-gate``
reports without failing (still 2 when there is nothing to pair).

``--delta BASE VAR`` switches to rank-transfer mode: both files must carry
both variants, and per-seed BASE->VAR blueTotal deltas must agree in sign.
Zero deltas are excluded, so an ineffective tweak exits 2, never PASS.
"""

import argparse
import math
import os
import re
import statistics
import sys
from collections import defaultdict

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from tools.score.compare import load as load_rows
from tools.tune.robust_fitness import classify_row, CLEAN, COLLAPSED, CONTAMINATED

THREW = "THREW"

# Signatures of a sim worker that threw mid-match but still wrote FINAL.
# Matches the Oct 09 headless throws (AIRobotSim NPE, simulationUpdate CME).
THROW_PATTERNS = re.compile(
    r"Exception|ConcurrentModification|NullPointer|UnsatisfiedLink|Unable to find|EXCEPTION_"
)

# Gate defaults. Agreement needs 6/8 clean seeds; fewer than 6 pairs refuses
# to certify (a CI on less is theatre — same stance as MIN_PAIRED_SEEDS).
MIN_PAIRS = 6
MIN_AGREEMENT = 0.75


def winner_of(blue, red):
    if blue > red:
        return "Blue"
    if red > blue:
        return "Red"
    return "Tie"


def log_hit(logs_dir, row):
    """True when the row's sweep log shows a mid-match throw. None when unknown
    (no --logs, or the log file is absent)."""
    if not logs_dir:
        return None
    tag = "%s_seed%s_r%s" % (row.get("variant", "baseline"), row.get("seed"),
                             row.get("replica", 0))
    found_any = False
    for ext in (".out.log", ".err.log"):
        path = os.path.join(logs_dir, tag + ext)
        if not os.path.isfile(path):
            continue
        found_any = True
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if THROW_PATTERNS.search(line):
                    return True
    if not found_any:
        print("?? no sweep log for %s (threw-scan unknown for this row)" % tag)
        return None
    return False


def ranks(values):
    """Average-tie ranks (1-based) for a list of numbers."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    out = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            out[order[k]] = avg
        i = j + 1
    return out


def spearman(xs, ys):
    """Spearman rho, or None when undefined (n < 3 or no variance)."""
    n = len(xs)
    if n < 3:
        return None
    rx, ry = ranks(xs), ranks(ys)
    mx, my = statistics.mean(rx), statistics.mean(ry)
    cov = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    vx = sum((a - mx) ** 2 for a in rx)
    vy = sum((b - my) ** 2 for b in ry)
    if vx == 0 or vy == 0:
        return None
    return cov / math.sqrt(vx * vy)


def clean_cells(rows, side, logs_dir, censored):
    """Censor rows (classify_row + fullsim throw-scan), return ({(variant, seed):
    [rows]}, scan_unknown_count). Standalone rows are never log-scanned."""
    cells = defaultdict(list)
    scan_unknown = 0
    for r in rows:
        status, reason = classify_row(r)
        if status != CLEAN:
            censored["%s %s: %s" % (side, status.lower(), reason)] += 1
            continue
        if side == "fullsim":
            hit = log_hit(logs_dir, r)
            if hit is True:
                censored["fullsim threw mid-match (sweep log)"] += 1
                continue
            if hit is None and logs_dir:
                scan_unknown += 1
        cells[(r.get("variant", "baseline"), r.get("seed"))].append(r)
    return cells, scan_unknown


def aggregate(rows):
    """Mean totals + winner for one side of one (variant, seed) cell."""
    blue = statistics.mean(r.get("blueTotal", 0) or 0 for r in rows)
    red = statistics.mean(r.get("redTotal", 0) or 0 for r in rows)
    margin = statistics.mean(abs((r.get("blueTotal", 0) or 0) - (r.get("redTotal", 0) or 0))
                             for r in rows)
    blue_fuel = statistics.mean(r.get("blueFuel", 0) or 0 for r in rows)
    red_fuel = statistics.mean(r.get("redFuel", 0) or 0 for r in rows)
    return {"n": len(rows), "blue": blue, "red": red, "margin": margin,
            "winner": winner_of(blue, red),
            "fuel_winner": winner_of(blue_fuel, red_fuel)}


def delta_main(args, clean_s, clean_f, s_shas, f_shas):
    """Rank-transfer mode: per-seed BASE->VAR deltas must agree in sign across
    sims. A zero delta on either side is excluded (no effect, no sign) — so an
    ineffective tweak pairs nothing and exits 2, never PASS."""
    base, var = args.delta
    deltas = []
    for key in sorted(set(clean_s) & set(clean_f), key=str):
        variant, seed = key
        if variant != base:
            continue
        cells = (clean_s.get((base, seed)), clean_s.get((var, seed)),
                 clean_f.get((base, seed)), clean_f.get((var, seed)))
        if any(not c for c in cells):
            continue
        s_base, s_var, f_base, f_var = (aggregate(c) for c in cells)
        deltas.append({"seed": seed,
                       "d_s": s_var["blue"] - s_base["blue"],
                       "d_f": f_var["blue"] - f_base["blue"]})
    print("-- %s->%s deltas on %d seed(s) --" % (base, var, len(deltas)))
    decided = [d for d in deltas if d["d_s"] != 0 and d["d_f"] != 0]
    for d in deltas:
        status = ("AGREE" if (d["d_s"] > 0) == (d["d_f"] > 0) else "differ") \
            if d in decided else "ZERO"
        print("  seed %-6s standalone %+.1f  fullsim %+.1f  %s"
              % (d["seed"], d["d_s"], d["d_f"], status))
    if not deltas:
        print("VERDICT: nothing pairable (need both variants on both sides).")
        return 2
    if not decided:
        print("VERDICT: nothing decidable (every delta is zero — the tweak has")
        print("  no measurable effect on either side).")
        return 2
    agreed = sum(1 for d in decided if (d["d_s"] > 0) == (d["d_f"] > 0))
    rho = spearman([d["d_s"] for d in decided], [d["d_f"] for d in decided])
    print("delta sign agreement: %d/%d decided seeds%s" %
          (agreed, len(decided),
           " (+%d zero-delta excluded)" % (len(deltas) - len(decided))
           if len(deltas) != len(decided) else ""))
    print("delta Spearman rho=%s (n=%d%s)" %
          ("%.2f" % rho if rho is not None else "n/a", len(decided),
           ", n.s. below n=10" if len(decided) < 10 else ""))
    reasons = []
    if set(s_shas) != set(f_shas):
        reasons.append("commit mismatch across files (different simulators)")
    if len(decided) < args.min_pairs:
        reasons.append("only %d decided seed(s), need %d"
                       % (len(decided), args.min_pairs))
    agreement = (agreed / len(decided)) if decided else 0.0
    if agreement < args.min_agreement:
        reasons.append("agreement %.2f < %.2f" % (agreement, args.min_agreement))
    print("-" * 78)
    if reasons:
        print("VERDICT: FAIL")
        for rsn in reasons:
            print("  - %s" % rsn)
        return 0 if args.no_gate else 1
    print("VERDICT: PASS")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--standalone", required=True, help="standalone JSONL rows")
    ap.add_argument("--fullsim", required=True, help="full-sim sweep JSONL rows")
    ap.add_argument("--logs", default=None,
                    help="sweep log dir for the sim-throw scan (default: skip it)")
    ap.add_argument("--min-pairs", type=int, default=MIN_PAIRS)
    ap.add_argument("--min-agreement", type=float, default=MIN_AGREEMENT)
    ap.add_argument("--no-gate", action="store_true", help="report only; never exit 1")
    ap.add_argument("--delta", nargs=2, metavar=("BASE", "VAR"), default=None,
                    help="rank-transfer mode: compare per-seed BASE->VAR deltas "
                         "across sims instead of absolute winners")
    args = ap.parse_args(argv)

    s_rows, s_bad = load_rows(args.standalone)
    f_rows, f_bad = load_rows(args.fullsim)
    for label, bad in (("standalone", s_bad), ("fullsim", f_bad)):
        for b in bad[:5]:
            print("!! %s unusable: %s" % (label, b))
    if not s_rows or not f_rows:
        print("no usable rows on %s side" % ("standalone" if not s_rows else "fullsim"))
        return 2

    # Provenance: different commits are different simulators.
    s_shas = sorted({r.get("gitSha") or "?" for r in s_rows})
    f_shas = sorted({r.get("gitSha") or "?" for r in f_rows})
    print("standalone sha(s): %s   fullsim sha(s): %s" % (",".join(s_shas), ",".join(f_shas)))

    censored = defaultdict(int)
    clean_s, _ = clean_cells(s_rows, "standalone", args.logs, censored)
    clean_f, scan_unknown = clean_cells(f_rows, "fullsim", args.logs, censored)

    if censored:
        print("-- censored (counted, never paired) --")
        for reason, n in sorted(censored.items(), key=lambda kv: -kv[1]):
            print("  %4d  %s" % (n, reason))
    if args.logs is None:
        print("-- threw-scan skipped (no --logs); NPE/CME rows read as CLEAN --")
    elif scan_unknown:
        print("-- threw-scan unknown for %d row(s) (log file absent) --" % scan_unknown)

    if args.delta:
        return delta_main(args, clean_s, clean_f, s_shas, f_shas)

    pairs = []
    for key in sorted(set(clean_s) & set(clean_f), key=str):
        sa, fa = aggregate(clean_s[key]), aggregate(clean_f[key])
        agree = sa["winner"] == fa["winner"] and sa["winner"] != "Tie"
        tie = sa["winner"] == "Tie" or fa["winner"] == "Tie"
        pairs.append({"key": key, "sa": sa, "fa": fa, "agree": agree, "tie": tie})
    print("-- %d paired seed(s) --" % len(pairs))
    for p in pairs:
        (variant, seed) = p["key"]
        print("  %-14s seed %-6s standalone %-4s (%6.1f-%6.1f)  fullsim %-4s (%6.1f-%6.1f)  %s"
              % (variant, seed, p["sa"]["winner"], p["sa"]["blue"], p["sa"]["red"],
                 p["fa"]["winner"], p["fa"]["blue"], p["fa"]["red"],
                 "AGREE" if p["agree"] else ("TIE" if p["tie"] else "differ")))

    if not pairs:
        print("VERDICT: nothing pairable.")
        print("  standalone cells: %s"
              % sorted(set(clean_s), key=str)[:12])
        print("  fullsim cells:    %s"
              % sorted(set(clean_f), key=str)[:12])
        print("  (a variant-name mismatch pairs nothing: standalone defaults to")
        print("   --variant standalone; pass --variant baseline to match a sweep)")
        return 2

    decided = [p for p in pairs if not p["tie"]]
    agreed = sum(1 for p in decided if p["agree"])
    fuel_agreed = sum(1 for p in decided
                      if p["sa"]["fuel_winner"] == p["fa"]["fuel_winner"]
                      and p["sa"]["fuel_winner"] != "Tie")
    rho = spearman([p["sa"]["margin"] for p in pairs], [p["fa"]["margin"] for p in pairs])
    s_blue = statistics.mean(p["sa"]["blue"] for p in pairs)
    f_blue = statistics.mean(p["fa"]["blue"] for p in pairs)
    print("winner agreement: %d/%d decided seeds%s" %
          (agreed, len(decided),
           " (+%d tie(s) excluded)" % (len(pairs) - len(decided)) if len(pairs) != len(decided) else ""))
    print("fuel-only agreement: %d/%d" % (fuel_agreed, len(decided)))
    print("margin Spearman rho=%s (n=%d%s)" %
          ("%.2f" % rho if rho is not None else "n/a", len(pairs),
           ", n.s. below n=10" if len(pairs) < 10 else ""))
    print("level: standalone mean blue %.1f vs fullsim %.1f (ratio %.2f)" %
          (s_blue, f_blue, (s_blue / f_blue) if f_blue else float("nan")))

    reasons = []
    if set(s_shas) != set(f_shas):
        reasons.append("commit mismatch across files (different simulators)")
    if len(pairs) < args.min_pairs:
        reasons.append("only %d paired seed(s), need %d" % (len(pairs), args.min_pairs))
    agreement = (agreed / len(decided)) if decided else 0.0
    if agreement < args.min_agreement:
        reasons.append("agreement %.2f < %.2f" % (agreement, args.min_agreement))
    print("-" * 78)
    if reasons:
        print("VERDICT: FAIL")
        for rsn in reasons:
            print("  - %s" % rsn)
        return 0 if args.no_gate else 1
    print("VERDICT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
