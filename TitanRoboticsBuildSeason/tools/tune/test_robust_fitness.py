"""Self-test for robust_fitness.py. Run: python tools/tune/test_robust_fitness.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from tools.tune.robust_fitness import (
    classify_row, robust_single_score, evaluate_paired_robust,
    CLEAN, COLLAPSED, CONTAMINATED,
)


def row(seed, replica=0, blue_tele=200, red_tele=180, path=450.0,
        dirty=0, overruns=0, max_ms=20.0, resid=0, wasted=0, fouls=0,
        winner="Blue", blueTotal=200, redTotal=180):
    return {
        "schemaVersion": 2, "seed": seed, "replica": replica, "dirty": dirty,
        "winner": winner, "blueTotal": blueTotal, "redTotal": redTotal,
        "blueTeleopFuel": blue_tele, "redTeleopFuel": red_tele,
        "blueWastedFuel": wasted, "redPenaltyPoints": fouls,
        "blueReconciliationResidual": resid, "redReconciliationResidual": 0,
        "blueUnattributedFuel": 0, "redUnattributedFuel": 0,
        "health": {"loopOverruns": overruns, "maxRobotPeriodicMs": max_ms},
        "blueBots": {"pathLengthM": [path] * 3,
                     "maxContiguousStallSec": [0.2] * 3,
                     "maxConsecutiveRecoveries": [0] * 3},
        "redBots": {"pathLengthM": [path] * 3,
                    "maxContiguousStallSec": [0.2] * 3,
                    "maxConsecutiveRecoveries": [0] * 3},
    }


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        raise SystemExit(1)


# 1. clean rows classify + score with the right sign
b = row(7, blue_tele=135)
t = row(7, blue_tele=148)
check("clean classify", classify_row(b)[0] == CLEAN)
check("efficiency delta positive", robust_single_score(t) > robust_single_score(b))

# 2. collapse censored (rig §3 signature: cratered teleop + stationary bots)
c = row(42, blue_tele=7, path=13.0)
check("collapse censored", classify_row(c)[0] == COLLAPSED)
check("collapse unscored", robust_single_score(c) == -9999.0)

# 3. contamination censored
check("dirty censored", classify_row(row(1, dirty=1))[0] == CONTAMINATED)
check("unknown provenance censored",
      classify_row({k: v for k, v in row(1).items() if k != "dirty"})[0] == CONTAMINATED)
check("health censored", classify_row(row(1, overruns=99))[0] == CONTAMINATED)
check("canary censored", classify_row(row(1, resid=5))[0] == CONTAMINATED)

# 4. LOW_N refusal, then OK at 4 pairs (margin +10/pair, capped at +3)
seeds = [7, 11, 42]
base = [row(s, blue_tele=135) for s in seeds]
trial = [row(s, blue_tele=148, blueTotal=210) for s in seeds]
fit, st = evaluate_paired_robust(trial, base)
check("low-n refused", fit == float("-inf") and st["reason"] == "LOW_N")
base.append(row(101, blue_tele=135))
trial.append(row(101, blue_tele=148, blueTotal=210))
fit, st = evaluate_paired_robust(trial, base)
check("n=4 certified", st["reason"] == "OK" and fit == 3.0 and st["sign_rate"] == 1.0
      and st["win_edge"] == 0.0 and st["margin_median"] == 10.0)

# 5. outlier robustness: 4× +1 delta, 1× clean -10 (collapsed would censor, so
#    use a legit bad-but-clean row): median stays +1, mean goes negative.
seeds5 = [7, 11, 42, 101, 500]
base5 = [row(s, blue_tele=135) for s in seeds5]
trial5 = [row(s, blue_tele=148) for s in seeds5[:4]] + [row(500, blue_tele=0)]
fit5, st5 = evaluate_paired_robust(trial5, base5)
check("median robust to outlier", abs(st5["median"] - 1.0) < 0.3 and st5["mean"] < 0)
check("bootstrap lo<=median", st5["bootstrap_lo"] <= st5["median"])

# 6. collapsed pairs censored, not averaged: trial collapse on one seed of 4
trial6 = [row(s, blue_tele=148) for s in [7, 11, 42]] + [row(101, blue_tele=7, path=13.0)]
base6 = [row(s, blue_tele=135) for s in [7, 11, 42, 101]]
fit6, st6 = evaluate_paired_robust(trial6, base6)
check("collapse pair censored", st6["n_censored_collapsed"] == 1 and st6["n_pairs"] == 3
      and st6["reason"] == "LOW_N")

# 7. provenance: pre-dirty-field rows censored by default, analyzable relaxed
nodirty = {k: v for k, v in row(7, blue_tele=148).items() if k != "dirty"}
check("unknown provenance strict", classify_row(nodirty)[0] == CONTAMINATED)
check("unknown provenance relaxed", classify_row(nodirty, False)[0] == CLEAN)
fit7, st7 = evaluate_paired_robust(
    [nodirty], [dict(nodirty, seed=7, blue_tele=135)], strict_provenance=False)
check("relaxed flag recorded", st7["provenance_relaxed"] is True and st7["reason"] == "LOW_N")

# 8. winning decides: trial wins every pair while scoring worse on fuel —
#    the scalar stays positive on the win edge alone.
seeds8 = [7, 11, 42, 101]
trial8 = [row(s, blue_tele=50, winner="Blue") for s in seeds8]
base8 = [row(s, blue_tele=200, winner="Red") for s in seeds8]
fit8, st8 = evaluate_paired_robust(trial8, base8)
check("wins outrank efficiency", fit8 == 10.0 and st8["win_edge"] == 1.0
      and st8["median"] < 0)

# 9. ties count half: all-tied pairs score the capped margin only.
trial9 = [row(s, winner="Tie", blueTotal=200) for s in seeds8]
base9 = [row(s, winner="Tie", blueTotal=190) for s in seeds8]
fit9, st9 = evaluate_paired_robust(trial9, base9)
check("ties halve out", fit9 == 3.0 and st9["pairs_even"] == 4 and st9["win_edge"] == 0.0)

# 10. winner missing falls back to totals (win counts accrue even on LOW_N).
trial10 = [dict(row(s, blueTotal=210, winner=None)) for s in [7, 11, 42]]
base10 = [dict(row(s, blueTotal=170, winner=None)) for s in [7, 11, 42]]
fit10, st10 = evaluate_paired_robust(trial10, base10)
check("totals fallback", st10["reason"] == "LOW_N" and st10["pairs_won"] == 3)

print("ALL ROBUST_FITNESS TESTS PASS")
