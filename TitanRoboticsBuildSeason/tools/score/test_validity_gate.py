"""Self-test for validity_gate.py. Run: python tools/score/test_validity_gate.py
NOT EXECUTED HERE — no Python runtime on this machine. Re-run before trusting.
"""
import contextlib
import io
import json
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from tools.score.validity_gate import main, spearman, ranks


def row(seed, replica=0, variant="baseline", blue=200, red=180, blue_tele=200,
        path=450.0, dirty=0, overruns=0, max_ms=20.0, sha="abc1234"):
    return {
        "schemaVersion": 2, "gitSha": sha, "dirty": dirty, "seed": seed,
        "variant": variant, "replica": replica,
        "winner": "Blue" if blue > red else ("Red" if red > blue else "Tie"),
        "blueTotal": blue, "redTotal": red, "margin": abs(blue - red),
        "blueFuel": blue, "redFuel": red,
        "blueTeleopFuel": blue_tele, "redTeleopFuel": 180,
        "bluePenaltyPoints": 0, "redPenaltyPoints": 0,
        "blueReconciliationResidual": 0, "redReconciliationResidual": 0,
        "blueUnattributedFuel": 0, "redUnattributedFuel": 0,
        "health": {"loopOverruns": overruns, "maxRobotPeriodicMs": max_ms},
        "blueBots": {"pathLengthM": [path] * 3},
        "redBots": {"pathLengthM": [path] * 3},
    }


def write(rows):
    fh = tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False,
                                     encoding="utf-8")
    for r in rows:
        fh.write(json.dumps(r) + "\n")
    fh.close()
    return fh.name


def run_gate(s_rows, f_rows, logs=None, extra=()):
    s_path, f_path = write(s_rows), write(f_rows)
    argv = ["--standalone", s_path, "--fullsim", f_path] + list(extra)
    if logs is not None:
        argv += ["--logs", logs]
    with contextlib.redirect_stdout(io.StringIO()):
        code = main(argv)
    os.unlink(s_path)
    os.unlink(f_path)
    return code


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        raise SystemExit(1)


SEEDS = [7, 11, 42, 101, 500, 1337, 2026, 9999]

# 1. perfect 8/8 agreement passes (margins perfectly rank-aligned too).
s = [row(sd, blue=200 + sd % 50, red=150) for sd in SEEDS]
f = [row(sd, blue=190 + sd % 50, red=150) for sd in SEEDS]
check("8/8 agreement passes", run_gate(s, f) == 0)

# 2. 2/8 agreement fails.
s2 = [row(sd, blue=200, red=150) if i < 2 else row(sd, blue=150, red=200)
      for i, sd in enumerate(SEEDS)]
check("2/8 agreement fails", run_gate(s2, f) == 1)
check("--no-gate reports zero", run_gate(s2, f, extra=("--no-gate",)) == 0)

# 3. dirty fullsim rows censored: 8 agree but 3 dirty -> 5 pairs < 6 -> fail.
f3 = [row(sd, blue=190 + sd % 50, red=150, dirty=1 if i < 3 else 0)
      for i, sd in enumerate(SEEDS)]
check("dirty censoring drops pairs", run_gate(s, f3) == 1)

# 4. threw-scan: log with an exception censors the seed (7/8 agree otherwise).
#    7 pairs still pass; with --min-pairs 8 the censoring is what fails it.
logdir = tempfile.mkdtemp()
with open(os.path.join(logdir, "baseline_seed7_r0.out.log"), "w") as fh:
    fh.write("[Headless] tick\njava.lang.NullPointerException at AIRobotSim.update\n")
check("throw censored, 7 pairs pass",
      run_gate(s, f, logs=logdir, extra=("--min-pairs", "7")) == 0)
check("throw censored, 8 required fails",
      run_gate(s, f, logs=logdir, extra=("--min-pairs", "8")) == 1)
check("no --logs skips scan, 8 pairs pass", run_gate(s, f) == 0)

# 5. ties excluded from decided seeds, not counted as agreement.
s5 = [row(sd, blue=200, red=150) for sd in SEEDS[:7]] + [row(9999, blue=150, red=150)]
f5 = [row(sd, blue=190, red=150) for sd in SEEDS[:7]] + [row(9999, blue=150, red=150)]
check("ties excluded, 7/7 passes", run_gate(s5, f5) == 0)

# 6. commit mismatch fails.
f6 = [row(sd, blue=190 + sd % 50, red=150, sha="def5678") for sd in SEEDS]
check("commit mismatch fails", run_gate(s, f6) == 1)

# 7. nothing pairable -> exit 2.
check("disjoint seeds exit 2",
      run_gate([row(1, blue=200, red=150)], [row(2, blue=200, red=150)]) == 2)

# 8. rank math: perfect, reversed, ties, degenerate.
check("spearman perfect", spearman([1, 2, 3, 4], [10, 20, 30, 40]) == 1.0)
check("spearman reversed", spearman([1, 2, 3, 4], [40, 30, 20, 10]) == -1.0)
check("spearman n<3 none", spearman([1, 2], [1, 2]) is None)
check("spearman constant none", spearman([5, 5, 5], [1, 2, 3]) is None)
check("tie ranks averaged", ranks([10, 20, 20, 30]) == [1.0, 2.5, 2.5, 4.0])

# 9. delta mode: matching +20 deltas pass; sign flip fails; zero fails safe.
VAR = "vacuumHi"
s_base = [row(sd, blue=200, red=150) for sd in SEEDS]
s_var = [row(sd, blue=220, red=150, variant=VAR) for sd in SEEDS]
f_base = [row(sd, blue=190, red=150) for sd in SEEDS]
f_var = [row(sd, blue=210, red=150, variant=VAR) for sd in SEEDS]
D = ("--delta", "baseline", VAR)
check("delta agreement passes",
      run_gate(s_base + s_var, f_base + f_var, extra=D) == 0)
f_flip = [row(sd, blue=170, red=150, variant=VAR) for sd in SEEDS]
check("delta sign flip fails",
      run_gate(s_base + s_var, f_base + f_flip, extra=D) == 1)
check("delta zero-delta exits 2",
      run_gate(s_base + s_base, f_base + f_base, extra=D) == 2)
check("delta missing variant exits 2",
      run_gate(s_base, f_base + f_var, extra=D) == 2)

print("all validity_gate self-tests passed")
