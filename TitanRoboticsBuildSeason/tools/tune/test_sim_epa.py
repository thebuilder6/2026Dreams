"""Self-test for sim_epa.py. Run: python tools/tune/test_sim_epa.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from tools.tune.sim_epa import (
    match_component_epa,
    defense_residuals,
    iterative_ratings,
    _endgame_epa,
    _teleop_rate,
)


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        raise SystemExit(1)


def bot(roster, alliance, archetype, auto=0, teleop=0, wasted=0,
        climbed=False, arrival=-1.0, minor=0, major=0, hub_sec=100.0):
    return {
        "roster": roster, "alliance": alliance, "archetype": archetype,
        "autoScored": auto, "teleopScored": teleop, "scored": auto + teleop,
        "pickedUp": 0, "attemptedShots": 0, "missedShots": 0, "escapes": 0,
        "pathLengthM": 0.0, "wastedFuel": wasted, "shuttledFuel": 0,
        "climbed": climbed, "climbArrivalSec": arrival,
        "minorFouls": minor, "majorFouls": major, "hubActiveTeleopSec": hub_sec,
    }


# 1. endgame discount
check("climb at window start is full credit", _endgame_epa(bot(0, "blue", "X", climbed=True, arrival=0.0)) == 10.0)
check("climb at window end is half credit", _endgame_epa(bot(0, "blue", "X", climbed=True, arrival=20.0)) == 5.0)
check("no climb is zero", _endgame_epa(bot(0, "blue", "X")) == 0.0)

# 2. teleop rate
check("teleop rate is per hub-active second", abs(_teleop_rate(bot(0, "blue", "X", teleop=30, hub_sec=100)) - 0.3) < 1e-9)
check("zero hub time is zero rate", _teleop_rate(bot(0, "blue", "X", teleop=30, hub_sec=0)) == 0.0)

# 3. component breakdown
row = {
    "perBot": [bot(0, "blue", "CYCLER", auto=6, teleop=20, wasted=4, climbed=True, arrival=0.0, minor=1)],
    "markSeconds": [[0.0]],
}
comp = match_component_epa(row)[0]
check("auto component", comp["auto"] == 6.0)
check("teleop component subtracts waste", comp["teleop"] == 18.0)
check("endgame component", comp["endgame"] == 10.0)
check("penalty component", comp["penalty"] == 5.0)
check("total combines with sign", abs(comp["total"] - (6 + 18 + 10 + 0 - 5)) < 1e-9)

# 4. defense residual: marked opponent scores below its archetype baseline
per_bot = [
    bot(0, "blue", "DEFENDER", teleop=0),
    bot(1, "red", "CYCLER", teleop=20, hub_sec=100),   # rate 0.2
    bot(2, "red", "CYCLER", teleop=60, hub_sec=100),   # rate 0.6 -> archetype mean 0.4
]
marks = [[0.0, 30.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
res = defense_residuals(per_bot, marks)
check("defense residual credits suppression", abs(res[0] - 30.0 * (0.4 - 0.2)) < 1e-9)
check("unmarked defenders get nothing", res[1] == 0.0 and res[2] == 0.0)

# 5. within-match baseline is circular by design -> zero when status quo holds
circ = defense_residuals([bot(0, "blue", "D"), bot(1, "red", "C", teleop=30, hub_sec=100)],
                         [[0.0, 10.0], [0.0, 0.0]])
check("single-archetype within-match baseline yields zero residual", circ[0] == 0.0)

# 6. missing perBot is a hard error
try:
    match_component_epa({"perBot": []})
    check("empty perBot raises", False)
except ValueError:
    check("empty perBot raises", True)

# 7. iterative pass returns an entity per (variant, roster) and populates defense
rows = [
    {"variant": "candidate", "seed": 7, "replica": 0, "durationSec": 150.0, "autoSec": 15.0,
     "perBot": per_bot, "markSeconds": marks},
]
ratings = iterative_ratings(rows, alpha=0.5, rounds=4)
check("iterative returns entities", len(ratings) == 3)
check("defense rating becomes positive", ratings["candidate#0"]["defense"] > 0.0)

print("\nall sim_epa self-tests passed")
