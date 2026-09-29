---
title: Headless Sweep Findings — Score Rig Limits and the Contested-Target Deadlock
audience: [human, ai]
owner: programming-leads
last_verified: 2026-09-29
status: current
---

# Headless Sweep Findings

Findings from a 56-match headless 3v3 sweep (8 seeds x 2 replicas, then 22
replicas each on seeds 7 and 90210) run 2026-09-28 23:29 - 2026-09-29 00:20 on a
dirty working tree. Recorded because **the rig cannot currently measure the thing
it was built to measure**, and that blocks any A/B on policy.

## Scope

Two things: (1) what the score rig can and cannot see, and (2) the failure mode
the sweep exposed, which is a deadlock rather than a tuning problem. This is a
findings document, not a fix; no code changed as a result of it.

## The rig's blind spots

Three measurement gaps, all of which produced *clean-looking* results for runs
that were actually broken. Any of these alone would keep hiding the defect.

**1. `maxContiguousStallSec` cannot see a stationary robot.** It counts only
windows where a bot *commands* motion above `STALL_CMD_SPEED_MIN` and fails to
achieve it. A bot that has stopped commanding — because it believes it has
arrived, or because it is yielding — is invisible to it. The best run of the
sweep (seed 7777 rep 1, Blue 315 / Red 226) reported stalls of 1.88 / 1.40 /
1.68 s for the three Blue bots: clean. The log shows all six bots stationary for
the final 20 s, moving above 0.5 m/s in only 3-15% of samples and scoring
**zero** in that window.

**2. No commanded-velocity topic exists.** `AIRobotInstance.java:846` already
computes `commandedSpeed` and uses it for the stall predicate, then discards it.
Nothing is published, so the rig cannot distinguish "told to stand still" from
"told to move and did not". **This one line of instrumentation would have made
most of the diagnosis below direct rather than inferential, and it is the
cheapest high-value change on this list.**

**3. `UnreachableNoProgressSec` was available and unused for this purpose.** It is
logged per bot and would flag a wedged bot immediately, but nothing in the rig
summary reads it.

Also unproven: every row stamped `gitSha=b2a85f0`, a commit predating all the
work in the tree, because the working tree was dirty and uncommitted throughout.
`gitSha` therefore cannot distinguish binaries. All 56 runs are unattributable to
a specific build.

## What the sweep measured

Pooled within-seed SD on Blue total, from 22 replicas each on two seeds:

```
seed 7     n=22   Blue 185.7 +/- 82.8    Red 194.1 +/- 55.4   BlueWins 11 / RedWins 11
seed 90210 n=22   Blue 191.5 +/- 77.3    Red 180.3 +/- 60.2   BlueWins 12 / RedWins 10
```

**CV ~23%, roughly 4x worse than a 2-replica sample suggested** (an interim
6-pair estimate gave SD 22.7 / CV 11% and was wrong — the two worst seeds, 7777
at a 130-point spread and 90210 at 127, landed last). The sorted values are
clearly bimodal, with a low cluster (36-119) and a high cluster (191-335) and a
gap between:

```
seed 7      36  51  65  80 106 119 154 155 157 191 195 199 221 229 229 230 234 264 267 277 292 335
seed 90210  68  70  82  82 145 153 156 157 163 172 177 184 203 220 226 231 254 262 272 288 319 330
```

Detection cost scales as (sigma/delta)^2, so at this noise floor:

| Effect | matches (both arms) | wall time @4 workers |
|---|---|---|
| 5%  | ~678  | ~8 h |
| 10% | ~170  | ~2 h |
| 20% | ~44  | ~30 m |

**Conclusion: the rig cannot resolve a 5% or 10% policy change.** More replicas
will not fix that, because the variance is a mixture of two behaviours, not
Gaussian noise. Fixing the deadlock is the prerequisite for the rig being usable
at all.

Integrity was clean throughout: 56/56 rows `ok`, not `degraded`, with
`blueReconciliationResidual = 0`, `redReconciliationResidual = 0` and both
unattributed canaries zero on every row. The per-bot fuel attribution fix holds
on live match data.

## The defect: contested-target deadlock

Confirmed by replay inspection. This is **not** a pathfinding fault and **not**
an arbitration gap.

Observed endgame state, all six robots stalled:

- **Four robots pathing to the same point in centre field.** Fuel selection has no
  notion of a claim, so the best piece on the field is the best piece for every
  bot simultaneously. They converge and wedge. Racing for fuel is correct
  behaviour in this game — 54+ loose pieces, three robots per alliance — so the
  fix must not reserve pieces or introduce queueing. The winner taking a piece is
  the game working.
- **A fifth robot pathing to a stalled robot** that is itself blocking another.
  This is a mark/intercept target rather than a fuel claim, so it is a second,
  separate failure and would survive a fuel-only fix.
- **No robot shooting.** The deadlock completes before anyone reaches a shooting
  position.

The signature is now fully explained:

1. A bot converges on a contested piece and comes within `ARRIVED_M` of it.
2. It **believes it has arrived** and commands ~0 velocity.
3. The stall predicate requires commanded speed above `STALL_CMD_SPEED_MIN`, so
   **no stall is ever registered**. The same applies to `ContactWatchdog` and
   `TargetProgressWatchdog`, which all key off commanded-vs-achieved.
4. It is physically wedged against a peer sitting on the same piece, and will sit
   there for the rest of the match.

**Arrival is assumed rather than validated.** Every recovery mechanism in the
stack switches off the moment a bot decides it has arrived, which is exactly the
state in which a physical wedge becomes undetectable. This also explains the
harvester stalls: a 97-107 s stall is not a pathfinding failure but a bot that
committed to a piece, arrived at a crowd, and stayed wedged — same mechanism,
longer fuse.

Corroborating evidence that it is not a latch or a stuck-hold: objectives kept
changing every 0.6-1.7 s during the freeze (`Bot1` last switched 13.7 s before
the end), so the decision layer was actively re-deciding and choosing to hold.
And `ShootGate` read `ready` ~50% of the time with 2-4 deg heading error while
the score stayed at zero — a separate contradiction, still unexplained, worth
chasing between "gate ready" and "fuel counted".

## Proposed direction (not implemented)

Give the no-progress detector a **stationary arm**: a bot that has held a target
for a couple of seconds, is not gaining fuel, and has no other reason to be there
should blacklist that point through the **existing** `blockedFuel` path that
`TargetProgressWatchdog` already owns (1.0 m radius, 20 s TTL). The next
`findBestFuelTarget` then naturally moves to the next piece.

This deliberately does not introduce claim arbitration or reservation:

- it reuses the watchdog's existing "this point is not working for me" concept, so
  it is per-bot and needs no cross-bot bookkeeping
- the race continues; the loser re-targets instead of parking, which is the
  human-shaped outcome — a player who finds a piece gone looks for the next one
- unlike a claim, it cannot go stale, because it is derived from the bot's own
  recent progress rather than stored

Two things to read before implementing, since guessing here has been expensive:

- where arrival is decided and what gates the chassis command to zero
- whether the watchdog timer runs at all in the arrived state

If the second is false, the fix belongs in the arrival condition itself, and that
version would have to address both the endgame freeze and the harvester stalls.
If it is true, a stationary arm on the watchdog is sufficient for the endgame case
but leaves the mark deadlock as a second, separate fix.

## Verification

No code change. Findings rest on 56 rig rows (`results/sweep.jsonl`, plus
`logs/sweep/*.wpilog`) and on replay inspection. Two caveats on that evidence:
the runs are not attributable to a build (dirty tree, `gitSha` pinned to a
pre-session commit), and the automated per-bot pose analysis was abandoned — a
hand-written `struct:Pose2d` decoder produced garbage, so the per-bot motion and
target numbers above come from the plain-double telemetry topics
(`UnreachableNoProgressSec`, `Objective`, `Score`, `ShootGate`) and from the
replay, not from decoded poses. `TargetPose` and `ActualPose` are still
unanalysed programmatically and should be read in AdvantageScope.

Sample-size figures in the table above are arithmetic on a noisy SD and should
not be quoted as decision-grade; they are reproduced here because the interim
2-replica numbers that misled this investigation are worth recording alongside
the corrected ones.

## Related

- `KNOWN_ISSUES.md` §A (decisions, determinism), §E (desired features)
- `ARCHITECTURE.md` §3A (navigation contracts), §3J (Jev architecture)
- `docs/KNOWLEDGE_MODEL.md` (clairvoyant vs observed knowledge)
- `docs/COORDINATION.md` (why concurrent builds invalidate a run)
- `tools/score/sweep.ps1`, `tools/score/compare.py` (the rig)
