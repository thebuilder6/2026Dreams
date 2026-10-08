---
title: Logic Evolution & Individual EPA
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-08
status: proposal
---

# Logic Evolution & Individual EPA

A research direction for evolving the Jev decision logic itself (not just its
scalar weights), plus the per-bot performance telemetry and evaluator that any
such search needs. The GA/search machinery is **proposed**; the telemetry and
the `sim_epa.py` evaluator are **implemented** as of 2026-10-08.

## Scope

Covers: the substrate decision (which match engine a search may trust), the
definition and measurement of **individual component EPA**, the standalone
runner's per-bot telemetry contract, and the phased plan to grow an evolvable
expression tree.

Does NOT cover (link, don't copy):

- Decision-layer contracts: `ARCHITECTURE.md` §3J, `docs/KNOWLEDGE_MODEL.md`.
- The full-sim score rig: `docs/SCORE_RIG_RESULTS.md`, `docs/SWEEP_FINDINGS.md`,
  `SIMULATION_GUIDE.md`.
- Weight tuning (as opposed to logic evolution): `tools/tune/optimize_policy.py`,
  `docs/AI_CO_PILOT_GUIDE.md` P5.

## Content

### 1. Substrate: standalone primary, full sim as a check

The full MapleSim score rig is **not** a valid fitness function — a seeded
match is bimodal (`KNOWN_ISSUES.md` §E: Blue ~200-280 or ~7-62 at random, sd
151). The MapleSim-free runner (`Sim/StandaloneMatchRunner`) is deterministic
(bit-identical same-seed reruns) and fast (~0.3-0.8 s for a 6-bot 150 s match),
so it is the substrate a search should optimize on. Its *rank ordering* is
cross-checked against the full sim rather than treated as ground truth.

**Fidelity blocker — recycling now modelled; margin rank still unvalidated.** The full sim's RebuiltHub recycles scored fuel back onto the field, so its throughput is unbounded; the standalone originally ran a fixed pool and saturated. As of 2026-10-08 the runner models that recycle (`FuelStore` pending queue + `StandaloneBot.recycleLanding`, derived from the hub chutes), so the field is conserved. Measured A/B (2026-10-08, 8 seeds): standalone combined score rose 864 → 1549 (from 3.7× to 2.05× below the full sim) and per-seed **winner agreement rose 2/8 → 7/8**; per-seed **margin correlation is still ≈0** (Pearson −0.24, Spearman −0.14). The *level* and *winner* now track; the margin *rank* does not, and the full-sim side is one replica of a non-reproducible run (sd 151). **So the fidelity gate is only partly closed: a search on the standalone is now closer but still not validated against the full sim**, and any champion must still be re-validated there. See `KNOWN_ISSUES.md` §E.

### 2. Individual component EPA

Because the standalone emits exact per-bot telemetry, a bot's contribution is
measured, not blind-estimated the way Statbotics must from alliance totals:

```
EPA_total = EPA_auto + EPA_teleop + EPA_endgame + EPA_defense - EPA_penalty
```

| Component | Definition (implemented in `tools/tune/sim_epa.py`) |
|---|---|
| `EPA_auto` | fuel scored in AUTO |
| `EPA_teleop` | fuel scored in TELEOP − 0.5 × wasted fuel |
| `EPA_endgame` | 10 pts if climbed, discounted up to 50% by arrival lateness in the 20 s window |
| `EPA_penalty` | points conceded: 5 × minor fouls + 15 × major fouls (subtracted) |
| `EPA_defense` | model-based residual — see below |

`EPA_defense` is deliberately **not** a true counterfactual replay. The runner
records *who marked whom* (`markSeconds[i][j]`), not what the marked opponent
would have scored unmarked. So the residual is
`mark_seconds × max(0, baseline_rate − actual_rate)`, where the baseline
"unmarked rate" is:

- **within one match:** the marked opponent's archetype mean teleop rate; and
- **across a tournament:** the opponent's own running rating from the previous
  pass.

Using the *same-match* rate makes the residual identically zero — that is why
the iterative pass exists. Ratings update with the Statbotics/Elo rule
`new = old + α · (match − old)`.

### 3. Telemetry contract

`StandaloneMatchRunner.Result` carries a `BotTelemetry[]` (roster order, Blue
slots first in `default3v3`) and a `double[][] markSeconds` matrix. The JSONL
(`toJsonLine`) emits them as `perBot` (array of objects) and `markSeconds`
(2-D array), alongside the alliance-level `blueClimb` / `redClimb` /
`blueFouls` / `bluePenaltyPoints` / `blueWastedFuel` fields, which are now
computed instead of hardcoded zero.

The penalty channel is honest rather than fabricated: the standalone has no
rigid-body contact, only a positional push-out, so `Sim/StandaloneReferee`
re-implements only the G418 pin rule on plain bot state, requires **active
closing** (passive proximity in the contact band is not a PIN), and caps
escalations at `MAX_PIN_VIOLATIONS = 3` per engagement (separation resets).
Without the cap, the 0.90 m push-out keeps opposing pairs in the contact band
indefinitely and fouls run away (measured: 165-380 penalty EPA before the fix).

`--climb` on the runner CLI gives every slot a climber, so Endgame EPA is
measurable (the default roster has none, exactly as `default3v3` does in the
full sim).

### 4. Evaluator usage

```
# one or more standalone matches, with climbers for endgame:
java -cp build/libs/TitanRoboticsBuildSeason.jar frc.robot.Sim.StandaloneMatchRunner \
    --seeds 7,11 --duration 150 --auto 15 --climb --jsonl results/standalone.jsonl

python tools/tune/sim_epa.py results/standalone.jsonl --alpha 0.2 --rounds 12
python tools/tune/test_sim_epa.py        # self-test, no JVM needed
```

### 5. Phased plan to evolve logic (proposed)

1. **Telemetry + evaluator** — done (this doc).
2. **Validity gate** — before any search, Spearman-correlate the standalone
   component EPAs against a clean sequential full-sim sweep over a few policy
   variants. Blocked by the fidelity gap in §1.
3. **Expression-tree engine** — **library landed 2026-10-08**:
   `Intelligence/utility/ast` has the sealed `ExpressionNode` (Constant,
   Terminal, Product/Sum/Min/Max, IfThenElse, Threshold/`threshold_ge`, Sigmoid,
   Gaussian, Power, Scale, Clamp, Not), a tier-tagged `Terminal` set with an
   `EvalContext` that zeroes clairvoyant terminals under the observed tier, an
   `SExpressions` text codec (the genome↔Java wire form), and `GeneticOperators`
   (randomTree / mutate / crossover / prune). **`CYCLE_SCORE_HUB` parity proven
   2026-10-08:** `ObjectiveExpressions.cycleScoreHub` reproduces the live engine's
   value to `1e-9` across a >200-state grid (`CycleScoreHubParityTest`), with the
   harvest-deadline-force states excluded. **Remaining:** wire the engine to *use*
   the expression (the `ObjectiveUtility` seam), and convert the other objectives.
   Frozen scaffolding stays hand-written: archetype zeroing, harvest-deadline
   force, G420 endgame suppression, the Tier-1 no-information path, and the
   `ObjectiveCommitment` inertia.
4. **Tier 0/1** — Tier 0 is a curated *legality/invariance* card subset (never
   climb at t=140, never shoot a dead hub, never shoot out of zone, alliance
   symmetry), **not** the 97-card preference set, which would bias the search
   toward reproducing the incumbent. Tier 1 is micro-drills.
5. **Search + report** — S-expression genomes seeded from the extracted default
   trees, holdout seeds, a complexity/bloat penalty, and `toReadableString()`
   output as the deliverable. End goal is **advisory** (an interpreted policy),
   not an auto-deployed genome.

## Verification

- Verified against: 2026-10-08, WPILib 2026 JDK. `StandaloneMatchRunnerTest`
  (16 run / 1 skipped) and `StandaloneRefereeTest` (4) green under the
  `gradle-build` lock; `python tools/tune/test_sim_epa.py` 16 checks pass;
  end-to-end `StandaloneMatchRunner --climb` → `sim_epa.py` produces component
  tables and bounded penalties. The expression-tree library is covered by
  `AstExpressionTest` (9/9: evaluation, natural veto, tier masking, genome codec
  round-trip, malformed-genome rejection, genetic-operator well-formedness) and
  `CycleScoreHubParityTest` (2/2: the engine-parity grid + the Scale / inclusive
  gate codec).
  Fuel recycling added 2026-10-08
  (`recyclingKeepsTheFieldStocked`); full suite re-run the same day: 73 files /
  637 tests, 0 failures (clean re-run — see `docs/CHANGELOG.md`).
- Next review due: 2026-11-07, or when a clean sequential full-sim sweep can run
  the validity gate (§5.2) and confirm the standalone's margin rank, not just its
  level and winner.

## Related

- `KNOWN_ISSUES.md` §E — standalone fidelity gap, score-as-fitness blockers.
- `docs/SCORE_RIG_RESULTS.md`, `docs/SWEEP_FINDINGS.md` — why the full-sim rig
  cannot yet be the optimizer's target.
- `docs/KNOWLEDGE_MODEL.md` — clairvoyant vs observed, which terminals a genome
  may legally read.
- `docs/AI_CO_PILOT_GUIDE.md` — weight-level tuning (P5) and proposed deltas.
