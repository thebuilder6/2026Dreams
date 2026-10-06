---
title: Score Rig — Measured Results
audience: [human, ai]
owner: leads
last_verified: 2026-10-06
status: authoritative
---

# Score Rig — Measured Results (2026-09-28)

Evidence log for the score measurement rig. **What the numbers are**, not how to run
it — commands live in `SIMULATION_GUIDE.md`, the contract in `ARCHITECTURE.md` §3L,
and the open questions in `KNOWN_ISSUES.md` §E.

## Scope

Covers: the first noise-floor baseline of the headless 3v3, the sequential control
that isolated the variance mechanism, and the nine defects the rig surfaced in its
first day of use (four match defects 1-4 plus five rig-validity defects 5-9).

Does **not** cover: any claim that a policy change improved score. No such comparison
has been run, and none can be until §"Verdict" below is resolved.

## 1. Headline

**The headless 3v3 is not currently usable as a fitness function.** Two independent
causes, only one of which is ours:

| Cause | Ours? | Effect |
|---|---|---|
| CPU / port contention in parallel sweeps | yes — mitigable | outright score collapse (three replicas at ~7-62, one marginal at 107) |
| Unseeded MapleSim physics | no — upstream | ±124 point spread even run one at a time |

`SE = 53.4` on a mean Blue score of ~180 → a 95% CI of **±105 points**. Any policy
change worth less than that is indistinguishable from a re-run. `compare.py` is
written to refuse to certify a variant in exactly this situation.

## 2. Noise floor — 8 seeds × 2 replicates, 150 s, 12-way parallel

Command: `tools/score/sweep.ps1 -Seeds 7,11,42,101,500,1337,2026,9999 -Replicas 2`
→ 16 matches in **352 s** wall (48 min of match time, 8× speedup).

Paired `blueTotal` deltas, same seed, same code:

| seed | r0 | r1 | Δ |
|---|---|---|---|
| 7 | 191 | 199 | −8 |
| 11 | 184 | 197 | −13 |
| 42 | **7** | 240 | **−233** |
| 101 | 265 | 279 | −14 |
| 500 | 235 | **12** | **+223** |
| 1337 | 166 | 262 | −96 |
| 2026 | 202 | **107** | **+95** |
| 9999 | **61** | 274 | **−213** |

`sd=151.03  SE=53.40  median=−13.5  max|Δ|=233`

**4 of 8 seeds collapsed in exactly one replica** (three deep collapses to ≤61 and one marginal — seed 2026 at 107, still a 95-point replica split with the same stall-free holding signature). The median/sd gap is the
diagnostic: a median of −13.5 against an sd of 151 is not noise, it is a discrete
failure mode sitting inside a mostly-tight distribution.

## 3. What a collapse looks like

Seed 42 replica 0 (Blue 7) against a healthy seed 42 replica (Blue 240):

| bot | collapsed | healthy |
|---|---|---|
| `blue[0]` AUTONOMOUS_CYCLER | fuel 0, **13.7 m** | 475 m |
| `blue[1]` ADAPTIVE_COMPETITOR | fuel 2, **10.0 m** | 483 m |
| `blue[2]` TACTICAL_DEFENDER | fuel 5, 42.6 m | 428 m |
| `red[0]` AUTONOMOUS_CYCLER | fuel 100, 376 m | — |

- **Red is unaffected.** Blue alone collapses.
- **AUTO fuel is normal** (3–18). The collapse is entirely in teleop.
- **The bots are not stuck.** `maxContiguousStallSec` 0.3–0.7 s, consecutive-recovery
  count 0. The stall predicate never fires because the bots are *commanding* ~zero
  speed — they are choosing to hold. This is the case that justifies gating on
  distance travelled as well as stall: the freeze metric alone reports "healthy"
  for a robot that never moved.
- Collapsed runs also produce a visibly smaller `.wpilog` (~5.6 MB vs ~20 MB).

## 4. Sequential control — isolating the mechanism

Seed 42, two replicas, `-MaxWorkers 1` (one match at a time):

| | r0 | r1 | Δ | collapse |
|---|---|---|---|---|
| **Sequential** | 147 | 271 | 124 | **no** |
| Parallel ×2 | 62 | *process exited before FINAL* | — | yes |
| Parallel ×12 | 7 | 240 | 233 | yes |

**The collapse is contention-induced.** Under load the workers log:

```
NT: NT3 server socket error: address already in use
NT: NT4 server socket error: address already in use
CS: ERROR: bind() to port 1182 failed: Only one usage of each socket address
Warning: IterativeRobotBase: Loop time of 0.02s overrun
```

Every worker contends for NT3 1735, NT4 5810, CameraServer 1181-1182, WebServer 5800,
and PortForwarder 5801-5805. `GameSim.updateSimulationTime` selects its clock branch
by whether the DriverStation reports a valid match time, so a load-induced loop
overrun can flip that branch mid-match and change what `timeUntilHubShift` returns
to the Jev engine. Mitigation landed: `Robot.robotInit` no longer starts the
WebServer or PortForwarder when `frc.headless` is set (six bindings removed). NT and
CameraServer still contend.

**But contention is not the whole story.** Sequential replicates still differ by 124
points, and **AUTO fuel alone swung 35 vs 51**. AUTO is a 15 s phase over an
identically-seeded fuel layout (`GameSim.rng` from `scenario.seed()`), so a 16-point
swing there means the divergence is in the physics, not the decision layer. This
independently reproduces the earlier `-PfreezeObjective` isolation result recorded in
`KNOWN_ISSUES.md` priority 5. `MatchDeterminism` seeds only *our* `Random` uses; it
cannot seed MapleSim's solver or its static `RebuiltHub.rng`.

## 5. Defects the rig surfaced

| # | Defect | Evidence | Status |
|---|---|---|---|
| 1 | **Per-bot fuel attribution leak.** `recordPlayerScore` called `recordFuelScore` directly, so player fuel reached the alliance total with no bot slot. Structurally Blue-only: the player always shoots at its own hub. | 13 of 20 archived reports missed the per-bot sum by 2–10, always on Blue. Live confirm: `playerBlueFuel=5`, `78+5=83=blueTotal`. | Fixed; residual + canary getters, report line, and tests |
| 2 | **Training archetypes discarded.** `AIRobotInstance.update()` re-derived the archetype every tick from dashboard choosers, *after* the scenario applied it. No operator → choosers return null → hardcoded defaults won. | `default3v3`'s `TACTICAL_DEFENDER` never appeared in any headless match; Blue had no defender. Post-fix: Blue `[CYCLER, ADAPTIVE, TACTICAL_DEFENDER]`, Red `[CYCLER, ADAPTIVE, DEFENSE_BULLY]`. | Fixed; scenario is authoritative when active |
| 3 | **Replay logs overwrote each other.** `resolveLogPath()` stamped to the second, so both replicas of a seed started together wrote the same path. | 16 matches produced 8 wpilogs. | Fixed; variant + replica in filename via `logTag` |
| 4 | **`compare.py` false green on a single pair.** `sd` fell back to `0.0`, printing "CI ±0.0 / 1 seed is sufficient". | Single-replica run. | Fixed; refuses to form a CI below 2 pairs |
| 5 | **Markdown reports overwrote each other too** — the same second-resolution collision as #3, on the *report* path, which the #3 fix missed. | Two workers, one file; the archive survived twice by 1 s. | Fixed; `reportFileName` reuses `logTag` so one rule covers both |
| 6 | **Every worker was recorded "ok" while being contaminated.** Success was only "a FINAL line exists and a JSONL row exists", so loop overruns, port collisions and a live dashboard all passed silently. | Archived 12-way batch: 27-53 WPILib overrun warnings per 150 s match including one `robotPeriodic()` epoch of 0.81 s, and all 16 rows logged `ok`. | Fixed; `degraded` / `why` audit in `sweep.ps1`, non-zero exit, `-MaxWorkers` default 12 → 4 |
| 7 | **A live dashboard was sharing each worker's NetworkTables namespace**, including the `Auto Mission` chooser read at `autonomousInit`. | An `elastic_dashboard` process had been running since 10:52; every worker logged `NT: CONNECTED NT4 client 'Elastic@1'`. Two workers logged *different* `Auto selected:` values against the chooser's `Do Nothing` default. | Fixed; `sweep.ps1` refuses to start while a dashboard runs, `Robot.autonomousInit` pins the mission when headless |
| 8 | **A CPU-starved worker produced a *different* match rather than failing**, so a perturbed row was indistinguishable from a policy difference. | Width curve at 60 s: 2-wide and 4-wide clean (0 overruns, 4-11 ms), 6-wide not (2-13 overruns, 37-126 ms). | Fixed; `Sim/LoopHealth` stamps `health` into every row, `compare.py` excludes degraded rows and **refuses** unmeasured ones |
| 9 | **v1 rows cannot be distinguished from clean ones**, so a contaminated sweep could be silently re-read. | — | Fixed; `JSONL_SCHEMA_VERSION` 1 → 2, `compare.py` rejects a v1 row by name |

Defects 1 and 2 also invalidated earlier analysis in this repo: every per-role fuel-share
statistic derived from the 20 archived reports was reading mislabelled roles. Defects 6-9
invalidate the *baseline itself*: the 16-row 12-way batch is quarantined rather than
compared, which is why §2's numbers are historical evidence and not a control.

## 5a. Decision cards (physics-free probe)

`tools/score/DecisionCards.java` evaluates `evaluatePolicy` directly, so it needs no
match, no seed and no variance. Re-run 2026-09-28 on the current binary:

- **39 cards**, not the 45 previously recorded in `SIMULATION_GUIDE.md`,
  `KNOWN_ISSUES.md` §E and `docs/CHANGELOG.md`. Corrected in all three.
- **Alliance symmetry holds: every card's Blue and Red passes agree.** The conclusion
  stands; only the count was wrong. (The 18 false asymmetries in the tool's first version
  remain a useful historical note about flipping the seed and the hub flags together.)
- **`0 PASS / 0 MISMATCH / 39 UNREVIEWED`** — the `expected` column is blank for every
  card. The PASS/MISMATCH workflow was documented as if it had been exercised; it has
  not. The tool is currently a *report*, not a *gate*.
- **One card is `INVALID STATE` by design (`Z99`)** and the report says so explicitly
  ("must be reported INVALID STATE; if it is not, the checker is broken"). It is a canary
  for the schedule cross-check — **do not "fix" it.**
- **6 of 39 cards (15%) pick a different objective under the observed tier** — the same
  card, clock and geometry, with a robot that has no opponent tracker and no field-fuel
  sensor. Five of the six are `TACTICAL_DEFENDER` / `DEFENSE_BULLY` cards where the
  clairvoyant answer is a *defensive* objective (`LEAD_INTERCEPT`, `DENY_SHOOTING_LANE`)
  and the observed answer is `VACUUM_MIDFIELD` or `STAGE_STANDOFF`. This is direct
  evidence for the open item in `KNOWN_ISSUES.md` §A: the defensive policy is not
  merely degraded by the honest tier, it **disappears**. Note this card set does *not*
  reproduce the `RUSH_CLIMB` fallback seen in the unit tests — it falls to harvesting or
  staging instead. Both are the same class of defect (objective choice silently changes
  when information is removed); they differ in which wrong answer is picked.
- Two structural findings stand, unchanged: `inShootingRange` (<= 4.0 m from the hub)
  covers the whole home zone and the whole home half out to the centerline, and the
  harvest-deadline force at `JevDecisionEngine.java:614` overrides `minFuelToScore` with
  a hardcoded `heldFuelCount >= 8`, making the effective teleop threshold 1 in range and
  8 out of it. The 4 and 16 rungs are unreachable.

## 6. What the rig is good for today

As a **detector**, not an optimizer. In one day it surfaced the variance problem itself plus two latent match defects (1, 2) that were invisible to the unit suite, the
single-match smoke runs, and the markdown reports. A follow-up pass surfaced five more
about the rig's own validity (5-9). That is the return on the work so far.

As an **optimizer** it is blocked, and `compare.py` is designed to say so rather than
produce a confident wrong answer.

## 7. Verdict and next step

Do not start the `PolicyWeights` extraction yet. Parameterising ~25 utility weights is
only worth doing once a measurement can detect the effect; today a sweep would mostly
be measuring which run hit the collapse.

Two honest paths:

1. **Move the fitness function to the headless scripted player on a real Driver
   Station.** Directly actionable, makes AI opponents a fixed control instead of a
   variable, and is the only environment that validates the shift clock. Largest
   initial job. Note the objective is not continuous across this change — G418 pinning
   goes live, so pre/post baselines are not comparable.
2. **Keep the sim as a detector and wait on upstream physics seeding.** Cheaper, but
   leaves the optimization goal unmet, and the prior investigation already concluded
   this cannot be bisected by seed.

If a comparison is wanted sooner: run at **`-MaxWorkers 4`** (measured clean: 2-wide and
4-wide give 0 overruns and 4-11 ms max, while 6-wide gives 2-13 overruns and 37-126 ms),
accept `SE` far above the ~5-point threshold, and treat `compare.py`'s refusal as the
correct answer rather than a bug to work around. The earlier advice here was to cap
`-MaxWorkers 1`; that predated `Sim/LoopHealth`, which now measures the perturbation
in-process and **fails the sweep** on a degraded row instead of leaving you to judge by
eye. Do not go back to 1 as a ritual — it is slower and it hides the actual limit.

## Verification

- Verified against: full suite **45 files / 450 tests, 0 failures** (clean re-run, offline,
  `--no-daemon`, 2026-10-06). The "4 failing" episode of 2026-09-28 is retained as history in
  `KNOWN_ISSUES.md` §A and `docs/CHANGELOG.md`: all four tests pass on the current binary (two were
  renamed in the rewrite), and the count discrepancy (360 vs 361) was a stale-report artifact —
  450 `@Test` annotations are on disk and 450 execute.
  **Any "325 tests green" figure previously recorded here was wrong twice over** — wrong
  count, and the suite was red through the `MatchKnowledge` refactor until the Sep 29 clean re-run closed it.
- Baseline artifacts: `results/baseline-12way-contaminated.jsonl` (16 rows — **quarantined,
  do not compare against**), `results/diag_seq.jsonl` (2 rows, sequential control).
  `results/` is gitignored, so re-run the sweep to regenerate rather than expecting them
  in the repo. The 12-way baseline is retained only as the evidence for defects 5-9 above.
- Card re-run 2026-09-28: `tools/score/run-cards.ps1` reports **39 cards**, not the 45
  previously claimed here and in `SIMULATION_GUIDE.md` / `KNOWN_ISSUES.md` §E /
  `docs/CHANGELOG.md`. See "Decision cards" below.
- Next review due: 2026-10-28

## Related

- `SIMULATION_GUIDE.md` — commands for the sweep and comparison tools
- `ARCHITECTURE.md` §3L — the rig contract (JSONL schema, guardrails, frozen parameters)
- `KNOWN_ISSUES.md` §E — the open score-as-target work and the variance blocker
- `KNOWN_ISSUES.md` priority 5 — the pre-existing simulation-fidelity investigation
- `docs/CHANGELOG.md` — per-change entries for everything in §5

