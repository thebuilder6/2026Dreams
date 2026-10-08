---
title: Match Knowledge Model
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-07
status: current
---

# Match Knowledge Model

## Scope

Covers **what the decision layer is allowed to know**, and why that is modelled as two
types (`ClairvoyantKnowledge` / `ObservedKnowledge`) rather than one record plus a
flag. Owns the boundary between honest sensor truth and operator omniscience, the
zone-fuel counts, and the field fuel positions (`fieldFuel()`) that used to bypass the record entirely.

Explicitly **not** covered:

- **Vision-tracked opponent estimates for the real robot.** That is the missing sensor
  that would let `ObservedKnowledge` carry a non-empty opponent list; it is
  `KNOWN_ISSUES.md` §E, not here.
- **The no-information policy path.** This doc records that it is missing; designing it
  belongs to the decision layer (`JevDecisionEngine`).
- Hub schedule rules, utility weights, and the card harness's own mechanics.

Two distinct kinds of knowledge, deliberately separate types rather than one record
with a flag. The previous `MatchKnowledge` conflated them: a single record with an
`opponentObserved` boolean, where the sim-sparring tier carried the data and the
driver-assist tier was the same record with `false`.

## The two kinds (as implemented)

**Clairvoyant** — everything a human watching the match would know. All robot
poses (self, allies, opponents), all robot velocities, the live score
differential, roughly how many fuel each side holds and has scored, and the
**number of fuel pieces in each of the three field zones** (alliance zone,
midfield, opponent zone). In simulation the sim operator has all of this, so a
sparring bot runs clairvoyant.

**Observed** — only what the robot's own sensors can produce. Self pose from
odometry, its own hopper state (`Intake.hasFuel()`), a single detected fuel
piece (`Vision.getGamePieceFieldPose()`), a detected opponent bumper
(`Vision.registerDetectedBumperObstacle()`). It has **no** sensor for the
global fuel distribution and **no** tracker for other robots, so the zone counts
are zero and other robots are absent. Zero is not a degraded guess here — it is
the truth about what a real robot can know. It does know all the data about the shifts, such as time until next shift, and the current active hub.

The three zone boundaries are single-owned by `FieldMap.AllianceZones`
(`isInAllianceZone`, `isInMidfield`), so both tiers bucket by the same geometry.

## The defect this fixes

`JevDecisionEngine.countFuelInZone` reached directly into `SimulatedArena`
(`SimulatedArena.getInstance()`, via `MatchDeterminism.fuelOnFieldSorted()`) and
ignored the knowledge record entirely, behind a `catch (Exception)` that returned
`0`. Consequences:

- The tier gate was decorative for the whole harvest branch. `MatchKnowledge.unknown()`
  claimed opponents were unobserved while fuel was still read perfectly from the sim.
- The real robot got `SimulatedArena.getInstance() == null`, so it *always* took the
  `return 0` fallback. Every zone-fuel objective was silently unreachable on real
  hardware and the engine could not distinguish "no fuel there" from "I cannot see any".
- The card suite set hub state by hand but received **real** zone counts from
  whatever arena was live, so a card verdict could depend on unrelated sim state.

Fixing this means zone counts become a field of the knowledge type, computed once
per tick by the builder. `observed` reports zeros explicitly.

## Shapes

Two options were considered.

**Option A — keep one record, add the three counts and a tier enum.** Smallest diff
(5 construction sites). Keeps the existing failure mode available: a caller can
still build a record whose `opponentObserved` contradicts its contents, which is
what let the original bug through.

**Option B — two types behind a sealed interface, `MatchKnowledge` as the
supertype.** The engine keeps a single `MatchKnowledge` parameter, so the signature
does not change and the 5 call sites construct the concrete type instead. Each tier
exposes only the data it can honestly have, so an `observed` instance has no way to
report a zone count. Tier mismatches become compile errors rather than a runtime
boolean that can lie.

**Option B was chosen and is now implemented.** The engine reads 7 accessors
(`opponentObserved`, `scoreDifferential`, `alliesHeldFuel`, `allyPoses`,
`opponentPoses`, `allianceZoneFuel`, `opponentZoneFuel`), so the surface is small.
`midfieldFuel()` is carried and published to the cloud but has **no policy
consumer** — `countFuelInZone` folds the non-strict branch into
`allianceZoneFuel`. It is worth deciding whether to give it a consumer or drop it
from the interface, rather than leaving a knowledge field that looks load-bearing
and is not.

## Honest knowledge is not the same as safe behaviour

The 2-arg `evaluatePolicy(world, archetype)` overload and a `null` knowledge both
default to `ObservedKnowledge.selfOnly()`. That is the correct default for the
*honesty* problem described above — the old `legacyObserved()` claimed
`opponentObserved = true` with four empty lists. But it is a **double** change,
and the second half is not about honesty at all: `countFuelInZone` reads the zone
counts off the knowledge type, so the new default also drops all three to `0` and
disables every fuel-driven objective. An empty knowledge does **not** degrade the
policy to something safe — it empties the utility matrix, and the argmax then falls
through to whatever carries a non-zero base, which is `RUSH_CLIMB` for a defender.

So the type system fixes "a lying combination is inexpressible" and does *not* fix
"no information must not look like a confident decision". The second half is open
work in the decision layer, not in this design.

It is **unmeasured by any unit test**. The behaviour was first caught as 5 failing
tests, and those tests were then made green by passing an explicit
`ClairvoyantKnowledge` — right for tests that are about fuel-gated objectives, but
it left the defect with no test at all. ~27 call sites still go through the
empty-knowledge default, so the path is exercised and unasserted. It *is* measured
by the decision cards, in the next section. See `KNOWN_ISSUES.md` §A, "Passing
knowledge is load-bearing in both directions", and `ARCHITECTURE.md` §3J.

## Consequences for the decision cards

**Knowledge tier is now a card axis, and it is already paying for itself.** Every
card runs three times: Blue clairvoyant, mirrored Red clairvoyant, and the
observed tier (`ObservedKnowledge.selfOnly()`). The summary table carries an
`observed` column, and a card whose clairvoyant answer is not reproduced under the
observed tier is flagged `TIER DIFFERS`. The verdict is judged against Blue
clairvoyant, so this is measurement rather than a second opinion on the answer.

Current result across 39 cards: **6 tier diffs, five of them defender cards**
(`C5`, `F1`-`F4`; the sixth, `D3`, is an `ADAPTIVE_COMPETITOR` card). Defensive objectives read `world.opponentPose()`
(`DENY_SHOOTING_LANE`) or gate on `opponentObserved` (`LEAD_INTERCEPT`,
`CHOKE_TRENCH`), so with no poses the tier-1 gate at
`JevDecisionEngine.java:552-556` zeroes lane denial / shadow / intercept (the
`CHOKE_TRENCH` gate at `:526-527` is a separate `opponentObserved` check, and the
post-argmax fallback covering it is at `:664-671`) and the defender falls back to
`VACUUM_MIDFIELD` or `STAGE_STANDOFF`. That is the intended, honest consequence of the observed tier
rather than a bug — but it means **a defensive card can look correct while the
objective it chose is unreachable for a real robot**, which is precisely what a
single-tier suite cannot show. 33 of 39 cards are tier-invariant, so the split is
specific to vision-dependent branches and not a general property.

### The `oppObserved` column was dead, and that was a real defect

Building this axis exposed a harness bug worth recording, because the type system
was supposed to make the mistake inexpressible and the harness reintroduced it
anyway. `ClairvoyantKnowledge.opponentObserved()` is *derived* from
`!opponentPoses.isEmpty()` rather than passed as a flag. The card harness was
feeding `opponentPoses` from the `oppZonePieces` column while the `oppObserved`
column was read for nothing — so a card asserting "opponent visible" with zero
opponent-zone pieces claimed vision it did not have, and every defensive card
silently measured the *no-opponent* path.

The observable symptom was a false finding: `F3` (opponent hub live, opponent
0.09 m from their own hub) reported `VACUUM_MIDFIELD`, which looked like lane
denial being dead. It was not. With the tracked opponent now driven by
`oppObserved`, `F3` correctly returns `DENY_SHOOTING_LANE` at 0.989. The lesson
generalises: a derived field makes the record honest, but any *caller* that
populates the source of that derivation from somewhere else has reintroduced the
same lie through the back door.

## Target Selection Decoupling (Implemented 2026-10-07)

Target selection (identifying candidate fuel piece coordinates for harvest tours and cluster targeting)
previously bypassed the knowledge model by reading `SimulatedArena` directly. It is now routed
through `MatchKnowledge.fieldFuel()`:
- `ClairvoyantKnowledge` receives and exposes a defensive copy of field fuel positions populated via `WorldStateBuilder.snapshotZoneFuel()`.
- `ObservedKnowledge` returns an empty list, maintaining honest sensor truth on physical hardware.
- `JevDecisionEngine` and `FuelTourOptimizer` operate purely on `List<Translation2d>`, eliminating all direct dependencies on `swervelib.simulation.*` and `MatchDeterminism`.

## Verification

- Verified against: `.\gradlew test --offline --no-daemon`, 2026-09-28 — **36 files /
  360 tests, 2 failing**: `JevDecisionEngineTest.hubScheduleNeverLeavesBothHubsDark`
  (`isHubActive(_, DONE, _) && !isHubActive(_, SHIFT2, _)`, so both hubs *can* be dark
  in DONE) and `HubScheduleTest.afterNextPhaseAlternatesTheActiveHubAcrossShifts`
  (asserts the current phase's polarity for a method that evaluates the *next* one).
  Both are pre-existing and neither is a knowledge-model defect; see
  `KNOWN_ISSUES.md` §A. `compileJava` and `compileTestJava` pass.
  - An earlier run the same day reported **4** failures; the other two
    (`shuttlePassIsAllowedWhenOpponentHubActivatesNext` /
    `shuttlePassStillAllowedWhenOpponentHubIsLiveNow`) are now green. Re-run on the
    current binary before citing any count — the suite is not hermetic and a stale
    figure here is worse than a red one, because it is trusted.
- `tools/score/run-cards.ps1` re-run the same day: 39 cards, 6 `TIER DIFFERS`,
  alliance-symmetric, `0 PASS / 0 MISMATCH / 39 UNREVIEWED`.
- **The knowledge model is not the cause of the 2 failures**, and the two concerns
  should not be conflated when either is fixed.
- Next review due: 2026-10-28

## Related

- `ARCHITECTURE.md` §3J — Jev System 1/2 architecture
- `KNOWN_ISSUES.md` §A — decisions, determinism, and the `blockedFuel` live-lock
- `tools/score/DecisionCards.java` — the card harness
- `docs/SCORE_RIG_RESULTS.md` §5a — card re-run results

