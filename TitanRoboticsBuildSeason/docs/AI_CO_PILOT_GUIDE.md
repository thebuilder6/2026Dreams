---
title: Bot AI & Driver Assist Co-Pilot Proposals
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-07
status: proposal
---

# Bot AI & Driver Assist Co-Pilot Proposals

Candidate deltas on top of the current Jev System 1/2 core. Nothing here is
implemented by landing this doc — each item names its measurement gate.

## Scope

Covers proposed deltas to the shared Jev core: shift-clock default, shuttle /
poach retunes, tour-waypoint execution, fleet partitioning, collision-avoidance
options, an optimizer harness, plus Co-Pilot crossover proposals (haptics/HUD
cues, Ball-Hunt tour wiring, SOTF assist).

Does NOT cover (link, don't copy):

- Current contracts: `ARCHITECTURE.md` §3J/§3K/§3L, `docs/KNOWLEDGE_MODEL.md`.
- Sim operation / score rig: `SIMULATION_GUIDE.md`, `docs/SCORE_RIG_RESULTS.md`,
  `docs/SWEEP_FINDINGS.md`.
- Controls / tuning: `OPERATORS_GUIDE.md`, `docs/SWERVE_TUNING_GUIDE.md`,
  `docs/VISION_GUIDE.md`, `docs/AUTONOMOUS_GUIDE.md`.

## Content

### Current shared core (ground truth, not a proposal)

- One engine, two tiers: `Intelligence/JevDecisionEngine.java:294-304`
  (`evaluatePolicy`), `ClairvoyantKnowledge` for sim bots vs
  `ObservedKnowledge.selfOnly()` for the real robot (`JevDecisionEngine.java:281-296`).
- Weights are parameterised: `Intelligence/PolicyWeights.java:96-119`
  (`DEFAULT`; override via `-Dfrc.jev.weights=`). Current defaults stay:
  shuttle `6.0 m / held>=16 / 0.87`, poach `0.78 / 0<t<=6.0 s`,
  `WorldStateBuilder.java:206` `useRealShiftClock=false`.
- Shared authority is single-owned in
  `Intelligence/AutonomousTeleopAgent.java:53-55,157-182`: thresholds
  `BREAKOUT_TRANSLATION=0.65`, `BREAKOUT_ROTATION=0.60`, `BLEND_MIN=0.10`
  (normalized by `MAX_SPEED` / `MAX_ROTATION_SPEED`). Translation blends as
  `(1-0.5*alpha)*planned + alpha*driver`; rotation blends as
  `(1-alphaRot)*omega + alphaRot*driver`. Breakout + `RUSH_CLIMB` arrival
  (`<0.12 m`) in `updateSmartAssist` (`:117-143`); `Teleop.java:392-419`
  delegates. Auto-feed interlocks (`:184-227`): in-zone + open-ceiling +
  `solutionOk` + heading within `ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG (3.0)`.
- Tours exist but execute one step: `JevDecisionEngine.java:828-881` consumes
  only `tour.immediateTargetPose()`; no production call feeds tour waypoints
  into `TrajectoryController.setExplicitWaypoints` (tests only).

### Proposed deltas (each needs its gate before landing)

- **P1 — Real shift clock.** PROPOSED, NOT IMPLEMENTED. Flip
  `useRealShiftClock` default only after ≥5 seeds x2 replicas on a clean tree
  plus a real-DS clock check (`KNOWN_ISSUES.md` Top-6 #2/#5).
- **P2 — Shuttle / poach retune.** PROPOSED, NOT IMPLEMENTED. Current
  shuttle gate is unsatisfiable in practice and poach `0.78` never wins the
  argmax by design (tripwire `testPoachUtilityIsCurrentlyUnreachable`).
  Any new distance/utility must first move `tools/score/decision_cards.tsv`
  expectations and survive a paired `compare.py` sweep; frozen params in
  `tools/score/compare.py:72-80` stay out.
- **P3 — Continuous tour execution.** PROPOSED, NOT IMPLEMENTED. Pass
  `FuelTourOptimizer` waypoints as explicit paths; A/B vs greedy scent on the
  rig before making it default.
- **P4 — Fleet partitioning.** PROPOSED, NOT IMPLEMENTED. No
  `partitionFuelCandidates` exists in `src/`; design a Voronoi / market
  assignment behind a flag and measure stalls + teleop share, not just score.
- **P5 — Collision avoidance + optimizer harness.** PROPOSED, NOT
  IMPLEMENTED. No ORCA / velocity-obstacle code in `src/` (`DynamicRouter`
  is APF); no `tools/tune/optimize_policy.py` exists. Score is not yet a
  usable fitness function (`docs/SCORE_RIG_RESULTS.md`), so no CMA-ES/Optuna
  loop until the rig noise floor is closed.
- **P6 — Haptic / HUD cues from shared intent.** PROPOSED, NOT IMPLEMENTED.
  Current truth: `HUB_PHASE_SHIFT` enum exists with no trigger caller
  (`Hardware/Controller.java:19`), `triggerDirectionalFlankAlert` has no
  production caller (`:137`), `TARGET_LOCKED` fires at `Teleop.java:656`
  only, and `MatchCoach.java:260-294` tips are `[HUB SHIFT]`, `[SCORING
  READY]`, `[STAGE STANDOFF]`, `[RELOAD]`, `[HUNTING]`, `[ENDGAME]` with
  grades A+–D (`:301-321`). Candidates (shift-countdown rumble, lock-confirm
  with the real 150 RPM kicker gate rather than 50, directional flank alert)
  must not override driver authority and need drive-team sign-off plus
  `DriverAssistTest` coverage before landing.
- **P7 — Ball-Hunt tour wiring.** PROPOSED, NOT IMPLEMENTED. Current Left
  Bumper is direct visual pursuit on `vision yaw/distance`
  (`Teleop.java:419-468`), not a `FuelTourOptimizer` arc. Wiring Hunt to
  `optimizeTour` goes behind a flag with a fallback to current pursuit, gated
  on cards + paired rig A/B + driver test.
- **P8 — SOTF assist for Co-Pilot.** PROPOSED, NOT IMPLEMENTED. Current SOTF
  is a 0.13 s predictive lookahead (`Subsystems/shooter/ShooterConstants.java:45`)
  with rotation override in `Navigation/TrajectoryController.java:100-110,341-374`.
  Free-translation / locked-yaw assist stays a proposal until the heading +
  solution interlocks (`AutonomousTeleopAgent.java:184-227`) are proven on
  hardware.

## Verification

- Docs-only proposal: code refs checked against HEAD 2026-10-07; no behavior
  change, no test re-run claimed.
- Gate: `tools/dev/check-docs.ps1` must pass (new-guide frontmatter + INDEX row).
- Next review due: 2026-11-06, or when any P1–P8 item produces card/rig evidence.

## Related

- `ARCHITECTURE.md` §3J/§3K/§3L — owning contracts.
- `docs/KNOWLEDGE_MODEL.md` — knowledge tiers + no-information path.
- `docs/SCORE_RIG_RESULTS.md`, `docs/SWEEP_FINDINGS.md` — why the rig gates P1–P5.
- `KNOWN_ISSUES.md` §A/§E — shift clock, poach/shuttle, tour, cloud cap.
