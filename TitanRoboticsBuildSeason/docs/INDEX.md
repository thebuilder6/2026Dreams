---
title: Docs Index
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Docs Index — TitanRoboticsBuildSeason

Single map for all durable human + AI knowledge. Code wins over prose on conflict
(`build.gradle` / source > any `.md`).

## Read order for AI agents (strict)

1. `AGENTS.md` (repo root) — build commands, generated-code ban, conventions.
2. This `INDEX.md` — locate the right guide, do not guess.
3. Topic guide below for the task at hand.
4. `KNOWN_ISSUES.md` (repo root) — check `[OPEN]` / `[PARTIAL]` before adding work.
5. `docs/RESOURCES.md` — external vendor docs, prefer over web search.

## Durable guides (do not duplicate — link, don't copy)

| Doc | Path | Covers | Owner |
|-----|------|--------|-------|
| Platform overview + quickstart | `TitanRoboticsBuildSeason/README.md` | Compile/sim/test/deploy commands, guide links (contracts live in `ARCHITECTURE.md`) | leads |
| Architecture contracts | `TitanRoboticsBuildSeason/ARCHITECTURE.md` | Subsystem contracts (§3A–I), Jev System1/2 (§3J), sim match engine (§3K), score measurement rig (§3L), 2027 roadmap (§4) | leads |
| Bot AI & Co-Pilot proposals | `TitanRoboticsBuildSeason/docs/AI_CO_PILOT_GUIDE.md` | Proposed Jev deltas (shift clock, shuttle/poach, tours, fleet, avoidance, optimizer) with measurement gates; not implemented | programming-leads |
| Simulation setup | `TitanRoboticsBuildSeason/SIMULATION_GUIDE.md` | SimGUI, Elastic 7 tabs, AdvantageScope, multi-bot, scoring, drills, score-rig commands | sim-owner |
| Score rig — measured results | `TitanRoboticsBuildSeason/docs/SCORE_RIG_RESULTS.md` | First noise-floor baseline, sequential control isolating the variance mechanism, the nine defects the rig surfaced (match 1-4 + rig-validity 5-9), why score is not yet a usable fitness function | leads |
| Match knowledge model | `TitanRoboticsBuildSeason/docs/KNOWLEDGE_MODEL.md` | Why `MatchKnowledge` is a sealed interface (`ClairvoyantKnowledge` / `ObservedKnowledge`) rather than one record plus a flag; which knowledge is real-robot obtainable | leads |
| Headless sweep findings | `TitanRoboticsBuildSeason/docs/SWEEP_FINDINGS.md` | 56-match sweep: why the rig cannot measure a policy change (blind spots, ~23% CV, bimodal results) and the contested-target deadlock it exposed | leads |
| Operator map | `TitanRoboticsBuildSeason/OPERATORS_GUIDE.md` | Dual-controller layout, haptics, Glide, dashboard tabs, drills | drive-team |
| Test mode | `TitanRoboticsBuildSeason/src/main/java/frc/robot/Test/README.md` | TestMode categories, controller layout, tuning workflow, safety | test-owner |
| Shooter tuning guide | `TitanRoboticsBuildSeason/docs/SHOOTER_TUNING_GUIDE.md` | Flywheel SysId characterization, PID feedback, empirical distance lookup tables, SOTF, and `calibrate_shooter.py` automation | leads |
| Intake tuning guide | `TitanRoboticsBuildSeason/docs/INTAKE_TUNING_GUIDE.md` | Pivot arm Profiled PID, ArmFeedforward (kS, kG, kV), jam detection, trench geofence, and `calibrate_intake.py` | leads |
| Pit calibration checklist | `TitanRoboticsBuildSeason/docs/PIT_TUNING_CHECKLIST.md` | Pre-match 5-station rapid check: mechanical clearance, automated pre-flight, sensor zeroes, 3-shot carpet benchmark | drive-team |
| External links | `TitanRoboticsBuildSeason/docs/RESOURCES.md` | Merged vendor doc URLs (Photon, Choreo, Limelight, MapleSim, WPILib, AdvantageKit, YAGSL, REV, Elastic) | leads |
| Issues + roadmap | `KNOWN_ISSUES.md` (repo root) | §A–D resolved history, §E desired features, §F test roadmap, §G multi-robot | leads |
| Agent rules | `AGENTS.md` (repo root) | Build env, generated-code ban, Blue-origin, Alert/LED, TunableNumber, timing quirks, resource-coordination protocol | leads |
| Resource coordination | `TitanRoboticsBuildSeason/docs/COORDINATION.md` | The four shared resources (`gradle-build` / `sim-gui` / `sweep` / `deploy`), the `tools/lock` protocol, why ports cannot be offset, and recovery recipes for the two interrupt bugs it closed | leads |
| Nav roadmap diagram | `TitanRoboticsBuildSeason/docs/nav/roadmap.html` | Interactive SVG: 38-node roadmap, obstacle AABBs, 0.45 m inflated footprints, trench bands + drivable centres, per-node clearance. Guarded by `tools/nav/verify_roadmap.py` (106 checks) | leads |
| Navigation freeze mechanisms | `KNOWN_ISSUES.md` §I | The six robot-freeze mechanisms closed Sep 29 (trench self-masking, corridor severing, wall pockets, peer-independent unstick, pinned-churn, fuel-target churn) and what is still unmeasured | leads |
| Autonomous & Choreo guide | `TitanRoboticsBuildSeason/docs/AUTONOMOUS_GUIDE.md` | Custom action/mission framework, Choreo trajectory pipeline, WaitUntilMarkerAction, dynamic auto-discovery, creating new missions | programming-leads |
| Vision platform guide | `TitanRoboticsBuildSeason/docs/VISION_GUIDE.md` | Dual-vision platform: Limelight 3/3G MegaTag2, Orange Pi 5 PhotonVision, YOLOv8 Ball Hunt, median filtering, dynamic std-devs, port forwarding | programming-leads |
| Swerve drive tuning guide | `TitanRoboticsBuildSeason/docs/SWERVE_TUNING_GUIDE.md` | SDS MK4i specifications, CANcoder zero-calibration, wheel radius via `tune.py`, YAGSL PIDF, and SysId characterization | programming-leads |
| Developer & student onboarding | `TitanRoboticsBuildSeason/docs/ONBOARDING.md` | Onboarding path, 4-layer mental model, conventions (singletons, Blue-origin, alerts, tunables), test workflow, resource locks | programming-leads |
| Minimal rebuild checklist | `REBUILD_MINIMAL_CHECKLIST.md` (repo root) | Self-contained step-by-step checklist to rebuild drivable robot from scratch (swerve, intake, shooter, auto, teleop, AdvantageKit, sim) | leads |
| Rebuild sprint plan & tickets | `SPRINT_PLAN.md` (repo root) | 5-sprint project plan and tickets (Sprints 0–5) implementing the minimal robot rebuild | leads |
| Changelog | `TitanRoboticsBuildSeason/docs/CHANGELOG.md` | Agent-maintained per-change log | any-agent |
| New-doc template | `TitanRoboticsBuildSeason/docs/_TEMPLATE.md` | Required frontmatter + sections for new guides | leads |

## Ephemeral / generated (never treat as durable spec)

| Path | Type | Rule |
|------|------|------|
| `TitanRoboticsBuildSeason/reports/match_coach_report_*.md`, `TitanRoboticsBuildSeason/reports/headless_match_*.md`, `TitanRoboticsBuildSeason/reports/practice_review_*.md` | Generated coaching/headless/review output | Read for context only; never edit by hand; never parse programmatically (use the rig JSONL); prune older than 30 days |
| `TitanRoboticsBuildSeason/results/*.jsonl` | Score-rig match artifacts (one JSON object per match) | Machine-readable sweep output. `compare.py` refuses an unknown `schemaVersion`; rows from mixed git shas are reported as non-comparable |
| `TitanRoboticsBuildSeason/results/decision_cards.md` | Decision-card report | Generated by `tools/score/run-cards.ps1`; edit the TSV, never this |
| `TitanRoboticsBuildSeason/AdvantageScope *.json` | AdvantageScope layout export | Generated on export; gitignored. Do not commit a dated export |
| `TitanRoboticsBuildSeason/tools/score/decision_cards.tsv` | Decision-card **input** (hand-edited, tracked) | The one score-rig file a human is meant to edit: fill the `expected` column. It is currently blank for all 39 cards, so `run-cards.ps1` reports every card `UNREVIEWED` — the tool is a report, not a gate, until that column is filled |
| `src/main/java/frc/robot/BuildConstants.java`, `src/main/deploy/git_info.json` | Generated by `generateBuildConstants` | Never hand-edit |
| `.apt_generated*/`, `build/` | Annotation / build output | Never edit, never cite as spec |
| `.agents/teamwork/` (see `ARCHIVE.md`) | Transient orchestration chatter | Only `ORIGINAL_REQUEST.md`, `orchestrator_1/PROJECT.md`, `orchestrator_1/plan.md`, worker `handoff.md` summaries are history; `BRIEFING/DISPATCH/progress/*.json/*.py` are scratch |

## Where things live (so nobody re-discovers)

- Gradle project root: `TitanRoboticsBuildSeason/` — all `./gradlew` runs from here.
- Inter-agent locks: `tools/lock/` (`Lock.psm1` + `acquire.ps1` / `release.ps1` / `status.ps1` + `with-lock.ps1` for one command under a lock), state in the gitignored `TitanRoboticsBuildSeason/.locks/`. Guards `gradle-build`, `sim-gui`, `sweep`, `deploy`; see [Resource Coordination](COORDINATION.md) and `AGENTS.md` §Resource coordination.
- Dev gates (no lock needed): `tools/dev/check.ps1` (pre-flight: JDK + dirty-tree stamp + dashboard + lock probe; `-ForSweep` is strict), `tools/dev/verify.ps1` (single automated verifier: roadmap + counts + docs-contract, optional rig-schema), `tools/dev/sync-test-counts.ps1` (suite numbers from measured XML; refuses partial results dirs), `tools/dev/check-docs.ps1` (Docs-Contract gate: CHANGELOG touch, INDEX row, frontmatter).
- Deploy dir: `TitanRoboticsBuildSeason/src/main/deploy/` (`example.txt` explains RoboRIO deploy semantics).
- Elastic layout source: `TitanRoboticsBuildSeason/src/main/deploy/elastic-layout.json` (served on port 5800).
- Coaching tool: `TitanRoboticsBuildSeason/tools/coaching/jev_coach.py` (`--live` / `--report`).
- Sim scoring: `Sim/HubSchedule.java`, `Sim/ShotTracker.java`, `Sim/MatchScoreTracker.java`, `Sim/RefereeSim.java`.
- Jev brain: `Intelligence/JevDecisionEngine.java`, `Intelligence/WorldState.java`, `Intelligence/MatchKnowledge.java` (sealed: `ClairvoyantKnowledge` / `ObservedKnowledge`, see [Match Knowledge Model](KNOWLEDGE_MODEL.md)), `Intelligence/StrategicObjective.java`, `Intelligence/AIActionIntent.java` (plus `StrategicPlan.java`, `WorldStateBuilder.java`, `TypeSafeJevClient.java`, `AutonomousTeleopAgent.java`).
- Nav roadmap diagram: `TitanRoboticsBuildSeason/docs/nav/roadmap.html` (open in a browser; no build step). Re-verify after any `FieldMap` / `StaticPathfinder` geometry change: `python TitanRoboticsBuildSeason/tools/nav/verify_roadmap.py`.
- Training + headless: `Sim/TrainingMatchScenario.java`, `Sim/HeadlessMatchDriver.java` (flags in `build.gradle`: `-Pheadless`, `-Pseed/-PdurationSec/-PautoSec/-PdisabledGapSec/-PbootWaitSec/-PfieldFuelCount/-PlogDir/-PreportDir/-PresultJsonl/-Pvariant/-Preplica`, opt-in `-PrealDs`).
- Match reproducibility: `Sim/MatchDeterminism.java` (named per-purpose seeded PRNG sub-streams + stable fuel ordering) — Common Random Numbers, not bit-exact replay.
- Score measurement rig: `tools/score/sweep.ps1` (parallel resumable variant×seed×replica sweep), `tools/score/compare.py` (paired comparison, bootstrap CI, role-aware guardrails), `gradlew dumpSimLaunch` (authoritative headless launch recipe), `Sim/BotMatchMetrics.java` (per-bot path length / longest stall / consecutive recoveries), `results/*.jsonl` (match artifacts, gitignored). Contract in `ARCHITECTURE.md` §3L; commands in `SIMULATION_GUIDE.md`; **measured evidence in `docs/SCORE_RIG_RESULTS.md`**.
- Jev decision cards: `tools/score/DecisionCards.java` + `run-cards.ps1` + `decision_cards.tsv` (editable) — physics-free probe of the decision layer. **39 cards** (re-verified 2026-09-28; the "45" previously quoted in several docs was wrong). Each card runs three times — Blue clairvoyant, mirrored Red clairvoyant, observed tier — and prints the chosen objective, the full intent, and a held-fuel sweep. Report lands in `results/decision_cards.md`. **Current state: alliance-symmetric (all Blue/Red pairs agree), `0 PASS / 0 MISMATCH / 39 UNREVIEWED` because the `expected` column is blank, `Z99` is `INVALID STATE` by design (schedule-check canary — do not fix), and 6 cards flag `TIER DIFFERS`.** See `ARCHITECTURE.md` §3L, `docs/KNOWLEDGE_MODEL.md`, and `KNOWN_ISSUES.md` §A/§E.
- Tuning CLI suite: `TitanRoboticsBuildSeason/tools/tune/tune.py` (interactive terminal suite unifying shooter ballistics, intake kinematics, wheel radius calibration, and pre-match pit checklists).
- Mechanism calibration scripts: `TitanRoboticsBuildSeason/tools/tune/calibrate_shooter.py` (2D trajectory ballistics solver + lookup table generator) and `TitanRoboticsBuildSeason/tools/tune/calibrate_intake.py` (arm kinematics, gravity feedforward $kG$, trapezoid profile transit times).
- System identification: `Test/SysIdManager.java` (consolidated authoritative 5-mechanism SysId manager for Swerve Linear/Angular/Steer, Shooter Flywheels, and Intake Arm Pivot; see `src/main/java/frc/robot/Test/README.md`).
- Navigation seed sweep: `TitanRoboticsBuildSeason/tools/nav/sweep-seeds.ps1` (multi-seed navigation repeatability and freeze sweep).
- Electrical thermal & power budget: `Hardware/BreakerModel.java` (120 A main-breaker I²t thermal accumulation and cooling) and `Hardware/PowerBudgetManager.java` (authoritative electrical throttling and brownout derating manager).
- Fuel tour optimization: `Intelligence/FuelTourOptimizer.java` (kinematic Traveling Salesperson fuel tour solver with 2-opt search).
- Autonomous missions catalog: `Auto/Missions/` (8 standard match routines registered in `AutoMissionChooser.java`; see [`AUTONOMOUS_GUIDE.md`](AUTONOMOUS_GUIDE.md)).

## Doc health

- Every durable doc carries YAML frontmatter: `title, audience, owner, last_verified, status`.
- `last_verified` = last date a human or green-build agent confirmed the doc matches code.
- Stale threshold: 30 days without verification → mark `status: needs-review`.
- `AGENTS.md` §Docs Contract is the enforcement rule; this index is the map.

