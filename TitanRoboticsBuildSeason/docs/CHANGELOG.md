---
title: Changelog
audience: [human, ai]
owner: any-agent
last_verified: 2026-09-26
status: living
---

# Changelog (agent-maintained)

Newest first. One bullet per behavior-affecting change. Format:

`- YYYY-MM-DD — <area>: <what changed> (<test evidence>) [docs touched]`

## Unreleased

- 2026-09-26 — issues/roadmap: grouped related open work into a ranked top-five view (navigation safety, scoring execution, multi-robot coordination, full-sim validation, cloud readiness); corrected source test counts and status legend, and clarified wall-test failures vs coverage. No code change or test execution. [KNOWN_ISSUES.md]
- 2026-09-26 — roadmap: recorded the `DENY_SHOOTING_LANE` failure where defensive bots drive into the opponent Hub footprint; proposed a reachable outside staging target and alliance/path validation. No code change or tests. [KNOWN_ISSUES.md]
- 2026-09-26 — navigation: default obstacle handling is `IMPASSABLE`, and every split Hub core, ramp, and trench-wall AABB now blocks in all legacy mode labels; clearance uses the 0.45 m bumper half-width. A* validates roadmap edges and endpoint connectors against active inflated obstacles, and no route returns no movement instead of blocked fallback links. If the measured start overlaps an inflated obstacle, the planner inserts an outward escape waypoint and the controller holds it until clear. Dashboard mode applies only on remote-value updates, so local setter calls cannot be reverted by stale entries. Waypoint-plane progression requires cross-track error below 0.45 m to avoid skipping missed tunnel turns. Ramp-face/corner and far-side paths, `FieldMapTest`, `AIRobotSimTest.testHubAndRampObstacleAvoidanceInPathfinder`, and all 16 `DriverAssistTest` tests pass; full sim confirmation remains. [ARCHITECTURE.md, KNOWN_ISSUES.md]
- 2026-09-26 — AI: enabled optional TypeSafe requests for simulator bots when the existing feature toggle is on. Each bot now has isolated request/result/error state, per-bot telemetry, and a 1 s dispatch interval; local admissibility/safety checks still decide whether a response can be used. `compileJava --offline` and `compileTestJava --offline` passed; tests were not executed for this review change. [ARCHITECTURE.md, OPERATORS_GUIDE.md, SIMULATION_GUIDE.md, KNOWN_ISSUES.md]
- 2026-09-26 — AI: added a shared FIFO TypeSafe dispatcher that spaces requests by 150 ms and coalesces queued bot snapshots, preventing all simulator bots from dispatching in the same tick. Per-bot one-second intervals remain; a per-match spending cap is not implemented. `compileJava --offline` and `compileTestJava --offline` passed; tests were not run. [ARCHITECTURE.md, SIMULATION_GUIDE.md, KNOWN_ISSUES.md]
- 2026-09-26 — navigation/docs: obstacle handling now reads the SmartDashboard mode on obstacle queries; legacy ramp union bounds derive from the split geometry and are deprecated. Added the requested navigation/controller/simulation cleanup items to `KNOWN_ISSUES.md`. Focused tests were not rerun after these review follow-ups. [ARCHITECTURE.md, KNOWN_ISSUES.md]
- 2026-09-26 — AI: added opt-in asynchronous TypeSafe System One objective selection, initially for the player Co-Pilot, with local fallback, 1.0 s request timeout, 1.2 s decision freshness, confidence/eligibility checks, high-priority local objective protection, and an autonomous own-half objective allowlist. The follow-up above extends the same path to simulator bots. The initial `JevDecisionEngineModeTest`, `JevDecisionEngineTest`, and `TierKnowledgeTest` passed (36 focused tests, offline; no API key used; deprecation checks clean). [ARCHITECTURE.md, OPERATORS_GUIDE.md, RESOURCES.md, KNOWN_ISSUES.md]
- 2026-09-26 — navigation: split physical Hub cores, trench divider walls, and ramps into separate `FieldMap` AABBs; added `ObstacleHandling` modes; apply one 0.40 m pathfinder margin and keep ramps out of hard fuel-footprint checks. Focused `FieldMapTest`, ramp-mode, trench transit, and alliance-mirror checks pass. Full 205-test rerun reports 4 failures: AI player deflection, one trench-heading assertion, and two tower-adjacent wall-target assertions; tracked in `KNOWN_ISSUES.md`. [ARCHITECTURE.md, KNOWN_ISSUES.md]
- 2026-09-26 — AI: autonomous batch-ready, in-range, and clock-low scoring now suppresses home-zone sweep utility so harvesting cannot outrank the dump decision. Focused `JevDecisionEngineTest` passed (27 tests); CoPilot shooter execution, long-range envelope parity, and far-side trench targeting are tracked in `KNOWN_ISSUES.md`. [ARCHITECTURE.md, KNOWN_ISSUES.md]
- 2026-09-26 — AI: added seven strategic objectives (16 total), a current/next `StrategicPlan`, hub-shift harvest cutoff, alliance/opponent-zone targeting, directional fuel scent, defensive intake subsumption, and plan telemetry. Shuttle launches remain gated to legal alliance-zone positions. Full suite green: 202 tests. [ARCHITECTURE.md, KNOWN_ISSUES.md]
- 2026-09-26 — sim: `TrainingMatchScenario` now runs Blue slot 0 as an independent AI instance, routes training targeting/match knowledge/score/climb attribution through the Blue roster, and parks/restores the physical `SwerveBase` around training. Existing seeded fuel layout, duration, and per-robot configuration remain. `compileJava --offline --rerun-tasks` passed; scenario tests were not rerun. Headless lifecycle and result summary remain unfinished. [ARCHITECTURE.md, SIMULATION_GUIDE.md, KNOWN_ISSUES.md]
- 2026-09-26 — drive: contact watchdog now compares requested vs measured field-relative speeds and produces field-relative escape vectors; architecture and known-issues docs updated. (`compileJava --offline` passed with WPILib 2026 JDK; tests not run)
- 2026-09-26 — roadmap: added low-priority post-AI-integration features (opportunistic subsumption intake/harvest bias, 2-step horizon `StrategicPlan` + shift budget + corridor broadcast) to `KNOWN_ISSUES.md` §E. (no code change)

- 2026-09-26 — docs: review pass vs code (25 files / 190 `@Test` / 7 Elastic tabs confirmed; `Telemetry/`, `Hardware/`, `Navigation/` moves confirmed). Fixed stale referee values (MINOR 5 / MAJOR 15, auto-contact/G407/G418/G420), sim-parity `[OPEN]`→`[PARTIAL]`, Dashboard path, DynamicRouter link, RESOURCES deletion note, SIM GUIDE Tab 5 ally controls + Tab 7 link; stamped frontmatter on 5 guides; moved `ControllerHapticsTest` to `Hardware/` (class green). (docs-only + test move)
- 2026-09-26 — docs: full 5-audit refresh (Test README class/controller/NT/LED/Robot-snippet fixes; Operators slew/gyro/glide-cancel/pin/haptic/tab/drill/archetype fixes; Simulation battery/tabs/spawn/archetype/GamePieces/range fixes; Architecture shooter/intake/vision/diag/watchdog/WorldState/bot-count/latency/clock/commands fixes; README diagram dedup; AGENTS test-count + 7-tab; KNOWN_ISSUES baseline stamp). (no code change)
- 2026-09-26 — docs: consolidated index + resources + changelog + AGENTS contract; archived `.agents/teamwork` chatter policy. (no code change)

## History (imported from KNOWN_ISSUES §A–D, §G Sep 25)

- 2026-09-25 — sim: `SimulatedBattery.disableBatterySim()` in `Robot.simulationInit()` kills MapleSim multi-bot brownout spam; `BatterySim` in `simulationPeriodic` stays authoritative. (154/154 green)
- 2026-09-25 — scoring: `ShotTracker` hub-capture fix + `MatchScoreTracker` auto/teleop splits + Scoreboard tab. (sim reconcile)
- 2026-09-25 — auto: `AutoAimAction` drives `Shooter` state machine (not raw kicker voltage).
- 2026-09-25 — bots: `RUSH_CLIMB` gated on `!isAutonomous()` + CO_PILOT-only; climb eval gated vs auto. (`JevDecisionEngineTest`)
- 2026-09-25 — bots: fill-then-volley auto batching (`AUTO_BATCH_MIN_FUEL` 6→8, dump at 5 s left).
- 2026-09-25 — bots: `DeadlockResolver` yield-and-jink recovery (1.0 s stall, 1.10 m peer, 0.7 s jink). (`DeadlockResolverTest`)
- 2026-09-25 — bots: wall-adjacent fuel targeting via `isPointInHardObstacle`; central threat-based mark selection (`AIRobotSim.selectMark`).
- 2026-09-25 — referee: `RefereeSim` G401/G201/G407/G418/G420 + `MatchScoreTracker` penalty subsystem. (`RefereeSimTest`)
- 2026-09-26 — navigation: made ramps impassable by default because the planner does not model slope/traction limits; validate roadmap edges against active inflated obstacles and fail closed when no route exists. Build/tests not rerun. [ARCHITECTURE.md, KNOWN_ISSUES.md]
