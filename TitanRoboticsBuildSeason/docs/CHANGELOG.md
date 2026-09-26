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
