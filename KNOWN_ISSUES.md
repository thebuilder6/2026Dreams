# Known Issues & Desired Features

> Baseline Sep 26 2026 before field-map work: 25 test files, 202 `@Test` annotations on disk (full suite green after strategic-objective expansion). Current source snapshot: 27 test files / 214 `@Test` annotations. The latest full suite run recorded before TypeSafe integration had four failures (AI player deflection, intermittent trench heading correction, and two tower-adjacent wall-target cases); later targeted navigation and Jev checks are noted below. Counts are source annotations, not a claim that all tests currently pass. Historical counts below (154/154, 186/186, 24 classes) are checkpoints, not current totals.

Add entries as `- [ ] description`. Include repro or file refs so future sessions can verify.
Status tags: `[OPEN]`, `[PARTIAL]` (partly implemented), `[EXPLAINED]` (working as designed, UX problem), `[STALE]` (no repro since), and `[RESOLVED]` (fixed/closed). A checked box `[x]` marks a resolved item.

## Current Top 5 Priorities

Grouped by shared remediation and ranked by match safety, robot behavior, and validation risk. The detailed repros and status remain in the sections below.

1. **Navigation, obstacle clearance, and stuck recovery** — group the Smart Assist route/stall work and sparring-bot obstacle, wall-pickup, `CHOKE_TRENCH`, and `DENY_SHOOTING_LANE` issues. See [§A](#a-drive--performance), [§E](#e-desired-features-annotated-with-what-already-exists), and [§G](#g-multi-robot-interaction-4-of-5-done-sep-25-deadlock-recovery-is-v1).
2. **Safe, executable scoring actions** — wire CoPilot shooter/feed requests through the protected Shooter state machine and reconcile the 4.0 m simulator limit with the 4.3 m long-range utility. See [§E](#e-desired-features-annotated-with-what-already-exists).
3. **Multi-robot coordination** — add target/zone/corridor deconfliction and per-ally loaded-scorer context; existing deadlock recovery and threat-based marking are partial foundations. See [§E](#e-desired-features-annotated-with-what-already-exists) and [§G](#g-multi-robot-interaction-4-of-5-done-sep-25-deadlock-recovery-is-v1).
4. **End-to-end simulation and training validation** — finish the training-match lifecycle and result summary, then validate scoring, navigation, and roster behavior in full matches beyond focused unit/path tests. See [§B](#b-scoring-pipeline-reworked-sep-25--needs-a-full-sim-match-to-validate-end-to-end), [§E](#e-desired-features-annotated-with-what-already-exists), and [§G](#g-multi-robot-interaction-4-of-5-done-sep-25-deadlock-recovery-is-v1).
5. **Cloud Jev operational readiness** — retain opt-in/local fallback, complete credential and live-API validation when scheduled, and decide whether to add a per-match request/spend cap. See [§E](#e-desired-features-annotated-with-what-already-exists); live API testing is deferred.

## A. Drive / performance

- [x] `[RESOLVED]` Contact watchdog did not detect a blocked player robot and could issue a misdirected escape. **Fixed Sep 26: feed it measured and requested field-relative speeds, rotate robot-frame IMU acceleration to field coordinates, and keep escape vectors field-relative.**
- [x] `[RESOLVED]` Robot drives slowly on manual controls; suspect brownout logic. **Fixed: Simulation current draw model retuned.**
  - `SwerveBase.getSimulationCurrentDraw()` previously used an exaggerated heuristic (15A idle per module = 60A at rest, up to 160A drive alone). When summed with shooter and intake in `Robot.simulationPeriodic()`, the simulated battery voltage dropped below 9.5V, triggering brownout speed scaling down to 0.35 and slow recovery (+0.03/cycle).
  - Retuned `SwerveBase.getSimulationCurrentDraw()` to realistic physics (~0.5A idle per module quiescent, ~20A per module at full sprint = 82A max). Simulated battery voltage now rests at ~12.4V and drops to ~10.5V under full sprint with flywheels running, preserving full speed (scale 1.0) while protecting against real brownout spikes.
  - Related drive-feel fix (same day): `teleop.init()` is now called on `teleopInit`, resetting input limiters/slew state when entering teleop so stale limiter state can't cap initial response.
- [x] `[RESOLVED]` Smart Assist / Glide pathfinding issues (differs from AI sparring bots). **Fixed: Restored StaticPathfinder roadmap, eliminated Virtual Rail damper, and prevented 50Hz action reset thrashing.**
  - `DriveToPoseAction` constructor previously added `targetPose` to `waypoints` and passed it to `setExplicitWaypoints()`, which set `isExplicitPath = true` in `TrajectoryController`, permanently disabling `StaticPathfinder.findPath(...)` and driving blindly in a straight line into Hub/Ramp obstacles.
  - The Trench "Virtual Rail Damper" in `TrajectoryController` was overriding `vx`, `vy`, and `desiredHeading` while disabling dynamic avoidance whenever touching `y <= 1.55` or `y >= 6.50`, severely fighting both the pathfinder and the driver's shared-authority stick inputs. Replaced with clean heading alignment in low-clearance areas without corrupting holonomic translation or dynamic avoidance.
  - `AutonomousTeleopAgent` previously allowed `DriveToPoseAction` to finish immediately upon reaching 0.12m of standoff, destroying and recreating actions 50 times/sec. Added `setHoldPosition(true)` and dynamic `setTargetPose()` streaming so the robot holds its standoff stance smoothly, aims at the Hub on the fly, and transitions naturally when balls are depleted.
- [x] `[RESOLVED]` Static field obstacles double-counted bumper clearance and blocked the trench lanes. **Fixed Sep 26: FieldMap stores Hub cores, trench divider walls, and ramps separately; StaticPathfinder applies the 0.45 m bumper half-width once. All split pieces now block under every legacy mode label.** `FieldMapTest` covers mode selection and the Y=0.65/7.42 m trench routes.
- [ ] `[PARTIAL]` Tunnel-corner and far-side routes can stall when the controller skips a missed turn or the planner selects blocked links. **Code fix Sep 26: default mode is `IMPASSABLE`, A* validates roadmap edges and endpoint connectors, no-route results stop safely, stale local dashboard values no longer override mode setters, clearance uses the 0.45 m bumper half-width, and an unsafe start is evacuated to its projected clear pose before following the route. Waypoint-plane progression requires cross-track error below 0.45 m. Ramp-face/corner and far-side routes, `FieldMapTest`, `AIRobotSimTest.testHubAndRampObstacleAvoidanceInPathfinder`, and all 16 `DriverAssistTest` tests pass; full sim confirmation remains.**
- [ ] `[OPEN]` AI sim obstacle-avoidance assertions remain inconsistent in the full suite. The latest run failed player deflection and trench heading correction; trench centering passed on focused reruns. The player-deflection test fails alone with no lateral response, so investigate dynamic obstacle registration separately from the field map.
- [ ] `[OPEN]` Unify `StaticPathfinder` and `DynamicRouter` behind one clear navigation engine. Define ownership for static route planning, live obstacle response, replanning, and shared safety margins.
- [ ] `[OPEN]` Consolidate duplicate shared-authority blending and breakout detection in `AutonomousTeleopAgent.updateSmartAssist(...)` and `Teleop.java`; retain one implementation and test its authority boundaries.
- [ ] `[OPEN]` Clean up input deadbanding in `Hardware/Controller.java`; identify the canonical deadband stage and remove redundant shaping.
- [ ] `[OPEN]` Remove dead legacy math and redundant overloads across `Navigation/` and `Shooter`; verify callers before deleting compatibility entry points.
- [x] `[RESOLVED]` MapleSim brownout console spam (`[MapleSim] BrownOut Detected, protecting battery voltage...` every sub-tick via `DriverStation.reportError`). **Fixed Sep 25: `SimulatedBattery.disableBatterySim()` in `Robot.simulationInit()` (`Robot.java`).**
  - Root cause (verified in YAGSL 2026.1.14 sources): `SimulatedBattery` is a single **static** battery shared by every registered drivetrain — player + up to 3 opponents + 2 allies ≈ 48 motor sims on one 13.5 V model. Inevitable sag below brownout voltage → `reportError` every sub-tick (100s of lines/sec). The sagged voltage also fed our `SwerveBase` brownout throttle, compounding slow-drive symptoms in multi-bot sim.
  - `disableBatterySim()` is the library author's own escape hatch ("lazy quick fix to help the opponent simulation"): locks voltage to nominal 13.5 V, no more spam. Our `Robot.simulationPeriodic()` BatterySim model (runs after the arena update) remains authoritative for RoboRIO voltage, so our brownout protection still sees realistic sag. Sim-only call; compile + 154/154 tests green.
- [x] `[RESOLVED]` Console spam from exception-path `System.err`. **Closed per user Sep 25; residual risk noted.**
  - `GameSim` catch-blocks now use rate-limited `logRateLimitedError()` (committed). `AIRobotSim`/`AIRobotInstance` `System.err` calls (constructor attach failures, shot-launch errors) are still unthrottled but exception-only — if they ever spam, rate-limit them the same way.
  - No per-loop `System.out` found elsewhere (`Alert` never touches console; `Diagnostics`/`Vision` prints are event-driven). Per-loop SmartDashboard/NT writes (`Power/*`, `Scoreboard/*`, bot telemetry) are NT noise, not console.

## B. Scoring pipeline (reworked Sep 25 — needs a full sim match to validate end-to-end)

- [x] `[RESOLVED]` Score counting confusion. **Validated in sim Sep 25 night: scores reconcile.**
  - Shots into an **inactive** Hub score 0 by design. The Hub alternates per the official 2026 schedule (`Sim/HubSchedule.java`, rules 6.4/6.4.1; SHIFT 1 order seeded from AUTO fuel via `seedFromAutoResult()` in sim `teleopInit`), so ~half of teleop shots are *supposed* to score nothing.
  - Fixed along the way: `Sim/ShotTracker.java` (hub captures balls before analytic hit-time → hit-callback scoring dropped most scores), `MatchScoreTracker` auto/teleop fuel splits + foul points, and the committed "Match Scoreboard" Elastic tab (totals, Leader, fuel splits, fouls, climb).
  - Counter map for future confusion: `GameSim.score` (player only) vs `MatchScoreTracker` red/blue totals vs per-bot counts vs `ShooterSim` raw vs `RefereeSim` awards.
  - Test hygiene (Sep 25): `AIRobotSimTest` setup resets `GameSim` clock + `HubSchedule` every test (static hub state leaked between tests sharing a JVM).

## C. Autonomous (root causes fixed Sep 25; suite now 186/186 green)

- [x] `[RESOLVED]` "Advanced Choreo Shot" auto path does not shoot. **Fixed: `AutoAimAction` integrated with Shooter state machine.**
  - Refactored `AutoAimAction` (`Auto/Actions/AutoAimAction.java`) to command `shooter.shoot()` and `shooter.prepareToShoot()` instead of setting raw kicker voltage via `setKickerSpeed(...)`. The `Shooter` 50 Hz state machine now cleanly arbitrates flywheel velocity and kicker voltage in lockstep.
- [x] `[RESOLVED]` Bots perform poorly in autonomous — all take the same objective. **Fixed: Jev endgame logic gated on non-autonomous phase.**
  - Added `isAutonomous` tracking into `WorldState` (populated via `DriverStation.isAutonomous()` in `AIRobotSim`, `WorldStateBuilder`, and unit tests).
  - Gated `StrategicObjective.RUSH_CLIMB` and retreat defense in `JevDecisionEngine.java` on `!world.isAutonomous()`, preventing bots from treating autonomous (match time 15→0) as teleop endgame.
  - Gated `MatchScoreTracker.updateClimbEvaluation()` against autonomous mode, eliminating spurious climb scoring in auto.
  - Added unit test `testAutonomousModePreventsPrematureRushClimb` in `JevDecisionEngineTest.java`.

## D. JVM crashes (stale — keep logs, close on no repro)

- [x] `[RESOLVED]` Test-worker crash (`hs_err_pid12944.log`, `hs_err_pid51348.log`, `hs_err_pid55220.log`): `EXCEPTION_ACCESS_VIOLATION` in `wpiHal.dll` / C2 compiler crash under JDWP. **Fixed: Pinned Gradle test worker JVM to WPILib 2026 JDK. Suite now 186/186 green (24 classes), exit 0.**
  - Gradle test workers (`forkEvery = 1`) previously defaulted to the system Eclipse Temurin JDK, which failed native JNI calls in WPILib HAL. Explicitly set `executable = wpilibJava.absolutePath` in `build.gradle`, ensuring all test forks run on `C:\Users\Public\wpilib\2026\jdk\bin\java.exe`.
- [ ] `[EXPLAINED]` Sim prints `bind() to port 1181 failed` on startup. Non-fatal (CameraServer vs PhotonVision/Limelight sim ports; see `SIMULATION_GUIDE.md`). Ignore; listed so nobody "fixes" it.

## E. Desired features (annotated with what already exists)

- [x] `[RESOLVED]` Referee/penalty awareness in sim & penalty score tracking. **Implemented + expanded: `RefereeSim` & `MatchScoreTracker` penalty subsystem.**
  - `RefereeSim` enforces AUTO centerline contact (MAJOR), G407 alliance-zone shooting (MAJOR, `checkShotLegality` called on every player/bot shot in `ShooterSim`/`AIRobotSim`/`AIRobotInstance`), G418 pinning (MINOR at 3 s, MAJOR per extra uncorrected 3 s), and G420 tower protection in the last 30 s (MAJOR).
  - MINOR FOUL (5 pts) / MAJOR FOUL (15 pts) tracked in `MatchScoreTracker`, awarded to the opponent alliance total, published under `Scoreboard/Referee/*` and the scoreboard tab. Covered by `RefereeSimTest` + `HubShiftShotAllowanceTest` + `PlayerPickupShootTest`.
- [x] `[RESOLVED]` Coordinated bot autonomous plans + starting positions. **Fixed: Defense suppression and centerline isolation in autonomous mode.**
  - Bots spawn across staggered lanes (Y=2.25, 4.035, 5.80m).
  - In autonomous mode (`world.isAutonomous()`), defense archetypes (`TACTICAL_DEFENDER`, `DEFENSE_BULLY`, `LEAD_PURSUIT_INTERCEPTOR`) suppress illegal cross-field pursuit and lane denial (preventing FRC G201 centerline penalties). Bots with preloaded fuel prioritize scoring into the active hub, and fuel harvesting is strictly bounded to the alliance half (X <= 8.12m Blue, X >= 8.42m Red).
- [x] `[RESOLVED]` CAN bus utilization & motor current protection. **Implemented: CAN status frame throttling & smart current limits.**
  - Added `NeoSparkMaxMotor.optimizeCanBusUtilization()` throttling unused auxiliary sensors (analog, alternate encoder, absolute encoder) to 500ms and tuning velocity/position frames to subsystem needs.
  - Added smart current limits (40A arm pivot, 30A rollers, 30A hopper) on `IntakeIOSparkMax` and applied frame throttling across intake and shooter SparkMax controllers.
- [ ] Headless AI-vs-AI training matches + scenario control. `[PARTIAL]` `GameSim.resetGame(TrainingMatchScenario)` applies duration, seeded preplaced fuel, and the full Blue/Red rosters' archetypes, poses, and preloads. Training Blue slot 0 is now an independent `AIRobotInstance`; the physical `SwerveBase` is parked off-field and excluded from training targeting/scoring, then returned to its pre-training pose. The autonomous match lifecycle, headless runner, and result summary remain unwired.
- [ ] Test the AI "brain" outside full sim. `[PARTIAL]` `JevDecisionEngineTest` + `AIRobotSimTest` currently contain 61 `@Test` methods combined (27 + 34) — extend, don't start over.
- [ ] Better match UI (both alliances' scores, etc.). `[PARTIAL]` "Match Scoreboard" tab committed Sep 25 (totals, Leader, auto/teleop fuel, fouls, climb) — needs in-sim eyeball check during the §B validation run.
- [ ] Richer AI action/move options. `[OPEN]`
- [ ] `[OPEN]` CoPilot does not execute `SHOOTING` or `triggerFeedKicker` from `AIActionIntent` for the new long-range and shuttle objectives. `AutonomousTeleopAgent.manageSubsystems()` currently only handles flywheel preparation and intake; Teleop consumes navigation/aim but not the feed request. **Fix:** route shooter state and feed requests through the existing Shooter state machine with its RPM, alliance-zone, and operator-override interlocks; confirm the intended driver authority before enabling automatic feed.
- [ ] `[OPEN]` `LONG_RANGE_SNIPE` utility permits Hub distances through 4.3 m, while `AIRobotSim.isValidShootingLocation()` rejects distances above 4.0 m. **Fix:** choose one validated shared shot envelope for the decision engine, simulator, and shooter distance tables; don't expand the sim limit until trajectories and legality are validated.
- [ ] `[OPEN]` `CHOKE_TRENCH` activates for an opponent in either trench but computes target X using our own alliance trench bounds, so an opponent in the far-side trench can produce a target on our side. **Fix:** derive the target corridor from the occupied trench, then validate the cross-field route and intended defense behavior before enabling it.
- [ ] `[OPEN]` Defensive bots selecting `DENY_SHOOTING_LANE` often drive toward or into the opponent Hub and get stuck against its obstacle footprint. **Fix:** compute a lane-blocking point from the shooter-to-Hub line, clamp it to a reachable staging corridor outside the Hub/ramp safety bounds, and verify both alliances and opponent positions with path/obstacle tests.
- [ ] TypeSafe AI API for decisions. `[PARTIAL]` `tools/coaching/jev_coach.py --live/--report`, the opt-in player Co-Pilot, and enabled simulator bots can use the async TypeSafe client with `TYPESAFE_API_KEY`; credential provisioning and field validation remain. Cloud control is disabled by default. Each simulator bot has independent request/decision state and a 1 s dispatch interval; a shared FIFO dispatcher spaces callers by 150 ms and coalesces each queued bot to its latest snapshot. Local safety/admissibility checks gate every remote objective. There is no per-match request or spending cap, so five bots can still account for up to five requests per second. See `SIMULATION_GUIDE.md` for setup.
- [ ] Team coordination message system. `[OPEN]`
  - First step landed (parallel workstream, staged): `Sim/MatchKnowledge.java` — shared match picture tier (score differential, both sides' poses/velocities, held/scored estimates) built per bot via `WorldStateBuilder.buildMatchKnowledgeForSimBot`, with a 3-arg `evaluatePolicy(world, knowledge, archetype)` overload now used by `AutonomousTeleopAgent` and `MatchCoach`. Covered by `TierKnowledgeTest`. Score differential is consumed (+0.03 chase bias when behind); mark exclusion and zone agreements remain future work.
- [ ] Post-match LLM log review per bot (actions → suggested changes). `[PARTIAL]` `jev_coach.py --report` writes `reports/match_coach_report_*.md`; per-bot analysis + suggestions missing.
- [ ] Opportunistic subsumption behaviors ("2 things at once"). `[PARTIAL]` Defensive/staging intake subsumption and homeward dot-product harvest bias are implemented and tested. Low-clearance and full-hopper checks gate intake. Pin-duration evidence is not yet part of `WorldState`/`MatchKnowledge`, so `BAIT_PIN_FOUL` remains unavailable.
- [ ] 2-step horizon task planning (`StrategicPlan(current, next, timeToTransitionSec)`). `[PARTIAL]` Current/next objective, estimated shift budget, harvest cutoff, and co-pilot/sim-bot `NextIntent` telemetry are implemented. Trench corridor broadcast/yield is still open; ally loading remains aggregate knowledge and does not support per-ally loaded-scorer screening. Shuttle utility is gated to legal alliance-zone launch positions under G407.
- [ ] Practice/coaching mode UX (start/stop, driver-station/coach/bot loading). `[OPEN]`
- [ ] Coaching mode that makes sense. `[OPEN]`
- [ ] Robot-with-missing-parts mode (no intake/climb/vision). `[PARTIAL]` Intake encoder fallback + vision-degraded odometry exist; no general "missing subsystem" config.
- [ ] Driver-assist transparency (what it is doing). `[PARTIAL]` `CoPilot/Objective` published; needs UI/explanation pass.
- [ ] Sim info parity (robot sees only sensor/API data; bots may keep perfect knowledge). `[PARTIAL]` Two-tier model landed: co-pilot/coach evaluate with `MatchKnowledge.unknown()` (opponent unobserved, lane left to driver), sim bots get populated player-visible context. Covered by `TierKnowledgeTest`. Still open: vision-tracked opponent estimates for the real robot.
- [ ] Scale simulated game pieces without lag. `[OPEN]` (`Arena2026Rebuilt` efficiency mode + 54-ball cap is the current mitigation.)
- [ ] Sim→real→sim iteration workflow. `[OPEN]`
- [ ] AI-assisted robot/path/auto design. `[OPEN]`
- [ ] Streamlined AdvantageScope setup. `[OPEN]`
- [ ] Elastic UI cleanup across tabs; consider bespoke dashboard replacement. `[OPEN]`
- [ ] Git details + robot name on Elastic. `[PARTIAL]` Already published (`Telemetry/Dashboard.java:91-97` → `Build/*` keys from generated `BuildConstants`); likely just needs layout wiring.
- [ ] YAGSL feature review. `[OPEN]`
- [ ] Next-year readiness (new game, Systemcore, hardware). `[OPEN]` (`ARCHITECTURE.md` §4 roadmap: Elastic primary, AdvantageScope 3D, Telemetry/Tunables APIs, Commands v3 coroutines.)
- [ ] Competitor codebase survey for ideas. `[OPEN]`
- [ ] New-programmer friendliness + comprehensive docs + agent-maintained changelogs. `[OPEN]` (`AGENTS.md` + `Test/README.md` + guides exist; onboarding path doesn't.)
- [ ] Code best-practices review. `[OPEN]`
- [ ] `[OPEN]` Consolidate redundant SysID and characterization classes; compare overlapping lifecycle, mechanism selection, and command implementations before choosing a single owner.
- [ ] `[OPEN]` Review the responsibilities and utility of `AIRobotSim` versus `AIRobotInstance`; document the intended ownership boundary and remove duplicated orchestration where safe.

## F. Original doc-sourced roadmap (kept)

- [ ] Automated test sequences in TestMode
- [ ] Test data export + historical comparison
- [ ] Remote test control / advanced analytics
- [ ] Full AdvantageKit integration for test sessions

## G. Multi-robot interaction (4 of 5 done Sep 25; deadlock recovery is v1)

These are expected hard problems. The HOLD is lifted item by item as directed; intent-sharing and zone agreements remain future work.
- [x] `[RESOLVED v1]` Robots deadlock against each other — allies head-on in a trench, or multiple bots converging on the same spot, neither replans. **Fixed Sep 25 night: detection + randomized yield-and-jink recovery.**
  - New `Sim/DeadlockResolver.java` (unit-tested, seeded RNG): triggers after 1.0 s of commanded-but-stalled motion pressed within 1.10 m of a peer; recovery scales forward drive to 0.3× and adds a randomized ±1.2 m/s lateral jink for 0.7 s, then 2 s cooldown. Wired into `AIRobotInstance.update` (all sparring bots) and Bot 0 in `AIRobotSim`; state shown as `DEADLOCK_RECOVERY`, recoveries counted.
  - Deliberately deferred (still open as future work): intent-sharing (bots reading each other's advertised targets to deconflict destinations) and team zone/direction agreements.
  - Note: parallel workstream also touched this area (`MatchKnowledge` tier + 3-arg `evaluatePolicy` overload in `JevDecisionEngine`; also retuned `AUTO_BATCH_MIN_FUEL` 6→8). Shared-file edits interleaved — full suite re-verified green after both landed: 186/186.
  - Tests: new `DeadlockResolverTest` (6 tests: no-trigger cases, trigger timing, recovery length/cooldown/re-trigger, seeded determinism, reset).
- [ ] `[PARTIAL]` Bots don't pick up balls near walls or obstacles. **Last observed Sep 26:** `WallPickupTest.blueWallBallIsTargeted` and `redWallBallIsTargeted` project the approach around the tower footprint to >1.2 m from the ball. Revisit the tower footprint and wall-normal approach together; do not bypass hard-obstacle checks without validating physical clearance.
  - Root cause was the targeting filter, not the pickup radius: `StaticPathfinder.isPointInObstacle` treated the whole perimeter wall safety band as blocked, so the Jev scent search never proposed wall-adjacent balls. New `isPointInHardObstacle` (hub ramps, tower poles only — walls reachable via wall-normal approach) plus a dynamic-obstacle proximity check; fuel targeting uses those instead.
  - Test coverage: six `WallPickupTest` cases cover target selection for all four walls and standoff collection by both bot types. The two Blue/Red target-selection cases above were the observed failures; this coverage note does not mean all six currently pass. Temp repro file removed.
- [x] `[RESOLVED]` Bots attempt endgame climb but have no climber. **Fixed navigation only — climb scoring evaluation kept as-is per user.**
  - `JevDecisionEngine.evaluatePolicy`: `RUSH_CLIMB` utility gated on `archetype == CO_PILOT` (the only player-facing archetype). Bots keep playing in endgame (cycle/stage/vacuum/defend fall out of the normal utility race).
  - Belt-and-suspenders in `case RUSH_CLIMB`: `RUSH_CLIMB` is inserted first in the utility map and max-selection uses strict `>`, so an all-zero tie would previously select it — non-CO_PILOT now holds position instead of navigating to the tower.
  - `MatchScoreTracker.updateClimbEvaluation` intentionally untouched: bot/ally climb scoring still evaluated (bots just never navigate there on their own anymore).
  - Tests: `testEndgameRushClimbTransition` + teleop-endgame assert bots cycle while CO_PILOT climbs; `testAllyTowerClimbAttribution` unchanged. Green in the 186/186 suite.
- [x] `[RESOLVED]` Bots collect and shoot one ball at a time in auto instead of loading then firing volleys. **Fixed Sep 25 night: fill-then-volley batching in the Jev auto branch.**
  - `JevDecisionEngine`: `AUTO_BATCH_MIN_FUEL` (retuned 6→8 by parallel workstream) / `AUTO_DUMP_SECONDS_LEFT = 5.0`. In auto, bots harvest until a full batch before committing to a scoring trip; exceptions: already in shooting range (finish the volley) or auto clock nearly out (dump the hopper). Once committed, the existing 80 ms re-trigger in `AIRobotInstance`/`AIRobotSim` fires the volley.
  - Tests: new `testAutoFillThenVolleyBatching` (partial batch harvests, full batch commits, in-range finishes, low clock dumps); existing auto tests (10–18 fuel preloads) still cycle. Green in the 186/186 suite.
- [x] `[RESOLVED]` Defense bots only mark the player, never other opponents/allies. **Fixed Sep 25 night: central threat-based mark selection.**
  - New `AIRobotSim.MarkCandidate` + `selectMark` (pure function): threat = 3×held fuel + 1×scored fuel − 0.25×travel distance. Opponent defenders choose among player + ally bots; ally defenders choose among Bot 0–2. Jev engine untouched — the mark is simply passed as the opponent pose/velocity (`WorldStateBuilder.buildForSimBot` overload; `AIRobotInstance.update` overload).
  - Pin watchdog and Bot 0 pin tracking now reference the assigned mark instead of hardcoded peer-0/player. Active mark published per bot (`Simulation/BotN/Mark`, `Simulation/AllyN/Mark`, `Simulation/Bot0/Mark`) so target choice is visible on the dashboard.
  - Deliberately out of scope: zone constraints on defenders ("stay in zone" mused but not implemented).
  - Architecture decision (Sep 25): mark selection stays **outside** the Jev engine in `AIRobotSim`. The engine evaluates per-bot and statelessly, so two bots seeing the same enemy list would pick the same mark — only the central selector (which sees all bots) can later add exclusion/deconfliction. Natural follow-up: greedy mark assignment with exclusion so two defenders don't pile onto one carrier.
  - Tests: `testDefensiveMarkSelectsHottestCarrier` (loaded scorer beats nearby empty robot) + tie-break/fallback test. Green in the 186/186 suite.
