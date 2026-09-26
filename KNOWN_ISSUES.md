# Known Issues & Desired Features

> Baseline Sep 26 2026: 25 test files, ~190 `@Test` on disk. Historical counts below (154/154, 186/186, 24 classes) are checkpoints, not current totals — re-baseline before citing.

Add entries as `- [ ] description`. Include repro or file refs so future sessions can verify.
Status tags: `[OPEN]`, `[EXPLAINED]` (working as designed, UX problem), `[STALE]` (no repro since), `[PARTIAL]` (partly implemented).

## A. Drive / performance

- [x] `[RESOLVED]` Robot drives slowly on manual controls; suspect brownout logic. **Fixed: Simulation current draw model retuned.**
  - `SwerveBase.getSimulationCurrentDraw()` previously used an exaggerated heuristic (15A idle per module = 60A at rest, up to 160A drive alone). When summed with shooter and intake in `Robot.simulationPeriodic()`, the simulated battery voltage dropped below 9.5V, triggering brownout speed scaling down to 0.35 and slow recovery (+0.03/cycle).
  - Retuned `SwerveBase.getSimulationCurrentDraw()` to realistic physics (~0.5A idle per module quiescent, ~20A per module at full sprint = 82A max). Simulated battery voltage now rests at ~12.4V and drops to ~10.5V under full sprint with flywheels running, preserving full speed (scale 1.0) while protecting against real brownout spikes.
  - Related drive-feel fix (same day): `teleop.init()` is now called on `teleopInit`, resetting input limiters/slew state when entering teleop so stale limiter state can't cap initial response.
- [x] `[RESOLVED]` Smart Assist / Glide pathfinding issues (differs from AI sparring bots). **Fixed: Restored StaticPathfinder roadmap, eliminated Virtual Rail damper, and prevented 50Hz action reset thrashing.**
  - `DriveToPoseAction` constructor previously added `targetPose` to `waypoints` and passed it to `setExplicitWaypoints()`, which set `isExplicitPath = true` in `TrajectoryController`, permanently disabling `StaticPathfinder.findPath(...)` and driving blindly in a straight line into Hub/Ramp obstacles.
  - The Trench "Virtual Rail Damper" in `TrajectoryController` was overriding `vx`, `vy`, and `desiredHeading` while disabling dynamic avoidance whenever touching `y <= 1.55` or `y >= 6.50`, severely fighting both the pathfinder and the driver's shared-authority stick inputs. Replaced with clean heading alignment in low-clearance areas without corrupting holonomic translation or dynamic avoidance.
  - `AutonomousTeleopAgent` previously allowed `DriveToPoseAction` to finish immediately upon reaching 0.12m of standoff, destroying and recreating actions 50 times/sec. Added `setHoldPosition(true)` and dynamic `setTargetPose()` streaming so the robot holds its standoff stance smoothly, aims at the Hub on the fly, and transitions naturally when balls are depleted.
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
- [ ] Headless AI-vs-AI training matches + scenario control. `[OPEN]`
- [ ] Test the AI "brain" outside full sim. `[PARTIAL]` `JevDecisionEngineTest` + `AIRobotSimTest` (32 tests) already do this — extend, don't start over.
- [ ] Better match UI (both alliances' scores, etc.). `[PARTIAL]` "Match Scoreboard" tab committed Sep 25 (totals, Leader, auto/teleop fuel, fouls, climb) — needs in-sim eyeball check during the §B validation run.
- [ ] Richer AI action/move options. `[OPEN]`
- [ ] TypeSafe AI API for decisions. `[PARTIAL]` `tools/coaching/jev_coach.py --live/--report` already calls it with `TYPESAFE_API_KEY`; robot-side (real-time) integration missing.
- [ ] Team coordination message system. `[OPEN]`
  - First step landed (parallel workstream, staged): `Sim/MatchKnowledge.java` — shared match picture tier (score differential, both sides' poses/velocities, held/scored estimates) built per bot via `WorldStateBuilder.buildMatchKnowledgeForSimBot`, with a 3-arg `evaluatePolicy(world, knowledge, archetype)` overload now used by `AutonomousTeleopAgent` and `MatchCoach`. Covered by `TierKnowledgeTest`. Score differential is consumed (+0.03 chase bias when behind); mark exclusion and zone agreements remain future work.
- [ ] Post-match LLM log review per bot (actions → suggested changes). `[PARTIAL]` `jev_coach.py --report` writes `reports/match_coach_report_*.md`; per-bot analysis + suggestions missing.
- [ ] Opportunistic subsumption behaviors ("2 things at once"). `[OPEN] [LOW PRIORITY — after AI integration]` Do not build before the package restructure + green test suite.
  - Cowcatcher intake: when the objective is defensive/transit (`LEAD_INTERCEPT`, trench choke, return-to-zone) and not low-clearance and held < 30, keep intake deployed + spinning (`intakeCmd = INTAKING`) so the bot vacuums stray fuel mid-defense. Chassis (`navigationTarget`) and mechanisms (`intakeCommand`) are already decoupled in `AIActionIntent` — ~1 line in `JevDecisionEngine`.
  - Directional harvest bias: add a dot-product bonus toward the next zone into the cluster-scent score so midfield sweeps drift homeward instead of stranding the bot at the far wall.
- [ ] 2-step horizon task planning (`StrategicPlan(current, next, timeToTransitionSec)`). `[OPEN] [LOW PRIORITY — after AI integration]` Do not build before the package restructure + green test suite.
  - Shift time budget: `timeLeftToHarvest = hubShiftTimeRemaining − transitTime` (transit ≈ 2.5–3.5 s); if ≤ 0, cut harvest and transit now so the volley lands while the Hub is active.
  - Alliance intent broadcast: each bot publishes `Alliance/BotN/NextCorridor` + `NextIntent` to NT; peers yield contested corridors (e.g. TOP vs BOTTOM trench) — broadcast-and-yield, no negotiation protocol.
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
- [x] `[RESOLVED]` Bots don't pick up balls near walls or obstacles. **Fixed by parallel workstream Sep 25 night.**
  - Root cause was the targeting filter, not the pickup radius: `StaticPathfinder.isPointInObstacle` treated the whole perimeter wall safety band as blocked, so the Jev scent search never proposed wall-adjacent balls. New `isPointInHardObstacle` (hub ramps, tower poles only — walls reachable via wall-normal approach) plus a dynamic-obstacle proximity check; fuel targeting uses those instead.
  - Tests: `WallPickupTest` (6 tests: all four walls targeted + both bot types collect at standoff). Temp repro file removed.
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
