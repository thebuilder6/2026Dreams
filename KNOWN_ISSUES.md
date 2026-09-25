# Known Issues & Desired Features

Add entries as `- [ ] description`. Include repro or file refs so future sessions can verify.
Status tags: `[OPEN]`, `[EXPLAINED]` (working as designed, UX problem), `[STALE]` (no repro since), `[PARTIAL]` (partly implemented).

## A. Drive / performance

- [x] `[RESOLVED]` Robot drives slowly on manual controls; suspect brownout logic. **Fixed: Simulation current draw model retuned.**
  - `SwerveBase.getSimulationCurrentDraw()` previously used an exaggerated heuristic (15A idle per module = 60A at rest, up to 160A drive alone). When summed with shooter and intake in `Robot.simulationPeriodic()`, the simulated battery voltage dropped below 9.5V, triggering brownout speed scaling down to 0.35 and slow recovery (+0.03/cycle).
  - Retuned `SwerveBase.getSimulationCurrentDraw()` to realistic physics (~0.5A idle per module quiescent, ~20A per module at full sprint = 82A max). Simulated battery voltage now rests at ~12.4V and drops to ~10.5V under full sprint with flywheels running, preserving full speed (scale 1.0) while protecting against real brownout spikes.
- [x] `[RESOLVED]` Smart Assist / Glide pathfinding issues (differs from AI sparring bots). **Fixed: Restored StaticPathfinder roadmap, eliminated Virtual Rail damper, and prevented 50Hz action reset thrashing.**
  - `DriveToPoseAction` constructor previously added `targetPose` to `waypoints` and passed it to `setExplicitWaypoints()`, which set `isExplicitPath = true` in `TrajectoryController`, permanently disabling `StaticPathfinder.findPath(...)` and driving blindly in a straight line into Hub/Ramp obstacles.
  - The Trench "Virtual Rail Damper" in `TrajectoryController` was overriding `vx`, `vy`, and `desiredHeading` while disabling dynamic avoidance whenever touching `y <= 1.55` or `y >= 6.50`, severely fighting both the pathfinder and the driver's shared-authority stick inputs. Replaced with clean heading alignment in low-clearance areas without corrupting holonomic translation or dynamic avoidance.
  - `AutonomousTeleopAgent` previously allowed `DriveToPoseAction` to finish immediately upon reaching 0.12m of standoff, destroying and recreating actions 50 times/sec. Added `setHoldPosition(true)` and dynamic `setTargetPose()` streaming so the robot holds its standoff stance smoothly, aims at the Hub on the fly, and transitions naturally when balls are depleted.
- [ ] `[OPEN]` Console/dashboard log spam from AI diagnostics + brownout. **Partially checked, needs a riolog capture to confirm.**
  - No per-loop `System.out/err` found in `Sim/`, `MatchCoach`, `Utils/`, `Teleop`: `Alert` (`Utils/Alert.java`) never touches console; `Diagnostics` prints are event-driven (`Test/Diagnostics.java:350-491`); `Vision` prints are setter-driven (`Subsystems/Vision.java:340-354`); `GameSim`/`AIRobot*` print only on caught exceptions.
  - Two real spam risks remain: (a) exception-path `System.err` fires every loop if the same exception recurs (e.g. `GameSim: Error in ...`, `AIRobotSim: ...`); (b) per-loop SmartDashboard/NT writes (`SwerveBase.log()` ~15 `Power/*` keys, `MatchScoreTracker.publishTelemetry()` ~30 `Scoreboard/*` keys, bot telemetry) — noisy in NT logs, not console.
  - Next step: capture riolog during a spam episode and match lines to source before changing anything.

## B. Scoring is confusing (likely working as designed)

- [ ] `[EXPLAINED]` Score counting does not make sense. **No counting bug found; there are 4 overlapping counters plus a 0-point rule that looks like lost points.**
  - Shots into an **inactive** Hub score 0 by design (`Sim/ShooterSim.java:123-137` → `recordWastedShot`). In teleop the Hub alternates every 25 s (`Subsystems/Dashboard.java:241-312`), so ~half of all teleop shots are *supposed* to score nothing. In autonomous the Hub is forced active (`:248-251`), so auto shots always count.
  - The counters disagree on purpose: `GameSim.score` (`Simulation/Score`, player only, capped by balls consumed `:388-408`) vs `MatchScoreTracker` red/blue totals (`Scoreboard/Match/*`, all robots) vs `AIRobotSim` bot counts (`Simulation/BotN/Score`) vs `ShooterSim` raw (`simScoreCount`). Before "fixing" scoring, say *which two counters* disagree and when (auto/teleop, hub active/inactive).
  - Real UX gap (matches feature list): no single match UI showing both alliances, wasted shots, and per-bot breakdown together.

## C. Autonomous is broken (two root causes found, neither fixed)

- [x] `[RESOLVED]` "Advanced Choreo Shot" auto path does not shoot. **Fixed: `AutoAimAction` integrated with Shooter state machine.**
  - Refactored `AutoAimAction` (`Auto/Actions/AutoAimAction.java`) to command `shooter.shoot()` and `shooter.prepareToShoot()` instead of setting raw kicker voltage via `setKickerSpeed(...)`. The `Shooter` 50 Hz state machine now cleanly arbitrates flywheel velocity and kicker voltage in lockstep.
- [x] `[RESOLVED]` Bots perform poorly in autonomous — all take the same objective. **Fixed: Jev endgame logic gated on non-autonomous phase.**
  - Added `isAutonomous` tracking into `WorldState` (populated via `DriverStation.isAutonomous()` in `AIRobotSim`, `WorldStateBuilder`, and unit tests).
  - Gated `StrategicObjective.RUSH_CLIMB` and retreat defense in `JevDecisionEngine.java` on `!world.isAutonomous()`, preventing bots from treating autonomous (match time 15→0) as teleop endgame.
  - Gated `MatchScoreTracker.updateClimbEvaluation()` against autonomous mode, eliminating spurious climb scoring in auto.
  - Added unit test `testAutonomousModePreventsPrematureRushClimb` in `JevDecisionEngineTest.java`.

## D. JVM crashes (stale — keep logs, close on no repro)

- [ ] `[STALE]` Debug-sim JVM crash (`hs_err_pid12944.log`, Sep 23): C2 `refcount has gone to zero` under JDWP + Temurin 17.0.16+8 (wrong JDK). No recurrence since; always run sim under `JAVA_HOME=C:\Users\Public\wpilib\2026\jdk` per `AGENTS.md`.
- [x] `[RESOLVED]` Test-worker crash (`hs_err_pid51348.log`, `hs_err_pid55220.log`): `EXCEPTION_ACCESS_VIOLATION` in `wpiHal.dll`. **Fixed: Pinned Gradle test worker JVM to WPILib 2026 JDK.**
  - Gradle test workers (`forkEvery = 1`) previously defaulted to the system Eclipse Temurin JDK, which failed native JNI calls in WPILib HAL. Explicitly set `executable = wpilibJava.absolutePath` in `build.gradle`, ensuring all test forks run on `C:\Users\Public\wpilib\2026\jdk\bin\java.exe`.
- [ ] `[EXPLAINED]` Sim prints `bind() to port 1181 failed` on startup. Non-fatal (CameraServer vs PhotonVision/Limelight sim ports; see `SIMULATION_GUIDE.md`). Ignore; listed so nobody "fixes" it.

## E. Desired features (annotated with what already exists)

- [x] `[RESOLVED]` Referee/penalty awareness in sim & penalty score tracking. **Implemented: `RefereeSim` & `MatchScoreTracker` penalty subsystem.**
  - Created `RefereeSim` enforcing FRC G401 (pinning duration >2.4s without 3ft backoff) and G201 (autonomous centerline crossing >0.40m past midfield).
  - Integrated Minor Foul (2 pts) and Tech Foul (5 pts) tracking in `MatchScoreTracker`, cleanly awarding penalty points to the opponent alliance score total and publishing `Scoreboard/Referee/*` telemetry. Full unit test coverage in `RefereeSimTest`.
- [ ] Coordinated bot autonomous plans + starting positions. `[PARTIAL]` staggered spawns exist (`AIRobotSim.java:333-354`). Missing: distinct auto objectives (blocked by §C endgame bug), coordinated multi-bot plans.
- [ ] Smarter Jev strategy/tactics (lookahead, allies, opponent modeling). `[OPEN]` Engine is unit-tested (`JevDecisionEngineTest`, 17 tests) but purely reactive single-step policy.
- [ ] Headless AI-vs-AI training matches + scenario control. `[OPEN]`
- [ ] Test the AI "brain" outside full sim. `[PARTIAL]` `JevDecisionEngineTest` + `AIRobotSimTest` (32 tests) already do this — extend, don't start over.
- [ ] Better match UI (both alliances' scores, etc.). `[OPEN]` Data exists (`Scoreboard/*`), layout doesn't.
- [ ] Richer AI action/move options. `[OPEN]`
- [ ] TypeSafe AI API for decisions. `[PARTIAL]` `tools/coaching/jev_coach.py --live/--report` already calls it with `TYPESAFE_API_KEY`; robot-side (real-time) integration missing.
- [ ] Team coordination message system. `[OPEN]`
- [ ] Post-match LLM log review per bot (actions → suggested changes). `[PARTIAL]` `jev_coach.py --report` writes `reports/match_coach_report_*.md`; per-bot analysis + suggestions missing.
- [ ] Practice/coaching mode UX (start/stop, driver-station/coach/bot loading). `[OPEN]`
- [ ] Coaching mode that makes sense. `[OPEN]`
- [ ] Robot-with-missing-parts mode (no intake/climb/vision). `[PARTIAL]` Intake encoder fallback + vision-degraded odometry exist; no general "missing subsystem" config.
- [ ] Driver-assist transparency (what it is doing). `[PARTIAL]` `CoPilot/Objective` published; needs UI/explanation pass.
- [ ] Sim info parity (robot sees only sensor/API data; bots may keep perfect knowledge). `[OPEN]` Real audit, no partial credit claimed.
- [ ] Scale simulated game pieces without lag. `[OPEN]` (`Arena2026Rebuilt` efficiency mode + 54-ball cap is the current mitigation.)
- [ ] Sim→real→sim iteration workflow. `[OPEN]`
- [ ] AI-assisted robot/path/auto design. `[OPEN]`
- [ ] Streamlined AdvantageScope setup. `[OPEN]`
- [ ] Elastic UI cleanup across tabs; consider bespoke dashboard replacement. `[OPEN]`
- [ ] Git details + robot name on Elastic. `[PARTIAL]` Already published (`Dashboard.java:86-91` → `Build/*` keys from generated `BuildConstants`); likely just needs layout wiring.
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
