# Sprint 3 Tickets — Teleop + auto + wiring (goal: full match loop)

Demo: drive → intake → shoot in teleop; chooser auto drives 2 m and shoots; e-stop works. Total: 10 pts.

---

## S3-1 Teleop input + drive (3 pts) — depends: S1-4

**Tasks:**
1. `Teleop.java`: own `XboxController`s (0 driver / 1 operator), `Vector2dSlewRateLimiter(16)` + `SlewRateLimiter(10)`.
2. Cleanup chain (verbatim): circular 2D deadband `0.08` (if `hypot < 0.08` ⇒ zero; else rescale `[db,1]→[0,1]`) → cubic `sign·(0.7|x|³+0.3|x|)` → slew → scale `MAX_SPEED`/`MAX_ROT` → on Red negate X/Y. Rotation: 1D deadband `0.06` → same cubic → slew.
3. Slow mode (left-stick click toggles): 0.35× translation / 0.50× rotation. Double-tap A within 0.4 s (WPILib `Trigger.multiPress(2, 0.4)`) ⇒ `zeroGyroWithAlliance` + continue driving (no lockup).
4. Tests: `(0.05,0.05)` ⇒ `(0,0)`; `(1,0)` ⇒ full speed along same angle; Red flips sign of X/Y.

**Acceptance:** no stick drift; full stick = full speed; Red "away" works. **Verify:** tests + sim drive.

---

## S3-2 Teleop mechanism bindings (2 pts) — depends: S3-1, S2-2, S2-4

**Tasks:**
1. X (edge-toggle) arm up/down; LT `>0.3` ⇒ `Intaking`; B ⇒ `Reversed`/`StandbyReversed`; Y ⇒ hopper feed; RT `>0.3` in-zone ⇒ spool-to-solution + `shoot()` when `isAtCorrectSpeed`, else `manualFire`; Back/Start ⇒ everything stops (intake Disabled, shooter stop, swerve stop) and returns.
2. Operator merge (if controller 1 connected): triggers `max()`, buttons OR; operator POV up/down = arm stow/deploy. Solo controller-0 driving must work with operator unplugged (guard `isConnected()`).
3. Skip: Glide/assist, ball-hunt, auto-aim rotation, snap-to-heading, rumble, dashboard toggles.

**Acceptance:** each binding bench-verified; e-stop halts arm + rollers + kicker + drive within one cycle. **Verify:** sim + bench walkthrough.

---

## S3-3 Auto executor + mission (3 pts) — depends: S1-4, S2-4

**Tasks:**
1. `Auto/AutoMissionExecutor.java`: `setAutoMission` builds a `Thread(mission.run)` with crash-report; `start / stop / reset / isStarted`; `stop` nulls thread AFTER `mission.stop()`.
2. `Auto/Missions/MissionBase.java`: `mUpdateRate 1/50`; `run()` = optional delay → `routine()` → `done()`; `runAction(a)` = `start` → 20 ms `update` loop until `isFinished`/interrupt → `done()`; `isActiveWithThrow` aborts via `AutoMissionEndedException`.
3. One mission `Drive2mAndShoot extends MissionBase`: set start pose → drive forward 2 m (proportional on pose error, ≤2 m/s) → spool to solution → 3× (wait-at-speed + 0.3 s kicker pulse) → stop all. Plus `Do Nothing` (empty routine).
4. `SendableChooser<String>` (`Do Nothing` default) published to SmartDashboard.

**Acceptance:** auto completes in sim (ends ≤2.3 m, wheels spooled, kicker pulsed); disable mid-run stops the thread; `Do Nothing` never moves. **Verify:** 3× sim runs.

---

## S3-4 Robot wiring + E2E (2 pts) — depends: S3-1, S3-2, S3-3

**Tasks:** `Robot extends LoggedRobot` (logger config per Checklist §9d — do Sprint 5 AK-1 first or in parallel):
1. Constructor/`robotInit`: instantiate swerve/intake/shooter singletons → `new Teleop()` → `SubsystemManager.initializeSubsystems()` → `silenceJoystickConnectionWarning(true)` → LiveWindow off + `disableAllTelemetry()`.
2. `robotPeriodic`: update + log subsystems. `autonomousInit`: `executor.stop/reset → getSelected → setAutoMission → start`. `teleopInit/teleopPeriodic`: delegate to `Teleop`. `disabledInit`: executor stop, teleop reset, all stops, `setMotorBrake(true)`.
3. `simulationPeriodic`: `SubsystemManager.simulationUpdateSubsystems()` + BatterySim voltage from summed draws (Checklist §9d).
4. Skip: 100 Hz Notifier, WebServer/PortForwarder, NT juggling, GameSim/AI.

**Acceptance:** full loop — compile, sim teleop drive/intake/shoot, sim auto, suite green. **Verify:** `.\gradlew compileJava --offline`, `.\gradlew simulateJava --offline`, `.\gradlew test --offline --no-daemon`.
