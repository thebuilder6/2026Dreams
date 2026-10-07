# Minimal Robot Rebuild — Sprint Plan with Tickets

Spec: `REBUILD_MINIMAL_CHECKLIST.md` (all numbers/file paths live there; tickets below reference it as "Checklist §N").
Scope: swerve + intake + shooter + simple teleop + one auto mission + AdvantageKit logging + simple desktop sim (IO split, FlywheelSim/ArmSim, battery model, AdvantageScope replay). Out of scope: AI, vision, GameSim fuel/score, multi-bot, headless rig, advanced nav, dashboard/LEDs, test mode.

Suggested cadence: 5 sprints × 1 week (solo dev, ~6–8 hrs/sprint). Each sprint ends with a demo. Verify env every shell:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"; $env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
```

---

## Sprint 0 — Scaffold (goal: empty project compiles)

**Demo:** `.\gradlew compileJava --offline` green on template + swerve JSON present.

- **S0-1 Env + repo setup (1 pt)**
  - Tasks: install WPILib 2026; confirm `C:\Users\Public\wpilib\2026\jdk` and `...\2026\maven` exist; create project dir; write `.wpilib/wpilib_preferences.json` (`java/2026/8334`).
  - Accept: `java -version` runs from the WPILib JDK; prefs file committed.
- **S0-2 Gradle files (2 pts)**
  - Tasks: write `settings.gradle` (local WPILib maven first, then plugin portal); write minimal `build.gradle` (GradleRIO `2026.2.1`, Java 17, `ROBOT_MAIN_CLASS frc.robot.Main`, `wpi.java.deps.wpilib()` + `wpi.java.vendor.java()`, JUnit 5, desktop sim on). See Checklist §0.
  - Accept: `.\gradlew compileJava --offline` green on the default template.
- **S0-3 Vendordeps + swerve JSON (2 pts)**
  - Tasks: install via WPILib extension: YAGSL `2026.1.14`, REVLib `2026.0.5`, AdvantageKit 26.x, WPILibNewCommands (keep required-but-unused Phoenix/Redux/Thrifty JSONs so YAGSL resolves); place all 8 `src/main/deploy/swerve/*.json` files (root: navX, `invertedIMU false`, 4 module names); fill module CAN IDs/offsets via YAGSL configurator — never hand-guess offsets.
  - Accept: compile still green; `deploy/swerve/swervedrive.json` lists 4 modules.

## Sprint 1 — Framework + geometry + drive (goal: robot drives field-relative in sim)

**Demo:** push left stick in sim, robot drives; double-tap A re-zeroes.

- **S1-1 Main + Subsystem contract (1 pt)**
  - Tasks: create `Main.java` (`RobotBase.startRobot(Robot::new)`, Checklist §1a); create `Interfaces/Subsystem.java` (`update/initialize/log/isEnabled/getName`, §1b).
  - Accept: compiles; no statics in `Main`.
- **S1-2 SubsystemManager + PortMap + Constants (2 pts)**
  - Tasks: `SubsystemManager` static list + register/initialize/update/log with per-subsystem try/catch → `DriverStation.reportWarning` (§1c); `PortMap` exact IDs (driver 0/op 1; arm 10, wheels 14, hopper 9, DIO 0; shooter 12/11, kicker 13) (§1d); trimmed `Constants` (MAX_SPEED 15 ft/s ≈ 4.57 m/s, MAX_ROT 8 rad/s, slew 16/10, auto P 10.0/7.5) (§1e).
  - Accept: unit-less compile; one bad subsystem can't kill the loop (code review).
- **S1-3 Alliance geometry (1 pt)**
  - Tasks: hardcode field numbers (`16.541×8.069`, hub `(4.6256, 4.0346)`, rim `1.575`, envelope `4.20`); implement `AllianceFlipUtil` (`isRedAlliance`, `apply` mirror `X_red = LEN − X_blue`, zone test Blue `x≤4.6256` / Red `x≥11.9154`, driver-relative heading +180° on Red). Checklist §2.
  - Accept: tests — Blue `(2,4)` in-zone, `(6,4)` out; Red mirror `(14.5,4)` in-zone.
- **S1-4 SwerveBase wrapper (5 pts)**
  - Tasks: singleton + self-register; build YAGSL `SwerveDrive` from `deploy/swerve` with start pose Blue `(1,4,0°)` / Red `(16,4,180°)`; `setHeadingCorrection(true)`; expose `drive / driveFieldOriented / stop / zeroGyroWithAlliance / getPose / getHeading`; `update` = odometry refresh, `log` = Field2d pose. Skip vision fusion, watchdogs, path following (§3).
  - Accept: sim drive field-relative; gyro re-zero snaps to 0°/180° per alliance.

## Sprint 2 — Mechanisms (goal: arm sweeps, flywheels spool, kicker gated)

**Demo:** X toggles arm 347°↔250°; RT spools wheels; kicker fires only in-zone at speed.

- **S2-1 Intake constants + arm control (3 pts)**
  - Tasks: `IntakeConstants` (up `347°`, down/ground/horizontal `250°`; limits `400°/s, 400°/s²`; PID `0.1/0/0.01`; FF `kS 0.2 kG 0.34`; soft stops cut below `240°`/above `355°`); `Intake` singleton with `ProfiledPIDController` (continuous `[0,360]`) + `ArmFeedforward`; `updateArmController`: `volts = pid.calculate(deg) + ff.calculate(rad − horizRad, velRadS)`.
  - Accept: arm steps 347↔250 smoothly, no oscillation/limit violation.
- **S2-2 Intake rollers + jam (2 pts)**
  - Tasks: states `Standby/Down/Intaking/Reversed/StandbyReversed/Disabled`; roller duties intake `0.7` / hopper `0.5`; `Debouncer(0.5 s)` on `>30 A` → 1 s auto-reverse; SparkMax wiring (arm 10 inverted + DIO-0 absolute encoder w/ relative fallback; wheels 14; hopper 9); sim = `ArmSim` + duty model behind same IO.
  - Accept: LT spins both forward; B reverses; forced stall (>30 A, 0.5 s) triggers 1 s eject.
- **S2-3 Shooter wheels + table (3 pts)**
  - Tasks: `ShooterConstants` (kicker `12 V`, FF `kS 0/kV 0.0022/kA 0`, PID `kP 0.0007`, integrator ±1.5, idle 60); two PID+FF loops (L 12 / R 11); interpolated RPM table `1.2→2400/2450 … 6.0→4500/4550` (right +50; Checklist §5 for full table).
  - Accept: wheel steps to table RPM ±150 within 2 s at 2–5 m.
- **S2-4 Firing gate + kicker (2 pts)**
  - Tasks: `calculateShootingSolution(pose)` → `(angle, rpmL, rpmR, possible)`; `possible` = in-zone AND `1.2–6.5 m` AND geometry clears; `isAtCorrectSpeed` = both errors `<150` (hysteresis release at `>750`); kicker `12 V` only when `possible && atSpeed`; `manualFire` fallback 3200 RPM; `shoot/prepareToShoot/stop`.
  - Accept: kicker NEVER fires out-of-zone or with either error ≥150 (test both negatives); fires in-zone at speed.

## Sprint 3 — Teleop + auto + wiring (goal: full match loop)

**Demo:** drive → intake → shoot in teleop; chooser auto drives 2 m and shoots; e-stop works.

- **S3-1 Teleop input + drive (3 pts)**
  - Tasks: controllers 0/1; cleanup chain — circular deadband (`0.08` trans / `0.06` rot) → cubic `0.7|x|³+0.3|x|` → slew (`16` + `10`) → scale (`MAX_SPEED`/`MAX_ROT`) → negate X/Y on Red; left-stick-click slow mode (0.35×/0.50×); double-tap-A (0.4 s) re-zero.
  - Accept: stick drift ⇒ zero output; full stick ⇒ full speed; Red flip verified.
- **S3-2 Teleop mechanism bindings (2 pts)**
  - Tasks: X toggle arm; LT `>0.3` intake; B eject; Y feed; RT `>0.3` shoot (zone-gated via S2-4); Back/Start e-stop (all stop); operator merge (`max` triggers, OR buttons, POV up/down arm) with solo-controller-0 fallback.
  - Accept: each binding verified on bench/sim; e-stop halts arm+wheels+kicker+drive.
- **S3-3 Auto executor + mission (3 pts)**
  - Tasks: `AutoMissionExecutor` (thread set/start/stop/reset); `MissionBase` 50 Hz `run/routine/runAction` with interrupt; one mission `Drive2mAndShoot` + `Do Nothing`; `SendableChooser` wired in `autonomousInit`; `disabledInit` always stops executor.
  - Accept: auto runs to completion in sim; disable mid-mission stops thread; `Do Nothing` stays still.
- **S3-4 Robot wiring + E2E (2 pts)**
  - Tasks: `Robot extends LoggedRobot` (logger config per Checklist §9d — do Sprint 5 AK-1 first or in parallel): init singletons + `initializeSubsystems` + silence joystick warnings + kill LiveWindow; `robotPeriodic` update+log; wire auto/teleop/disabled per Checklist §8; `simulationPeriodic` subsystem sim-updates + BatterySim voltage; brake on disable.
  - Accept: full loop green — `compileJava`, sim teleop, sim auto, `test --offline --no-daemon`.

## Sprint 4 — Harden + prove (goal: shippable minimal)

**Demo:** green suite + sim recording of drive/intake/shoot/auto.

- **S4-1 Smoke tests (2 pts):** input shaping symmetry/zero-deadband; zone gate Blue/Red true+false; shooter gate (out-of-zone blocked, under-speed blocked, at-speed fires); executor start/stop. Accept: `test --offline --no-daemon` green; single-test shortcut documented.
- **S4-2 Sim validation (2 pts):** drive 5 m field-relative error check; arm 10× toggle soak; 5× spool-and-fire at 3 m; auto mission 3× runs; open the `.wpilog` in AdvantageScope and confirm pose/arm/RPM traces replay. Accept: no brownout/overrun warnings; findings logged.
- **S4-3 Docs + backlog (1 pt):** update checklist counts; file follow-up tickets for deferred items (vision, GameSim fuel/score, Choreo auto, dashboard). Accept: next-phase backlog exists; this doc's verification dates bumped.

## Sprint 5 — AdvantageKit + simple sim (goal: every sensor logged, sim drives the same code as hardware)

**Demo:** sim run with AdvantageScope open live (NT) + a `.wpilog` replay showing pose, arm, flywheel traces. Depends: S0-2 (build), S1-4/S2-4 (subsystems to attach IO to); do alongside Sprints 1–2, must finish before S3-4.

- **AK-1 Logger wiring (2 pts)**
  - Tasks: `build.gradle` akit annotation processor (jar fallback, Checklist §9a); `Robot extends LoggedRobot` with per-mode receivers (REAL: WPILOGWriter + NT4Publisher; SIM: NT4Publisher; REPLAY: reader + suffixed writer, `setUseTiming(false)`); `Logger.start()`; `simulationPeriodic` BatterySim voltage from summed current draw (§9d).
  - Accept: sim publishes live NT traces; a saved `.wpilog` replays identically in AdvantageScope.
- **AK-2 Mechanism IO split + physics (3 pts)**
  - Tasks: `@AutoLog` IO interfaces + Spark (`IOSparkMax`) + Sim IO for intake and shooter (§9b); `FlywheelSim` (NEO, MOI `0.001`, gearing `1.0`) + kicker duty model; `SingleJointedArmSim` (NEO, gearing `100`, `0.4 m`/`3.0 kg`, radian limits converted from 250°/347°) + `Mechanism2d` ligament; roller/hopper duty model (`8 A`/`4 A`); `Logger.processInputs` every cycle; constructor selects IO by `RobotBase.isSimulation()`.
  - Accept: sim arm tracks 347↔250 under the real PID; sim flywheel spools to table RPM; jam detector reads sane currents.
- **AK-3 Drive sim IO + battery (2 pts)**
  - Tasks: `DriveIOSim` wrapping the YAGSL `SwerveDrive` (module vel/pos + gyro yaw into inputs); subsystem `getSimulationCurrentDraw()` implementations; `simulationPeriodic` sums currents → `BatterySim` → `RoboRioSim`.
  - Accept: sim drive pose/voltage/current visible in AdvantageScope; brownout-free under normal load.

---

## Ticket template (copy/paste)

`ID + title | Goal (1 line) | Tasks (bullets, files + values) | Acceptance (measurable) | Verify (exact command) | Points`
