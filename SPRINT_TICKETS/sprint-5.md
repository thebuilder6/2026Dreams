# Sprint 5 Tickets — AdvantageKit + simple sim (goal: every sensor logged, sim drives the same code as hardware)

Demo: sim run with AdvantageScope open live (NT) + a `.wpilog` replay showing pose, arm, flywheel traces. Total: 7 pts. Do alongside Sprints 1–2; finish before S3-4.

---

## AK-1 Logger wiring (2 pts) — depends: S0-2

**Tasks:**
1. `build.gradle`: add the akit annotation processor (generates `*InputsAutoLogged`; without it `@AutoLog` never compiles):
   ```gradle
   if (file("lib/akit-autolog-26.0.2.jar").exists()) {
     annotationProcessor files('lib/akit-autolog-26.0.2.jar', 'lib/javapoet-1.13.0.jar')
   } else {
     annotationProcessor "org.littletonrobotics.akit:akit-autolog:<version from vendordeps/AdvantageKit.json>"
   }
   annotationProcessor wpi.java.deps.wpilibAnnotations()
   ```
2. `Robot extends LoggedRobot` (not `TimedRobot`). Constructor: `Logger.recordMetadata(...)` (project, sha, branch, build date) then mode switch on `Constants.getMode()`:
   - REAL → `WPILOGWriter` + `NT4Publisher`; SIM → `NT4Publisher`; REPLAY → `setUseTiming(false)`, `WPILOGReader(logPath)` source + suffixed writer. Then `Logger.start()`.
3. `simulationPeriodic`: sum `getSimulationCurrentDraw()` over `SubsystemManager.getSubsystems()` → `BatterySim.calculateDefaultBatteryLoadedVoltage(amps)` → `RoboRioSim.setVInVoltage(v)`.

**Acceptance:**
- [ ] Compile green (proves the processor generated the inputs classes).
- [ ] Sim publishes live NT traces visible in AdvantageScope; saved `.wpilog` replays identically (File → Open Log).

**Verify:** `.\gradlew compileJava --offline`, then `simulateJava` + AdvantageScope.

---

## AK-2 Mechanism IO split + physics (3 pts) — depends: S2-1, S2-3, AK-1

**Tasks:**
1. Interfaces `ShooterIO` / `IntakeIO` with `@AutoLog` inputs structs (fields per Checklist §9b). Hardware impls `ShooterIOSparkMax` / `IntakeIOSparkMax` (SparkMax writes + sensor reads; arm inverted; DIO-0 absolute encoder + relative fallback).
2. `ShooterIOSim`: `FlywheelSim(createFlywheelSystem(NEO(1), MOI 0.001, gearing 1.0), NEO(1), 1.0)`; per cycle `update(lVolts, rVolts)` → RPM + current into inputs (mirror one wheel for right on the simple build). Kicker: `rpm = (v/12)·5676`, `2.5 A` when driven.
3. `IntakeIOSim`: `SingleJointedArmSim(NEO(1), gearing 100.0, estimateMOI(0.4 m, 3.0 kg), 0.4, minRad, maxRad, gravity true, start)` — limits in RADIANS converted from 250°/347°; `Mechanism2d` ligament published for the mechanism view. Rollers/hopper duty model: `rpm = (v/12)·5676`, `8 A` / `4 A` when driven.
4. Subsystems: constructor picks `RobotBase.isSimulation() ? Sim : SparkMax`; every cycle `io.updateInputs(inputs); Logger.processInputs("Name", inputs);`.

**Acceptance:**
- [ ] Sim arm follows 347↔250 under the real ProfiledPID+FF (no sim-only shortcuts).
- [ ] Sim flywheel spools to table RPM ±150; jam detector reads believable currents.
- [ ] Zero `new SparkMax` / hardware calls reachable in sim (code inspection).

**Verify:** sim arm sweep + spool traces in AdvantageScope.

---

## AK-3 Drive sim IO + battery (2 pts) — depends: S1-4, AK-1

**Tasks:**
1. `DriveIO` interface (`@AutoLog` inputs: per-module drive vel/pos/volts, steer pos/volts, gyro yaw, accel, odometry pose) + `DriveIOSparkMax` + `DriveIOSim` (wraps the SAME YAGSL `SwerveDrive`: reads module velocity/position, gyro yaw, pose into inputs; writes volts through to sim motors; `zeroGyro`/`setPose` delegate to `resetOdometry`).
2. `getSimulationCurrentDraw()` on each subsystem (drive sums module currents; intake = arm + roller + hopper; shooter = both wheels + kicker); `Robot.simulationPeriodic` sums → battery model (AK-1 step 3).

**Acceptance:**
- [ ] Sim drive pose/voltage/current stream in AdvantageScope; no brownout under normal load.
- [ ] Replay of a drive run reproduces the pose trace.

**Verify:** `simulateJava` drive run + `.wpilog` replay.
