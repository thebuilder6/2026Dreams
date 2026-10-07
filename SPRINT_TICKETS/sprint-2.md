# Sprint 2 Tickets — Mechanisms (goal: arm sweeps, flywheels spool, kicker gated)

Demo: X toggles arm 347°↔250°; RT spools wheels; kicker fires only in-zone at speed. Total: 10 pts.

---

## S2-1 Intake constants + arm control (3 pts) — depends: S1-2

**Tasks:**
1. `Subsystems/intake/IntakeConstants.java` (exact): up/stow `347°`, down/ground/horizontal `250°`; limits `400 °/s` + `400 °/s²`; PID `kP 0.1 / kI 0 / kD 0.01`; FF `kS 0.2 / kG 0.34 / kV 0 / kA 0`; soft stops — cut voltage below `240°` / above `355°`.
2. `Subsystems/Intake.java` singleton: `ProfiledPIDController` (constraints above, `enableContinuousInput(0,360)`) + `ArmFeedforward`. Per-cycle `updateArmController`: `pid.setGoal(goal)`; `volts = pid.calculate(angleDeg) + ff.calculate(toRadians(angle − 250), toRadians(setpointVel))`; `io.setArmVoltage(volts)`.
3. IO behind `IntakeIO` interface: real = arm SparkMax ID 10 (inverted) + DIO-0 absolute encoder with relative-encoder fallback; sim = `ArmSim`. Same interface, chosen by `RobotBase.isSimulation()`.

**Acceptance:** arm steps 347↔250 smoothly, settles <1 s, never crosses soft stops. **Verify:** compile + bench/sim sweep.

---

## S2-2 Intake rollers + jam (2 pts) — depends: S2-1

**Tasks:**
1. States: `Standby` (347°, off) / `Down` (250°, off) / `Intaking` (250°, rollers+hopper on) / `Reversed`+`StandbyReversed` (both reversed) / `Disabled`. `setState(String)` parses case-insensitively, defaults `Standby`.
2. Duties: intake `0.7`, hopper `0.5` (wheels ID 14 inverted, hopper ID 9). Jam: `Debouncer(0.5 s, kRising)` on roller current `>30 A` → start 1 s eject timer (both reversed), then resume prior state; raise only a dashboard flag (no console spam).

**Acceptance:** LT spins both forward; B reverses; forced stall (>30 A held 0.5 s) auto-reverses exactly 1 s. **Verify:** bench stall test (pin roller briefly) or sim current injection.

---

## S2-3 Shooter wheels + RPM table (3 pts) — depends: S1-2, S1-3

**Tasks:**
1. `Subsystems/shooter/ShooterConstants.java`: kicker `12 V`; FF `kS 0 / kV 0.0022 V-per-RPM / kA 0`; PID `kP 0.0007 / kI 0 / kD 0`, integrator range ±1.5; idle `60 RPM`.
2. `Subsystems/Shooter.java` singleton: two PID+FF loops (L CAN 12 / R CAN 11); per cycle `volts = FF.calculate(targetRPM) + PID.calculate(measuredRPM, targetRPM)`; reset PID when spooling from 0.
3. Interpolated distance→RPM table (linear between points; right ≈ +50 for spin): `1.20→2400/2450, 1.92→2700/2750, 2.47→2900/2950, 3.05→3500/3550, 3.48→3550/3600, 4.18→3750/3800, 5.00→4100/4150, 6.00→4500/4550`. Use `InterpolatingDoubleTreeMap` or lerp helper.

**Acceptance:** step to table RPM settles ±150 RPM within 2 s at 2, 3.5, 5 m. **Verify:** tachometer/logged RPM trace.

---

## S2-4 Firing gate + kicker (2 pts) — depends: S2-3, S1-3

**Tasks:**
1. `calculateShootingSolution(pose)`: hub from alliance (`BLUE (4.6256,4.0346)` / mirrored Red); shooter point = robot + `SHOOTER_OFFSET −0.2032 m` rotated by heading; `possible = inZone && dist 1.2–6.5 m && dist·tan(70°) − heightDiff > 0` (heightDiff = `1.575 − 0.53`).
2. `isAtCorrectSpeed()`: both errors `<150 RPM` (ignore targets ≤100); latched — stays true until either error `>750`. Kicker (ID 13) `12 V` ONLY when `possible && atSpeed`; else 0. API: `setTargetRPM / shoot / prepareToShoot / stop / manualFire` (3200 RPM fallback).
3. Tests: out-of-zone pose ⇒ `possible false`; in-zone but wheels slow ⇒ kicker off; spooled in-zone ⇒ kicker on.

**Acceptance:** all three tests pass; no code path fires the kicker without the gate. **Verify:** `.\gradlew test --offline --tests "frc.robot.Subsystems.ShooterGateTest"`
