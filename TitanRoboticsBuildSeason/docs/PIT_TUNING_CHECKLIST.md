---
title: Pit Tuning & Pre-Match Checklist
audience: [human, ai]
owner: drive-team
last_verified: 2026-10-06
status: authoritative
---

# Pit Tuning & Pre-Match Checklist — FRC Team 8334

A single-page, quick-reference calibration checklist designed for pit crew members, drive coaches, and programmers between matches.

---

## Scope

### What this document covers
- Fast physical pre-checks for all robot mechanisms before power-on.
- Step-by-step procedure for the 15-second automated Pre-Flight Diagnostics routine.
- Zero and sensor calibration sanity checks.
- Rapid 3-shot field carpet benchmark before practice matches.

### What this document does NOT cover
- Deep controller derivations or math modeling (see [`docs/SHOOTER_TUNING_GUIDE.md`](SHOOTER_TUNING_GUIDE.md) and [`docs/INTAKE_TUNING_GUIDE.md`](INTAKE_TUNING_GUIDE.md)).
- Swerve drive kinematic code details (see [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3A).

---

## Content

```
================================================================================
                    TEAM 8334 TEST MODE CONTROLLER QUICK MAP
================================================================================
Hold [LB] + D-Pad on Driver Controller (Port 0):
  * LB + UP    (POV   0): SysId Characterization
  * LB + DOWN  (POV 180): Shooter Tuning
  * LB + LEFT  (POV 270): Intake Testing
  * LB + RIGHT (POV  90): System Diagnostics (Automated Pit Check)
================================================================================
```

### Station 1: Mechanical Clearance & Idle Modes (Power OFF)
- [ ] **Flywheel Freedom:** Spin both flywheels and kicker by hand. Confirm zero friction or rub against polycarbonate hood plates.
- [ ] **Coast Mode Verification:** Confirm flywheel motors spin down freely (`kCoast` mode, NOT `kBrake`).
- [ ] **Intake Arm Travel:** Manually move intake arm through full range ($250^\circ$ to $347^\circ$). Ensure wiring and chain do not catch.
- [ ] **Belt & Chain Tension:** Check tension on swerve azimuth belts, intake drive chain, and shooter polycord belts.

---

### Station 2: 15-Second Automated Pre-Flight (Tethered / Test Mode)
1. Power robot ON and connect driver station via Ethernet tether or radio.
2. Enable **Test Mode** on Driver Station.
3. On Driver Controller, press **LB + D-Pad Right** to select `SYSTEM_DIAGNOSTICS`.
4. Pull **Right Trigger** (> 0.5) to run the automated sequence:
   - [ ] **Swerve Pulse:** Each drive motor spins at +1.5V (verifies forward encoder sign).
   - [ ] **Steer Check:** Azimuth modules rotate to assert encoder delta $\ge 1.0^\circ$.
   - [ ] **Intake Profile:** Arm traverses travel while confirming current $< 25\text{ A}$.
   - [ ] **Shooter Ramp:** Flywheels ramp to 1500 RPM and check error $< \pm 50\text{ RPM}$.
   - [ ] **Vision Link:** Verifies Limelight and PhotonVision network heartbeat.
5. Inspect Elastic Dashboard tab **Pre-Flight Diagnostics**:
   - [ ] Confirm all scorecard items show **GREEN**.

---

### Station 3: Sensor Zero Alignment Check
- [ ] **Arm Absolute Encoder:** In Standby, verify `/SmartDashboard/Intake/ArmPosition` reads $347.0^\circ \pm 1.0^\circ$.
- [ ] **Swerve Wheel Alignment:** All 4 swerve modules point true forward ($0.0^\circ$) with zero drift.
- [ ] **Cameras:** Lenses inspected and wiped clean of dust, grease, or polycarbonate debris.

---

### Station 4: Practice Carpet Benchmark (3 Quick Shots)
Run the unified solver or use Test Mode presets:
```powershell
python tools/tune/tune.py shooter 3.0
```
- [ ] **Shot 1 (Close / 1.8m):** Setpoint `2700 L / 2750 R RPM`. Confirm clean entry into inner funnel.
- [ ] **Shot 2 (Mid / 3.0m):** Setpoint `3500 L / 3550 R RPM`. Confirm ball centers inner funnel with backspin.
- [ ] **Shot 3 (Recovery Dip):** Feed ball while at 3500 RPM. Verify flywheel recovers within $\pm 150\text{ RPM}$ in $< 0.15\text{ s}$.

---

### Station 5: Match Readiness Sign-Off
- [ ] **Battery Condition:** Resting voltage $\ge 12.6\text{ V}$, internal resistance $\le 0.015\,\Omega$.
- [ ] **Driver Controller Haptics:** Verify crisp rumble confirmation on ball acquire and target lock.
- [ ] **Autonomous Routine:** Set chosen auto mission on Elastic Dashboard `Driver Dashboard` tab.

---

## Verification

- **Verified against:** Commit `e52c6dd` (2026-10-06).
- **Test Suite Status:** 47 test files / 461 tests green.
- **Tooling:** Integrated with `python tools/tune/tune.py checklist`.
- **Next Review Due:** 2026-11-05.

---

## Related

- Unified calibration CLI: [`tools/tune/tune.py`](../tools/tune/tune.py)
- Shooter calibration guide: [`docs/SHOOTER_TUNING_GUIDE.md`](SHOOTER_TUNING_GUIDE.md)
- Intake calibration guide: [`docs/INTAKE_TUNING_GUIDE.md`](INTAKE_TUNING_GUIDE.md)
- Diagnostics subsystem: [`src/main/java/frc/robot/Test/Diagnostics.java`](../src/main/java/frc/robot/Test/Diagnostics.java)
- Master documentation index: [`docs/INDEX.md`](INDEX.md)
