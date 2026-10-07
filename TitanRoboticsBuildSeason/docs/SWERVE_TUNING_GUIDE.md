---
title: Swerve Drive Calibration & Tuning Guide
audience: [human, ai, programming-leads]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Swerve Drive Calibration & Tuning Guide — FRC Team 8334

Complete technical and procedures manual for Team 8334's 4-module SDS MK4i swerve drivebase: hardware architecture, absolute encoder zeroing, wheel radius calibration via `tune.py`, YAGSL configuration, closed-loop PIDF tuning, and SysId characterization.

---

## Scope

### What this document covers
- Physical specifications and kinematics of SDS MK4i swerve modules ([`SwerveBase.java`](../src/main/java/frc/robot/Subsystems/SwerveBase.java) and `src/main/deploy/swerve/*.json`).
- Procedure for zeroing CANcoder absolute azimuth encoders and updating configuration files.
- Effective wheel radius calibration on competition carpet using [`tools/tune/tune.py`](../tools/tune/tune.py).
- Tuning YAGSL drive and steering PIDF gains in [`modules/pidfproperties.json`](../src/main/deploy/swerve/modules/pidfproperties.json).
- Multi-mode system identification via [`SysIdManager.java`](../src/main/java/frc/robot/Test/SysIdManager.java).
- Troubleshooting drift, skew, brownout current limits, and inversion issues.

### What this document does NOT cover
- High-level autonomous path following and Choreo trajectories (see [`docs/AUTONOMOUS_GUIDE.md`](AUTONOMOUS_GUIDE.md)).
- Dynamic obstacle avoidance and artificial potential fields (see [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3A).
- Vision pose measurement fusion (see [`docs/VISION_GUIDE.md`](VISION_GUIDE.md)).

---

## Content

### 1. Drivebase Hardware Architecture

The robot utilizes an **SDS MK4i Swerve** configuration powered by Yet Another Generic Swerve Library (YAGSL 2026.1.14):

| Parameter | Specification | Location |
|---|---|---|
| **Module Type** | SDS MK4i (Inverted steering motor layout) | Hardware |
| **Drive Motors** | 4x REV NEO Brushless (CAN 2, 4, 6, 8) | SparkMax |
| **Steer Motors** | 4x REV NEO Brushless (CAN 1, 3, 5, 7) | SparkMax |
| **Azimuth Encoders** | 4x CTRE CANcoder (CAN 15, 16, 17, 18) | CANivore / CAN Bus |
| **IMU / Gyro** | Kauai Labs NavX2-MXP (SPI) | RoboRIO MXP Port |
| **Drive Gear Ratio** | $6.75 : 1$ (MK4i L2 standard ratio) | [`physicalproperties.json`](../src/main/deploy/swerve/modules/physicalproperties.json) |
| **Steer Gear Ratio** | $21.42857 : 1$ ($150 / 7 : 1$) | [`physicalproperties.json`](../src/main/deploy/swerve/modules/physicalproperties.json) |
| **Track Width / Base** | $22.25 \times 22.25\text{ inches}$ ($\pm 11.125\text{ in}$ from center) | Module JSONs |
| **Wheel Diameter** | $4.0\text{ inches}$ ($0.1016\text{ m}$) nominal | Tread wear calibrated |
| **Max Linear Speed** | $15.0\text{ ft/s}$ ($4.572\text{ m/s}$) | [`Constants.java`](../src/main/java/frc/robot/Data/Constants.java) |
| **Max Angular Speed** | $8.0\text{ rad/s}$ ($458.3^\circ/\text{s}$) | [`Constants.java`](../src/main/java/frc/robot/Data/Constants.java) |

#### Current Limits & Brownout Protection
- **Drive Motors**: 40A continuous smart current limit.
- **Steer Motors**: 20A continuous smart current limit.
- Voltage compensation enabled at $12.0\text{ V}$.
- Ramp rate set to $0.25\text{ s}$ open and closed loop to prevent carpet tearing and battery voltage sag.

---

### 2. Module Mapping & CAN Layout

```
         FRONT OF ROBOT
      ┌──────────────────┐
  FL  │ [16]        [15] │  FR
      │ Drive: 6    Drive: 4 │
      │ Steer: 5    Steer: 3 │
      │                      │
      │       [NavX]         │
      │                      │
  BL  │ Drive: 8    Drive: 2 │  BR
      │ Steer: 7    Steer: 1 │
      │ [17]        [18] │
      └──────────────────┘
         BACK OF ROBOT
```

- **Front-Left (`frontleft.json`)**: Drive CAN 6, Steer CAN 5, CANcoder CAN 16
- **Front-Right (`frontright.json`)**: Drive CAN 4, Steer CAN 3, CANcoder CAN 15
- **Back-Left (`backleft.json`)**: Drive CAN 8, Steer CAN 7, CANcoder CAN 17
- **Back-Right (`backright.json`)**: Drive CAN 2, Steer CAN 1, CANcoder CAN 18

---

### 3. Absolute Encoder Zero-Calibration Procedure

Whenever a module is physically removed, a motor replaced, or a CANcoder swapped, the module's absolute zero offset **must** be re-calibrated.

#### Physical Alignment (Rule of Bevels)
1. Place the robot securely on a cart with wheels completely off the ground.
2. Clamp an aluminum straightedge across the wheel faces or insert the 3D-printed MK4i alignment jig.
3. **Bevel Gear Convention**:
   - For all four modules, orient the large bevel gear facing the **RIGHT** side of the robot when looking forward.
   - All four wheels must point perfectly parallel to the chassis frame rail ($0.0^\circ$ heading).

#### Reading the Raw Angles
1. Open **Phoenix Tuner X** on your driver station laptop connected to the robot.
2. Select CANcoders 15, 16, 17, 18.
3. Observe the `Absolute Position` (in degrees, range $0.0^\circ$ to $360.0^\circ$).
4. Note the raw angle for each module.

#### Updating Configuration Files
Open each module's configuration JSON in `src/main/deploy/swerve/modules/`:
```json
{
  "drive": { "type": "sparkmax_neo", "id": 6 },
  "angle": { "type": "sparkmax_neo", "id": 5 },
  "encoder": { "type": "cancoder", "id": 16 },
  "inverted": { "drive": true, "angle": true },
  "absoluteEncoderOffset": 350.59572,
  "location": { "front": 11.125, "left": 11.125 }
}
```
1. Set `"absoluteEncoderOffset"` to the raw reading from Phoenix Tuner X.
2. Deploy code (`.\gradlew deploy`).
3. Power-cycle the robot. When booted, enable Teleop: wheels should stay aligned straight forward with zero twitching or 180° inversion.

---

### 4. Effective Wheel Radius Calibration via `tune.py`

Tread wear over a weekend will cause a 4.0-inch wheel to shrink to 3.85 inches, creating odometry scaling errors ($3\text{–}4\%$ drift). Calibrate effective radius using [`tools/tune/tune.py`](../tools/tune/tune.py):

#### Step 1: Physical Test
1. Set robot at a marked baseline on the carpet.
2. In Driver Station Test Mode, zero odometry.
3. Drive forward exactly $10.0\text{ meters}$ measured with a physical laser or tape measure.
4. Record drive motor rotations from AdvantageKit / NetworkTables: `/SmartDashboard/Drive/AverageMotorRotations`.

#### Step 2: Run `tune.py`
Execute the interactive tuning suite:
```powershell
python TitanRoboticsBuildSeason/tools/tune/tune.py
```
Select `[3] Swerve: Calculate Effective Wheel Radius`:
```
Target distance traveled (m): 10.0
Average drive motor shaft rotations: 172.4
Drive gear ratio (default 6.75): 6.75
Number of modules (default 4): 4
```

The script calculates:
$$\text{Wheel Rotations} = \frac{172.4}{6.75} = 25.54\text{ rev}$$
$$\text{Circumference} = \frac{10.0\text{ m}}{25.54} = 0.3915\text{ m}$$
$$\text{Effective Radius} = 0.0623\text{ m} \quad (2.45\text{ inches} \rightarrow \text{Diameter } 3.91\text{ in})$$

Update `"diameter": 3.91` in `src/main/deploy/swerve/modules/physicalproperties.json`.

---

### 5. YAGSL Closed-Loop PIDF Tuning

Gains reside in `src/main/deploy/swerve/modules/pidfproperties.json`:

```json
{
  "drive": {
    "p": 0.0001,
    "i": 0.0,
    "d": 0.0,
    "f": 0.00017,
    "iz": 0.0
  },
  "angle": {
    "p": 0.004,
    "i": 0.0,
    "d": 0.0,
    "f": 0.0,
    "iz": 0.0
  }
}
```

- **Steering PID**: Angle error in degrees. Increase `angle.p` until the module tracks rapid stick changes aggressively without oscillating on carpet.
- **Drive Feedforward (`f`)**: $f = \frac{1.0}{\text{Max NEO RPM}} = \frac{1.0}{5676} \approx 0.000176$. Provides baseline velocity feedforward.
- **Heading Correction**: `setHeadingCorrection(true)` is enabled in [`SwerveBase.java`](../src/main/java/frc/robot/Subsystems/SwerveBase.java). When rotating stick is at rest (deadband <0.06), YAGSL locks heading and compensates for translation-induced chassis rotational skew.

---

### 6. Automated SysId Characterization

System identification generates precise $kS, kV, kA$ feedforward and $kP$ feedback gains for WPILib trajectory tracking.

All routines are consolidated under [`Test/SysIdManager.java`](../src/main/java/frc/robot/Test/SysIdManager.java):
1. Place robot on carpet in open space ($>6\text{ m}$ lane).
2. Enable Driver Station in **Test Mode**.
3. Category: **SysId Characterization** (`LB + D-Pad Up` on Driver Controller).
4. Select mechanism on Elastic Dashboard:
   - `SWERVE_LINEAR`: Quasistatic and dynamic translation tests for drive wheels.
   - `SWERVE_ANGULAR`: In-place chassis rotation tests for inertia estimation.
   - `SWERVE_STEER`: Azimuth steering response tests.
5. Execute sequence (hold button to run; releasing immediately aborts with brake):
   - **Quasistatic Forward** (Slow ramp)
   - **Quasistatic Reverse** (Slow ramp)
   - **Dynamic Forward** (Step voltage)
   - **Dynamic Reverse** (Step voltage)
6. Export the `.wpilog` to AdvantageScope / WPILib SysId tool to calculate updated feedforwards.

---

## Verification

- **Wheel Alignment Invariant**:
  - Double-tap A button (`zeroGyroTrigger`) re-zeroes heading.
  - Pushing left stick straight forward drives the robot forward without crab-walking or turning.
- **Unit Tests**:
  - [`SysIdManagerTest.java`](../src/test/java/frc/robot/Test/SysIdManagerTest.java) pins characterization lifecycle and safety aborts (7/7 tests pass).
  - Full suite green: 53 test files / 499 tests pass (2026-10-07).
- **Next review due**: 2026-11-06.

---

## Related

- [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3A: SwerveBase contracts and stall recovery layers.
- [`docs/PIT_TUNING_CHECKLIST.md`](PIT_TUNING_CHECKLIST.md): Rapid 5-station pre-match verification checklist.
- [`src/main/java/frc/robot/Test/README.md`](../src/main/java/frc/robot/Test/README.md): TestMode controller layout and diagnostics.
- [`docs/INDEX.md`](INDEX.md): Central documentation directory.
