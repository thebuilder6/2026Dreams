---
title: Dual-Vision Platform & Calibration Guide
audience: [human, ai, programming-leads]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Dual-Vision Platform & Calibration Guide — FRC Team 8334

Comprehensive engineering manual for the overhauled vision subsystem: declarative multi-camera configuration ([`CameraConfig`](../src/main/java/frc/robot/Subsystems/vision/CameraConfig.java)), central configuration & live tunables ([`VisionConfig`](../src/main/java/frc/robot/Subsystems/vision/VisionConfig.java)), Limelight 3/3G MegaTag2, Orange Pi 5 coprocessor running PhotonVision, multi-tag AprilTag localization, neural network ball detection ("Ball Hunt"), connection watchdogs, transparent rejection matrix, and pose estimator fusion.

---

## Scope

### What this document covers
- Architecture of [`Vision.java`](../src/main/java/frc/robot/Subsystems/Vision.java) and the AdvantageKit IO abstraction layer ([`Subsystems/vision/`](../src/main/java/frc/robot/Subsystems/vision/)).
- Declarative camera configuration via [`CameraConfig`](../src/main/java/frc/robot/Subsystems/vision/CameraConfig.java) with fluent builders for 1, 2, or N cameras.
- Central tuning and live pit calibration via [`VisionConfig`](../src/main/java/frc/robot/Subsystems/vision/VisionConfig.java) and NetworkTables `TunableNumber` controls.
- Primary camera: Limelight 3/3G MegaTag2 with live gyro angular velocity injection and automatic robot-space mounting pose transmission (`setCameraPose_RobotSpace`).
- Secondary coprocessor: Orange Pi 5 running PhotonVision with dual-pipeline PhotonLib/NetworkTables support and YOLOv8 object detection.
- Transparent rejection matrix ([`RejectionReason`](../src/main/java/frc/robot/Subsystems/Vision.java#L36-L50)): latency, distance, yaw rate, ambiguity, and field boundary gates.
- Connection watchdog and Elastic Dashboard [`AlertManager`](../src/main/java/frc/robot/Telemetry/AlertManager.java) warnings for unplugged/offline cameras.
- Dynamic standard deviation weighting math for WPILib Kalman filtering in [`SwerveBase`](../src/main/java/frc/robot/Subsystems/SwerveBase.java).
- Network configuration, static IP assignments, and RoboRIO port forwarding (ports 5801–5805).
- Desktop simulation support via [`VisionIOSim`](../src/main/java/frc/robot/Subsystems/vision/VisionIOSim.java).

### What this document does NOT cover
- Swerve drive odometry kinematics and dead reckoning (see [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md)).
- Ballistics calculation and shooter distance lookup tables (see [`docs/SHOOTER_TUNING_GUIDE.md`](SHOOTER_TUNING_GUIDE.md)).
- AI decision engine targeting policy (see [`docs/KNOWLEDGE_MODEL.md`](KNOWLEDGE_MODEL.md)).

---

## Quickstart: Configuring a Camera in 60 Seconds

The overhauled vision subsystem uses declarative camera definitions in [`VisionConfig.java`](../src/main/java/frc/robot/Subsystems/vision/VisionConfig.java). You no longer need to modify subsystem plumbing, hardcode offsets across files, or write custom NetworkTables loops.

### Adding a New Limelight
In `VisionConfig.java`:
```java
VisionConfig.addCamera(
    CameraConfig.limelight("limelight-back")
        .withTransform(new Transform3d(
            new Translation3d(-0.28, 0.0, 0.45), // 28cm behind robot center, 45cm off carpet
            new Rotation3d(0.0, Math.toRadians(15.0), Math.PI))) // 15° pitch up, facing backward
        .withRole(CameraRole.APRILTAG)
        .withStdDevMultiplier(1.0)
        .withMegaTag2(true)
);
```

### Adding a PhotonVision Coprocessor Camera
```java
VisionConfig.addCamera(
    CameraConfig.photonVision("photon-intake")
        .withTransform(new Transform3d(
            new Translation3d(0.25, 0.0, 0.45),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0))) // 15° downward tilt toward carpet
        .withRole(CameraRole.OBJECT_DETECTION)
        .withStdDevMultiplier(1.2)
);
```

### Rapid Presets
Switch setups instantly using built-in presets in [`VisionConfig.Presets`](../src/main/java/frc/robot/Subsystems/vision/VisionConfig.java#L81-L140):
- `VisionConfig.Presets.dualDefault()`: Front Limelight + Orange Pi 5 PhotonVision coprocessor.
- `VisionConfig.Presets.singleLimelight()`: Front Limelight only.
- `VisionConfig.Presets.singlePhotonVision()`: PhotonVision coprocessor only.
- `VisionConfig.Presets.dualPhotonVision()`: Dual PhotonVision (front + back).

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                   PRIMARY: Limelight 3 / 3G                            │
│  - Hostname: "limelight-front"                                         │
│  - MegaTag2 Pose Estimation with live gyro feedback                    │
│  - Automatic robot-space mount configuration via setCameraPose         │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Pose2d, timestamp, avgTagDist, ambiguity
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│            MANAGED CAMERA PIPELINE (Vision.java)                       │
│  - 5-sample MedianFilter on tag and game piece distances               │
│  - Transparent Rejection Matrix (Reason published to NT/AdvantageKit)  │
│  - Dynamic std-dev calculation: stdDev = (base + penalties) * mult     │
│  - Connection watchdog: Disconnect Alert raised if camera drops offline│
│  - Injects valid measurement into SwerveBase.addVisionMeasurement(...) │
└───────────────────────────────────▲────────────────────────────────────┘
                                    │ Pose2d, timestamp, avgTagDist, ambiguity
                                    │ + Neural Network Game Piece (yaw/pitch)
┌───────────────────────────────────┴────────────────────────────────────┐
│             SECONDARY: Orange Pi 5 (PhotonVision)                      │
│  - Hostname: "rubik-pi-coprocessor"                                    │
│  - Native PhotonLib + NetworkTables dual-pipeline                      │
│  - YOLOv8 Neural Network pipeline ("Ball Hunt" Fuel piece detector)    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Data Gating & Noise Rejection Matrix

Every camera measurement is evaluated against strict rejection gates before admission into the Kalman pose estimator. If rejected, the specific reason is published live to `/Vision/<CameraName>/RejectionReason`:

| Rejection Reason | Gate Threshold | Rationale |
|---|---|---|
| `NO_TARGET` | $\text{tagCount} == 0$ | No AprilTag fiducial in view. |
| `CAMERA_OFFLINE` | No packet in $> 1.5\text{ s}$ | Camera disconnected or network stalled. |
| `VISION_DISABLED` | `MasterEnabled} == 0$ or camera disabled | Operator/dashboard software lockout. |
| `HIGH_YAW_RATE` | $|\omega_{\text{gyro}}| > 360.0^\circ/\text{s}$ (`MAX_YAW_RATE`) | Motion blur degrades sub-pixel corner accuracy during fast spins. |
| `HIGH_LATENCY` | $\text{latency} > 150.0\text{ ms}$ (`MAX_LATENCY_MS`) | Stale network buffers invalidate pose synchronization. |
| `HIGH_DISTANCE` | $\text{dist} > 4.0\text{ m}$ (or $> 3.0\text{ m}$ single-tag) | Angular error amplifies perspective projection noise quadratically. |
| `HIGH_AMBIGUITY` | $\text{ambiguity} > 0.40$ (`MAX_AMBIGUITY`) | Single-tag coplanar ambiguity flip protection. |
| `OUTSIDE_FIELD` | $X \notin [-0.5, 17.04]$, $Y \notin [-0.5, 8.55]$ | Rejects wild optical misidentifications outside the physical arena. |
| `ACCEPTED` | Passes all above gates | Measurement valid and forwarded to `SwerveBase`. |

---

## Dynamic Standard Deviation Weighting

Rather than fixed trust constants, measurement variance is dynamically calculated using visual conditions and scaled by the camera's individual trust multiplier:

$$\sigma_{xy} = \left( \sigma_{\text{base}} + \Delta\sigma_{\text{single}} + \frac{d^2}{K_{\text{dist}}} \right) \cdot M_{\text{camera}}$$

In [`Vision.java`](../src/main/java/frc/robot/Subsystems/Vision.java#L225-L236) and [`VisionConfig.java`](../src/main/java/frc/robot/Subsystems/vision/VisionConfig.java):
- $\sigma_{\text{base}} = 0.08\text{ m}$ (`BASE_STD_DEV`, tunable)
- $\Delta\sigma_{\text{single}} = 0.15\text{ m}$ applied when only 1 tag is visible (`SINGLE_TAG_PENALTY`, tunable)
- Distance penalty divisor $K_{\text{dist}} = 25.0$ (`DIST_PENALTY_DIVISOR`, tunable)
- Camera multiplier $M_{\text{camera}} = \text{config.getStdDevMultiplier()}$ ($1.0$ for primary Limelight, $1.2$ for secondary)
- Heading trust $\sigma_\theta = 900^\circ$ ($15.7\text{ rad}$): We explicitly **do not** trust vision heading for swerve rotation. High-frequency NavX/Pigeon gyro dead reckoning remains strictly authoritative.

---

## Live Dashboard Tuning & Elastic "Vision & Cameras" Tab

A dedicated **Vision & Cameras** tab is provided in Elastic Dashboard (`elastic-layout.json`) giving pit crew and programmers complete real-time visibility:
- **Per-Camera Status Cards**: Live `Enabled` toggle switch, `Connected` & `Accepted` indicators, text rejection reason, tag count, filtered distance bar, latency, and dynamic standard deviation.
- **Hardware Defect Isolation**: If a camera or USB cable fails at an event, flip its dashboard `Enabled` toggle switch to **OFF**. The vision pipeline immediately drops the camera, updates the watchdog to ignore it, and continues fusing remaining cameras into odometry without code restarts or redeployments.
- **Neural "Ball Hunt" Tracking Card**: Real-time target acquisition boolean, filtered distance to game piece, area, and yaw/pitch error bars.
- **Field Map View**: Live 2D arena overlay displaying fused robot pose alongside detected game pieces and AprilTag visual benchmarks.

### Tunable Parameters (NetworkTables)

All rejection gates and weighting parameters can be tuned in real-time from Elastic Dashboard or SmartDashboard without rebuilding code:

| NetworkTables Key | Default | Function |
|---|---|---|
| `/TunableNumbers/Vision/Config/MasterEnabled` | `1.0` | Master enable/disable toggle for all vision pose updates |
| `/TunableNumbers/Vision/Config/MaxTagDistMeters` | `4.0 m` | Maximum tag distance for multi-tag measurements |
| `/TunableNumbers/Vision/Config/SingleTagMaxDistMeters` | `3.0 m` | Maximum tag distance when only 1 tag is visible |
| `/TunableNumbers/Vision/Config/MaxAmbiguity` | `0.40` | Maximum ambiguity ratio for single-tag estimates |
| `/TunableNumbers/Vision/Config/MaxYawRateDegPerSec` | `360.0°/s` | Maximum chassis spin rate permitted during vision capture |
| `/TunableNumbers/Vision/Config/MaxLatencyMs` | `150.0 ms` | Maximum frame latency accepted |
| `/TunableNumbers/Vision/Config/BaseStdDev` | `0.08 m` | Base translation standard deviation |
| `/TunableNumbers/Vision/Config/SingleTagPenalty` | `0.15 m` | Additional standard deviation penalty for single tags |
| `/TunableNumbers/Vision/Config/DistPenaltyDivisor` | `25.0` | Divisor governing distance quadratic penalty |

---

## Pit & Field Calibration CLI

For rapid carpet calibration of camera mounting pitch and AprilTag distance scaling:
```powershell
python tools/tune/tune.py vision
```
Or directly:
```powershell
python tools/tune/calibrate_vision.py --interactive
```
The CLI wizard provides:
1. **Camera Mounting Pitch Calibration**: Places robot at measured carpet distances (e.g. 1.5m, 2.5m, 3.5m) and analytically solves for the exact mounting pitch angle $\theta_c$, accounting for camera height $h_c$ and target height $h_t$.
2. **AprilTag Distance Validation**: Compares NetworkTables measured distances against carpet tape benchmarks and outputs linear scaling corrections.
3. **Java Code Generation**: Generates ready-to-paste `CameraConfig` Java snippets with solved transforms.

---

## Object Detection ("Ball Hunt") & Geometry

Ground Fuel game piece targeting uses fixed camera geometry defined in `CameraConfig`:
$$d = \frac{h_{\text{camera}} - h_{\text{target}}}{\tan(\theta_{\text{camera}} + \theta_{\text{target}})}$$

- $h_{\text{camera}} = \text{config.getCameraHeightMeters()}$ ($0.45\text{ m}$)
- $h_{\text{target}} = \text{FUEL\_TARGET\_HEIGHT\_METERS}$ ($0.08\text{ m}$, radius of fuel sphere)
- $\theta_{\text{camera}} = \text{config.getCameraPitchDegrees()}$ (e.g., $-15^\circ$ downward tilt)
- $\theta_{\text{target}} = \text{inputs.gamePiecePitch}$ (measured pitch from YOLOv8)

A 5-sample median filter (`gamePieceDistFilter`) rejects transient false-positive frames.

---

## Network Configuration & Port Forwarding

| Device | Hostname / mDNS | Static IP | Subnet Mask | Gateway |
|---|---|---|---|---|
| **RoboRIO 2.0** | `roborio-8334-frc.local` | `10.83.34.2` | `255.255.255.0` | `10.83.34.1` |
| **Driver Station** | `—` | `10.83.34.5` | `255.255.255.0` | `10.83.34.1` |
| **Limelight 3/3G** | `limelight-front.local` | `10.83.34.11` | `255.255.255.0` | `10.83.34.1` |
| **Orange Pi 5** | `rubik-pi-coprocessor.local` | `10.83.34.12` | `255.255.255.0` | `10.83.34.1` |

USB Tether port forwarding in [`Robot.java`](../src/main/java/frc/robot/Robot.java#L105-L110):
- Port `5801` → `10.83.34.11:5801` (Limelight web config)
- Port `5802` → `10.83.34.11:5800` (Limelight stream)
- Port `5803` → `10.83.34.12:5800` (PhotonVision dashboard)
- Port `5804` → `10.83.34.12:1181` (PhotonVision stream 1)
- Port `5805` → `10.83.34.12:1182` (PhotonVision stream 2)

---

## Verification

- **Automated Pre-Flight Check**:
  - Run Pre-Flight Diagnostics from Elastic Dashboard. Station 6 inspects live connection heartbeats across all configured cameras via `vision.isAllCamerasConnected()` (`Diagnostics.java:314`).
- **Telemetry Verification**:
  - Check NetworkTables keys under `/Vision/<CameraName>/`:
    - `Enabled`: Boolean toggle input/output.
    - `Connected`: Boolean reporting frame freshness.
    - `IsAccepted`: Boolean indicating admission into SwerveBase.
    - `RejectionReason`: String (`ACCEPTED`, `HIGH_DISTANCE`, `VISION_DISABLED`, etc.).
    - `StdDev`: Live dynamic translation standard deviation in meters.
- **Unit Tests**:
  - Dedicated suite [`VisionTest.java`](../src/test/java/frc/robot/Subsystems/VisionTest.java): 7/7 tests passing (configuration, presets, multi-camera, rejection matrix, dynamic std-dev, neural tracking, per-camera enable toggle).
  - Full suite passes: 54 test files / 513 tests green (2026-10-07).
- **Next review due**: 2026-11-06.

---

## Related

- [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3D: Vision platform specification.
- [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md): Swerve drive odometry and vision pose fusion.
- [`docs/PIT_TUNING_CHECKLIST.md`](PIT_TUNING_CHECKLIST.md): Pre-match 15-second diagnostics and camera health checks.
- [`docs/INDEX.md`](INDEX.md): Central documentation directory.
