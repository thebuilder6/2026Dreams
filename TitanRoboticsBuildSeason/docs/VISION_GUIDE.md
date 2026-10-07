---
title: Dual-Vision Platform & Calibration Guide
audience: [human, ai, programming-leads]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Dual-Vision Platform & Calibration Guide — FRC Team 8334

Comprehensive engineering manual for the dual-camera vision subsystem: Limelight 3/3G MegaTag2, Orange Pi 5 coprocessor running PhotonVision, multi-tag AprilTag localization, neural network ball detection ("Ball Hunt"), and pose estimator fusion.

---

## Scope

### What this document covers
- Architecture of [`Vision.java`](../src/main/java/frc/robot/Subsystems/Vision.java) and the AdvantageKit IO abstraction layer ([`Subsystems/vision/`](../src/main/java/frc/robot/Subsystems/vision/)).
- Primary camera: Limelight 3/3G MegaTag2 with live gyro angular velocity injection.
- Secondary coprocessor: Orange Pi 5 running PhotonVision with multi-tag PNP and YOLOv8 object detection.
- Filtering and rejection logic: 5-sample median filters, latency cutoffs, high-angular-velocity rejection, and distance boundaries.
- Dynamic standard deviation weighting math for WPILib Kalman filtering in [`SwerveBase`](../src/main/java/frc/robot/Subsystems/SwerveBase.java).
- Network configuration, static IP assignments, and RoboRIO port forwarding (ports 5801–5805).
- Desktop simulation support via [`VisionIOSim`](../src/main/java/frc/robot/Subsystems/vision/VisionIOSim.java).

### What this document does NOT cover
- Swerve drive odometry kinematics and dead reckoning (see [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md)).
- Ballistics calculation and shooter distance lookup tables (see [`docs/SHOOTER_TUNING_GUIDE.md`](SHOOTER_TUNING_GUIDE.md)).
- AI decision engine targeting policy (see [`docs/KNOWLEDGE_MODEL.md`](KNOWLEDGE_MODEL.md)).

---

## Content

### 1. Dual-Vision System Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                   PRIMARY: Limelight 3 / 3G                            │
│  - NetworkTables hostname: "limelight-front"                           │
│  - MegaTag2 Pose Estimation (High-speed multi-tag with gyro)           │
│  - Receives gyro yaw, yaw rate (deg/s), pitch, roll at 50 Hz           │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Pose2d, timestamp, avgTagDist
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│            VISION SUBSYSTEM LOGIC (Vision.java)                        │
│  - 5-sample Median Filters on tag and game piece distances             │
│  - Rejection Watchdog (latency > 150ms, tag > 4.0m, yawRate > 360°/s)  │
│  - Dynamic std-dev calculation: stdDev = base + penalties              │
│  - Injects measurement into SwerveBase.addVisionMeasurement(...)       │
└───────────────────────────────────▲────────────────────────────────────┘
                                    │ Pose2d, timestamp, avgTagDist
                                    │ + Neural Network Game Piece (yaw/pitch)
┌───────────────────────────────────┴────────────────────────────────────┐
│             SECONDARY: Orange Pi 5 (PhotonVision)                      │
│  - NetworkTables table: "photonvision/rubik-pi-coprocessor"            │
│  - Multi-tag PNP AprilTag pose estimation                              │
│  - YOLOv8 Neural Network pipeline ("Ball Hunt" Fuel piece detector)    │
└────────────────────────────────────────────────────────────────────────┘
```

---

### 2. Primary Camera: Limelight MegaTag2

The primary camera is a **Limelight 3 or 3G** mounted on the front chassis facing forward:
- **Hostname**: `limelight-front`
- **NetworkTables Key**: `limelight-front`
- **Algorithm**: MegaTag2. MegaTag2 incorporates the robot's high-frequency gyro data directly on the Limelight to resolve camera tilt ambiguities and prevent perspective flipping.

#### Orientation Feedback
Every robot periodic loop ($20\text{ ms}$), [`Vision.java`](../src/main/java/frc/robot/Subsystems/Vision.java#L79-L83) transmits the chassis IMU state:
```java
primaryIO.setRobotOrientation(
    swerve.getHeading().getDegrees(),
    swerve.getGyroYawVelocityDegPerSec(),
    swerve.getPitch().getDegrees(),
    0.0
);
```
Passing instantaneous gyro yaw rate ($\text{deg/s}$) is mandatory for MegaTag2's motion blur and rolling shutter compensation.

---

### 3. Secondary Coprocessor: Orange Pi 5 (PhotonVision)

The secondary vision system runs on an **Orange Pi 5** coprocessor running PhotonVision:
- **Coprocessor Hostname**: `rubik-pi-coprocessor`
- **NetworkTables Table**: `/photonvision/rubik-pi-coprocessor`
- **Neural Subtable**: `/photonvision/rubik-pi-coprocessor-neural`

#### Neural Network Object Detection ("Ball Hunt")
In addition to AprilTag pose estimation, the Orange Pi 5 executes a custom YOLOv8 model trained on 2026 Fuel game pieces.
The coprocessor streams:
- `hasTarget`: Boolean flag indicating a detected game piece.
- `targetYaw`: Azimuth angle offset (degrees) from camera optical center.
- `targetPitch`: Elevation angle offset (degrees) relative to horizon.
- `targetArea`: Normalized contour area percentage.

#### Distance Calculation via Trigonometry
[`Vision.getGamePieceDistanceMeters()`](../src/main/java/frc/robot/Subsystems/Vision.java#L166-L185) derives distance to ground pieces using fixed camera geometry:
$$d = \frac{h_{\text{camera}} - h_{\text{target}}}{\tan(\theta_{\text{camera}} + \theta_{\text{target}})}$$

Where:
- $h_{\text{camera}} = \text{RUBIK\_PI\_CAMERA\_HEIGHT\_METERS}$ ($0.65\text{ m}$)
- $h_{\text{target}} = \text{FUEL\_TARGET\_HEIGHT\_METERS}$ ($0.08\text{ m}$, radius of fuel sphere)
- $\theta_{\text{camera}} = \text{RUBIK\_PI\_CAMERA\_PITCH\_DEG}$ (downward tilt in degrees)
- $\theta_{\text{target}} = \text{targetPitch}$ (measured pitch from neural pipeline)

A 5-sample median filter (`gamePieceDistFilter`) rejects transient false-positive frames.

---

### 4. Data Gating & Noise Rejection Matrix

Pose estimates from both cameras are subjected to strict validation gates before being permitted into the swerve pose estimator:

| Gate | Rejection Threshold | Rationale |
|---|---|---|
| **Latency Gate** | $\text{latency} > 150.0\text{ ms}$ | Rejects old frames from stale network buffers. |
| **Angular Rate Gate** | $|\omega_{\text{gyro}}| > 360.0^\circ/\text{s}$ | Motion blur severely degrades sub-pixel corner accuracy during aggressive spins. |
| **Distance Gate** | $\text{avgTagDist} > 4.0\text{ m}$ | Perspective projection noise increases quadratically with distance. |
| **Median Filter** | 5-sample sliding window | Eliminates single-frame optical spike reflections or camera misidentifications. |

---

### 5. Dynamic Standard Deviation Weighting

Rather than fixed trusting values, vision measurement variance is adjusted dynamically based on visual conditions:

$$\sigma_{xy} = \sigma_{\text{base}} + \Delta\sigma_{\text{single}} + \Delta\sigma_{\text{secondary}} + \frac{d^2}{K_{\text{dist}}}$$

In [`Vision.java`](../src/main/java/frc/robot/Subsystems/Vision.java#L95-L101):
- $\sigma_{\text{base}} = 0.05\text{ m}$ (`VISION_BASE_STD_DEV`)
- $\Delta\sigma_{\text{single}} = 0.10\text{ m}$ if only 1 AprilTag is visible (`VISION_SINGLE_TAG_PENALTY`)
- $\Delta\sigma_{\text{secondary}} = 0.15\text{ m}$ applied to the Orange Pi 5 (Limelight MegaTag2 given higher base trust)
- Distance penalty divisor $K_{\text{dist}} = 30.0$ (`VISION_DIST_PENALTY_DIVISOR`)
- $\sigma_\theta = 900^\circ$ ($15.7\text{ rad}$): We explicitly do **not** trust vision yaw for swerve rotation. High-frequency NavX/Pigeon gyro dead reckoning remains strictly authoritative for heading.

```java
Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(900));
swerve.addVisionMeasurement(inputs.estimatedPose, inputs.timestamp, visionStdDevs);
```

---

### 6. Network Configuration & Port Forwarding

All hardware is networked over static 10.TE.AM.x IP addressing on the competition VLAN:

| Device | Hostname / mDNS | Static IP | Subnet Mask | Gateway |
|---|---|---|---|---|
| **RoboRIO 2.0** | `roborio-8334-frc.local` | `10.83.34.2` | `255.255.255.0` | `10.83.34.1` |
| **Driver Station** | `—` | `10.83.34.5` | `255.255.255.0` | `10.83.34.1` |
| **Limelight 3/3G** | `limelight-front.local` | `10.83.34.11` | `255.255.255.0` | `10.83.34.1` |
| **Orange Pi 5** | `rubik-pi-coprocessor.local` | `10.83.34.12` | `255.255.255.0` | `10.83.34.1` |

#### Port Forwarding (USB Tether Access)
In [`Robot.java`](../src/main/java/frc/robot/Robot.java#L105-L110), WPILib `PortForwarder` forwards coprocessor web interfaces over the USB tether:
- Port `5801` → `10.83.34.11:5801` (Limelight web configuration interface)
- Port `5802` → `10.83.34.11:5800` (Limelight camera stream)
- Port `5803` → `10.83.34.12:5800` (Orange Pi 5 PhotonVision web dashboard)
- Port `5804` → `10.83.34.12:1181` (PhotonVision stream 1)
- Port `5805` → `10.83.34.12:1182` (PhotonVision stream 2)

Connecting your laptop via USB cable and opening `http://172.22.11.2:5801` or `http://172.22.11.2:5803` provides immediate access in the pit without switching WiFi networks.

---

### 7. Simulation Architecture (`VisionIOSim`)

In simulation (`.\gradlew simulateJava`), physical camera IO is replaced by [`VisionIOSim`](../src/main/java/frc/robot/Subsystems/vision/VisionIOSim.java):
- Utilizes `PhotonCameraSim` and `VisionSystemSim` with simulated AprilTags positioned at 2026 field coordinates.
- Simulates realistic camera resolution ($1280\times800$), field of view ($70^\circ$ diagonal), frame rates ($30\text{ fps}$), and Gaussian pixel noise.
- Generates simulated NetworkTables traffic under the `/photonvision` and `/limelight-front` namespaces.

---

## Verification

- **Automated Pre-Flight Check**:
  - Run Pre-Flight Diagnostics from Elastic Dashboard or Driver Station Test Mode. Station 6 verifies vision heartbeats and camera availability (`Diagnostics.java:314`).
- **Telemetry Verification**:
  - Check NetworkTables keys:
    - `/Vision/Primary/HasTarget`: True when AprilTag is visible.
    - `/Vision/Primary/TagCount`: Number of tags in view.
    - `/Vision/Primary/AvgTagDist`: Filtered distance in meters.
    - `/Vision/Secondary/HasGamePiece`: True when Fuel piece is detected by YOLOv8.
- **Unit Tests**:
  - Full suite passes: 53 test files / 500 tests green (2026-10-07).
- **Next review due**: 2026-11-06.

---

## Related

- [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3D: Dual-Vision platform specification.
- [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md): Odometry integration and Kalman filter tuning.
- [`docs/PIT_TUNING_CHECKLIST.md`](PIT_TUNING_CHECKLIST.md): Pre-match 15-second diagnostics and camera health checks.
- [`docs/INDEX.md`](INDEX.md): Central documentation directory.
