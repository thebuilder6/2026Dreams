# 🤖 2026 Robot Operator's Guide

Welcome to the **Titan Robotics 2026 Driver and Operator Manual**. This guide documents the unified dual-controller layout, input shaping dynamics, automated assist features, smart intake mechanics, and diagnostic telemetry indicators.

---

## 🎮 Controller Layouts

The robot supports **Dual Xbox Controllers** (Driver on Port 0, Operator on Port 1) with automatic **Single-Controller Fallback** (all operator commands work seamlessly on the Driver controller if the Operator controller is disconnected).

---

### 🕹️ Driver Controller (Port 0)

| Control | Function | Description |
| :--- | :--- | :--- |
| **Left Stick (X/Y)** | **Field-Oriented Translation** | Non-linear cubic response ($0.7x^3 + 0.3x$) with $4.5\text{ m/s}^2$ slew rate acceleration smoothing (tunable via `Operator/TranslationSlewRate`). |
| **Right Stick (X)** | **Manual Rotation** | Precision cubic angular response ($7.0\text{ rad/s}^2$ slew limited, tunable via `Operator/RotationSlewRate`). |
| **Left Stick Click** | **Slow Mode (Toggle)** | Caps linear speed to 35% and angular speed to 50% for precision alignment. |
| **D-Pad (POV)** | **Cardinal Snap-to-Heading** | **Up**: Face $0^\circ$ (Forward)<br>**Right**: Face $-90^\circ$ / $270^\circ$ (Right)<br>**Down**: Face $180^\circ$ (Backward)<br>**Left**: Face $+90^\circ$ (Left) |
| **A Button** | **Zero Gyro** | Re-calibrates field orientation zero relative to current alliance. |
| **Right Trigger (Hold > 30%)** | **Auto-Aim & Shoot** | Locks swerve heading onto the Hub, spools dual flywheels to distance-interpolated RPM, triggers haptic confirmation buzz, and automatically fires when lined up ($<3^\circ$ error) and at target speed. |
| **Left Trigger (Hold > 30%)** | **Ground Intake (Hold-to-Run)** | Deploys arm to ground ($250^\circ$), runs intake rollers and hopper. Retracts to standby ($347^\circ$) upon release. |
| **Right Bumper (Hold)** | **Smart Glide Mode** | Autonomously navigates to the optimal waypoint arbitrated dynamically by the Jev AI Decision Engine (Hub when loaded & active, Midfield hunt when empty, Depot when inactive). Features **Smart Tunnel Navigation** with automatic corridor diversion if an opponent blocks a trench. Manual stick deflection ($>30\%$) cancels cleanly. |
| **Left Bumper (Hold)** | **Auto Ball Pick Up** | Activates vision object tracking and autonomous intake alignment. Features **Shared Driver Authority** (driver stick nudges search area without cancelling) and **350ms Blindspot Memory** for seamless bumper-level ingestion. |
| **X Button (Press)** | **Arm Toggle** | Manually toggles intake arm between Standby ($347^\circ$) and Deployed ($250^\circ$). |
| **B Button (Hold)** | **Eject / Unjam** | Reverses rollers and hopper to clear obstructions. |
| **Y Button (Hold)** | **Feed / Pass** | Runs hopper and intake rollers in standby position to pass balls without shooting. |
| **Back / Start** | **E-Stop / Abort** | Immediately stops all drive motors, halts shooter, retracts intake, and cancels active actions. |

---

### 🎯 Operator Controller (Port 1 - Optional Co-Pilot)

| Control | Function | Description |
| :--- | :--- | :--- |
| **Right Trigger (Hold > 30%)** | **Auto-Aim & Shoot** | Overrides shooter spooling and auto-firing sequence. |
| **Left Trigger (Hold > 30%)** | **Ground Intake** | Deploys arm and runs rollers/hopper. |
| **X Button** | **Arm Toggle** | Toggles arm deploy/standby. |
| **B Button (Hold)** | **Eject / Unjam** | Clears intake jams. |
| **Y Button (Hold)** | **Feed / Pass** | Passes balls to alliance partners. |
| **D-Pad Up (POV 0)** | **Arm Standby** | Commands arm directly to $347^\circ$ standby position. |
| **D-Pad Down (POV 180)** | **Arm Ground** | Commands arm directly to $250^\circ$ ground position. |
| **Back / Start** | **E-Stop / Abort** | Immediate safety override. |

---

## 📳 Tactile Haptic Feedback Patterns

Both controllers feature non-blocking rumble patterns to communicate real-time robot state without requiring the driver to look away from the field:

1. **Target Locked (Crisp Double Pulse)**: Fires on right rumble motor when flywheels reach target RPM and robot heading aligns within $3^\circ$ with an active Hub.
2. **Ball Acquired (Solid Medium Buzz)**: Fires when a fuel piece is ingested into the intake / hopper.
3. **Pin Warning (Rapid Double Buzz)**: Alerts the driver when bumper contact against an opponent robot approaches the $2.0\text{s}$ FRC G-rule pin limit.
4. **Hub Phase Shift (Rhythmic Double Pulse)**: Warns the driver $\le 3.0\text{s}$ before the Hub active/inactive scoring cycle switches.
5. **Collision Impact (High-G Shock Pulse)**: Instantaneous full-intensity pulse ($160\text{ms}$) triggered by IMU accelerometer jerk ($\|\vec{J}\| > 35\text{ m/s}^3$) upon frame or bumper contact.
6. **Directional Flank Alert (Left / Right Grip Vibration)**: Vibrates the corresponding controller grip when an opponent robot approaches within $2.2\text{m}$ in a driver blindspot.
7. **Endgame Reminders (Long Deep Rumble)**: Alerts the drive team at **30 seconds** and **15 seconds** remaining in the match for climbing.
8. **Hardware / Vision Warning (Rapid Triple Buzz)**: Alerts the driver if AprilTag vision drops into pure odometry mode or an active subsystem fault occurs.

---

## 📍 Strategic Navigation: Glide Points & Smart Auto-Tunneling

Holding **Right Bumper** calculates a smooth, obstacle-aware trajectory:
- **Alliance Feeders (Top/Bottom)**: Fast alignment for rapid human loading.
- **Alliance Hub (Front/Back)**: Ideal perimeter positions for scoring.
- **Smart Trench Auto-Tunneling**:
  - **Bi-Directional**: Automatically determines traversal direction (Alliance Zone $\to$ Midfield vs Midfield $\to$ Alliance Zone) and aligns heading ($0^\circ$ or $180^\circ$).
  - **Opponent Blockage Detection & Auto-Diversion**: Continuously monitors dynamic obstacles in the Top and Bottom trenches; if an opponent blocks the preferred trench, the router instantly and safely diverts to the open corridor.
  - **4-Stage Funneling & Centerline Lock**: Smooth pre-entry funneling ($X \pm 0.60\text{m}$) prevents clipping the steel truss, while stiff cross-track centering locks the chassis onto the corridor centerline.
- **Midfield Crossings (Top/Bottom)**: Protected lanes across the center zone.

---

## 📊 Elastic Dashboard Telemetry & Controls

The Elastic Dashboard (`elastic-layout.json`) provides real-time situational awareness across 5 tabs:

### 1. Driver Dashboard HUD
- **Shooter Ready Status**: Green light when flywheels are within $\pm 50\text{ RPM}$ of target.
- **Lined Up Indicator**: Green when Limelight tag tracking is within $\pm 2.0^\circ$.
- **Alliance Hub Status Banner**: Live countdown timer showing remaining seconds before the next 25-second Hub scoring shift ('R' / 'B' shift rules).
- **Nearest Glide Target**: Displays the target waypoint name before trigger engagement.
- **3D Robot Field View**: Live holonomic pose, vision ghost, trajectory pathing, and Hub timing ring.

### 2. Pre-Flight Diagnostics
- Live Scorecard displaying **CAN Bus**, **Drivebase**, **Steer Alignment**, **Intake & Jam Protection**, **Dual Flywheels**, and **Vision Links**.
- 12-motor manual jog testing bench.

### 3. SysID & Characterization
- Dedicated execution buttons for **Drive Linear**, **Drive Angular**, **Steer Azimuth**, **Flywheels**, and **Intake Arm**.

### 4. Tuning & PID (Live Tuning Hub)
- **Shooter Dual Flywheels**: Real-time RPM telemetry & live PID ($kP, kI, kD$) + Feedforward ($kS, kV, kA$) inputs, physical dimensions, and launch efficiency coefficients.
- **Intake Arm Pivot**: Real-time angle telemetry vs goal & live Profiled PID ($kP, kI, kD$) + Gravity Feedforward ($kS, kG, kV, kA$).
- **Autonomous Holonomic Pathfinding**: Live Choreo/Pure Pursuit Drive ($kP, kI, kD$) and Heading Turn ($kP, kI, kD$) controllers.
- **Driver Response Shaping**: Live Slew Rate Limiters ($4.5\text{ m/s}^2$ translation, $7.0\text{ rad/s}^2$ rotation) and assist toggles.

### 5. Simulation & Match Telemetry
- MapleSim 3D physics feed, battery sag estimator, ball respawner, opponent AI defense toggle, and match clock.
