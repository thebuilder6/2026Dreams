---
title: Operator Map
audience: [human, drive-team]
owner: drive-team
last_verified: 2026-09-26
status: authoritative
---

# 🤖 2026 Robot Operator's Guide

Welcome to the **Titan Robotics 2026 Driver and Operator Manual**. This guide documents the unified dual-controller layout, input shaping dynamics, automated assist features, smart intake mechanics, and diagnostic telemetry indicators.

---

## 🎮 Controller Layouts

The robot supports **Dual Xbox Controllers** (Driver on Port 0, Operator on Port 1) with automatic **Single-Controller Fallback** (all operator commands work seamlessly on the Driver controller if the Operator controller is disconnected).

---

### 🕹️ Driver Controller (Port 0)

| Control | Function | Description |
| :--- | :--- | :--- |
| **Left Stick (X/Y)** | **Field-Oriented Translation** | Non-linear cubic response ($0.7x^3 + 0.3x$) with slew rate acceleration smoothing (tunable via `Operator/TranslationSlewRate`, default 16, `Constants.java:131`). |
| **Right Stick (X)** | **Manual Rotation** | Precision cubic angular response (slew limited, tunable via `Operator/RotationSlewRate`, default 10, `Constants.java:132`). |
| **Left Stick Click** | **Slow Mode (Toggle)** | Caps linear speed to 35% and angular speed to 50% for precision alignment. |
| **D-Pad (POV)** | **Cardinal Snap-to-Heading** | **Up**: Face $0^\circ$ (Forward)<br>**Right**: Face $-90^\circ$ / $270^\circ$ (Right)<br>**Down**: Face $180^\circ$ (Backward)<br>**Left**: Face $+90^\circ$ (Left)<br>Headings are driver-relative via `AllianceFlipUtil` (+180° on Red). Gated by Dashboard Snap-Turn toggle; any rotation input clears snap. |
| **A Button** | **Zero Gyro** | **Double-tap within 0.4s** re-calibrates field orientation (`zeroGyroTrigger.multiPress(2, 0.4)`, `Teleop.java:116-119`). Single press does nothing. |
| **Right Trigger (Hold > 30%)** | **Auto-Aim & Shoot** | Locks swerve heading onto the Hub, spools dual flywheels to distance-interpolated RPM, triggers haptic confirmation buzz, and automatically fires when lined up ($<3^\circ$ error) and at target speed. **Rule Constraint**: Firing is permitted *only within your Alliance Zone* (Blue $X \le 4.5974\text{m}$, Red $X \ge 11.938\text{m}$, `Navigation/FieldMap.java:130-132`); shooter solution covers 1.2–6.5 m (`Shooter.java:194,247`). |
| **Left Trigger (Hold > 30%)** | **Ground Intake (Hold-to-Run)** | Deploys arm to ground ($250^\circ$), runs intake rollers and hopper. Retracts to standby ($347^\circ$) upon release. |
| **Right Bumper (Hold)** | **Smart Glide Mode** | Autonomously navigates to the optimal waypoint arbitrated dynamically by the Jev AI Decision Engine. Manual stick deflection (drive > 0.65 or rotation > 0.60) cancels cleanly (`AutonomousTeleopAgent.java:120`). |
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
3. **Pin Warning (Rapid Double Buzz, aspirational — `PIN_WARNING` currently has no callers)**: Intended alert when bumper contact approaches the pin limit (code warns at 1.8 s, max 2.4 s, `LegalPinningWatchdog.java:17`).
4. **Hub Phase Shift (Rhythmic Double Pulse, aspirational — currently unwired)**: Intended warning before Hub active/inactive switches.
5. **Collision Impact (Directional Opposing Deceleration Pulse)**: Instantaneous full-intensity pulse ($160\text{ms}$) triggered by opposing deceleration ($a_{\text{opposing}} = -(\vec{a}_{\text{filt}} \cdot \hat{u}_v) > 10.0\text{ m/s}^2$ and $J_{\text{opposing}} > 120.0\text{ m/s}^3$) filtered with a 1st-order low-pass filter ($\alpha = 0.35$). Normal driving/acceleration produces negative opposing deceleration, mathematically preventing false positives. Gated by dashboard switch (`Operator/HapticCollisionEnabled`, default disabled in simulation, enabled on real hardware).
6. **Directional Flank Alert (aspirational — `triggerDirectionalFlankAlert()` currently has no callers)**: Intended left/right grip vibration on blindspot approach.
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

The Elastic Dashboard (`elastic-layout.json`) provides real-time situational awareness across 7 tabs (Driver Dashboard, AI Coach & Practice, Pre-Flight Diagnostics, SysID & Characterization, Simulation & Match Info, Match Scoreboard, Tuning & PID). The layout can be loaded directly from the robot or simulation via **`Ctrl + D`** (Remote Layout Downloading over HTTP port 5800) or by opening `elastic-layout.json`:

### 1. Driver Dashboard HUD
- **Match Time Countdown**: Dedicated large-format clock widget with automatic color transitions (Blue > 60s, Green < 60s, Yellow <= 30s, Red <= 15s).
- **Shooter Ready Status & Hub Active**: Large status indicators showing Hub state and shoot lock.
- **Flywheel RPM Live Graph**: Real-time time-series wave graph displaying instantaneous flywheel spool-up, recovery after firing, and target stability.
- **Held Fuel Gauge**: Visual 0-5 ball fullness bar.
- **Assist Feature Toggles**: Interactive `Toggle Switch` controls for Snap Turn, Auto Aim, Ball Hunt, Glide Points, Slow Mode, and optional TypeSafe Jev. TypeSafe is off by default; when enabled with a configured key, the player Co-Pilot and simulator bots can make cloud requests. Set `JevAI/DecisionMode` to `LOCAL_HEURISTIC`, `TYPESAFE_CLOUD`, or `AUTO_FALLBACK`. Missing keys, stale replies, API errors, low-confidence replies, and locally unsafe or low-priority choices use the local policy; simulator bot setup and request limits are in `SIMULATION_GUIDE.md`.
- **3D Robot Field View**: Live holonomic pose, vision ghost, trajectory pathing, and Hub timing ring.

### 2. AI Coach & Practice Proving Ground
- **Driver Grade Display**: Dynamic letter grade ($A+$ to $D$) reflecting cycle speed and shot timing discipline.
- **Live Shooting Accuracy Bar**: Percentage of shots taken with locked heading and target RPM during active Hub periods.
- **Cycle Timing Gauges**: Real-time display of average cycle duration, fastest cycle record, and total completed cycles.
- **Practice Drill Chooser**: Switch between `Free Play Match`, `Rapid Cycling Sprint`, `Trench Defense`, and `Anti-Defense SOTF` drills.
- **Reset Practice Arena**: Interactive `Toggle Button` to clear stats, respawn all field balls, and teleport robot to starting line.
- **Haptic Collision Alert Switch**: Interactive `Toggle Switch` for controller collision rumble.

### 3. Pre-Flight Diagnostics
- Live Scorecard displaying **CAN Bus**, **Drivebase**, **Steer Alignment**, **Intake & Jam Protection**, **Dual Flywheels**, and **Vision Links**.
- Automated 15-second pre-flight routine with progress bar.
- 12-motor manual jog test bench with interactive `Toggle Button` controls to pulse individual steer/drive azimuths, intake arm, rollers, and flywheels.

### 4. SysID & Characterization
- Dedicated execution buttons (`Toggle Button`) for **Quasistatic Forward / Reverse**, **Dynamic Forward / Reverse**, and **ABORT / E-STOP**.
- Real-time `Graph` widgets for **Live Applied Voltage** (-12V to +12V) and **Live Velocity** for instant waveform visualization.

### 5. Simulation & Multi-Bot Match Telemetry
- **Embedded Arena Field View**: 2D holonomic field tracking the player robot alongside up to 3 AI opponent bots (`OpponentBot0`, `OpponentBot1`, `OpponentBot2`) with target waypoints and heading vectors.
- **Opponent Count Dropdown Chooser**: Select between 1, 2, or 3 simultaneous opponent bots.
- **Independent Bot Archetype Dropdowns**: Dropdown menus for Bot 0, Bot 1, and Bot 2 strategy assignments (Autonomous Fuel Cycler, Aggressive Defense Bully, Adaptive Match Competitor, Tactical Defender, Lead Pursuit Interceptor — `Archetype.java:7-12`) with live status rationale.
- **Speed Slider**: Interactive `Number Slider` for opponent velocity scaling (20% to 100%).
- **Interactive Action Triggers**: `Toggle Button` controls to Reset Simulation and Respawn Fuel Balls.
- **Multi-Bot Scoring & Ball Count**: Live tally of individual bot scores and total opponent points scored against the driver.
- **Pit Mode**: Interactive `Toggle Switch` to lock swerve wheels in X-brake configuration.

### 6. Match Scoreboard
- Live red/blue totals, Leader, auto/teleop fuel splits, foul points, and climb status (`Scoreboard/*`, `MatchScoreTracker.java`). Added after the original 6-tab layout — see `SIMULATION_GUIDE.md` §8.

### 7. Tuning & PID (Live Tuning Hub)
- **Shooter Dual Flywheels**: Real-time RPM telemetry & live PID ($kP, kI, kD$) + Feedforward ($kS, kV, kA$) text inputs with submit buttons.
- **Intake Arm Pivot**: Real-time angle telemetry bars vs goal & live Profiled PID ($kP, kI, kD$) + Gravity Feedforward ($kS, kG, kV, kA$).
- **Autonomous Holonomic Pathfinding**: Live Choreo/Pure Pursuit Drive ($kP, kI, kD$) and Heading Turn ($kP, kI, kD$) controllers.
- **Driver Response Shaping**: Live Slew Rate Limiters ($4.5\text{ m/s}^2$ translation, $7.0\text{ rad/s}^2$ rotation) and assist toggles (`Toggle Switch`).

---

## 🎯 Practice Drills & Real-Time AI Match Coach

The robot codebase integrates an automated driver training system (`MatchCoach.java`):

### Practice Drills
1. **Free Play Match (`FREE_PLAY`)**: Full match scrimmage against an autonomous cycling competitor.
2. **Rapid Cycling Sprint (`RAPID_CYCLING`)**: Defense disabled with automatic ball respawning for solo cycle time-trials.
3. **Trench Defense & Pirouette Drill (`TRENCH_DEFENSE`)**: Defense sparring partner patrolling trenches at 80% speed to practice trench funneling and contact-breaking pirouettes.
4. **Anti-Defense SOTF Drill (`ANTI_DEFENSE_SHOOTING`)**: 85% speed lead-pursuit defender to train moving shots under heavy pursuit.
5. **Triple Threat Scrum**: Manual dashboard config — set Opponent Count to 3 with all bots on `AUTONOMOUS_CYCLER`. Trains fast visual identification and contested ground pick-up reaction time when 3 opponents are actively harvesting midfield fuel clusters.
6. **2-on-1 Gauntlet Defense**: Manual dashboard config — set Opponent Count to 2 with Bot 0 as `DEFENSE_BULLY` and Bot 1 as `LEAD_PURSUIT_INTERCEPTOR`. Practice escape spins, legal pin evasion (<2.4 s max, warn 1.8 s), and finding narrow shooting windows while under coordinated double-team pressure.

### External AI Coach Tool (`tools/coaching/jev_coach.py`)
Run the standalone coaching tool during practice sessions:
```powershell
# Live terminal telemetry HUD
python tools/coaching/jev_coach.py --live

# Generate post-match markdown debrief report
python tools/coaching/jev_coach.py --report
```
Flags `--ip`/`--port` default to `127.0.0.1:5810` (`jev_coach.py:268-269`). When configured with a `TYPESAFE_API_KEY` (or `OPENROUTER_API_KEY` fallback), the tool queries the TypeSafe Jev API (`https://api.typesafe.ai/v1/systemone`).

