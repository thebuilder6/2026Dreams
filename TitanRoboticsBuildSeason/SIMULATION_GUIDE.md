# 🎮 2026 Desktop Simulation Setup & User Guide

Welcome to the **Titan Robotics 2026 Simulation Environment**. Our simulation stack provides a complete, high-fidelity virtual proving ground for testing swerve drive kinematics, ballistics shooter trajectories, intake mechanisms, autonomous pathfinding, and AI opponent sparring without requiring physical robot hardware.

---

## 🏗️ Simulation Architecture & Capabilities

The simulation environment integrates several real-time physics engines and telemetry streams:

| System | Simulation Engine | Capabilities |
| :--- | :--- | :--- |
| **Swerve Drivebase** | **IronMaple 2D Physics** | Rigid-body swerve kinematics with wheel slip, realistic carpet friction, inertia, and bumper-to-bumper collision dynamics. |
| **Field & Game Pieces** | **`GameSim` + IronMaple** | 54 dynamic Fuel game pieces (12 Blue, 12 Red, 30 Neutral Midfield) with ground collection, hopper capacity limits, scoring detection, and respawning. |
| **Dual Flywheel Shooter** | **`ShooterSim`** | Physics-based projectile trajectory math incorporating dual flywheel slip efficiency ($\eta = 0.42$), launch angle, and 3D parabolic flight into the Hub goal. |
| **Ground Intake Arm** | **`ArmSim` + DC Motor Sim** | Trapezoid-profiled arm pivot with gravity feedforward, ground plane collision damping, and roller ingestion volume. |
| **AI Opponent Robot** | **`AIRobotSim` + Jev AI** | Autonomous sparring partner capable of tactical shooting lane denial, lead pursuit interception, midfield shadowing, aggressive bumper pinning, or 2-player manual control. |
| **Vision & AprilTags** | **PhotonVision Sim + Rubik Pi Sim** | Desktop simulation of multi-camera AprilTag pose estimation and YOLO neural network ball detection ("Ball Hunt"). |
| **Electrical System** | **WPILib `BatterySim` & `RoboRioSim`** | Dynamic battery voltage sag calculation based on instantaneous current draw across all 12 simulated motors. |

---

## 📋 Prerequisites

Before launching the simulation, ensure you have the following installed:

1. **WPILib 2026 Suite**:
   - Contains the required Java 17 JDK (located at `C:\Users\Public\wpilib\2026\jdk` on Windows) and the WPILib Simulation GUI (`SimGUI`).
2. **Game Controller** *(Recommended)*:
   - Xbox 360 / Xbox One / Xbox Series X controller (or Logitech F310 in XInput mode).
   - Alternatively, you can use keyboard-mapped joysticks within the WPILib SimGUI.
3. **Visualization Tools**:
   - **[Elastic Dashboard](https://github.com/Gold872/elastic-dashboard)**: Driver HUD, diagnostic scorecards, SysID bench, and Tuning & PID interface.
   - **[AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)**: 3D field rendering, robot poses, arm articulation, and moving game pieces.

---

## 🚀 Launching the Simulation

### Option A: From VS Code (Recommended)

1. Open the project root folder (`TitanRoboticsBuildSeason`) in VS Code.
2. Press `Ctrl + Shift + P` (or `Cmd + Shift + P` on macOS) to open the Command Palette.
3. Type and select **`WPILib: Simulate Robot Code on Desktop`** (or press `F5`).
4. If prompted to select simulation extensions, check **`Sim GUI`** and click **OK**.

### Option B: From Terminal / PowerShell

Run the following commands in PowerShell from the repository root:

```powershell
# Set Java environment to the WPILib 2026 JDK
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:Path = "$($env:JAVA_HOME)\bin;$($env:Path)"

# Launch simulation
.\gradlew simulateJava
```

The **WPILib Simulation GUI (SimGUI)** window will appear automatically.

---

## 🕹️ Configuring Controllers in SimGUI

To control the robot with a physical gamepad:

1. In the WPILib SimGUI window, locate the **`System Joysticks`** panel on the left.
2. Find your connected controller (e.g., `Xbox Controller (XInput...)`).
3. Drag and drop it into **`Joystick 0`** under the **`Joysticks`** panel for the **Driver Controller**.
4. *(Optional)* Drag a second controller into **`Joystick 1`** for the **Operator Controller**.
5. *(Optional)* Drag a third controller into **`Joystick 2`** to manually pilot the **Opponent AI Robot** in 2-Player sparring mode.

> [!TIP]
> **No Gamepad?** In SimGUI, you can assign keyboard keys to axes and buttons by selecting **Joysticks -> Keyboard 0** and mapping keys (e.g. WASD for Left Stick, Arrow keys for Right Stick).

---

## 📊 Connecting Elastic Dashboard

Elastic Dashboard is pre-configured with 6 tabs tailored for this robot:

1. Launch **Elastic Dashboard**.
2. Go to **Settings** (gear icon in upper right) and set:
   - **Server Address**: `127.0.0.1` (or `localhost`)
   - **Port**: `5810` (NT4 default)
3. Click the layout dropdown and select **Open Layout File**.
4. Open the pre-built layout: `TitanRoboticsBuildSeason/elastic-layout.json`.
5. You will now have 6 synchronized tabs:
   - **Driver Dashboard**: 3D field overview, match clock, Hub countdown timer, auto-aim lock indicator, Jev strategy, and feature switches.
   - **AI Coach & Practice**: Real-time driver grading ($A+$ to $D$), cycle timing gauges, shooting accuracy bar, practice drill selector, 1-click arena reset, and contextual AI coaching directives.
   - **Pre-Flight Diagnostics**: Automated 15-second scorecard and manual motor test bench.
   - **SysID & Characterization**: Routine selectors and execution controls.
   - **Simulation & Match Info**: Live score counter, held ball count, respawn trigger, and opponent AI defense toggle.
   - **Tuning & PID**: Live-tunable gains for flywheels ($kP, kI, kD, kS, kV, kA$), intake arm, auton pathfinding, and driver slew rates ($4.5\text{ m/s}^2$ translation, $7.0\text{ rad/s}^2$ rotation).

---

## 🔭 Connecting AdvantageScope (3D Visualizer)

AdvantageScope gives you a live 3D rendering of the arena, robot, articulated mechanisms, and game pieces:

1. Launch **AdvantageScope**.
2. Select **File -> Connect to NetworkTables**.
3. Set the address to `127.0.0.1` and connect.
4. **Configure 3D Field**:
   - Open a **3D Field** tab.
   - Select the field model: **2026 Rebuilt** (or 2024 Crescendo as fallback).
   - Under **Robot Poses**, add `/SmartDashboard/Field` or `/RealOutputs/Pose`.
   - Under **Game Pieces**, add `/SmartDashboard/FieldSim/GamePieces` (`Pose3d[]`) to see all 54 Fuel balls.
5. **Configure Mechanism 3D**:
   - Add `/Subsystems/Intake/ArmPose3d` to observe the intake arm rotating between standby ($347^\circ$) and ground ($250^\circ$).
   - Add `/Subsystems/Shooter/ShooterPose3d` to visualize the shooter flywheel angle and position.

---

## 🎮 How to Test & Operate in Simulation

### 1. Enabling the Robot
In the WPILib SimGUI:
- Click **`Teleoperated`** and then **`Enabled`** in the DriverStation control panel to start manual driving.
- Click **`Autonomous`** and then **`Enabled`** to test the auto routine selected in the Elastic Dashboard dropdown.

### 2. Driving & Handling
- **Left Stick (X/Y)**: Field-oriented translation with smooth acceleration ($4.5\text{ m/s}^2$ slew rate).
- **Right Stick (X)**: Holonomic heading rotation ($7.0\text{ rad/s}^2$ slew rate).
- **Left Stick Click**: Toggles **Slow Mode** (35% speed) for precision positioning.
- **D-Pad (POV)**: Cardinal snap-to-heading:
  - **Up**: Face Away ($0^\circ$)
  - **Right**: Face Right ($-90^\circ$)
  - **Down**: Face Backward ($180^\circ$)
  - **Left**: Face Left ($+90^\circ$)
- **A Button**: Zero Gyro field heading relative to current alliance.

### 3. Intaking Fuel Balls
- Drive towards any Fuel ball on the carpet.
- **Hold Left Trigger (>30%)**: The intake arm automatically deploys down to ground ($250^\circ$) and spins the rollers.
- When the bumper intersects a ball, it is ingested into the hopper. The `Held Balls` counter on the dashboard will increment.
- **Release Left Trigger**: The arm automatically retracts to the standby upright position ($347^\circ$).

### 4. Auto-Aiming & Scoring in the Hub
- Drive to any shooting position within $1.8\text{--}4.5\text{ meters}$ of the Alliance Hub **strictly inside your Alliance Zone** ($X \le 4.60\text{m}$ for Blue, $X \ge 11.94\text{m}$ for Red). Shots from Midfield are automatically inhibited.
- **Hold Right Trigger (>30%)**:
  - The robot locks heading onto the Hub center.
  - Dual flywheels spool up to the interpolated target RPM based on distance.
  - The dashboard displays **Ready to Fire** (green indicator) once aligned within $3^\circ$, flywheels reach target velocity, and the robot is verified inside the Alliance Zone.
  - The kicker feed automatically fires the balls.
  - Watch the balls arc across the field into the Hub in AdvantageScope. The **Simulation Score** counter will increment!

### 5. Glide Points & Tactical Waypoints
- **Hold Right Bumper**: The robot autonomously plans a path and navigates to the nearest tactical waypoint (Alliance Feeder, Hub perimeter, Trench auto-tunnel, or Midfield crossing).
- Deflecting any manual joystick (>15%) instantly cancels Glide mode and restores full driver control.

### 6. Sparring Against Opponent AI (Autonomous Offense & Defense)
- In Elastic Dashboard (`Simulation & Match Info` tab), check **`Opponent AI Defense`** (or toggle `Features/Opponent Robot`).
- An opponent robot will spawn on the field with active bumper collision physics and realistic game piece intake/shooting capabilities.
- Select the opponent's behavior via **`Simulation/AIModeChooser`** (defaults to **`Autonomous Fuel Cycler`**):
  - **`Autonomous Fuel Cycler`**: Full offense mode with comprehensive **WHERE & WHEN** shooting intelligence:
    - **WHERE It Shoots**:
      - *Alliance Zone Enforcement*: Strictly fires only when inside its designated Alliance Zone ($X \le 4.60\text{m}$ for Blue, $X \ge 11.94\text{m}$ for Red). Shots from Midfield are strictly prohibited.
      - *Distance Window*: Strictly validates firing distance ($1.60\text{m} \le d \le 4.20\text{m}$) within the Alliance Zone to clear Hub base collision while maintaining high parabolic ballistic accuracy.
      - *Low-Clearance Trench Exclusion*: Evaluates field geofencing (`Intake.isPoseInTrenchLowClearanceZone`) to never fire under low-overhead steel trusses ($Z < 1.2\text{m}$), preventing ceiling deflections.
      - *Dynamic Standoff & Lateral Evasion*: Projects to an optimal $2.40\text{m}$ standoff arc within the Alliance Zone, clamped to the open trench-free corridor ($Y \in [2.2, 5.8]$). If a defender guards the spot or blocks the direct shooting lane, the bot executes a lateral strafe to re-open line of sight.
      - *Opportunistic Transitions*: If the bot gathers fuel and enters an unblocked valid shooting window within its Alliance Zone while the Hub is active, it fires immediately without driving to a rigid static waypoint.
    - **WHEN It Shoots**:
      - *Hub Active State Discipline*: Verifies ground-truth Hub scoring state (`Arena2026Rebuilt.isActive` and match shift timing). If the Hub is inactive, the bot hoards up to 5 Fuel pieces or stages at the standoff line instead of wasting game pieces.
      - *Fuel Inventory & Cooldown*: Requires held fuel ($> 0$) and enforces a $0.30\text{s}$ firing cadence between shots.
      - *Aim Alignment*: Verifies heading alignment within $< 8^\circ$ of the target Hub funnel.
      - *Shoot-On-The-Fly (SOTF) Ballistics*: Automatically applies virtual target motion compensation ($\vec{P}_{\text{virtual}} = \vec{P}_{\text{goal}} - \vec{v}_{\text{chassis}} \cdot t_{\text{tof}}$) so shots fired on the move arc directly into the center funnel ($Z = 1.48\text{m}$) without tangential drift.
  - **`Tactical Defense (Jev AI)`**: High-frequency defensive decision engine that shadows the player along midfield, blocks player shooting lanes, contests neutral depots, and opportunistically intakes loose balls and fires them into its Hub when in range.
  - **`Lead Pursuit Intercept`**: Intercepts player travel paths using quadratic lead pursuit.
  - **`Aggressive Pinning Bully`**: Charges player bumpers to practice spin-outs and test the 2-second legal pinning watchdog.
  - **`Manual 2-Player (Port 2)`**: Connect a second controller to Joystick Port 2 to drive the opponent robot manually against your teammate.
- **Topological Roadmap & Pure Pursuit Navigation Architecture**:
  - *AABB Obstacle Modeling & Topological Visibility Graph* ([`StaticPathfinder.java`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Auto/StaticPathfinder.java)):
    - Full 2026 field obstacle geometry represented via axis-aligned bounding boxes (AABBs) with $O(1)$ Liang-Barsky line-box intersection testing.
    - Preserves dedicated 53-inch Trench corridors ($Y = 7.42\text{m}$ Top, $Y = 0.65\text{m}$ Bottom) with $>0.42\text{m}$ bumper clearance from divider walls.
    - Deterministic A* graph search over 34 strategic nodes with string-pulling line-of-sight shortcutting.
  - *Resilient Pure Pursuit Lookahead Tracker* ([`AIRobotSim.java`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Sim/AIRobotSim.java)):
    - Computes lookahead point $P_{\text{look}}$ along polyline path with adaptive lookahead radius $R_{\text{look}} \in [0.45\text{m}, 0.85\text{m}]$.
    - Monotonic path progression advances waypoints only when the robot crosses the segment tangent normal plane or reaches within $0.45\text{m}$.
    - Goal deadband and replan hysteresis ($0.85\text{m}$ / $0.5\text{s}$) prevent waypoint index resets and rubber-banding during continuous target tracking.
    - Trapezoidal deceleration profiling ($V_{\text{target}} = \min(V_{\text{max}}, \sqrt{2 \cdot A_{\text{max}} \cdot d_{\text{remaining}}})$) ensures smooth deceleration into targets with zero overshoot.
  - *Trench Passage Lockout*:
    - Automatically activates inside Trench corridors ($X \in [3.20, 6.10]$ or $[10.40, 13.30]$ with $Y \ge 6.50$ or $Y \le 1.55$).
    - Locks heading parallel to the corridor ($0^\circ$ or $180^\circ$) to eliminate corner catch.
    - Applies active cross-track centering ($V_y = -3.0 \cdot (Y - Y_{\text{centerline}})$) to keep the bot centered along the corridor centerline.
    - Suppresses Artificial Potential Field (APF) player repulsion while inside trenches to prevent wall pinning.
- **AdvantageScope Visualizations**:
  - Add `/FieldSimulation/OpponentSuccessfulShotsTrajectory` and `/FieldSimulation/OpponentMissedShotsTrajectory` to the 3D Field tab to see the AI's 3D parabolic projectile arcs in real-time.
  - Monitor `/Simulation/OpponentScoreCount`, `/Simulation/OpponentFuelCount`, and `/Simulation/OpponentCyclerPhase` for live AI offensive stats.

### 7. AI Coach & Practice Proving Ground
Switch to the **`AI Coach & Practice`** tab in Elastic Dashboard for focused driver training:

- **Practice Drill Modes** (Select via `Practice Drill Mode` chooser):
  - **`Free Play Match`**: Standard match simulation against an autonomous cycling opponent (`AIRobotSim` at 75% speed).
  - **`Rapid Cycling Sprint`**: Disables opponent defense and enables automatic ball respawns for solo time-trial throughput drills.
  - **`Trench Defense & Pirouette Drill`**: Spawns an 80% speed sparring partner patrolling the trenches to practice automated Smart Tunnel diversions, bumper pirouettes, and 2.0-second legal pinning evasion.
  - **`Anti-Defense SOTF Drill`**: Spawns an 85% speed lead-pursuit interceptor to practice shoot-on-the-fly (SOTF) accuracy while under heavy pursuit.
- **1-Click Proving Ground Reset**:
  - Click **`Reset Practice Arena`** (`Coaching/ResetPractice`).
  - Instantly resets session metrics, teleports the robot back to the alliance starting line, respawns all 54 Fuel balls across the arena, and configures the sparring AI for the selected drill.
- **Dynamic Driver Grade**:
  - Real-time rating from **`A+`** to **`D`** based on cycle speed, shooting accuracy, and Hub active timing discipline.
- **TypeSafe Jev AI Coach Terminal & Markdown Reports**:
  - Launch the terminal coaching tool in another PowerShell window while running simulation:
    ```powershell
    # Live ANSI Telemetry & Tactical Directive HUD
    python tools/coaching/jev_coach.py --live

    # Generate Post-Match Debrief Report (Saved to reports/)
    python tools/coaching/jev_coach.py --report
    ```
  - If a `TYPESAFE_API_KEY` or `OPENROUTER_API_KEY` environment variable is defined, the tool queries the TypeSafe Jev API (`https://docs.typesafe.ai`) for automated System One AI tactical critiques.

---

## 🛠️ Troubleshooting & FAQs

### Q: `bind() to port 1181 failed: Only one usage of each socket address is normally permitted`
- **Cause**: WPILib CameraServer attempts to bind to default RTSP/HTTP ports that may already be in use by another local process.
- **Solution**: This is a non-fatal warning during simulation startup and can be safely ignored. PhotonVision and Limelight simulations operate independently over NetworkTables.

### Q: The robot does not respond to controller inputs
- Check the **`Joysticks`** panel in the WPILib SimGUI. Ensure your controller is placed in **`Joystick 0`**.
- Verify that Driver Station is set to **`Teleoperated`** and **`Enabled`**.

### Q: Build failure: `Unsupported class file major version` or Java errors
- Ensure you are running Gradle with the WPILib 2026 JDK:
  ```powershell
  $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
  $env:Path = "$($env:JAVA_HOME)\bin;$($env:Path)"
  .\gradlew simulateJava
  ```

### Q: How do I reset the match or respawn balls?
- On the **Simulation & Match Info** tab of the Elastic Dashboard, click the **Respawn Balls** or **Reset Sim** buttons.
- Alternatively, disable and re-enable the robot in SimGUI.

---

## 📚 Related Documentation
- 📖 [System Architecture Specification](ARCHITECTURE.md): Deep-dive into subsystem layers and IO abstraction.
- 🎮 [Operator's Guide](OPERATORS_GUIDE.md): Complete driver and operator control mappings.
- 🧪 [Testing & Diagnostics Guide](src/main/java/frc/robot/Test/README.md): Pre-flight routines and SysId characterization.
