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

Elastic Dashboard is pre-configured with 5 tabs tailored for this robot:

1. Launch **Elastic Dashboard**.
2. Go to **Settings** (gear icon in upper right) and set:
   - **Server Address**: `127.0.0.1` (or `localhost`)
   - **Port**: `5810` (NT4 default)
3. Click the layout dropdown and select **Open Layout File**.
4. Open the pre-built layout: `TitanRoboticsBuildSeason/elastic-layout.json`.
5. You will now have 5 synchronized tabs:
   - **Driver Dashboard**: 3D field overview, match clock, Hub countdown timer, auto-aim lock indicator, and feature switches.
   - **Pre-Flight Diagnostics**: Automated 15-second scorecard and manual motor test bench.
   - **SysID & Characterization**: Routine selectors and execution controls.
   - **Tuning & PID**: Live-tunable gains for flywheels ($kP, kI, kD, kS, kV, kA$), intake arm, auton pathfinding, and driver slew rates ($4.5\text{ m/s}^2$ translation, $7.0\text{ rad/s}^2$ rotation).
   - **Simulation & Match Info**: Live score counter, held ball count, respawn trigger, and opponent AI defense toggle.

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
- Drive to any shooting position within $1.8\text{--}4.5\text{ meters}$ of the Alliance Hub.
- **Hold Right Trigger (>30%)**:
  - The robot locks heading onto the Hub center.
  - Dual flywheels spool up to the interpolated target RPM based on distance.
  - The dashboard displays **Ready to Fire** (green indicator) once aligned within $3^\circ$ and flywheels reach target velocity.
  - The kicker feed automatically fires the balls.
  - Watch the balls arc across the field into the Hub in AdvantageScope. The **Simulation Score** counter will increment!

### 5. Glide Points & Tactical Waypoints
- **Hold Right Bumper**: The robot autonomously plans a path and navigates to the nearest tactical waypoint (Alliance Feeder, Hub perimeter, Trench auto-tunnel, or Midfield crossing).
- Deflecting any manual joystick (>15%) instantly cancels Glide mode and restores full driver control.

### 6. Sparring Against Opponent AI
- In Elastic Dashboard (`Simulation & Match Info` tab), check **`Opponent AI Defense`**.
- An opponent robot will spawn on the field with active bumper collision physics.
- The AI dynamically switches behaviors using the Jev decision engine:
  - Shadows the player along the midfield line.
  - Blocks shooting lanes when the player approaches the Hub.
  - Contests neutral depots and harvests balls.
- To spar manually with a second player, set the AI mode dropdown to **`Manual 2-Player (Port 2)`** and connect a gamepad to Joystick Port 2.

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
