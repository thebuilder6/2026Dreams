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

Elastic Dashboard is pre-configured with 6 specialized tabs adhering strictly to the official [Elastic Widget Reference](https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference).

### Instant Setup via Remote Layout Downloading (Recommended)
Our robot code serves the official layout directly over HTTP port 5800 (`edu.wpi.first.net.WebServer`):
1. Launch **Elastic Dashboard**.
2. Connect to the robot / simulation (`127.0.0.1` on port `5810`).
3. Press **`Ctrl + D`** (or go to **File -> Load Layout From Robot**).
4. Select `elastic-layout.json` and choose **Full Reload** (or **Overwrite**).
5. Elastic will pull the exact, validated layout directly from the robot deploy directory!

### Manual Setup (Alternative)
1. In Elastic Dashboard, click the layout dropdown (or File menu) and select **Open Layout File**.
2. Open `TitanRoboticsBuildSeason/elastic-layout.json` (or `src/main/deploy/elastic-layout.json`).

### Widget Features Across Tabs
- **Tab 1: Driver Dashboard**: Dedicated `Match Time` countdown clock (auto-transitions Blue -> Green -> Yellow at 30s -> Red at 15s), 3D Field2d view, live Hub active indicator, `Graph` widget displaying live Flywheel RPM response, held fuel `Number Bar`, and clickable `Toggle Switch` controls for Snap Turn, Auto Aim, Ball Hunt, Glide Points, and Slow Mode.
- **Tab 2: AI Coach & Practice**: Real-time driver grading ($A+$ to $D$), cycle timing bars, shooting accuracy bar, drill mode chooser, `Toggle Button` for 1-click arena reset, `Toggle Switch` for haptic collision rumble, and Jev AI coaching directives.
- **Tab 3: Pre-Flight Diagnostics**: Automated 15-second scorecard with progress bar and individual `Toggle Button` widgets to pulse each swerve steer/drive motor, intake arm, intake rollers, and flywheels.
- **Tab 4: SysID & Characterization**: `Toggle Button` for Quasistatic / Dynamic Forward / Reverse and ABORT / E-STOP, with real-time `Graph` widgets for live applied voltage and velocity response waves.
- **Tab 5: Simulation & Match Info**: Dropdown menus for Opponent Count (1, 2, or 3 bots) and per-bot Archetypes (Bot 0 Lead, Bot 1 Bully, Bot 2 Adaptive), interactive `Number Slider` for opponent speed (20-100%), interactive `Toggle Button` controls for Sim Reset and Respawn Balls, multi-bot state/score telemetry, and `Toggle Switch` for Opponent AI.
- **Tab 6: Tuning & PID**: Flywheel dual-RPM bars, pivot arm setpoint/goal bars, interactive `Toggle Switch` settings, and text displays with `show_submit_button: true` to edit PID constants live.

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

### 6. Sparring Against Multiple Opponent AI Robots (1 to 3 Autonomous Agents)

The simulation engine supports scaling from a single sparring opponent up to **3 simultaneous AI robots** running concurrently on the field, powered by the unified System 1 (Tactical Reflex) + System 2 (Executive Strategy) Jev cognitive architecture:

- **Activating Multi-Bot Simulation**:
  - In Elastic Dashboard (`Simulation & Match Info` tab), toggle **`Opponent AI Active`** (`Features/Opponent Robot`).
  - Set the number of active opponent bots via the **`Opponent Count (1-3)`** bar (`Simulation/OpponentCount`). Select `1`, `2`, or `3`.
  - Adjust sparring speed with **`Opponent Speed %`** (`Simulation/OpponentSpeedPercent`, 20% to 100%, defaults to 75%).
  - Bots spawn at staggered, non-overlapping starting coordinates along their alliance wall:
    - **Bot 0**: Centerline spawn ($X=2.00\text{m}, Y=4.035\text{m}$ for Blue)
    - **Bot 1**: Upper corridor spawn ($X=2.00\text{m}, Y=5.80\text{m}$ for Blue)
    - **Bot 2**: Lower corridor spawn ($X=2.00\text{m}, Y=2.25\text{m}$ for Blue)

- **Selectable AI Behavioral Archetypes**:
  Each bot can be independently configured with distinct behavioral strategies via SmartDashboard or the Elastic Dashboard:
  - **Bot 0** (`Simulation/AIModeChooser` / `Simulation/Bot0/Archetype`): Defaults to `Autonomous Fuel Cycler`.
  - **Bot 1** (`Simulation/Bot1/Archetype`): Defaults to `DEFENSE_BULLY`.
  - **Bot 2** (`Simulation/Bot2/Archetype`): Defaults to `ADAPTIVE_COMPETITOR`.

  | Archetype | Macro Strategy | Tactical Behaviors |
  | :--- | :--- | :--- |
  | **`AUTONOMOUS_CYCLER`** | High-Throughput Fuel Scoring | Evaluates Gaussian cluster density scent to target rich fuel patches. Adheres to Alliance Zone firing geofencing, standoff arcs ($2.40\text{m}$), and shoot-on-the-fly ballistics. |
  | **`DEFENSE_BULLY`** | Aggressive Physical Harassment | Pursues player bumpers, pins against walls up to the 2-second legal limit, and disrupts player intake lanes. |
  | **`ADAPTIVE_COMPETITOR`** | Hybrid Two-Way Play | Scavenges loose balls when the Hub is active; transitions to lane denial and player harassment when its Hub is inactive. |
  | **`TACTICAL_DEFENDER`** | Positional Lane & Depot Denial | Shadows player along the midfield boundary ($X = 8.27\text{m}$), blocks direct shooting corridors to the Hub, and contests neutral depots. |
  | **`LEAD_PURSUIT_INTERCEPTOR`** | Predictive Path Interception | Projects the player's instantaneous velocity vector and executes quadratic lead intercept to cut off travel routes. |
  | **`MANUAL_2_PLAYER`** | Human Sparring Partner | Map Joystick Port 2 to drive Bot 0 directly against the primary driver using standard gamepad controls. |

- **Multi-Robot Collision Avoidance & Flocking Separation**:
  - **Soft Peer Separation**: All AI instances evaluate peer robot distances in real time. If another robot approaches within $1.10\text{m}$ (bumper-to-bumper proximity), a smooth inverse-distance repulsive force is applied, preventing multi-bot scrums or mechanical lockups.
  - **Obstacle Registration**: Each active bot registers its pose and velocity in [`DynamicRouter`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Auto/DynamicRouter.java), enabling player trajectory pathfinding to cleanly circumnavigate moving opponents.

- **Elastic Dashboard Multi-Bot Controls (`Simulation & Match Info` Tab)**:
  - **Arena Field View**: Embedded 2D field widget displaying the player robot alongside `OpponentBot0`, `OpponentBot1`, and `OpponentBot2` with live heading orientations and lookahead target markers.
  - **Per-Bot Status Cards**:
    - **Mode & Objective**: Live displays for Bot 0, Bot 1, and Bot 2 active states (e.g. `CYCLE_SCORE_HUB`, `DENY_SHOOTING_LANE`, `STAGE_STANDOFF`).
    - **Held Fuel & Scores**: Dedicated counters tracking individual fuel counts and points scored per bot.
  - **Aggregate Telemetry**: Live indicators for `Total Opponent Score`, `Total Opponent Fuel`, and `Active Opponents Count`.

- **AdvantageScope 3D Multi-Robot Scrimmage Setup**:
  - Load the pre-configured layout: Open AdvantageScope -> **File -> Open Layout** -> select [`advantagescope-layout.json`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/advantagescope-layout.json).
  - Pre-configured views include:
    - **3D Arena Scrimmage**: Complete 3D field rendering with player (Blue) and up to 3 opponents (Orange, Coral, Crimson) driving with 3D projectile arcs and dynamic fuel balls.
    - **2D Tactical Field Map**: Simultaneous tracking of all robot poses, navigation waypoints, glide points, and pathfinder detours.
    - **Multi-Bot Scrimmage Scoring**: Real-time line graphs comparing player scoring throughput against individual and aggregate AI bot scores.
    - **Fuel Inventory & Drive Dynamics**: Multi-bot hopper tracking and flywheel RPM response.

- **NetworkTables Telemetry Reference**:
  - *Bot 0*: `/AI_Telemetry/Bot0/ActualPose`, `/Simulation/Bot0/StateDetail`, `/Simulation/Bot0/Score`, `/Simulation/Bot0/Fuel`, `/Simulation/Bot0/Archetype`
  - *Bot 1*: `/AI_Telemetry/Bot1/ActualPose`, `/Simulation/Bot1/StateDetail`, `/Simulation/Bot1/Score`, `/Simulation/Bot1/Fuel`, `/Simulation/Bot1/Archetype`
  - *Bot 2*: `/AI_Telemetry/Bot2/ActualPose`, `/Simulation/Bot2/StateDetail`, `/Simulation/Bot2/Score`, `/Simulation/Bot2/Fuel`, `/Simulation/Bot2/Archetype`
  - *Aggregates*: `/Simulation/TotalOpponentScore`, `/Simulation/TotalOpponentFuel`, `/Simulation/MultiBotActiveCount`
  - *Field2d Objects*: `/SmartDashboard/Field/OpponentBot0`, `/SmartDashboard/Field/OpponentBot1`, `/SmartDashboard/Field/OpponentBot2`

### 7. Ally Bots & Full 3v3 FRC Match Simulation (Player + 2 Allies vs 3 Opponents)

In addition to opposing sparring robots, the simulation engine allows you to spawn **1 or 2 autonomous Ally Bots** on your own alliance team. This enables complete **3v3 FRC match simulation** with full alliance coordination:

- **Activating Ally Bots**:
  - In Elastic Dashboard (`Simulation & Match Info` tab), toggle **`Ally Bots Active`** (`Features/Ally Bots`) or select the number of allies via **`Ally Count Chooser`** (`Simulation/AllyCountChooser`).
  - Available configurations:
    - **`0 Ally Bots (Solo)`**: Standard player solo practice or 1vX sparring.
    - **`1 Ally Bot (2v3 / 2v2)`**: Spawns Ally 1 alongside the player.
    - **`2 Ally Bots (Full 3v3)`**: Spawns both Ally 1 and Ally 2, forming a full 3-robot alliance!
  - Allies line up along your alliance driver wall alongside the player robot:
    - **Player**: Center start position ($X \approx 2.00\text{m}, Y \approx 4.035\text{m}$ for Blue)
    - **Ally 1**: Left flank start position ($X = 2.00\text{m}, Y = 5.80\text{m}$ for Blue, facing $0^\circ$)
    - **Ally 2**: Right flank start position ($X = 2.00\text{m}, Y = 2.25\text{m}$ for Blue, facing $0^\circ$)
    *(Coordinates automatically mirror to $X = 14.54\text{m}$, facing $180^\circ$ when on Red Alliance).*

- **Ally Bot Archetypes & Behavior**:
  - Each ally can be assigned an independent behavioral archetype via SmartDashboard:
    - **Ally 1** (`Simulation/Ally1/ArchetypeChooser`): Defaults to `Autonomous Fuel Cycler`. Focuses on collecting midfield fuel and rapid cycling into your Alliance Hub.
    - **Ally 2** (`Simulation/Ally2/ArchetypeChooser`): Defaults to `Adaptive Match Competitor`. Cycles fuel when your Hub is active, and switches to midfield containment or depot defense when the Hub is inactive.
  - **Alliance Awareness**: Unlike opponents, Ally Bots target your alliance's Hub, harvest balls from your alliance depots, never pin the player, and park at your alliance's climbing tower during the endgame.
  - **Multi-Robot Soft Separation**: All 6 robots active on the field (Player, 2 Allies, 3 Opponents) constantly compute mutual bumper distances. When any robot approaches within $1.10\text{m}$, smooth repulsion velocities prevent mechanical jams and scrums.

- **Field2d & Telemetry Representation**:
  - *Field2d Objects*: `AllyBot1`, `AllyTarget1`, `AllyBot2`, `AllyTarget2` displayed in real-time in Elastic Dashboard and AdvantageScope.
  - *Telemetry Channels*:
    - `AI_Telemetry/Ally1/ActualPose`, `Simulation/Ally1/Fuel`, `Simulation/Ally1/Score`, `Simulation/Ally1/StateDetail`
    - `AI_Telemetry/Ally2/ActualPose`, `Simulation/Ally2/Fuel`, `Simulation/Ally2/Score`, `Simulation/Ally2/StateDetail`
    - `Simulation/TotalAllyScore`, `Simulation/TotalAllyFuel`, `Simulation/AllyActiveCount`

---

### 8. Unified 2026 Match Scoring & Scoreboard System (`MatchScoreTracker`)

The simulation runs an automated, authoritative FRC match scoring engine via [`MatchScoreTracker`](file:///c:/Users/jumpi/Documents/Github/2026Dreams/TitanRoboticsBuildSeason/src/main/java/frc/robot/Sim/MatchScoreTracker.java), providing live scoreboards and Ranking Point calculations for both alliances:

- **Scoring Rules**:
  - **Fuel Ball in Active Hub**: $1\text{ point}$ per ball scored.
  - **Wasted Shots**: Balls launched into an inactive Hub during opposing shifts score $0\text{ points}$ and are logged as wasted fuel.
  - **Endgame Tower Climb**: $10\text{ points}$ per robot positioned within $1.20\text{m}$ of the alliance climbing pole during the final 20 seconds of the match ($t \le 20.0\text{s}$).

- **FRC Ranking Points (RP)**:
  - **Match Outcome**: $2\text{ RP}$ for a win, $1\text{ RP}$ for a tie.
  - **Energized RP (Fuel)**: $+1\text{ RP}$ awarded to any alliance scoring $\ge 40$ active fuel balls.
  - **Supercharged RP (Climb)**: $+1\text{ RP}$ awarded to any alliance with $\ge 2$ robots successfully climbed.

- **Full Alliance Score Attribution**:
  - Fuel scored and tower climbs achieved by **Ally 1** and **Ally 2** automatically credit your alliance's score and RP totals!
  - Real-time scoring streams published to Elastic Dashboard and AdvantageKit:
    - Main Scoreboard: `Scoreboard/Match/RedScore`, `Scoreboard/Match/BlueScore`, `Scoreboard/Match/LeadMargin`, `Scoreboard/Match/Leader`
    - Player Team Summary: `Scoreboard/Player/ShotsAttempted`, `Scoreboard/Player/ShotsScored`, `Scoreboard/Player/AccuracyPercent`, `Scoreboard/Player/Climbed`
    - Ally Breakdown: `Scoreboard/Allies/Ally1_FuelScored`, `Scoreboard/Allies/Ally2_FuelScored`, `Scoreboard/Allies/TotalFuelScored`, `Scoreboard/Allies/Ally1_Climbed`
    - Opponent Breakdown: `Scoreboard/Opponents/Bot0_FuelScored`, `Scoreboard/Opponents/Bot1_FuelScored`, `Scoreboard/Opponents/Bot2_FuelScored`

---

### 9. AI Coach & Practice Proving Ground
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
