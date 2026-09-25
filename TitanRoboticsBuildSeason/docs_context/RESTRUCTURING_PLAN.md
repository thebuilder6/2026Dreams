# Architecture Restructuring Plan

## Goal
The primary goal of this restructuring is to **reduce complexity** and **group related systems together**, establishing strict boundaries between Hardware/IO, Physical Simulation, AI/Intelligence, Navigation/Pathing, and Driver Assistance.

## Exhaustive File Mapping (`src/main/java/frc/robot/`)

Below is the exact mapping of every existing file to its proposed new location.

### Core & Base
*Files remaining in `frc/robot/` or `frc/robot/Interfaces/`*
*   `Robot.java` -> *No Change*
*   `Main.java` -> *No Change*
*   `Teleop.java` -> *No Change*
*   `Interfaces/Subsystem.java` -> *No Change*
*   `Interfaces/Actions.java` -> *No Change*

### 1. `Data/` (Constants & Maps)
*Currently scattered, consolidating static state and numbers here.*
*   `Data/Constants.java` -> *No Change* (Consider splitting into `RobotConstants`, `FieldConstants`)
*   `Data/FieldMap.java` -> *No Change*
*   `Data/PortMap.java` -> *No Change*
*   `Data/TunableNumber.java` -> *No Change*
*   `Data/GlideConstants.java` -> `Navigation/GlideConstants.java` (It's math for routing, not raw data)
*   **Merge Candidate:** Combine `IntakeConstants` and `ShooterConstants` (currently in IO folders) into `Data/MechanismConstants.java`.

### 2. `Hardware/` (Renamed from `Devices/` & Subsystem IOs)
*Consolidating all raw hardware abstraction and devices.*
*   `Devices/Controller.java` -> `HMI/Controller.java` (Moved to Human-Machine Interface)
*   `Devices/NeoSparkMaxMotor.java` -> `Hardware/NeoSparkMaxMotor.java`
*   `Subsystems/drive/DriveIO*.java` -> `Hardware/Drive/`
*   `Subsystems/intake/IntakeIO*.java` -> `Hardware/Intake/`
*   `Subsystems/shooter/ShooterIO*.java` -> `Hardware/Shooter/`
*   `Subsystems/vision/VisionIO*.java` -> `Hardware/Vision/`
*   `ThirdParty/LimelightHelpers.java` -> `Hardware/Vision/LimelightHelpers.java`

### 3. `Subsystems/` (Pure Robot Logic)
*Stripping out UI and AI, leaving only state machines and hardware orchestration.*
*   `Subsystems/SubsystemManager.java` -> *No Change*
*   `Subsystems/SwerveBase.java` -> *No Change*
*   `Subsystems/Shooter.java` -> *No Change*
*   `Subsystems/Intake.java` -> *No Change*
*   `Subsystems/Vision.java` -> *No Change*
*   `Subsystems/LEDs.java` -> `HMI/LEDs.java` (LEDs are driver feedback, not a mechanical system)
*   `Subsystems/Dashboard.java` -> `Telemetry/Dashboard.java`
*   `Subsystems/MatchCoach.java` -> `Intelligence/MatchCoach.java` (It's AI)

### 4. `Simulation/` (Extracted from `Sim/`)
*Strictly physical phenomena and virtual hardware.*
*   `Sim/ArmSim.java` -> `Simulation/ArmSim.java`
*   `Sim/ShooterSim.java` -> `Simulation/ShooterSim.java`
*   `Sim/VisionSim.java` -> `Simulation/VisionSim.java`
*   `Sim/LimelightSim.java` -> `Simulation/LimelightSim.java`
*   `Sim/GameSim.java` -> `Simulation/GameSim.java`

### 5. `Intelligence/` (New! Extracted from `Sim/`)
*The "Brain". Game-agnostic decision-making.*
*   `Sim/JevDecisionEngine.java` -> `Intelligence/JevDecisionEngine.java`
*   `Sim/WorldState.java` -> `Intelligence/State/WorldState.java`
*   `Sim/WorldStateBuilder.java` -> `Intelligence/State/WorldStateBuilder.java`
*   `Sim/MatchScoreTracker.java` -> `Intelligence/State/MatchScoreTracker.java`
*   `Sim/StrategicObjective.java` -> `Intelligence/StrategicObjective.java`
*   `Sim/AIActionIntent.java` -> `Intelligence/AIActionIntent.java`
*   `Sim/Archetype.java` -> `Intelligence/Archetype.java`
*   `Sim/AIRobotSim.java` -> `Intelligence/Sparring/AIRobotSim.java`
*   `Sim/AIRobotInstance.java` -> `Intelligence/Sparring/AIRobotInstance.java`

### 6. `Navigation/` (Extracted from `Auto/`)
*The "Legs". How to move from A to B.*
*   `Auto/StaticPathfinder.java` -> `Navigation/StaticPathfinder.java`
*   `Auto/DynamicRouter.java` -> `Navigation/Routing/DynamicRouter.java`
*   `Auto/SmartTunnelRouter.java` -> `Navigation/Routing/SmartTunnelRouter.java`
*   `Auto/DynamicObstacle.java` -> `Navigation/Routing/DynamicObstacle.java`
*   `Auto/CollisionDetector.java` -> `Navigation/CollisionDetector.java`
*   `Auto/TrajectoryController.java` -> `Navigation/TrajectoryController.java`
*   `Utils/Vector2dSlewRateLimiter.java` -> `Navigation/Math/Vector2dSlewRateLimiter.java`
*   `Utils/AllianceFlipUtil.java` -> `Navigation/Math/AllianceFlipUtil.java`

### 7. `Autonomous/` (Simplified `Auto/`)
*Sequential match scripts.*
*   `Auto/AutoMissionChooser.java` -> `Autonomous/AutoMissionChooser.java`
*   `Auto/AutoMissionExecutor.java` -> `Autonomous/AutoMissionExecutor.java`
*   `Auto/AutoMissionEndedException.java` -> `Autonomous/AutoMissionEndedException.java`
*   `Auto/Missions/*` -> `Autonomous/Missions/*`
*   `Auto/Actions/*` -> `Autonomous/Actions/*`

### 8. `HMI/` (Human-Machine Interface / Driver Assist)
*Bridging the gap between code and the Drive Team.*
*   `Devices/Controller.java` -> `HMI/Controller.java`
*   `Utils/Alert.java` -> `HMI/Alerts/Alert.java`
*   `Utils/AlertManager.java` -> `HMI/Alerts/AlertManager.java`
*   `Auto/AutonomousTeleopAgent.java` -> `HMI/CoPilot.java` (Renamed for clarity)
*   `Auto/LegalPinningWatchdog.java` -> `HMI/Watchdogs/LegalPinningWatchdog.java`

### 9. `Test/` and `Telemetry/`
*   `Test/*` -> *No Change (Keep Diagnostics, SysID routines here)*
*   *New Folder* `Telemetry/`: Move `Subsystems/Dashboard.java` here. Future AdvantageScope logging wrappers go here.

---

## Splitting and Combining Considerations

*   **Split `JevDecisionEngine`:** As AI complexity grows, `JevDecisionEngine.java` will become a god-class. We should split it into `TacticalReflex.java` (System 1) and `ExecutiveStrategy.java` (System 2).
*   **Combine Constants:** Instead of having `IntakeConstants.java` in the intake IO folder and `ShooterConstants.java` in the shooter IO folder, they should be aggregated into `Data/MechanismConstants.java`. This creates a single source of truth for mechanical properties.
*   **Split Field Math:** `FieldMap.java` currently contains both standard layout constants and game-piece tracking logic. Split into `StaticFieldConstants.java` and a live `GamePieceTracker.java` (perhaps under `Intelligence/State/`).

---

## Where Future Features Will Go

*   **A new auto routine?** -> `Autonomous/Missions/`
*   **A new way for AI to evaluate targets?** -> `Intelligence/ExecutiveStrategy.java`
*   **A new physical sensor (e.g., Time of Flight)?** -> `Hardware/` to define the IO layer, then used inside the relevant `Subsystems/` file.
*   **A new dashboard widget for drivers?** -> `Telemetry/` or `HMI/Alerts/`.
*   **A new obstacle avoidance algorithm?** -> `Navigation/Routing/`.

---

## Modifying the Robot: A Quick Guide

### 1. How to Add a New Hardware Component (e.g., a Climber)
1.  **Hardware Abstraction:** Create `Hardware/Climber/ClimberIO.java`, `ClimberIOSparkMax.java`, and `ClimberIOSim.java`.
2.  **Logic:** Create `Subsystems/Climber.java` that takes the `ClimberIO` interface. Add states (e.g., `STOWED`, `DEPLOYING`, `CLIMBING`).
3.  **Constants:** Add motor IDs to `Data/PortMap.java` and gearing/PID data to `Data/MechanismConstants.java`.
4.  **Simulation:** Create `Simulation/ClimberSim.java` to model the physics (e.g., gravity pulling the robot down) and link it to `ClimberIOSim`.
5.  **Actions:** Create `Autonomous/Actions/ClimbAction.java` if it needs to happen in Auto.

### 2. How to Change Field Layout or Game Rules
1.  **Coordinates:** Update POIs (Points of Interest) in `Data/FieldMap.java`.
2.  **Physics Rules:** If game pieces behave differently (e.g., sliding vs rolling), update `Simulation/GameSim.java`.
3.  **Pathing Zones:** If new field elements block paths, update boundaries in `Navigation/Routing/DynamicObstacle.java` or update the Choreo trajectory files in `deploy/choreo/`.

### 3. How to Change AI Behavior
1.  **New Goal?** Add a new enum to `Intelligence/StrategicObjective.java` (e.g., `HOARD_PIECES`).
2.  **New Output?** Update `Intelligence/AIActionIntent.java` if the robot needs a new verb (e.g., `deployDefenseShield`).
3.  **New Logic?** Edit `Intelligence/JevDecisionEngine.java` to change how it scores `StrategicObjectives` based on the `WorldState`. For example, change the utility weighting to make the AI prefer shooting over intaking in the last 15 seconds.
