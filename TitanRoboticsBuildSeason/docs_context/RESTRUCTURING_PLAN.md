# Architecture Restructuring Plan

## Goal
The primary goal of this restructuring is to **reduce complexity** and **group related systems together**. Specifically, we need to clarify the boundaries between Simulation, AI/Planning, Pathing, and Driver Assistance.

## Current Structure Analysis
Currently, the codebase has a few overloaded directories:
*   **`Sim/`**: Contains both hardware simulation (`ShooterSim`, `ArmSim`, `VisionSim`) **and** the entire AI cognitive architecture (`JevDecisionEngine`, `WorldState`, `AIRobotInstance`, `StrategicObjective`).
*   **`Auto/`**: Contains autonomous mission structures, pure pathing algorithms (`DynamicRouter`, `SmartTunnelRouter`), and driver assistance agents (`AutonomousTeleopAgent`).
*   **`Subsystems/`**: Contains physical hardware managers, but also has `MatchCoach`, which is a driver assist tool.

## Proposed New Directory Structure

We propose extracting these overlapping concepts into distinct, purpose-driven directories within `frc.robot`:

### 1. `Simulation/` (Renamed from `Sim/` for clarity)
**Purpose:** Pure virtual hardware and physics representations. Nothing in here should make "decisions."
*   `ArmSim.java`
*   `ShooterSim.java`
*   `VisionSim.java`
*   `LimelightSim.java`
*   `GameSim.java` (Physics rules of the game, like gravity on a note)

### 2. `Intelligence/` or `AI/` (New)
**Purpose:** The Jev AI engine and cognitive decision-making models. This is the "brain" that outputs intents, used by both Sim bots and the real robot (Co-Pilot).
*   `JevDecisionEngine.java`
*   `WorldState.java`, `WorldStateBuilder.java`
*   `StrategicObjective.java`
*   `AIActionIntent.java`
*   `Archetype.java`
*   `MultiBotManager/` (Extracted from Sim)
    *   `AIRobotSim.java` (Manages multiple AI instances, decoupled from pure physics)
    *   `AIRobotInstance.java`

### 3. `Navigation/` (Extracted from `Auto/`)
**Purpose:** The mathematical execution of movement. Given a target (from a human or AI), how do we get there while avoiding obstacles?
*   `StaticPathfinder.java`
*   `DynamicRouter.java`
*   `SmartTunnelRouter.java`
*   `DynamicObstacle.java`
*   `CollisionDetector.java`
*   `TrajectoryController.java`

### 4. `Autonomous/` (Simplified `Auto/`)
**Purpose:** Strict sequential mission logic (e.g., 15-second autonomous period).
*   `AutoMissionChooser.java`
*   `AutoMissionExecutor.java`
*   `Missions/` (All pre-baked mission files)
*   `Actions/` (All action nodes like `ShootAction`, `DriveToPoseAction`)

### 5. `DriverAssist/` or `HMI/` (New)
**Purpose:** Tools that augment human driving and situational awareness.
*   `AutonomousTeleopAgent.java` (Moved from `Auto/`)
*   `MatchCoach.java` (Moved from `Subsystems/`)
*   `LegalPinningWatchdog.java` (Moved from `Auto/` - acts as a driver warning/override)

## Benefits of this Restructuring
1.  **Clearer Dependencies:** Physics simulation no longer imports AI logic. Autonomous routines don't need to know about dynamic obstacle math unless explicitly routing.
2.  **Code Reusability:** The `Intelligence/` package can be dropped into future years largely unchanged, as it is decoupled from the `Simulation/` of the current year's game pieces.
3.  **Easier Onboarding:** New programmers can look at `Navigation/` to understand Swerve math, and `Autonomous/` to just string together a 15-second routine, without getting lost in the AI engine.
