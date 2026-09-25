# Code Quality & Javadoc Documentation Audit Report

**Author**: Explorer 2 (Code Quality & Javadocs)
**Target Codebase**: TitanRoboticsBuildSeason (`src/main/java/frc/robot`)
**Working Directory**: `C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1`
**Date**: 2026-09-25
**Parent**: orchestrator_1 (Conversation ID: `8b383374-9a92-411a-ba01-ca6b47135c41`)

## Executive Summary

A comprehensive, full-coverage static audit was performed across all **94 Java source files** comprising **15 packages** and **159 declared types** (classes, interfaces, enums, records, annotations) in the TitanRoboticsBuildSeason repository.

### High-Level Audit Findings:
1. **Documentation Deficit**:
   - **Declared Types**: 80 / 159 have Javadoc (50.3% coverage; **79 types completely lack Javadocs**).
   - **Public/Protected Methods**: 201 / 720 have Javadoc (27.9% coverage; **519 methods lack Javadocs**).
   - **Documented Engineering Units**: Only 85 / 720 methods (11.8%) and 71 / 457 constants (15.5%) have explicit physical units (e.g., meters, radians, seconds, volts, amperes, RPM) documented in signatures, names, or Javadocs.
   - **Constants (`static final`)**: Only 8 / 457 constants (1.8%) have formal Javadoc docstrings.
2. **Architectural & WPILib Convention Violations**:
   - **Custom Multithreaded Actions Framework vs WPILib CommandScheduler**: `AutoMissionExecutor` spawns autonomous routines on an unmanaged background Java thread (`new Thread(...)`), and `MissionBase` runs a polling loop using `Thread.sleep()`. Meanwhile, `CommandScheduler.getInstance().run()` is omitted from `robotPeriodic()` and `teleopPeriodic()`. This creates concurrency hazards and bypasses standard WPILib requirements and safety.
   - **Console Output Loop Overrun Risks**: **106 occurrences** of unbuffered `System.out.println`, `System.err.println`, and `printf` during active robot execution, risking RoboRIO loop overruns (>20ms).
   - **Encapsulation & Mutable State Leaks**: **167 public mutable fields**, including subsystem state variables (`Shooter.targetRpmLeft`, `AutoMissionChooser.delay`, `Teleop.joystickEnabled`) and exposed internal collections (`SubsystemManager.getSubsystems()`).
   - **REVLib Hardware Flash Parameter Burning**: `NeoSparkMaxMotor.java` dynamically executes `configure(..., PersistMode.kPersistParameters)` inside runtime setters (`setInverted`, `setBrakeMode`), wearing out SparkMax flash memory.
   - **Dead & Deprecated Code**: Legacy classes such as `Test/SysID.java` and boilerplate missions (`ExampleMission.java`) exist without active usage.

## 1. Observation

This section contains direct, verbatim evidence gathered from the filesystem, tool executions, and line-level code inspections.

### 1.1 Complete Catalog of Classes, Interfaces, Enums, Records, and Annotations

Every declared type in `src/main/java/frc/robot` was cataloged (159 total types):

| Package | File | Line | Kind | Modifiers | Name | Has Javadoc? | Summary / First Line |
|---|---|---|---|---|---|---|---|
| `frc.robot` | `BuildConstants.java` | 7 | `class` | `public final` | `BuildConstants` | ✅ Yes | Automatically generated file containing build and git versio... |
| `frc.robot` | `Main.java` | 14 | `class` | `public final` | `Main` | ✅ Yes | Do NOT add any static variables to this class, or any initia... |
| `frc.robot` | `Robot.java` | 42 | `class` | `public` | `Robot` | ✅ Yes | The methods in this class are called automatically correspon... |
| `frc.robot` | `Teleop.java` | 35 | `class` | `public` | `Teleop` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\AutoMission.java` | 14 | `@interface` | `public` | `AutoMission` | ✅ Yes | Annotation to mark a class as an autonomous mission that sho... |
| `frc.robot.Auto` | `Auto\AutoMissionChooser.java` | 24 | `class` | `public` | `AutoMissionChooser` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\AutoMissionEndedException.java` | 8 | `class` | `public` | `AutoMissionEndedException` | ✅ Yes | Class: AutoMissionEndedException |
| `frc.robot.Auto` | `Auto\AutoMissionExecutor.java` | 10 | `class` | `public` | `AutoMissionExecutor` | ✅ Yes | Class: AutoMissionExecutor |
| `frc.robot.Auto` | `Auto\AutonomousTeleopAgent.java` | 32 | `class` | `public` | `AutonomousTeleopAgent` | ✅ Yes | AutonomousTeleopAgent: Real-Robot Co-Pilot & One-Button Smar... |
| `frc.robot.Auto` | `Auto\CollisionDetector.java` | 9 | `class` | `public` | `CollisionDetector` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\DynamicObstacle.java` | 11 | `class` | `public` | `DynamicObstacle` | ✅ Yes | Represents a dynamic moving obstacle on the field (e.g. oppo... |
| `frc.robot.Auto` | `Auto\DynamicRouter.java` | 25 | `class` | `public` | `DynamicRouter` | ✅ Yes | DynamicRouter provides real-time local obstacle avoidance ar... |
| `frc.robot.Auto` | `Auto\DynamicRouter.java` | 27 | `enum` | `public` | `AvoidanceAlgorithm` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\DynamicRouter.java` | 377 | `class` | `private static` | `GridNode` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\LegalPinningWatchdog.java` | 15 | `class` | `public` | `LegalPinningWatchdog` | ✅ Yes | LegalPinningWatchdog enforces FRC G-rule pinning limits duri... |
| `frc.robot.Auto` | `Auto\SmartTunnelRouter.java` | 21 | `class` | `public` | `SmartTunnelRouter` | ✅ Yes | SmartTunnelRouter provides intelligent, collision-free trenc... |
| `frc.robot.Auto` | `Auto\SmartTunnelRouter.java` | 23 | `enum` | `public` | `TrenchCorridor` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\SmartTunnelRouter.java` | 28 | `class` | `public static` | `TunnelRoute` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 26 | `class` | `public` | `StaticPathfinder` | ✅ Yes | StaticPathfinder: 2026 Field Topological Roadmap & Visibilit... |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 31 | `interface` | `public` | `Obstacle` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 37 | `class` | `public static` | `CircularObstacle` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 68 | `class` | `public static` | `RectangularObstacle` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 97 | `class` | `public static` | `AABB` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 115 | `class` | `public static` | `RoadmapNode` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\StaticPathfinder.java` | 597 | `class` | `private static` | `NodeRecord` | ❌ Missing |  |
| `frc.robot.Auto` | `Auto\TrajectoryController.java` | 30 | `class` | `public` | `TrajectoryController` | ✅ Yes | High-performance holonomic trajectory controller for swerve ... |
| `frc.robot.Auto.Actions` | `Auto\Actions\AutoAimAction.java` | 17 | `class` | `public` | `AutoAimAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\BallHuntAction.java` | 24 | `class` | `public` | `BallHuntAction` | ✅ Yes | Intelligent Semi-Autonomous "Auto Ball Pick Up" Action (Ball... |
| `frc.robot.Auto.Actions` | `Auto\Actions\DriveToPoseAction.java` | 20 | `class` | `public` | `DriveToPoseAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\FollowChoreoPath.java` | 29 | `class` | `public` | `FollowChoreoPath` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\IntakeAction.java` | 7 | `class` | `public` | `IntakeAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\LambdaAction.java` | 9 | `class` | `public` | `LambdaAction` | ✅ Yes | A simple action that runs a Runnable once and finishes immed... |
| `frc.robot.Auto.Actions` | `Auto\Actions\ParallelAction.java` | 12 | `class` | `public` | `ParallelAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\ParallelRaceAction.java` | 8 | `class` | `public` | `ParallelRaceAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\SeriesAction.java` | 12 | `class` | `public` | `SeriesAction` | ✅ Yes | Executes a list of actions sequentially. |
| `frc.robot.Auto.Actions` | `Auto\Actions\ShootAction.java` | 16 | `class` | `public` | `ShootAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\WaitAction.java` | 10 | `class` | `public` | `WaitAction` | ❌ Missing |  |
| `frc.robot.Auto.Actions` | `Auto\Actions\WaitUntilMarkerAction.java` | 10 | `class` | `public` | `WaitUntilMarkerAction` | ✅ Yes | Action that waits until a specific marker has been passed in... |
| `frc.robot.Auto.Missions` | `Auto\Missions\AdvancedChoreoMission.java` | 17 | `class` | `public` | `AdvancedChoreoMission` | ✅ Yes | Example of a high-performance Choreo mission. |
| `frc.robot.Auto.Missions` | `Auto\Missions\DepotShootMission.java` | 15 | `class` | `public` | `DepotShootMission` | ❌ Missing |  |
| `frc.robot.Auto.Missions` | `Auto\Missions\DoNothingMission.java` | 11 | `class` | `public` | `DoNothingMission` | ❌ Missing |  |
| `frc.robot.Auto.Missions` | `Auto\Missions\DynamicChoreoMission.java` | 9 | `class` | `public` | `DynamicChoreoMission` | ✅ Yes | A generic autonomous mission that simply follows a Choreo tr... |
| `frc.robot.Auto.Missions` | `Auto\Missions\ExampleMission.java` | 18 | `class` | `public` | `ExampleMission` | ❌ Missing |  |
| `frc.robot.Auto.Missions` | `Auto\Missions\MissionBase.java` | 15 | `class` | `public abstract` | `MissionBase` | ❌ Missing |  |
| `frc.robot.Auto.Missions` | `Auto\Missions\ShooterMission.java` | 22 | `class` | `public` | `ShooterMission` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 24 | `class` | `public` | `Constants` | ✅ Yes | Global Robot & Platform Constants. |
| `frc.robot.Data` | `Data\Constants.java` | 32 | `enum` | `public static` | `Mode` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 68 | `class` | `public static final` | `AutonConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 93 | `class` | `public static final` | `DrivebaseConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 126 | `class` | `public static` | `OperatorConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 139 | `class` | `public static final` | `LEDConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 173 | `class` | `public static final` | `FieldConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 196 | `class` | `public static final` | `ShooterConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\Constants.java` | 243 | `class` | `public static final` | `IntakeConstants` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 24 | `class` | `public final` | `FieldMap` | ✅ Yes | Unified Field Map for the 2026 Rebuilt FIRST Robotics Compet... |
| `frc.robot.Data` | `Data\FieldMap.java` | 40 | `class` | `public static final` | `Hubs` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 69 | `class` | `public static final` | `Ramps` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 88 | `class` | `public static final` | `Trenches` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 128 | `class` | `public static final` | `AllianceZones` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 163 | `class` | `public static final` | `Depots` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 190 | `class` | `public static final` | `ClimbingTowers` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 203 | `class` | `public static` | `AABB` | ❌ Missing |  |
| `frc.robot.Data` | `Data\FieldMap.java` | 279 | `class` | `public static final` | `Obstacles` | ❌ Missing |  |
| `frc.robot.Data` | `Data\GlideConstants.java` | 17 | `class` | `public` | `GlideConstants` | ✅ Yes | Tactical navigation waypoints (Glide Points) and Auto-Tunnel... |
| `frc.robot.Data` | `Data\GlideConstants.java` | 26 | `class` | `public static` | `GlidePoint` | ❌ Missing |  |
| `frc.robot.Data` | `Data\PortMap.java` | 3 | `class` | `public final` | `PortMap` | ❌ Missing |  |
| `frc.robot.Data` | `Data\TunableNumber.java` | 13 | `class` | `public` | `TunableNumber` | ✅ Yes | Class for a tunable number. Gets value from dashboard in tun... |
| `frc.robot.Devices` | `Devices\Controller.java` | 10 | `class` | `public` | `Controller` | ❌ Missing |  |
| `frc.robot.Devices` | `Devices\Controller.java` | 12 | `enum` | `public` | `RumblePattern` | ❌ Missing |  |
| `frc.robot.Devices` | `Devices\NeoSparkMaxMotor.java` | 19 | `class` | `public` | `NeoSparkMaxMotor` | ❌ Missing |  |
| `frc.robot.Interfaces` | `Interfaces\Actions.java` | 8 | `interface` | `public` | `Actions` | ❌ Missing |  |
| `frc.robot.Interfaces` | `Interfaces\Subsystem.java` | 5 | `interface` | `public` | `Subsystem` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\AIActionIntent.java` | 11 | `record` | `public` | `AIActionIntent` | ✅ Yes | Concrete subsystem execution instructions generated by Syste... |
| `frc.robot.Sim` | `Sim\AIRobotInstance.java` | 39 | `class` | `public` | `AIRobotInstance` | ✅ Yes | Autonomous AI Robot Instance. |
| `frc.robot.Sim` | `Sim\AIRobotSim.java` | 50 | `class` | `public` | `AIRobotSim` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\AIRobotSim.java` | 52 | `enum` | `public` | `AIMode` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\AIRobotSim.java` | 78 | `enum` | `public` | `CyclerPhase` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\Archetype.java` | 6 | `enum` | `public` | `Archetype` | ✅ Yes | AI Competitor Archetypes governing System 2 macro utility we... |
| `frc.robot.Sim` | `Sim\ArmSim.java` | 13 | `class` | `public` | `ArmSim` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\GameSim.java` | 38 | `class` | `public` | `GameSim` | ✅ Yes | GameSim manages the simulation state for the 2026 robotics g... |
| `frc.robot.Sim` | `Sim\GameSim.java` | 41 | `class` | `private static final` | `Config` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 36 | `class` | `public` | `JevDecisionEngine` | ✅ Yes | Jev AI Decision Engine (TypeSafe AI) |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 38 | `enum` | `public` | `AIArchetype` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 46 | `enum` | `public` | `TacticalAction` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 53 | `class` | `public static` | `DecisionResult` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 636 | `enum` | `public` | `OffensiveStrategy` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\JevDecisionEngine.java` | 649 | `class` | `public static` | `StrategicAdvice` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\LimelightSim.java` | 20 | `class` | `public` | `LimelightSim` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\MatchScoreTracker.java` | 34 | `class` | `public` | `MatchScoreTracker` | ✅ Yes | MatchScoreTracker: Unified FRC 2026 Rebuilt Real-Time Match ... |
| `frc.robot.Sim` | `Sim\ShooterSim.java` | 24 | `class` | `public` | `ShooterSim` | ❌ Missing |  |
| `frc.robot.Sim` | `Sim\StrategicObjective.java` | 8 | `enum` | `public` | `StrategicObjective` | ✅ Yes | Universal FRC Strategic Objectives arbitrated by System 2 Ex... |
| `frc.robot.Sim` | `Sim\VisionSim.java` | 32 | `class` | `public` | `VisionSim` | ✅ Yes | PhotonVision simulation engine combining: |
| `frc.robot.Sim` | `Sim\WorldState.java` | 10 | `record` | `public` | `WorldState` | ✅ Yes | Immutable snapshot capturing all ground truth field informat... |
| `frc.robot.Sim` | `Sim\WorldStateBuilder.java` | 14 | `class` | `public final` | `WorldStateBuilder` | ✅ Yes | Factory utilities for building immutable WorldState snapshot... |
| `frc.robot.Subsystems` | `Subsystems\Dashboard.java` | 27 | `class` | `public` | `Dashboard` | ✅ Yes | Subsystem responsible for publishing aggregated driver telem... |
| `frc.robot.Subsystems` | `Subsystems\Intake.java` | 42 | `class` | `public` | `Intake` | ✅ Yes | Ground intake mechanism featuring an articulated pivot arm w... |
| `frc.robot.Subsystems` | `Subsystems\Intake.java` | 53 | `enum` | `public` | `IntakeState` | ✅ Yes | Discrete operational states for intake pivot and roller mech... |
| `frc.robot.Subsystems` | `Subsystems\LEDs.java` | 13 | `class` | `public` | `LEDs` | ❌ Missing |  |
| `frc.robot.Subsystems` | `Subsystems\MatchCoach.java` | 38 | `class` | `public` | `MatchCoach` | ✅ Yes | MatchCoach: Real-Time AI Coaching & Practice Proving Ground ... |
| `frc.robot.Subsystems` | `Subsystems\MatchCoach.java` | 40 | `enum` | `public` | `DrillMode` | ❌ Missing |  |
| `frc.robot.Subsystems` | `Subsystems\Shooter.java` | 39 | `class` | `public` | `Shooter` | ✅ Yes | Dual-flywheel shooter mechanism with indexer/kicker feed, cl... |
| `frc.robot.Subsystems` | `Subsystems\Shooter.java` | 46 | `enum` | `public` | `ShooterState` | ✅ Yes | Discrete operational states for shooter state machine. |
| `frc.robot.Subsystems` | `Subsystems\Shooter.java` | 102 | `record` | `public` | `ShootingSolution` | ✅ Yes | Shooting Solution record containing targeting calculations. |
| `frc.robot.Subsystems` | `Subsystems\SubsystemManager.java` | 7 | `class` | `public` | `SubsystemManager` | ❌ Missing |  |
| `frc.robot.Subsystems` | `Subsystems\SwerveBase.java` | 59 | `class` | `public` | `SwerveBase` | ❌ Missing |  |
| `frc.robot.Subsystems` | `Subsystems\Vision.java` | 27 | `class` | `public` | `Vision` | ✅ Yes | Vision Subsystem following the AdvantageKit IO abstraction p... |
| `frc.robot.Subsystems` | `Subsystems\Vision.java` | 288 | `class` | `public static` | `VisionTargetEstimate` | ❌ Missing |  |
| `frc.robot.Subsystems.drive` | `Subsystems\drive\DriveIO.java` | 12 | `interface` | `public` | `DriveIO` | ✅ Yes | Hardware IO abstraction interface for the Swerve Drivebase. |
| `frc.robot.Subsystems.drive` | `Subsystems\drive\DriveIO.java` | 15 | `class` | `public static` | `DriveIOInputs` | ❌ Missing |  |
| `frc.robot.Subsystems.drive` | `Subsystems\drive\DriveIOSim.java` | 12 | `class` | `public` | `DriveIOSim` | ✅ Yes | Desktop simulation implementation of DriveIO. |
| `frc.robot.Subsystems.drive` | `Subsystems\drive\DriveIOSparkMax.java` | 14 | `class` | `public` | `DriveIOSparkMax` | ✅ Yes | Physical hardware implementation of DriveIO interfacing with... |
| `frc.robot.Subsystems.intake` | `Subsystems\intake\IntakeConstants.java` | 8 | `class` | `public final` | `IntakeConstants` | ✅ Yes | Physical geometry, tuning parameters, and motion limits for ... |
| `frc.robot.Subsystems.intake` | `Subsystems\intake\IntakeIO.java` | 9 | `interface` | `public` | `IntakeIO` | ✅ Yes | Hardware IO abstraction interface for the articulated ground... |
| `frc.robot.Subsystems.intake` | `Subsystems\intake\IntakeIO.java` | 12 | `class` | `public static` | `IntakeIOInputs` | ❌ Missing |  |
| `frc.robot.Subsystems.intake` | `Subsystems\intake\IntakeIOSim.java` | 14 | `class` | `public` | `IntakeIOSim` | ✅ Yes | Desktop simulation implementation of IntakeIO wrapping ArmSi... |
| `frc.robot.Subsystems.intake` | `Subsystems\intake\IntakeIOSparkMax.java` | 13 | `class` | `public` | `IntakeIOSparkMax` | ✅ Yes | Real physical hardware implementation of IntakeIO using REV ... |
| `frc.robot.Subsystems.shooter` | `Subsystems\shooter\ShooterConstants.java` | 10 | `class` | `public final` | `ShooterConstants` | ✅ Yes | Physical geometry, tuning parameters, and simulation metrics... |
| `frc.robot.Subsystems.shooter` | `Subsystems\shooter\ShooterIO.java` | 9 | `interface` | `public` | `ShooterIO` | ✅ Yes | Hardware IO abstraction interface for the dual-flywheel shoo... |
| `frc.robot.Subsystems.shooter` | `Subsystems\shooter\ShooterIO.java` | 12 | `class` | `public static` | `ShooterIOInputs` | ❌ Missing |  |
| `frc.robot.Subsystems.shooter` | `Subsystems\shooter\ShooterIOSim.java` | 8 | `class` | `public` | `ShooterIOSim` | ✅ Yes | Desktop simulation implementation of ShooterIO wrapping Shoo... |
| `frc.robot.Subsystems.shooter` | `Subsystems\shooter\ShooterIOSparkMax.java` | 11 | `class` | `public` | `ShooterIOSparkMax` | ✅ Yes | Real physical hardware implementation of ShooterIO using REV... |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIO.java` | 10 | `interface` | `public` | `VisionIO` | ✅ Yes | Hardware IO abstraction interface for Vision coprocessors (L... |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIO.java` | 13 | `class` | `public static` | `VisionIOInputs` | ❌ Missing |  |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIOLimelight.java` | 9 | `class` | `public` | `VisionIOLimelight` | ✅ Yes | Real hardware implementation of VisionIO using Limelight Meg... |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIOPhotonVision.java` | 18 | `class` | `public` | `VisionIOPhotonVision` | ✅ Yes | Hardware IO implementation for PhotonVision running on a cop... |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIOSim.java` | 16 | `class` | `public` | `VisionIOSim` | ✅ Yes | Desktop simulation implementation of VisionIO supporting bot... |
| `frc.robot.Subsystems.vision` | `Subsystems\vision\VisionIOSim.java` | 18 | `enum` | `public` | `CameraType` | ❌ Missing |  |
| `frc.robot.Test` | `Test\Diagnostics.java` | 37 | `class` | `public` | `Diagnostics` | ✅ Yes | Diagnostics subsystem for safe hardware verification and aut... |
| `frc.robot.Test` | `Test\Diagnostics.java` | 49 | `class` | `public static` | `DiagTest` | ✅ Yes | A single diagnostic check: one motor in, one encoder/positio... |
| `frc.robot.Test` | `Test\Diagnostics.java` | 64 | `enum` | `public` | `PreFlightStep` | ❌ Missing |  |
| `frc.robot.Test` | `Test\DriveCharacterization.java` | 25 | `class` | `public` | `DriveCharacterization` | ✅ Yes | Swerve drive characterization and testing system. |
| `frc.robot.Test` | `Test\DriveCharacterization.java` | 27 | `enum` | `private` | `TestMode` | ❌ Missing |  |
| `frc.robot.Test` | `Test\IntakeTesting.java` | 19 | `class` | `public` | `IntakeTesting` | ✅ Yes | Intake system testing and calibration. |
| `frc.robot.Test` | `Test\IntakeTesting.java` | 21 | `enum` | `private` | `TestMode` | ❌ Missing |  |
| `frc.robot.Test` | `Test\ShooterTuning.java` | 22 | `class` | `public` | `ShooterTuning` | ✅ Yes | Comprehensive shooter tuning and testing system. |
| `frc.robot.Test` | `Test\ShooterTuning.java` | 24 | `enum` | `private` | `TestMode` | ❌ Missing |  |
| `frc.robot.Test` | `Test\SysID.java` | 12 | `class` | `public` | `SysID` | ✅ Yes | Legacy wrapper for SysId routines, now delegating to the uni... |
| `frc.robot.Test` | `Test\SysIdManager.java` | 32 | `class` | `public` | `SysIdManager` | ✅ Yes | Unified System Identification (SysID) Manager for Team 8334. |
| `frc.robot.Test` | `Test\SysIdManager.java` | 36 | `enum` | `public` | `MechanismType` | ❌ Missing |  |
| `frc.robot.Test` | `Test\TestMode.java` | 16 | `class` | `public` | `TestMode` | ✅ Yes | Comprehensive test mode for robot subsystem tuning, SysID, a... |
| `frc.robot.Test` | `Test\TestMode.java` | 20 | `enum` | `public` | `TestCategory` | ❌ Missing |  |
| `frc.robot.Test` | `Test\VisionTesting.java` | 21 | `class` | `public` | `VisionTesting` | ✅ Yes | Vision system testing and calibration. |
| `frc.robot.Test` | `Test\VisionTesting.java` | 23 | `enum` | `private` | `TestMode` | ❌ Missing |  |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 40 | `class` | `public` | `LimelightHelpers` | ✅ Yes | LimelightHelpers provides static methods and classes for int... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 47 | `class` | `public static` | `LimelightTarget_Retro` | ✅ Yes | Represents a Color/Retroreflective Target Result extracted f... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 143 | `class` | `public static` | `LimelightTarget_Fiducial` | ✅ Yes | Represents an AprilTag/Fiducial Target Result extracted from... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 244 | `class` | `public static` | `LimelightTarget_Barcode` | ✅ Yes | Represents a Barcode Target Result extracted from JSON Outpu... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 293 | `class` | `public static` | `LimelightTarget_Classifier` | ✅ Yes | Represents a Neural Classifier Pipeline Result extracted fro... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 326 | `class` | `public static` | `LimelightTarget_Detector` | ✅ Yes | Represents a Neural Detector Pipeline Result extracted from ... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 365 | `class` | `public static` | `LimelightResults` | ✅ Yes | Limelight Results object, parsed from a Limelight's JSON res... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 472 | `class` | `public static` | `RawFiducial` | ✅ Yes | Represents a Limelight Raw Fiducial result from Limelight's ... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 511 | `class` | `public static` | `RawDetection` | ✅ Yes | Represents a Limelight Raw Neural Detector result from Limel... |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 549 | `class` | `public static` | `PoseEstimate` | ✅ Yes | Represents a 3D Pose Estimate. |
| `frc.robot.ThirdParty` | `ThirdParty\LimelightHelpers.java` | 612 | `class` | `public static` | `IMUData` | ✅ Yes | Encapsulates the state of an internal Limelight IMU. |
| `frc.robot.Utils` | `Utils\Alert.java` | 9 | `class` | `public` | `Alert` | ✅ Yes | Class representing a persistent alert displayed on driver da... |
| `frc.robot.Utils` | `Utils\Alert.java` | 11 | `enum` | `public` | `AlertType` | ❌ Missing |  |
| `frc.robot.Utils` | `Utils\AlertManager.java` | 13 | `class` | `public` | `AlertManager` | ✅ Yes | Manages all robot alerts and publishes them to NetworkTables... |
| `frc.robot.Utils` | `Utils\AllianceFlipUtil.java` | 16 | `class` | `public` | `AllianceFlipUtil` | ✅ Yes | Standardizes coordinate geometry to Blue-origin coordinates ... |
| `frc.robot.Utils` | `Utils\Vector2dSlewRateLimiter.java` | 17 | `class` | `public` | `Vector2dSlewRateLimiter` | ✅ Yes | True 2D Vector Slew Rate Limiter. |

### 1.2 Quantitative Javadoc & Engineering Units Coverage by Package

| Package | Files | Types Total | Types w/ JD | Methods (Pub/Prot) | Methods w/ JD | Methods w/ Units | Constants Total | Consts w/ JD | Consts w/ Units | Smells |
|---|---|---|---|---|---|---|---|---|---|---|
| `frc.robot` | 4 | 4 | 3 (75%) | 31 | 15 (48%) | 4 (13%) | 8 | 0 (0%) | 0 (0%) | 5 |
| `frc.robot.Auto` | 12 | 22 | 10 (45%) | 84 | 25 (30%) | 10 (12%) | 80 | 0 (0%) | 12 (15%) | 4 |
| `frc.robot.Auto.Actions` | 12 | 12 | 4 (33%) | 65 | 5 (8%) | 2 (3%) | 3 | 0 (0%) | 1 (33%) | 0 |
| `frc.robot.Auto.Missions` | 7 | 7 | 2 (29%) | 17 | 0 (0%) | 0 (0%) | 0 | 0 (N/A) | 0 (N/A) | 8 |
| `frc.robot.Data` | 5 | 22 | 4 (18%) | 30 | 9 (30%) | 2 (7%) | 215 | 1 (0%) | 24 (11%) | 0 |
| `frc.robot.Devices` | 2 | 3 | 0 (0%) | 31 | 4 (13%) | 4 (13%) | 0 | 0 (N/A) | 0 (N/A) | 1 |
| `frc.robot.Interfaces` | 2 | 2 | 0 (0%) | 8 | 1 (12%) | 0 (0%) | 0 | 0 (N/A) | 0 (N/A) | 0 |
| `frc.robot.Sim` | 14 | 22 | 10 (45%) | 183 | 23 (13%) | 15 (8%) | 51 | 0 (0%) | 6 (12%) | 21 |
| `frc.robot.Subsystems` | 8 | 13 | 8 (62%) | 287 | 120 (42%) | 41 (14%) | 17 | 0 (0%) | 1 (6%) | 9 |
| `frc.robot.Subsystems.drive` | 3 | 4 | 3 (75%) | 19 | 7 (37%) | 6 (32%) | 0 | 0 (N/A) | 0 (N/A) | 16 |
| `frc.robot.Subsystems.intake` | 4 | 5 | 4 (80%) | 24 | 8 (33%) | 10 (42%) | 39 | 0 (0%) | 8 (21%) | 12 |
| `frc.robot.Subsystems.shooter` | 4 | 5 | 4 (80%) | 13 | 4 (31%) | 6 (46%) | 37 | 7 (19%) | 17 (46%) | 11 |
| `frc.robot.Subsystems.vision` | 4 | 6 | 4 (67%) | 12 | 2 (17%) | 2 (17%) | 0 | 0 (N/A) | 0 (N/A) | 13 |
| `frc.robot.Test` | 8 | 16 | 9 (56%) | 59 | 29 (49%) | 3 (5%) | 4 | 0 (0%) | 2 (50%) | 48 |
| `frc.robot.ThirdParty` | 1 | 11 | 11 (100%) | 95 | 55 (58%) | 18 (19%) | 0 | 0 (N/A) | 0 (N/A) | 132 |
| `frc.robot.Utils` | 4 | 5 | 4 (80%) | 35 | 22 (63%) | 1 (3%) | 3 | 0 (0%) | 0 (0%) | 0 |
| **TOTAL** | **94** | **159** | **80 (50.3%)** | **993** | **329 (33.1%)** | **124 (12.5%)** | **457** | **8 (1.8%)** | **71 (15.5%)** | **280** |

### 1.3 Detailed Code Quality Smells, Anti-Patterns & Convention Violations

#### Issue 1: Custom Actions Framework & Asynchronous Threading Race Conditions
- **Files & Lines**:
  - `src/main/java/frc/robot/Auto/AutoMissionExecutor.java:16-28`
  - `src/main/java/frc/robot/Auto/Missions/MissionBase.java:74-103`
  - `src/main/java/frc/robot/Interfaces/Actions.java:8-34`
  - `src/main/java/frc/robot/Robot.java:172-181, 270-272`
- **Verbatim Code Evidence**:
```java
// AutoMissionExecutor.java:16-28
mThread = new Thread(new Runnable() {
    @Override
    public void run() {
        if (mAutoMission != null) {
            try {
                mAutoMission.run();
            } catch (Exception e) {
                edu.wpi.first.wpilibj.DriverStation.reportError("AUTO MISSION CRASHED: " + e.getMessage(), e.getStackTrace());
            }
        }
    }
});
```
```java
// MissionBase.java:90-99
while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
    action.update();
    try {
        Thread.sleep(waitTime);
    } catch (InterruptedException e) {
        e.printStackTrace();
    }
}
```
```java
// Robot.java:172-181
@Override
public void robotPeriodic() {
    SubsystemManager.updateSubsystems();
    SubsystemManager.logSubsystems();
    AlertManager.update();
    if (testMode != null && (DriverStation.isTest() || testMode.isEnabled())) {
      testMode.update();
    }
    // NOTE: CommandScheduler.getInstance().run() IS MISSING!
}
```
- **Analysis**: WPILib Command-based architecture is fundamentally single-threaded to prevent data races and CAN bus synchronization issues. In this codebase, autonomous missions run inside a detached raw Java thread sleeping in 20ms slices (`Thread.sleep(waitTime)`). Actions call subsystem methods directly from this worker thread while `SubsystemManager.updateSubsystems()` runs concurrently on the main robot thread. Additionally, `CommandScheduler.getInstance().run()` is only invoked in `testPeriodic()`, preventing any standard WPILib Command/Trigger bindings from functioning during autonomous or teleop.

#### Issue 2: Console I/O Overrun Hazard (106 Occurrences)
- **Files & Lines**: Found across 24 files, most notably:
  - `Test/DriveCharacterization.java:144, 148, 152, 156, 163, 174, 179, 187, 192, 205, 210, 218, 223, 241, 245` (15 prints during real-time driver button handling)
  - `Sim/GameSim.java:152, 212, 234, 265, 293, 381, 406, 451...` (11 stderr/stdout prints)
  - `Sim/AIRobotSim.java:176, 300, 313, 995, 1098` (7 prints)
  - `Robot.java:112, 122, 124, 207` (4 prints)
  - `Devices/NeoSparkMaxMotor.java:39` (1 print)
  - `Subsystems/Vision.java:340, 347, 354` (3 prints)
- **Analysis**: Unbuffered console I/O on the RoboRIO over NetworkTables causes serial bus stalls, GC spikes, and frame jitter exceeding the 20ms boundary. In match environments, these calls must be replaced by AdvantageKit `Logger.recordOutput`, `DataLogManager.log`, or `DriverStation.reportWarning` / `DriverStation.reportError`.

#### Issue 3: Public Mutable Fields (Encapsulation Violations)
- **Files & Lines**:
  - `Subsystems/Shooter.java:76-77, 86-88`:
    ```java
    public double targetRpmLeft = 0;
    public double targetRpmRight = 0;
    public double normalDistanceToHub = 0;
    public double leftShooterVoltageCalc = 0;
    public double rightShooterVoltageCalc = 0;
    ```
  - `Auto/AutoMissionChooser.java:28`: `public static double delay;`
  - `Teleop.java:47`: `public static boolean joystickEnabled = false;`
  - `Auto/DynamicRouter.java:380-382`: `GridNode` public mutable `gCost`, `hCost`, `parent`
- **Analysis**: Subsystem internal states are exposed as public mutable primitives. Audit confirmed `Shooter.targetRpmLeft` is only modified internally, meaning its `public` visibility is an encapsulation anti-pattern.

#### Issue 4: REVLib 2025/2026 SparkMax Flash Wear Anti-Pattern
- **File & Lines**: `src/main/java/frc/robot/Devices/NeoSparkMaxMotor.java:85-100`
- **Verbatim Code Evidence**:
```java
public void setInverted(boolean inverted) {
    this.isInverted = inverted;
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(inverted);
    if (m_motor != null) {
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}

public void setBrakeMode(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    if (m_motor != null) {
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
```
- **Analysis**: `PersistMode.kPersistParameters` burns configuration into the SparkMax EEPROM/flash memory. Flash memory has a limited lifetime write cycle (~10,000 to 100,000 writes). Dynamically calling `configure` with `kPersistParameters` during runtime (e.g., toggling brake mode when disabling) degrades hardware memory and blocks CAN bus communications. Dynamic runtime updates must use `PersistMode.kNoPersistParameters`.

#### Issue 5: SubsystemManager Mutable Exposure & Thread Safety Contradiction
- **File & Lines**: `src/main/java/frc/robot/Subsystems/SubsystemManager.java:52-62`
- **Verbatim Code Evidence**:
```java
/**
 * Returns the list of registered subsystems. This list is unmodifiable, as the purpose is to...
 */
public static List<Subsystem> getSubsystems() {
    return subsystems;
}
```
- **Analysis**: The Javadoc explicitly promises an unmodifiable list, but the implementation returns the raw, mutable `ArrayList<Subsystem> subsystems`. Any external caller can clear, reorder, or corrupt registered subsystems. It must return `Collections.unmodifiableList(subsystems)`.

#### Issue 6: Dead, Deprecated, and Duplicate Code
- **Files & Lines**:
  - `Test/SysID.java:1-50`: Completely unreferenced legacy SysID class. All system identification is already handled by `SysIdManager.java`.
  - `Devices/Controller.java:58-87`: Overloads `getDebouncedButton(int button)` and `getDebouncedButton(Button button)` duplicate 15 lines of identical logic instead of `getDebouncedButton(button.value)`.
  - `Devices/NeoSparkMaxMotor.java:67-76`: `getVelocity()` is an exact duplicate of `getSpeed()`.
  - `Auto/Missions/ExampleMission.java:1-25`: Unused placeholder mission class.

#### Issue 7: In-Loop Heap Allocations & Autoboxing
- **Files & Lines**:
  - `Devices/Controller.java:25, 58-87`: `HashMap<Integer, Boolean> debounceButtons` creates autoboxed `Integer` and `Boolean` heap objects inside 20ms teleop cycles.
  - `Subsystems/SwerveBase.java:988-1002`: `getDriveMotorVoltages()` and `getDriveMotorPositions()` instantiate `new ArrayList<>()` every call.

#### Issue 8: Contradictory Constants & Undocumented Magic Numbers
- **Files & Lines**:
  - `Data/Constants.java:55`: `public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32 lbs * kg per pound`. 148 - 20.3 = 127.7 lbs, not 32 lbs! The comment is wildly contradictory.
  - `Teleop.java:340-341, 446-447`: Hardcoded slow mode scale factors `0.35` (translation) and `0.50` (rotation) are duplicated as raw magic numbers instead of residing in `Constants.OperatorConstants`.

#### Issue 9: Exception Swallowing via `printStackTrace()`
- **Files & Lines**:
  - `Auto/Missions/MissionBase.java:83, 97`: InterruptedException swallowed with `e.printStackTrace()` inside thread sleep loops.
  - `Sim/AIRobotSim.java:301, 314`: Opponent/Ally spawn failures printed to stderr.
  - `Sim/GameSim.java:213`: `simulationUpdate` exception printed to stderr.

### 1.4 Detailed Documentation & Cleanup Inventory Per File (All 94 Files)

Below is the complete, file-by-file audit of documentation status, missing Javadocs, missing engineering units, and cleanup recommendations for all 94 files in `src/main/java/frc/robot`:

#### 1. `BuildConstants.java`
- **Package**: `frc.robot`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (6)**: 0/6 have Javadoc, 6/6 lack explicit units
  - *Constants Lacking Units*: `ROBOT_NAME`, `GIT_SHA`, `GIT_BRANCH`, `GIT_DATE`, `BUILD_DATE`, `DIRTY`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 6 constants; Document physical engineering units on 6 constants or add unit suffixes.

#### 2. `Main.java`
- **Package**: `frc.robot`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (1)**: 1/1 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

#### 3. `Robot.java`
- **Package**: `frc.robot`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (14)**: 12/14 have Javadoc (2 missing)
  - *Missing Method Javadoc*: `robotInit()`, `close()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (4)**:
  - Line 112: `[CONSOLE_IO]` System.out.println("[WebServer] Notice: Elastic layout WebServer on port 5800 could not be
  - Line 122: `[CONSOLE_IO]` System.out.println("[PortForwarder] Coprocessor ports 5801-5805 successfully forwarded.");
  - Line 124: `[CONSOLE_IO]` System.out.println("[PortForwarder] Notice: Port forwarding setup encountered: " + t.getMe
  - Line 207: `[CONSOLE_IO]` System.out.println("Auto selected: " + m_autoSelected);
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 2 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 4. `Teleop.java`
- **Package**: `frc.robot`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `Teleop`)
- **Public/Protected Methods (16)**: 2/16 have Javadoc (14 missing)
  - *Missing Method Javadoc*: `init()`, `reset()`, `teleopPeriodic()`, `readControllers()`, `driveBaseControl()`, `intakeControl()`, `shooterControl()`, `isSlowModeActive()`, ... (+6 more)
- **Constants (2)**: 0/2 have Javadoc, 2/2 lack explicit units
  - *Constants Lacking Units*: `TRANSLATION_DEADBAND`, `ROTATION_DEADBAND`
- **Smells/Issues (1)**:
  - Line 47: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'joystickEnabled': public static boolean joystickEnabled = false;
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Teleop; Add descriptive Javadocs with `@param` and `@return` to 14 public/protected methods; Add formal Javadoc comments to 2 constants; Document physical engineering units on 2 constants or add unit suffixes; Encapsulate public mutable fields with private access and getters/setters.

#### 5. `Auto\AutoMission.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

#### 6. `Auto\AutoMissionChooser.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `AutoMissionChooser`)
- **Public/Protected Methods (7)**: 0/7 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `updateMissionCreator()`, `getAutoMissionForParams()`, `reset()`, `outputToSmartDashboard()`, `getRawChooser()`, `getSelected()`, `getAutoMission()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (1)**:
  - Line 28: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'delay': public static double delay;
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AutoMissionChooser; Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods; Encapsulate public mutable fields with private access and getters/setters.

#### 7. `Auto\AutoMissionEndedException.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `serialVersionUID`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 8. `Auto\AutoMissionExecutor.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (10)**: 0/10 have Javadoc (10 missing)
  - *Missing Method Javadoc*: `setAutoMission()`, `run()`, `start()`, `isStarted()`, `reset()`, `stop()`, `getAutoMission()`, `isInterrupted()`, ... (+2 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 10 public/protected methods.

#### 9. `Auto\AutonomousTeleopAgent.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (12)**: 3/12 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `getInstance()`, `isAssistActive()`, `checkAndClearBreakout()`, `getCurrentAction()`, `getActiveObjective()`, `getLatestIntent()`, `incrementBallCount()`, `decrementBallCount()`, ... (+1 more)
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `MAX_FUEL_CAPACITY`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 10. `Auto\CollisionDetector.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `CollisionDetector`)
- **Public/Protected Methods (5)**: 0/5 have Javadoc (5 missing)
  - *Missing Method Javadoc*: `isImpactDetected()`, `isStalled()`, `getStallDuration()`, `reset()`, `getLastJerkMagnitude()`
- **Constants (6)**: 0/6 have Javadoc, 4/6 lack explicit units
  - *Constants Lacking Units*: `COLLISION_JERK_THRESHOLD`, `COLLISION_DECEL_THRESHOLD`, `STALL_CMD_SPEED_MIN`, `STALL_ACTUAL_SPEED_MAX`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to CollisionDetector; Add descriptive Javadocs with `@param` and `@return` to 5 public/protected methods; Add formal Javadoc comments to 6 constants; Document physical engineering units on 4 constants or add unit suffixes.

#### 11. `Auto\DynamicObstacle.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (3)**: 3/3 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

#### 12. `Auto\DynamicRouter.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (3)**: 1/3 have Javadoc (Missing Javadoc: `AvoidanceAlgorithm, GridNode`)
- **Public/Protected Methods (10)**: 1/10 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `setAlgorithm()`, `getAlgorithm()`, `registerObstacle()`, `registerObstacle()`, `registerObstacle()`, `clearObstacles()`, `getActiveObstacles()`, `fCost()`, ... (+1 more)
- **Constants (17)**: 0/17 have Javadoc, 12/17 lack explicit units
  - *Constants Lacking Units*: `activeObstacles`, `REPULSION_STRENGTH`, `GRID_RESOLUTION`, `GRID_COLS`, `GRID_ROWS`, `STATIC_BLOCKED_GRID`, ... (+6 more)
- **Smells/Issues (3)**:
  - Line 380: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'gCost': public double gCost;
  - Line 381: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'hCost': public double hCost;
  - Line 382: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'parent': public GridNode parent;
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AvoidanceAlgorithm, GridNode; Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods; Add formal Javadoc comments to 17 constants; Document physical engineering units on 12 constants or add unit suffixes; Encapsulate public mutable fields with private access and getters/setters.

#### 13. `Auto\LegalPinningWatchdog.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (8)**: 4/8 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `getInstance()`, `getPinDuration()`, `getBackoffRemainingSec()`, `reset()`
- **Constants (4)**: 0/4 have Javadoc, 0/4 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods; Add formal Javadoc comments to 4 constants.

#### 14. `Auto\SmartTunnelRouter.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (3)**: 1/3 have Javadoc (Missing Javadoc: `TrenchCorridor, TunnelRoute`)
- **Public/Protected Methods (6)**: 3/6 have Javadoc (3 missing)
  - *Missing Method Javadoc*: `getWaypoints()`, `isDiverted()`, `isWestToEast()`
- **Constants (10)**: 0/10 have Javadoc, 10/10 lack explicit units
  - *Constants Lacking Units*: `BLUE_TRENCH_X_MIN`, `BLUE_TRENCH_X_MAX`, `RED_TRENCH_X_MIN`, `RED_TRENCH_X_MAX`, `TOP_TRENCH_Y_MIN`, `TOP_TRENCH_Y_MAX`, ... (+4 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TrenchCorridor, TunnelRoute; Add descriptive Javadocs with `@param` and `@return` to 3 public/protected methods; Add formal Javadoc comments to 10 constants; Document physical engineering units on 10 constants or add unit suffixes.

#### 15. `Auto\StaticPathfinder.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (7)**: 1/7 have Javadoc (Missing Javadoc: `Obstacle, CircularObstacle, RectangularObstacle, AABB, RoadmapNode, NodeRecord`)
- **Public/Protected Methods (16)**: 7/16 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `isBlocking()`, `getCenter()`, `getSafeRadius()`, `isBlocking()`, `getCenter()`, `getSafeRadius()`, `clampToField()`, `findNearestClearPoint()`, ... (+1 more)
- **Constants (39)**: 0/39 have Javadoc, 38/39 lack explicit units
  - *Constants Lacking Units*: `FIELD_LENGTH`, `FIELD_WIDTH`, `STATIC_OBSTACLES`, `NODES`, `N_BLUE_ALLIANCE_CTR`, `N_BLUE_ALLIANCE_TOP`, ... (+32 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Obstacle, CircularObstacle, RectangularObstacle, AABB, RoadmapNode, NodeRecord; Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods; Add formal Javadoc comments to 39 constants; Document physical engineering units on 38 constants or add unit suffixes.

#### 16. `Auto\TrajectoryController.java`
- **Package**: `frc.robot.Auto`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (7)**: 4/7 have Javadoc (3 missing)
  - *Missing Method Javadoc*: `getWaypoints()`, `getCurrentWaypointIndex()`, `isStalledActive()`
- **Constants (2)**: 0/2 have Javadoc, 2/2 lack explicit units
  - *Constants Lacking Units*: `MAX_ACCELERATION_MPS2`, `MAX_DECELERATION_MPS2`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 3 public/protected methods; Add formal Javadoc comments to 2 constants; Document physical engineering units on 2 constants or add unit suffixes.

#### 17. `Auto\Actions\AutoAimAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `AutoAimAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AutoAimAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 18. `Auto\Actions\BallHuntAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (7)**: 2/7 have Javadoc (5 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isTargetLocked()`, `isFinished()`, `done()`
- **Constants (3)**: 0/3 have Javadoc, 2/3 lack explicit units
  - *Constants Lacking Units*: `MAX_PURSUIT_SPEED`, `MIN_INGESTION_SPEED`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 5 public/protected methods; Add formal Javadoc comments to 3 constants; Document physical engineering units on 2 constants or add unit suffixes.

#### 19. `Auto\Actions\DriveToPoseAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `DriveToPoseAction`)
- **Public/Protected Methods (12)**: 0/12 have Javadoc (12 missing)
  - *Missing Method Javadoc*: `setDriverInput()`, `isBreakoutRequested()`, `start()`, `update()`, `isFinished()`, `done()`, `isTunnelTransit()`, `getTunnelHeading()`, ... (+4 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to DriveToPoseAction; Add descriptive Javadocs with `@param` and `@return` to 12 public/protected methods.

#### 20. `Auto\Actions\FollowChoreoPath.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `FollowChoreoPath`)
- **Public/Protected Methods (10)**: 3/10 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `start()`, `hasMarkerBeenPassed()`, `update()`, `isFinished()`, `done()`, `setPaused()`, `isPaused()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to FollowChoreoPath; Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods.

#### 21. `Auto\Actions\IntakeAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `IntakeAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to IntakeAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 22. `Auto\Actions\LambdaAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 23. `Auto\Actions\ParallelAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ParallelAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ParallelAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 24. `Auto\Actions\ParallelRaceAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ParallelRaceAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ParallelRaceAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 25. `Auto\Actions\SeriesAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 26. `Auto\Actions\ShootAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ShootAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ShootAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 27. `Auto\Actions\WaitAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `WaitAction`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to WaitAction; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 28. `Auto\Actions\WaitUntilMarkerAction.java`
- **Package**: `frc.robot.Auto.Actions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `start()`, `update()`, `isFinished()`, `done()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 29. `Auto\Missions\AdvancedChoreoMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 30. `Auto\Missions\DepotShootMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `DepotShootMission`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to DepotShootMission; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 31. `Auto\Missions\DoNothingMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `DoNothingMission`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (1)**:
  - Line 14: `[CONSOLE_IO]` System.out.println("Do nothing auto mission");
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to DoNothingMission; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 32. `Auto\Missions\DynamicChoreoMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 33. `Auto\Missions\ExampleMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ExampleMission`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ExampleMission; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 34. `Auto\Missions\MissionBase.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `MissionBase`)
- **Public/Protected Methods (11)**: 0/11 have Javadoc (11 missing)
  - *Missing Method Javadoc*: `routine()`, `setStartPose()`, `run()`, `done()`, `stop()`, `isActive()`, `isActiveWithThrow()`, `interrupt()`, ... (+3 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (7)**:
  - Line 45: `[CONSOLE_IO]` System.out.println("Auto mission done");
  - Line 65: `[CONSOLE_IO]` System.out.println("** Auto mission interrrupted!");
  - Line 70: `[CONSOLE_IO]` System.out.println("** Auto mission resumed!");
  - Line 80: `[THREAD_SLEEP]` Thread.sleep(waitTime);
  - Line 83: `[PRINT_STACK_TRACE]` e.printStackTrace();
  - ... (+2 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to MissionBase; Add descriptive Javadocs with `@param` and `@return` to 11 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry; Eliminate Thread.sleep and migrate to non-blocking WPILib command loops.

#### 35. `Auto\Missions\ShooterMission.java`
- **Package**: `frc.robot.Auto.Missions`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ShooterMission`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `routine()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ShooterMission; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 36. `Data\Constants.java`
- **Package**: `frc.robot.Data`
- **Types Declared (9)**: 1/9 have Javadoc (Missing Javadoc: `Mode, AutonConstants, DrivebaseConstants, OperatorConstants, LEDConstants, FieldConstants, ShooterConstants, IntakeConstants`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `getMode()`
- **Constants (147)**: 1/147 have Javadoc, 128/147 lack explicit units
  - *Constants Lacking Units*: `currentMode`, `TUNING_MODE`, `CHASSIS`, `MAX_SPEED`, `MAX_ROTATION_SPEED`, `MAX_SPEED_MEASURE`, ... (+122 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Mode, AutonConstants, DrivebaseConstants, OperatorConstants, LEDConstants, FieldConstants, ShooterConstants, IntakeConstants; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods; Add formal Javadoc comments to 146 constants; Document physical engineering units on 128 constants or add unit suffixes.

#### 37. `Data\FieldMap.java`
- **Package**: `frc.robot.Data`
- **Types Declared (9)**: 1/9 have Javadoc (Missing Javadoc: `Hubs, Ramps, Trenches, AllianceZones, Depots, ClimbingTowers, AABB, Obstacles`)
- **Public/Protected Methods (20)**: 2/20 have Javadoc (18 missing)
  - *Missing Method Javadoc*: `getHubLocation3d()`, `getHubLocation2d()`, `isLowClearance()`, `isInAllianceZone()`, `isInAllianceZone()`, `isInMidfield()`, `getDepotLoadPoint()`, `getDepotApproach()`, ... (+10 more)
- **Constants (53)**: 0/53 have Javadoc, 48/53 lack explicit units
  - *Constants Lacking Units*: `FIELD_LENGTH`, `FIELD_WIDTH`, `CENTERLINE_X`, `ROBOT_RADIUS`, `HUB_RADIUS`, `FUNNEL_TARGET_Z`, ... (+42 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Hubs, Ramps, Trenches, AllianceZones, Depots, ClimbingTowers, AABB, Obstacles; Add descriptive Javadocs with `@param` and `@return` to 18 public/protected methods; Add formal Javadoc comments to 53 constants; Document physical engineering units on 48 constants or add unit suffixes.

#### 38. `Data\GlideConstants.java`
- **Package**: `frc.robot.Data`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `GlidePoint`)
- **Public/Protected Methods (4)**: 2/4 have Javadoc (2 missing)
  - *Missing Method Javadoc*: `pose()`, `name()`
- **Constants (5)**: 0/5 have Javadoc, 5/5 lack explicit units
  - *Constants Lacking Units*: `FIELD_LENGTH`, `FIELD_WIDTH`, `Y_BOT_LANE`, `Y_TOP_LANE`, `RED_GLIDE_POINTS`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to GlidePoint; Add descriptive Javadocs with `@param` and `@return` to 2 public/protected methods; Add formal Javadoc comments to 5 constants; Document physical engineering units on 5 constants or add unit suffixes.

#### 39. `Data\PortMap.java`
- **Package**: `frc.robot.Data`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `PortMap`)
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (9)**: 0/9 have Javadoc, 9/9 lack explicit units
  - *Constants Lacking Units*: `DRIVER_CONTROLLER`, `OPERATOR_CONTROLLER`, `INTAKE_ARM_MOTOR_ID`, `INTAKE_WHEELS_MOTOR_ID`, `HOPPER_MOTOR_CANID`, `INTAKE_ENCODER_ID`, ... (+3 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to PortMap; Add formal Javadoc comments to 9 constants; Document physical engineering units on 9 constants or add unit suffixes.

#### 40. `Data\TunableNumber.java`
- **Package**: `frc.robot.Data`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (5)**: 5/5 have Javadoc (0 missing)
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `tableKey`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 41. `Devices\Controller.java`
- **Package**: `frc.robot.Devices`
- **Types Declared (2)**: 0/2 have Javadoc (Missing Javadoc: `Controller, RumblePattern`)
- **Public/Protected Methods (13)**: 4/13 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `setRumble()`, `getDebouncedButton()`, `getDebouncedButton()`, `getLeftX()`, `getLeftY()`, `getRightX()`, `getRightY()`, `isOperational()`, ... (+1 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Controller, RumblePattern; Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods.

#### 42. `Devices\NeoSparkMaxMotor.java`
- **Package**: `frc.robot.Devices`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `NeoSparkMaxMotor`)
- **Public/Protected Methods (18)**: 0/18 have Javadoc (18 missing)
  - *Missing Method Javadoc*: `configure()`, `set()`, `setVoltage()`, `setSpeed()`, `getSpeed()`, `getVelocity()`, `getPosition()`, `setInverted()`, ... (+10 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (1)**:
  - Line 39: `[CONSOLE_IO]` System.out.println("SparkMax error initializing CANID: " + CANID);
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to NeoSparkMaxMotor; Add descriptive Javadocs with `@param` and `@return` to 18 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 43. `Interfaces\Actions.java`
- **Package**: `frc.robot.Interfaces`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `Actions`)
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Actions.

#### 44. `Interfaces\Subsystem.java`
- **Package**: `frc.robot.Interfaces`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `Subsystem`)
- **Public/Protected Methods (8)**: 1/8 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `update()`, `initialize()`, `log()`, `isEnabled()`, `simulationUpdate()`, `getSimulationCurrentDraw()`, `getName()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Subsystem; Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods.

#### 45. `Sim\AIActionIntent.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

#### 46. `Sim\AIRobotInstance.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (21)**: 1/21 have Javadoc (20 missing)
  - *Missing Method Javadoc*: `getBotId()`, `isAlly()`, `getArchetype()`, `setArchetype()`, `getDriveSimulation()`, `getIntakeSimulation()`, `getActualPose()`, `setRobotPose()`, ... (+12 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (3)**:
  - Line 90: `[CONSOLE_IO]` System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Could not
  - Line 103: `[CONSOLE_IO]` System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Could not
  - Line 395: `[CONSOLE_IO]` System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Error lau
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 20 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 47. `Sim\AIRobotSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (3)**: 0/3 have Javadoc (Missing Javadoc: `AIRobotSim, AIMode, CyclerPhase`)
- **Public/Protected Methods (54)**: 3/54 have Javadoc (51 missing)
  - *Missing Method Javadoc*: `fromString()`, `getInstance()`, `setTrajectory()`, `setRobotPose()`, `reset()`, `simulationUpdate()`, `computeDriveToPoseSpeeds()`, `isStalled()`, ... (+43 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (7)**:
  - Line 176: `[CONSOLE_IO]` System.err.println("[AIRobotSim] Could not attach IntakeSimulation: " + e.getMessage());
  - Line 300: `[CONSOLE_IO]` System.err.println("[AIRobotSim] Failed to spawn additional opponent bot: " + e.getMessage
  - Line 301: `[PRINT_STACK_TRACE]` e.printStackTrace();
  - Line 313: `[CONSOLE_IO]` System.err.println("[AIRobotSim] Failed to spawn ally bot: " + e.getMessage());
  - Line 314: `[PRINT_STACK_TRACE]` e.printStackTrace();
  - ... (+2 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AIRobotSim, AIMode, CyclerPhase; Add descriptive Javadocs with `@param` and `@return` to 51 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 48. `Sim\Archetype.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (3)**: 0/3 have Javadoc (3 missing)
  - *Missing Method Javadoc*: `isDefensive()`, `isOffensive()`, `fromString()`
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `PINNING_BULLY`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 3 public/protected methods; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 49. `Sim\ArmSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ArmSim`)
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `update()`, `getAngleRads()`, `getVelocityRadsPerSec()`, `getCurrentDrawAmps()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ArmSim; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 50. `Sim\GameSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `Config`)
- **Public/Protected Methods (12)**: 3/12 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `getInstance()`, `getSimTimeRemainingSec()`, `setSimTimeRemainingSec()`, `getHeldBalls()`, `update()`, `initialize()`, `log()`, `isEnabled()`, ... (+1 more)
- **Constants (30)**: 0/30 have Javadoc, 25/30 lack explicit units
  - *Constants Lacking Units*: `MAX_HELD_BALLS`, `PICKUP_PER_CHECK_LIMIT`, `MIN_RESPAWN_INTERVAL`, `PICKUP_CHECK_INTERVAL`, `CENTER_HALF_X_MIN`, `CENTER_HALF_X_MAX`, ... (+19 more)
- **Smells/Issues (11)**:
  - Line 152: `[CONSOLE_IO]` System.err.println("GameSim: Invalid maxToConsume value: " + maxToConsume);
  - Line 212: `[CONSOLE_IO]` System.err.println("GameSim: Error in simulationUpdate: " + e.getMessage());
  - Line 213: `[PRINT_STACK_TRACE]` e.printStackTrace();
  - Line 234: `[CONSOLE_IO]` System.err.println("GameSim: Error handling dashboard commands: " + e.getMessage());
  - Line 265: `[CONSOLE_IO]` System.err.println("GameSim: Error updating simulation time: " + e.getMessage());
  - ... (+6 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to Config; Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods; Add formal Javadoc comments to 30 constants; Document physical engineering units on 25 constants or add unit suffixes; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 51. `Sim\JevDecisionEngine.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (6)**: 1/6 have Javadoc (Missing Javadoc: `AIArchetype, TacticalAction, DecisionResult, OffensiveStrategy, StrategicAdvice`)
- **Public/Protected Methods (7)**: 3/7 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `toSchemaJson()`, `getInstance()`, `getLastDecision()`, `calculatePolarStandoffPose()`
- **Constants (4)**: 0/4 have Javadoc, 4/4 lack explicit units
  - *Constants Lacking Units*: `BLUE_HUB_POS`, `BLUE_DEPOT_POS`, `CENTERLINE_X`, `RETREAT_X`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AIArchetype, TacticalAction, DecisionResult, OffensiveStrategy, StrategicAdvice; Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods; Add formal Javadoc comments to 4 constants; Document physical engineering units on 4 constants or add unit suffixes.

#### 52. `Sim\LimelightSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `LimelightSim`)
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `update()`
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `LL_TABLE`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to LimelightSim; Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 53. `Sim\MatchScoreTracker.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (48)**: 8/48 have Javadoc (40 missing)
  - *Missing Method Javadoc*: `getInstance()`, `getRedFuelScore()`, `getBlueFuelScore()`, `getRedClimbScore()`, `getBlueClimbScore()`, `getRedTotalScore()`, `getBlueTotalScore()`, `getPlayerScore()`, ... (+32 more)
- **Constants (5)**: 0/5 have Javadoc, 4/5 lack explicit units
  - *Constants Lacking Units*: `POINTS_PER_FUEL`, `POINTS_PER_CLIMB`, `FUEL_RP_THRESHOLD`, `CLIMB_RP_ROBOT_COUNT`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 40 public/protected methods; Add formal Javadoc comments to 5 constants; Document physical engineering units on 4 constants or add unit suffixes.

#### 54. `Sim\ShooterSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `ShooterSim`)
- **Public/Protected Methods (13)**: 2/13 have Javadoc (11 missing)
  - *Missing Method Javadoc*: `update()`, `update()`, `getVelocityRPM()`, `getLeftVelocityRPM()`, `getRightVelocityRPM()`, `getCurrentDrawAmps()`, `getLeftCurrentDrawAmps()`, `getRightCurrentDrawAmps()`, ... (+3 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ShooterSim; Add descriptive Javadocs with `@param` and `@return` to 11 public/protected methods.

#### 55. `Sim\StrategicObjective.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (2)**: 0/2 have Javadoc (2 missing)
  - *Missing Method Javadoc*: `isOffensive()`, `isDefensive()`
- **Constants (8)**: 0/8 have Javadoc, 8/8 lack explicit units
  - *Constants Lacking Units*: `HARVEST_FEEDER`, `HARVEST_FIELD_PIECES`, `HARVEST_FIELD_FUEL`, `SCORE_HUB`, `SCORE_GOAL`, `STAGE_SCORING_WINDOW`, ... (+2 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 2 public/protected methods; Add formal Javadoc comments to 8 constants; Document physical engineering units on 8 constants or add unit suffixes.

#### 56. `Sim\VisionSim.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (10)**: 2/10 have Javadoc (8 missing)
  - *Missing Method Javadoc*: `getInstance()`, `hasSimulatedNeuralTarget()`, `getSimulatedNeuralYaw()`, `getSimulatedNeuralPitch()`, `getSimulatedNeuralArea()`, `getSimulatedNeuralDistance()`, `getVisionSystemSim()`, `getCameraSim()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 8 public/protected methods.

#### 57. `Sim\WorldState.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (7)**: 0/7 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `isSelfHubActive()`, `isInventoryFull()`, `heldGamePieceCount()`, `isAllianceGoalActive()`, `isOpponentGoalActive()`, `timeUntilGoalShift()`, `inventoryRatio()`
- **Constants (2)**: 0/2 have Javadoc, 2/2 lack explicit units
  - *Constants Lacking Units*: `DEFAULT_MAX_CAPACITY`, `CO_PILOT_CAPACITY`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods; Add formal Javadoc comments to 2 constants; Document physical engineering units on 2 constants or add unit suffixes.

#### 58. `Sim\WorldStateBuilder.java`
- **Package**: `frc.robot.Sim`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (1)**: 1/1 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

#### 59. `Subsystems\Dashboard.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (29)**: 0/29 have Javadoc (29 missing)
  - *Missing Method Javadoc*: `getInstance()`, `isHapticCollisionEnabled()`, `update()`, `log()`, `initialize()`, `isEnabled()`, `getName()`, `getAutoChooser()`, ... (+21 more)
- **Constants (14)**: 0/14 have Javadoc, 14/14 lack explicit units
  - *Constants Lacking Units*: `table`, `snapToTurnSub`, `ballHuntSub`, `glidePointsSub`, `fieldOrientedSub`, `slowModeSub`, ... (+8 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 29 public/protected methods; Add formal Javadoc comments to 14 constants; Document physical engineering units on 14 constants or add unit suffixes.

#### 60. `Subsystems\Intake.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (2)**: 2/2 have Javadoc
- **Public/Protected Methods (52)**: 21/52 have Javadoc (31 missing)
  - *Missing Method Javadoc*: `getStateName()`, `fromString()`, `getArmPosition()`, `getTargetArmPosition()`, `setCharacterizationVoltage()`, `getArmAppliedVoltage()`, `getArmPositionRads()`, `getArmVelocityRads()`, ... (+23 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 31 public/protected methods.

#### 61. `Subsystems\LEDs.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `LEDs`)
- **Public/Protected Methods (8)**: 0/8 have Javadoc (8 missing)
  - *Missing Method Javadoc*: `getInstance()`, `setPattern()`, `initialize()`, `update()`, `simulationUpdate()`, `log()`, `isEnabled()`, `getName()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to LEDs; Add descriptive Javadocs with `@param` and `@return` to 8 public/protected methods.

#### 62. `Subsystems\MatchCoach.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `DrillMode`)
- **Public/Protected Methods (22)**: 2/22 have Javadoc (20 missing)
  - *Missing Method Javadoc*: `fromString()`, `getInstance()`, `initialize()`, `update()`, `getShootingAccuracyPercent()`, `getAverageCycleTimeSec()`, `getLastCycleDurationSec()`, `getFastestCycleSec()`, ... (+12 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to DrillMode; Add descriptive Javadocs with `@param` and `@return` to 20 public/protected methods.

#### 63. `Subsystems\Shooter.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (3)**: 3/3 have Javadoc
- **Public/Protected Methods (62)**: 25/62 have Javadoc (37 missing)
  - *Missing Method Javadoc*: `getStateName()`, `turretAngle()`, `flywheelRPM()`, `possible()`, `getTargetVelocityRPM()`, `getSpeed()`, `getActualRPM()`, `getFlywheelLeftVelocityRPM()`, ... (+29 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (5)**:
  - Line 76: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'targetRpmLeft': public double targetRpmLeft = 0;
  - Line 77: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'targetRpmRight': public double targetRpmRight = 0;
  - Line 86: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'normalDistanceToHub': public double normalDistanceToHub = 0;
  - Line 87: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'leftShooterVoltageCalc': public double leftShooterVoltageCalc = 0;
  - Line 88: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'rightShooterVoltageCalc': public double rightShooterVoltageCalc = 0;
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 37 public/protected methods; Encapsulate public mutable fields with private access and getters/setters.

#### 64. `Subsystems\SubsystemManager.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `SubsystemManager`)
- **Public/Protected Methods (6)**: 3/6 have Javadoc (3 missing)
  - *Missing Method Javadoc*: `updateSubsystems()`, `simulationUpdateSubsystems()`, `logSubsystems()`
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `subsystems`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to SubsystemManager; Add descriptive Javadocs with `@param` and `@return` to 3 public/protected methods; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 65. `Subsystems\SwerveBase.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (1)**: 0/1 have Javadoc (Missing Javadoc: `SwerveBase`)
- **Public/Protected Methods (77)**: 59/77 have Javadoc (18 missing)
  - *Missing Method Javadoc*: `getNearestGlidePoint()`, `drawGlidePointsOnField()`, `isVisionDegraded()`, `update()`, `getCollisionDetector()`, `isCollisionDetected()`, `initialize()`, `log()`, ... (+10 more)
- **Constants (2)**: 0/2 have Javadoc, 1/2 lack explicit units
  - *Constants Lacking Units*: `OFF_FIELD_POSE`
- **Smells/Issues (1)**:
  - Line 120: `[CONSOLE_IO]` System.out.println("SwerveBase: Simulation Mode is " + SwerveDriveTelemetry.isSimulation);
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to SwerveBase; Add descriptive Javadocs with `@param` and `@return` to 18 public/protected methods; Add formal Javadoc comments to 2 constants; Document physical engineering units on 1 constants or add unit suffixes; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 66. `Subsystems\Vision.java`
- **Package**: `frc.robot.Subsystems`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `VisionTargetEstimate`)
- **Public/Protected Methods (31)**: 10/31 have Javadoc (21 missing)
  - *Missing Method Javadoc*: `getInstance()`, `update()`, `hasTarget()`, `hasGamePiece()`, `getGamePieceYaw()`, `getGamePiecePitch()`, `getGamePieceArea()`, `getTX()`, ... (+13 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (3)**:
  - Line 340: `[CONSOLE_IO]` System.out.println("[Vision] Limelight enabled: " + enabled);
  - Line 347: `[CONSOLE_IO]` System.out.println("[Vision] Limelight LED mode: " + mode);
  - Line 354: `[CONSOLE_IO]` System.out.println("[Vision] PhotonVision pipeline: " + pipeline);
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to VisionTargetEstimate; Add descriptive Javadocs with `@param` and `@return` to 21 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 67. `Subsystems\drive\DriveIO.java`
- **Package**: `frc.robot.Subsystems.drive`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `DriveIOInputs`)
- **Public/Protected Methods (7)**: 7/7 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (16)**:
  - Line 17: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'drivePositionsMeters': public double[] drivePositionsMeters = new do
  - Line 18: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'driveVelocitiesMetersPerSec': public double[] driveVelocitiesMetersP
  - Line 19: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'driveAppliedVolts': public double[] driveAppliedVolts = new double[4
  - Line 20: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'driveCurrentAmps': public double[] driveCurrentAmps = new double[4];
  - Line 23: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'steerPositionsDeg': public double[] steerPositionsDeg = new double[4
  - ... (+11 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to DriveIOInputs; Encapsulate public mutable fields with private access and getters/setters.

#### 68. `Subsystems\drive\DriveIOSim.java`
- **Package**: `frc.robot.Subsystems.drive`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (6)**: 0/6 have Javadoc (6 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `setModuleDriveVoltage()`, `setModuleAngleVoltage()`, `setChassisSpeeds()`, `zeroGyro()`, `setPose()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 6 public/protected methods.

#### 69. `Subsystems\drive\DriveIOSparkMax.java`
- **Package**: `frc.robot.Subsystems.drive`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (6)**: 0/6 have Javadoc (6 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `setModuleDriveVoltage()`, `setModuleAngleVoltage()`, `setChassisSpeeds()`, `zeroGyro()`, `setPose()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 6 public/protected methods.

#### 70. `Subsystems\intake\IntakeConstants.java`
- **Package**: `frc.robot.Subsystems.intake`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (39)**: 0/39 have Javadoc, 31/39 lack explicit units
  - *Constants Lacking Units*: `INTAKE_ARM_INVERTED`, `INTAKE_WHEELS_INVERTED`, `INTAKE_POSITION_OFFSET`, `INTAKE_UP_POSITION`, `INTAKE_DOWN_POSITION`, `INTAKE_HORIZONTAL_POSITION`, ... (+25 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 39 constants; Document physical engineering units on 31 constants or add unit suffixes.

#### 71. `Subsystems\intake\IntakeIO.java`
- **Package**: `frc.robot.Subsystems.intake`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `IntakeIOInputs`)
- **Public/Protected Methods (7)**: 7/7 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (12)**:
  - Line 13: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'armPositionDeg': public double armPositionDeg = 0.0;
  - Line 14: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'armVelocityDegPerSec': public double armVelocityDegPerSec = 0.0;
  - Line 15: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'armAppliedVolts': public double armAppliedVolts = 0.0;
  - Line 16: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'armCurrentAmps': public double armCurrentAmps = 0.0;
  - Line 17: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'armMotorRotations': public double armMotorRotations = 0.0;
  - ... (+7 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to IntakeIOInputs; Encapsulate public mutable fields with private access and getters/setters.

#### 72. `Subsystems\intake\IntakeIOSim.java`
- **Package**: `frc.robot.Subsystems.intake`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (10)**: 1/10 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `getMapleIntakeSim()`, `getArmSim()`, `updateInputs()`, `setArmVoltage()`, `setRollerVoltage()`, `setRollerSpeed()`, `setHopperVoltage()`, `setHopperSpeed()`, ... (+1 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods.

#### 73. `Subsystems\intake\IntakeIOSparkMax.java`
- **Package**: `frc.robot.Subsystems.intake`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (7)**: 0/7 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `setArmVoltage()`, `setRollerVoltage()`, `setRollerSpeed()`, `setHopperVoltage()`, `setHopperSpeed()`, `stop()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods.

#### 74. `Subsystems\shooter\ShooterConstants.java`
- **Package**: `frc.robot.Subsystems.shooter`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (0)**: 0/0 have Javadoc (0 missing)
- **Constants (37)**: 7/37 have Javadoc, 20/37 lack explicit units
  - *Constants Lacking Units*: `FIRING_ANGLE`, `HEIGHT_DIFFERENCE`, `FEED_SPEED`, `RPM_TOLERANCE`, `FLYWHEEL_KI_VAL`, `FLYWHEEL_KD_VAL`, ... (+14 more)
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 30 constants; Document physical engineering units on 20 constants or add unit suffixes.

#### 75. `Subsystems\shooter\ShooterIO.java`
- **Package**: `frc.robot.Subsystems.shooter`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `ShooterIOInputs`)
- **Public/Protected Methods (4)**: 4/4 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (11)**:
  - Line 13: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'leftVelocityRPM': public double leftVelocityRPM = 0.0;
  - Line 14: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'rightVelocityRPM': public double rightVelocityRPM = 0.0;
  - Line 15: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'leftAppliedVolts': public double leftAppliedVolts = 0.0;
  - Line 16: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'rightAppliedVolts': public double rightAppliedVolts = 0.0;
  - Line 17: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'kickerAppliedVolts': public double kickerAppliedVolts = 0.0;
  - ... (+6 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to ShooterIOInputs; Encapsulate public mutable fields with private access and getters/setters.

#### 76. `Subsystems\shooter\ShooterIOSim.java`
- **Package**: `frc.robot.Subsystems.shooter`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (5)**: 0/5 have Javadoc (5 missing)
  - *Missing Method Javadoc*: `getShooterSim()`, `updateInputs()`, `setFlywheelVoltages()`, `setKickerVoltage()`, `stop()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 5 public/protected methods.

#### 77. `Subsystems\shooter\ShooterIOSparkMax.java`
- **Package**: `frc.robot.Subsystems.shooter`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (4)**: 0/4 have Javadoc (4 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `setFlywheelVoltages()`, `setKickerVoltage()`, `stop()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 4 public/protected methods.

#### 78. `Subsystems\vision\VisionIO.java`
- **Package**: `frc.robot.Subsystems.vision`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `VisionIOInputs`)
- **Public/Protected Methods (2)**: 2/2 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (13)**:
  - Line 14: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'hasTarget': public boolean hasTarget = false;
  - Line 15: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'tagCount': public int tagCount = 0;
  - Line 16: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'avgTagDist': public double avgTagDist = 0.0;
  - Line 17: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'timestamp': public double timestamp = 0.0;
  - Line 18: `[PUBLIC_MUTABLE_FIELD]` Public mutable field 'latencyMs': public double latencyMs = 0.0;
  - ... (+8 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to VisionIOInputs; Encapsulate public mutable fields with private access and getters/setters.

#### 79. `Subsystems\vision\VisionIOLimelight.java`
- **Package**: `frc.robot.Subsystems.vision`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (2)**: 0/2 have Javadoc (2 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `setRobotOrientation()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 2 public/protected methods.

#### 80. `Subsystems\vision\VisionIOPhotonVision.java`
- **Package**: `frc.robot.Subsystems.vision`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (2)**: 0/2 have Javadoc (2 missing)
  - *Missing Method Javadoc*: `updateInputs()`, `getCameraName()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 2 public/protected methods.

#### 81. `Subsystems\vision\VisionIOSim.java`
- **Package**: `frc.robot.Subsystems.vision`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `CameraType`)
- **Public/Protected Methods (6)**: 0/6 have Javadoc (6 missing)
  - *Missing Method Javadoc*: `setSimulatedPose()`, `setRobotOrientation()`, `setGamePieceDetected()`, `clearManualGamePieceOverride()`, `updateInputs()`, `getCameraType()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to CameraType; Add descriptive Javadocs with `@param` and `@return` to 6 public/protected methods.

#### 82. `Test\Diagnostics.java`
- **Package**: `frc.robot.Test`
- **Types Declared (3)**: 2/3 have Javadoc (Missing Javadoc: `PreFlightStep`)
- **Public/Protected Methods (17)**: 3/17 have Javadoc (14 missing)
  - *Missing Method Javadoc*: `getInstance()`, `resetScorecard()`, `registerTests()`, `update()`, `log()`, `initialize()`, `getName()`, `isEnabled()`, ... (+6 more)
- **Constants (3)**: 0/3 have Javadoc, 1/3 lack explicit units
  - *Constants Lacking Units*: `MODULE_NAMES`
- **Smells/Issues (9)**:
  - Line 350: `[CONSOLE_IO]` System.out.println("[Diagnostics] =========================================");
  - Line 351: `[CONSOLE_IO]` System.out.println("[Diagnostics] PRE-FLIGHT PIT CHECK RESULTS:");
  - Line 352: `[CONSOLE_IO]` scorecard.forEach((k, v) -> System.out.printf("[Diagnostics]   %-18s: %s%n", k, v));
  - Line 353: `[CONSOLE_IO]` System.out.println("[Diagnostics] =========================================");
  - Line 365: `[CONSOLE_IO]` System.out.printf(
  - ... (+4 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to PreFlightStep; Add descriptive Javadocs with `@param` and `@return` to 14 public/protected methods; Add formal Javadoc comments to 3 constants; Document physical engineering units on 1 constants or add unit suffixes; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 83. `Test\DriveCharacterization.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `TestMode`)
- **Public/Protected Methods (5)**: 5/5 have Javadoc (0 missing)
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `MODULE_NAMES`
- **Smells/Issues (15)**:
  - Line 144: `[CONSOLE_IO]` System.out.println("[DriveCharacterization] Mode: SYSID_QUASISTATIC");
  - Line 148: `[CONSOLE_IO]` System.out.println("[DriveCharacterization] Mode: SYSID_DYNAMIC");
  - Line 152: `[CONSOLE_IO]` System.out.println("[DriveCharacterization] Mode: MODULE_INDIVIDUAL");
  - Line 156: `[CONSOLE_IO]` System.out.println("[DriveCharacterization] Mode: KINEMATICS_TEST");
  - Line 163: `[CONSOLE_IO]` System.out.println("[DriveCharacterization] Mode: ODOMETRY_TEST");
  - ... (+10 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TestMode; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 84. `Test\IntakeTesting.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `TestMode`)
- **Public/Protected Methods (5)**: 5/5 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (7)**:
  - Line 83: `[CONSOLE_IO]` System.out.println("[IntakeTesting] Mode: ARM_CONTROL");
  - Line 87: `[CONSOLE_IO]` System.out.println("[IntakeTesting] Mode: ROLLER_TESTING");
  - Line 91: `[CONSOLE_IO]` System.out.println("[IntakeTesting] Mode: HOPPER_TESTING");
  - Line 95: `[CONSOLE_IO]` System.out.println("[IntakeTesting] Mode: JAM_DETECTION");
  - Line 102: `[CONSOLE_IO]` System.out.println("[IntakeTesting] Mode: POSITION_CALIBRATION");
  - ... (+2 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TestMode; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 85. `Test\ShooterTuning.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `TestMode`)
- **Public/Protected Methods (5)**: 5/5 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (5)**:
  - Line 87: `[CONSOLE_IO]` System.out.println("[ShooterTuning] Mode: MANUAL_VELOCITY");
  - Line 91: `[CONSOLE_IO]` System.out.println("[ShooterTuning] Mode: AUTO_AIM_TEST");
  - Line 95: `[CONSOLE_IO]` System.out.println("[ShooterTuning] Mode: PID_TUNING");
  - Line 99: `[CONSOLE_IO]` System.out.println("[ShooterTuning] Mode: CHARACTERIZATION");
  - Line 193: `[CONSOLE_IO]` System.out.println("[ShooterTuning] No shooting solution available for test pose");
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TestMode; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 86. `Test\SysID.java`
- **Package**: `frc.robot.Test`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (1)**: 0/1 have Javadoc (1 missing)
  - *Missing Method Javadoc*: `runTest()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 1 public/protected methods.

#### 87. `Test\SysIdManager.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `MechanismType`)
- **Public/Protected Methods (12)**: 3/12 have Javadoc (9 missing)
  - *Missing Method Javadoc*: `getInstance()`, `getActiveRoutine()`, `setActiveMechanism()`, `getActiveMechanism()`, `isRunning()`, `getRoutineState()`, `startDynamic()`, `setupDashboard()`, ... (+1 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (3)**:
  - Line 237: `[CONSOLE_IO]` System.out.println("[SysIdManager] Switched active mechanism to: " + mechanism.displayName
  - Line 263: `[CONSOLE_IO]` System.out.println("[SysIdManager] Scheduled Quasistatic " + direction.name() + " for " + 
  - Line 271: `[CONSOLE_IO]` System.out.println("[SysIdManager] Scheduled Dynamic " + direction.name() + " for " + acti
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to MechanismType; Add descriptive Javadocs with `@param` and `@return` to 9 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 88. `Test\TestMode.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `TestCategory`)
- **Public/Protected Methods (8)**: 2/8 have Javadoc (6 missing)
  - *Missing Method Javadoc*: `getInstance()`, `setupDashboard()`, `updateDashboard()`, `getActiveCategory()`, `isEnabled()`, `cleanup()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (3)**:
  - Line 119: `[CONSOLE_IO]` System.out.println("[TestMode] ENABLED - Category: " + activeCategory.name());
  - Line 128: `[CONSOLE_IO]` System.out.println("[TestMode] DISABLED");
  - Line 159: `[CONSOLE_IO]` System.out.println("[TestMode] Category switched via Dashboard: " + activeCategory.name())
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TestCategory; Add descriptive Javadocs with `@param` and `@return` to 6 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 89. `Test\VisionTesting.java`
- **Package**: `frc.robot.Test`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `TestMode`)
- **Public/Protected Methods (6)**: 6/6 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (6)**:
  - Line 92: `[CONSOLE_IO]` System.out.println("[VisionTesting] Mode: LIMELIGHT_TESTING");
  - Line 96: `[CONSOLE_IO]` System.out.println("[VisionTesting] Mode: PHOTONVISION_TESTING");
  - Line 100: `[CONSOLE_IO]` System.out.println("[VisionTesting] Mode: TARGET_TRACKING");
  - Line 104: `[CONSOLE_IO]` System.out.println("[VisionTesting] Mode: VISION_ODOMETRY");
  - Line 111: `[CONSOLE_IO]` System.out.println("[VisionTesting] Mode: CAMERA_CALIBRATION");
  - ... (+1 more smells)
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to TestMode; Replace console prints with AdvantageKit Logger or DriverStation telemetry.

#### 90. `ThirdParty\LimelightHelpers.java`
- **Package**: `frc.robot.ThirdParty`
- **Types Declared (11)**: 11/11 have Javadoc
- **Public/Protected Methods (95)**: 55/95 have Javadoc (40 missing)
  - *Missing Method Javadoc*: `getFamily()`, `getBotPose3d()`, `getBotPose3d_wpiRed()`, `getBotPose3d_wpiBlue()`, `getBotPose2d()`, `getBotPose2d_wpiRed()`, `getBotPose2d_wpiBlue()`, `equals()`, ... (+32 more)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues (132)**:
  - Line 666: `[CONSOLE_IO]` //System.err.println("Bad LL 3D Pose Data!");
  - Line 685: `[CONSOLE_IO]` //System.err.println("Bad LL 2D Pose Data!");
  - Line 863: `[CONSOLE_IO]` System.out.println("No PoseEstimate available.");
  - Line 867: `[CONSOLE_IO]` System.out.printf("Pose Estimate Information:%n");
  - Line 868: `[CONSOLE_IO]` System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
  - ... (+127 more smells)
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 40 public/protected methods; Replace console prints with AdvantageKit Logger or DriverStation telemetry; Encapsulate public mutable fields with private access and getters/setters.

#### 91. `Utils\Alert.java`
- **Package**: `frc.robot.Utils`
- **Types Declared (2)**: 1/2 have Javadoc (Missing Javadoc: `AlertType`)
- **Public/Protected Methods (7)**: 0/7 have Javadoc (7 missing)
  - *Missing Method Javadoc*: `set()`, `setText()`, `getText()`, `getGroup()`, `getType()`, `isActive()`, `getActiveStartTime()`
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add class-level Javadoc to AlertType; Add descriptive Javadocs with `@param` and `@return` to 7 public/protected methods.

#### 92. `Utils\AlertManager.java`
- **Package**: `frc.robot.Utils`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (7)**: 1/7 have Javadoc (6 missing)
  - *Missing Method Javadoc*: `register()`, `getActiveAlerts()`, `getHighestSeverity()`, `hasActiveErrors()`, `hasActiveWarnings()`, `update()`
- **Constants (1)**: 0/1 have Javadoc, 1/1 lack explicit units
  - *Constants Lacking Units*: `alerts`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add descriptive Javadocs with `@param` and `@return` to 6 public/protected methods; Add formal Javadoc comments to 1 constants; Document physical engineering units on 1 constants or add unit suffixes.

#### 93. `Utils\AllianceFlipUtil.java`
- **Package**: `frc.robot.Utils`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (13)**: 13/13 have Javadoc (0 missing)
- **Constants (2)**: 0/2 have Javadoc, 2/2 lack explicit units
  - *Constants Lacking Units*: `FIELD_LENGTH`, `FIELD_WIDTH`
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Add formal Javadoc comments to 2 constants; Document physical engineering units on 2 constants or add unit suffixes.

#### 94. `Utils\Vector2dSlewRateLimiter.java`
- **Package**: `frc.robot.Utils`
- **Types Declared (1)**: 1/1 have Javadoc
- **Public/Protected Methods (8)**: 8/8 have Javadoc (0 missing)
- **Constants (0)**: 0/0 have Javadoc, 0/0 lack explicit units
- **Smells/Issues**: None detected.
- **Required Cleanup & Documentation Action**: Verify unit documentation and formatting compliance.

## 2. Logic Chain

1. **Observation**: `AutoMissionExecutor.java:16` spawns autonomous routines inside a detached raw Java thread (`mThread = new Thread(...)`), and `MissionBase.java:90-99` runs a polling `while` loop calling `Thread.sleep(waitTime)` to tick actions.
   → **Inference**: Autonomous actions execute asynchronously from the RoboRIO's main periodic loop.
   → **Inference**: Actions call subsystem hardware methods (e.g. `swerveBase.drive(...)`, `intake.runRollers(...)`) from the background thread while `Robot.robotPeriodic()` executes `SubsystemManager.updateSubsystems()` on the main thread.
   → **Conclusion**: This is a direct multithreading race condition without locks or synchronization. In addition, `CommandScheduler.getInstance().run()` is completely missing from `robotPeriodic()`, breaking standard WPILib Command-based paradigms.

2. **Observation**: 106 occurrences of `System.out.println`, `System.err.println`, and `printf` were cataloged across 24 files, including 15 inside `Test/DriveCharacterization.java` and 11 inside `Sim/GameSim.java`.
   → **Inference**: Standard console output on the RoboRIO is synchronous and routed across NetConsole and NetworkTables.
   → **Conclusion**: Frequent or unbuffered console prints during robot operations cause loop time overruns (>20ms), degraded odometry integration, and intermittent communication stalls.

3. **Observation**: 167 public mutable fields were cataloged across the codebase, including `Shooter.targetRpmLeft`, `AutoMissionChooser.delay`, and `Teleop.joystickEnabled`.
   → **Inference**: External classes can mutate critical subsystem state without validation, side-effect triggers, or synchronization.
   → **Conclusion**: Violates core object-oriented encapsulation and introduces hidden state dependencies across disparate files.

4. **Observation**: In `NeoSparkMaxMotor.java:85-100`, runtime setter methods `setInverted(boolean)` and `setBrakeMode(boolean)` call `m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)`.
   → **Inference**: `PersistMode.kPersistParameters` writes configuration parameters directly to SparkMax non-volatile flash EEPROM.
   → **Conclusion**: Repeated invocations of these methods during a match or teleop toggle will exhaust SparkMax hardware write cycles and introduce CAN bus frame delays.

5. **Observation**: Quantitative Javadoc inspection showed only 50.3% of types, 27.9% of public/protected methods, and 1.8% of constants have formal Javadocs. Only 11.8% of methods and 15.5% of constants document physical engineering units.
   → **Inference**: Developers and autonomous tuning algorithms have no contractual guarantee whether angles are in radians or degrees, distances in meters or inches, speeds in RPM or duty cycle, or times in seconds or milliseconds.
   → **Conclusion**: A systematic documentation overhaul documenting explicit units on every method and constant is mandatory before competition deployment.

## 3. Caveats

1. **Third-Party Vendor Code Isolation**: `src/main/java/frc/robot/ThirdParty/LimelightHelpers.java` is an official, upstream vendor-supplied single-file utility from Limelight. While it contains 132 code smells (public fields, console prints), altering its core structure could complicate future upstream vendor drops. Refactoring should focus primarily on team-owned code in `frc.robot.*` while wrapping or safely consuming `LimelightHelpers`.
2. **AdvantageKit `@AutoLog` Struct Convention**: In files like `DriveIO.java`, `IntakeIO.java`, `ShooterIO.java`, and `VisionIO.java`, inner classes annotated with `@AutoLog` (`DriveIOInputs`, etc.) use public fields by design according to the AdvantageKit code generation pattern. These fields should NOT be encapsulated into private fields with getters/setters, but MUST have explicit engineering units documented via Javadoc docstrings and variable name suffixes.
3. **Simulation Code Scope**: The 14 files in `frc.robot.Sim.*` (`GameSim`, `AIRobotSim`, `JevDecisionEngine`, etc.) are desktop simulation models designed to run on development workstations, not on the RoboRIO during FRC matches. While code quality and Javadoc improvements are necessary, performance constraints (such as heap allocations) are less critical in simulation than on the physical RoboRIO embedded processor.
4. **ChoreoLib Integration**: Autonomous trajectories are defined via Choreo `.traj` files in `src/main/deploy/choreo`. Refactoring the autonomous action execution architecture must preserve compatibility with Choreo's trajectory sample format (`SwerveSample`) and timestamp tracking.

## 4. Conclusion

The TitanRoboticsBuildSeason codebase possesses a rich feature set (holonomic pathfinding, AdvantageKit replay logging, physics simulation, multi-agent AI sparring), but suffers from significant documentation gaps and critical architectural anti-patterns that jeopardize competition reliability.

### Actionable Overhaul Priorities for Implementers:
1. **Priority 1: Concurrency & Autonomous Unification**
   - Deprecate raw thread spawning in `AutoMissionExecutor` and blocking `Thread.sleep` loops in `MissionBase`.
   - Ensure `CommandScheduler.getInstance().run()` is called in `Robot.robotPeriodic()` so that WPILib commands, triggers, and scheduled actions run predictably and safely on the main thread.
2. **Priority 2: SparkMax Flash Memory Protection**
   - In `NeoSparkMaxMotor.java`, replace `PersistMode.kPersistParameters` with `PersistMode.kNoPersistParameters` in runtime methods (`setInverted`, `setBrakeMode`). Ensure parameter persistence occurs ONLY once during robot initialization.
3. **Priority 3: Console I/O Elimination & Telemetry Hygiene**
   - Replace all 106 occurrences of `System.out.println` and `System.err.println` with AdvantageKit `Logger.recordOutput`, `DataLogManager.log`, or `DriverStation.reportError`.
4. **Priority 4: Encapsulation & Mutability Protection**
   - Convert public mutable fields in `Shooter.java` (`targetRpmLeft`, `targetRpmRight`, etc.) and `AutoMissionChooser.java` (`delay`) to private fields with type-safe accessors.
   - Return `Collections.unmodifiableList(subsystems)` in `SubsystemManager.getSubsystems()`.
5. **Priority 5: Comprehensive Javadoc & Engineering Units Overhaul**
   - Add complete class-level Javadocs to the 79 missing types.
   - Add Javadoc docstrings with explicit `@param`, `@return`, and engineering units (meters, radians, degrees, seconds, volts, amperes, RPM) to all 519 missing methods and 449 missing constants.
   - Resolve contradictory comments (e.g. `Constants.ROBOT_MASS`).
6. **Priority 6: Dead Code Elimination**
   - Remove obsolete `Test/SysID.java` and redundant methods (`NeoSparkMaxMotor.getVelocity()`).

## 5. Verification Method

Any implementer executing the refactoring and documentation tasks can independently verify code integrity, build stability, and documentation coverage using the following commands and checks:

### 1. Build Compilation Verification
The project must compile cleanly without errors using the official WPILib 2026 JDK:

```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
.\gradlew build `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk`"
```
**Expected Outcome**: Exit code 0, BUILD SUCCESSFUL, with zero compile errors or broken symbol references.

### 2. Unit Test Suite Verification
Execute all existing unit tests in `src/test/java`:

```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
.\gradlew test `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk`"
```
**Expected Outcome**: Exit code 0, all 18 test suites pass without regressions.

### 3. Automated Documentation & Code Quality Audit Verification
Re-run the audit scripts created during this investigation to ensure coverage metrics increase to 100% and smells drop to 0:
```powershell
python "C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_script.py"
python "C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\summarize_audit.py"
```
**Target Metrics for Completion**:
- Types with Javadoc: 100% (159/159)
- Public/Protected Methods with Javadoc: 100% (720/720)
- Public/Protected Methods with Units: 100% of numeric methods
- Constants with Javadoc & Units: 100% (457/457)
- Console I/O Smells: 0 in robot code (`frc.robot.*`)
- Thread.sleep Occurrences: 0 in robot code (`frc.robot.*`)
- Public Mutable Fields: 0 (outside of AdvantageKit `@AutoLog` data classes)
