# Project: TitanRoboticsBuildSeason Architecture & Quality Overhaul

## Architecture
- **Framework**: WPILib 2026 Command-Based & AdvantageKit IO Abstraction (LoggedRobot, Logger, @AutoLog).
- **Subsystem Layer**:
  - `SwerveBase`: YAGSL swerve drive with AdvantageKit `DriveIO` (Spark Max hardware / IronMaple simulation).
  - `Shooter`: Dual-flywheel velocity closed loop with indexer kicker (`ShooterIO`).
  - `Intake`: Articulated pivot arm with ground pickup rollers and centering hopper (`IntakeIO`).
  - `Vision`: Multi-coprocessor AprilTag fiducial pose estimation (Limelight MegaTag2 + PhotonVision).
  - `LEDs`: REV Blinkin driver status indicator.
  - `Dashboard`: NetworkTables v4 publisher / Elastic driver dashboard.
  - `MatchCoach`: Autonomous tactical advisory and cycle evaluator.
- **Hardware Integration**:
  - 14 REV Spark Max controllers, 4 CTRE CANcoders, 1 REV PDH, NavX MXP IMU on 1 Mbps CAN bus.
- **Build & Execution**:
  - WPILib 2026 Gradle toolchain with OpenJDK 17 (`C:\Users\Public\wpilib\2026\jdk`).

## Feature Inventory
| # | Feature / Issue | Description | Milestone | Source |
|---|---|---|---|---|
| 1 | Gradle Wrapper JDK Detection | Add WPILib 2026 JDK fallback to `gradlew.bat` for seamless execution of `gradlew build -Dorg.gradle.java.home=...` across all shells | M1 | explorer_build_1 |
| 2 | Incremental Build Caching | Declare inputs/outputs for `generateBuildConstants` in `build.gradle` and set `DuplicatesStrategy.EXCLUDE` | M1 | explorer_build_1 |
| 3 | Prune Unused Vendordeps | Remove unused `vendordeps/yams.json` | M1 | explorer_build_1 |
| 4 | DriverAssistTest Ball Count Bug | Fix `AutonomousTeleopAgent.java:86` resetting `estimatedHeldBalls = 0` when `!intake.hasFuel()` so 144/144 tests pass | M1 | explorer_build_1 |
| 5 | Eliminate SparkMax Flash Burning | Change `NeoSparkMaxMotor.setBrakeMode` / `setInverted` to use `PersistMode.kNoPersistParameters` at runtime | M2 | explorer_arch_1 |
| 6 | CAN Bus Status Frame Throttling | Configure `SparkMaxConfig.signals` on auxiliary motors (kicker, hopper, rollers) and drive/steer to drop CAN utilization from ~80% to <50% | M2 | explorer_arch_1 |
| 7 | Subsystem Thread Safety & Synchronization | Synchronize public actuation methods on `SwerveBase`, `Intake`, and `Shooter` or migrate auto thread calls to safe dispatch | M2 | explorer_arch_1 |
| 8 | Drivetrain Simulation Battery Sag | Implement `SwerveBase.getSimulationCurrentDraw()` so `RoboRioSim` realistically simulates drivetrain voltage drops | M2 | explorer_arch_1 |
| 9 | CommandScheduler Integration | Safely invoke `CommandScheduler.getInstance().run()` in `robotPeriodic()` to ensure WPILib trigger evaluation and command compliance | M2 | explorer_arch_1 |
| 10 | 3D Pose Preallocation in Loops | Replace 50Hz dynamic instantiations of `Translation3d`, `Rotation3d`, `Pose3d` in `Shooter.log()` and `Intake.log()` with cached/static instances | M3 | explorer_arch_1 |
| 11 | Vision Covariance Matrix Caching | Preallocate/cache `Matrix<N3, N1>` in `Vision.java` and constant empty array in `VisionIOPhotonVision.java` | M3 | explorer_arch_1 |
| 12 | TunableNumber Autoboxing Elimination | Bypass map queries and boxing in `TunableNumber.hasChanged()` when not in tuning mode | M3 | explorer_arch_1 |
| 13 | AlertManager List Recycling | Reuse preallocated list buffers in `AlertManager.update()` instead of creating new `ArrayList` instances every 20ms | M3 | explorer_arch_1 |
| 14 | Telemetry & Console Streamlining | Eliminate redundant `System.out.println` console spam and redundant duplicate NetworkTables scalar writes | M3 | explorer_docs_1 |
| 15 | HTML Javadoc Syntax Errors | Fix all 31 malformed HTML tags/entities (`<=`, `&`, `<throttle>`) in `AllianceFlipUtil`, `LimelightHelpers`, `MatchCoach`, `MatchScoreTracker` | M4 | explorer_build_1 |
| 16 | Subsystem & HAL Javadoc Overhaul | Add comprehensive Javadocs with explicit physical units to all subsystems, IO interfaces, and hardware implementations | M4 | explorer_docs_1 |
| 17 | Constants & Configuration Javadoc | Document all constants in `Constants.java`, `FieldMap.java`, `GlideConstants.java`, and `PortMap.java` with explicit engineering units | M4 | explorer_docs_1 |
| 18 | Commands, Actions & Sim Javadoc | Add descriptive Javadoc docstrings and engineering units across `Auto/Actions`, `Auto/Missions`, and `Sim` types | M4 | explorer_docs_1 |
| 19 | Clean Javadoc Generation | Verify `gradlew javadoc` executes with 0 errors and 0 warnings | M4 | explorer_build_1 |
| 20 | Final Integration Verification | Execute complete clean `gradlew build` with WPILib 2026 JDK, 100% test pass rate, and forensic integrity audit | M5 | original_request |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|---|---|---|---|
| M1 | Build Toolchain & Dependency Modernization | Features 1, 2, 3, 4: `gradlew.bat` JDK fallback, `build.gradle` cache fixes, prune `yams.json`, fix `AutonomousTeleopAgent.java:86` unit test | None | IN_PROGRESS |
| M2 | Architecture, Thread Safety & CAN Performance | Features 5, 6, 7, 8, 9: `NeoSparkMaxMotor` flash writes, CAN status frame throttling, subsystem synchronization, battery current draw, `CommandScheduler` run | M1 | PLANNED |
| M3 | Periodic Loop Heap Optimization & Telemetry | Features 10, 11, 12, 13, 14: Preallocate 3D poses in `Shooter`/`Intake`, cache vision matrices, eliminate autoboxing in `TunableNumber`, recycle alert lists, prune console spam | M2 | PLANNED |
| M4 | Comprehensive Javadocs & Engineering Units | Features 15, 16, 17, 18, 19: Fix 31 HTML Javadoc syntax errors, add full Javadocs and explicit engineering units across all classes, methods, and constants | M3 | PLANNED |
| M5 | Final Build & Forensic Verification | Feature 20: Full clean WPILib 2026 Gradle build, test suite execution (144/144 tests), Javadoc generation, and forensic audit validation | M4 | PLANNED |

## Interface Contracts
### Subsystems ↔ Hardware Abstraction Layer (HAL)
- `DriveIO.updateInputs(DriveIOInputs inputs)`: Synchronous hardware sensor read; inputs logged via AdvantageKit `@AutoLog`.
- `ShooterIO.updateInputs(ShooterIOInputs inputs)`: Dual flywheel velocity and kicker status; all velocities in Radians/Sec or RPM.
- `IntakeIO.updateInputs(IntakeIOInputs inputs)`: Arm pivot position in Radians, roller current in Amperes.
- `VisionIO.updateInputs(VisionIOInputs inputs)`: Pose estimates, timestamps in Microseconds, target fiducial metrics.

### Threading & Concurrency Contract
- Main loop (`robotPeriodic`, 50Hz) owns subsystem periodic updates (`SubsystemManager.updateSubsystems()`).
- Autonomous actions or background routines interacting with `SwerveBase`, `Intake`, or `Shooter` must either be single-threaded via WPILib Command scheduler or execute through synchronized subsystem interfaces.
- Runtime Spark Max calls MUST NOT write to flash (`PersistMode.kNoPersistParameters`).

## Code Layout
- `src/main/java/frc/robot`: Core robot orchestration (`Robot.java`, `Teleop.java`, `Main.java`).
- `src/main/java/frc/robot/Subsystems`: Primary subsystem implementations (`SwerveBase.java`, `Shooter.java`, `Intake.java`, `Vision.java`, `LEDs.java`, `Dashboard.java`, `MatchCoach.java`).
- `src/main/java/frc/robot/Subsystems/{drive,shooter,intake,vision}`: AdvantageKit IO interfaces and Spark Max / Sim hardware implementations.
- `src/main/java/frc/robot/Auto`: Autonomous navigation, Choreo trajectory controllers, pathfinding.
- `src/main/java/frc/robot/Data`: Configuration constants (`Constants.java`, `FieldMap.java`, `GlideConstants.java`, `PortMap.java`).
- `src/main/java/frc/robot/Devices`: Hardware wrapper devices (`NeoSparkMaxMotor.java`, `Controller.java`).
- `src/main/java/frc/robot/Sim`: Simulation physics, competitor models, match state tracking.
- `src/main/java/frc/robot/Utils`: Mathematical utilities, coordinate transformations, alerts.
