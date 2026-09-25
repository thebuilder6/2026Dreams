# Feature and Subsystem Inventory

This document outlines the core features, subsystems, and structural components of the 2026/2027 robot software platform. It highlights user-facing features, testing infrastructure, and backend logic.

## 1. Driver and Operator Features (HMI & Assist)
These features directly impact how the human drive team interacts with the robot on the field.

*   **Driver Assist & Co-Pilot (`AutonomousTeleopAgent`)**
    *   One-button auto-cycle capabilities seamlessly integrating with teleop driving.
    *   Takes over trajectory and alignment while leaving micro-adjustments to the driver.
*   **Match Coach (`MatchCoach`)**
    *   Real-time HUD recommendations leveraging the AI engine to suggest strategic moves based on the current field state.
*   **Haptic Feedback & Rumbles (`Controller`)**
    *   **Non-Blocking Sequenced Rumbles:** Executes complex vibration patterns without pausing control loops.
    *   `TARGET_LOCKED`: Double pulse confirming alignment with the hub and flywheels at speed.
    *   `BALL_ACQUIRED`: Confirmation pulse when a piece is seated.
    *   `HARDWARE_WARNING`: Rapid triple pulse for sensor degradation (e.g., vision lost).
    *   `MATCH_TIME_WARNING`: Sustained rumble at T-30s and T-15s.
*   **Non-CLI Dashboard Alerts (`AlertManager`, `Alert`)**
    *   No spamming the console. Errors and warnings are pushed to a visual Elastic Dashboard banner.
*   **Status LEDs (`LEDs`)**
    *   Visual communication to drivers and human players (Strobe Red for fault, Solid Orange for warning, Solid Green for target locked).
*   **Legal Pinning Watchdog (`LegalPinningWatchdog`)**
    *   Monitors and warns drivers when they are close to violating the 5-second pinning rule.

## 2. Simulation & Physics Engine
Features designed for testing code without a physical robot and generating opponent AI.

*   **Multi-Bot Sparring (`AIRobotSim`, `AIRobotInstance`)**
    *   Can spawn 1 to 3 concurrent simulated robots on the field.
    *   Features inverse-distance repulsive forces to simulate collisions and prevent bots from stacking on each other.
*   **Physics Simulation Layer (`IronMaple`)**
    *   Rigid-body 2D simulation providing true carpet friction, wheel slip, and bumper collisions.
*   **Hardware Virtualization (`ShooterSim`, `ArmSim`, `VisionSim`, `LimelightSim`)**
    *   Simulated hardware inputs allowing full testing of PID controllers, intake gravity feedforwards, and camera pipelines in SimGUI.

## 3. SysId, Diagnostics & Testing
Tools for calibrating mechanisms, verifying hardware health, and troubleshooting.

*   **15-Second Automated Pit Check (`Diagnostics`)**
    *   *CAN Bus Audit:* Verifies all devices acknowledge heartbeats.
    *   *Swerve Motor Pulse:* Checks encoder velocity sign.
    *   *Steer Alignment Check:* Sweeps modules 90° to confirm absolute encoder offsets.
    *   *Intake Profile Check:* Flags mechanical binding by monitoring current draw (>25A).
    *   *Shooter Ramping:* Verifies steady-state RPM error bounds.
    *   *Vision Link Check:* Pings network latency and FPS.
*   **System Identification (`SysIdManager`, `SysID`, `DriveCharacterization`)**
    *   Automated routines to characterize drive base, shooter, and intake feedforward constants (kV, kA, kS).
*   **Test Modes (`TestMode`, `ShooterTuning`, `IntakeTesting`, `VisionTesting`)**
    *   Isolated testing environments accessible via DriverStation Test Mode to tune PID loops safely.

## 4. Telemetry & Replay
*   **AdvantageScope & Epilogue (`.wpilog`)**
    *   High-frequency deterministic logging.
    *   Byte-for-byte match replay for debugging post-match.
    *   3D visualizer for robot field poses, mechanism articulation, and vision raycasts.
*   **Elastic Dashboard Layouts**
    *   Custom pre-configured UI (`elastic-layout.json`) loaded automatically, containing the Pre-Flight Scorecard and Tunable Numbers.

## 5. Core Robot Subsystems
*   **SwerveBase (`SwerveBase`)**
    *   Kinematics via YAGSL.
    *   High-speed heading correction, skew compensation, and vision watchdog for dead-reckoning fallback.
*   **Shooter (`Shooter`)**
    *   Dual-flywheel system (independent PID + Feedforward).
    *   Empirical distance-to-RPM lookup tables.
    *   Kicker feeder sequenced actuation.
*   **Intake (`Intake`)**
    *   Articulated ground intake with continuous profiled PID control and gravity compensation.
    *   Absolute encoder fallback to relative encoders on failure.
    *   Current stall jam clearing (auto-reversal).
*   **Vision (`Vision`)**
    *   Dual-vision platform: Front Limelight (MegaTag2) and Coprocessor PhotonVision.
    *   YOLOv8 Object detection for automated game piece targeting.

## 6. Hardware IO Abstraction (AdvantageKit Pattern)
Abstracts physical hardware from logic, enabling replay and simulation.
*   **DriveIO** (`DriveIOSparkMax`, `DriveIOSim`)
*   **ShooterIO** (`ShooterIOSparkMax`, `ShooterIOSim`)
*   **IntakeIO** (`IntakeIOSparkMax`, `IntakeIOSim`)
*   **VisionIO** (`VisionIOLimelight`, `VisionIOPhotonVision`, `VisionIOSim`)

## 7. AI & Autonomous Planning
*   **Jev AI Decision Engine (`JevDecisionEngine`)**
    *   System 1 (Tactical Reflex) and System 2 (Executive Strategy).
    *   Evaluates macro-utility matrices based on Archetypes (Cycler, Bully, Defender).
*   **Choreo Integration & Pathing (`StaticPathfinder`, `DynamicRouter`)**
    *   Trajectory tracking & dynamic obstacle avoidance (`SmartTunnelRouter`).
*   **Autonomous Missions (`AutoMissionChooser`, `AutoMissionExecutor`)**
    *   Mission base classes and concrete actions (`DriveToPoseAction`, `ShootAction`).
*   **Alliance Flipping (`AllianceFlipUtil`)**
    *   Standardized Blue-origin coordinate geometry automatically mirrored based on DriverStation data.
