# Feature and Subsystem List

This document outlines the core features, subsystems, and structural components of the 2026/2027 robot software platform, based on `ARCHITECTURE.md` and the existing `src` structure.

## 1. Subsystems
*   **SwerveBase (`frc.robot.Subsystems.SwerveBase`)**
    *   Kinematics via YAGSL (Yet Another Generic Swerve Library).
    *   High-speed heading correction & skew compensation.
    *   Vision watchdog for dead-reckoning fallback.
*   **Shooter (`frc.robot.Subsystems.Shooter`)**
    *   Dual-flywheel system (Left/Right) with independent PID + Feedforward.
    *   Empirical distance-to-RPM lookup tables (`leftRpmTable`, `rightRpmTable`).
    *   Kicker feeder sequenced actuation.
*   **Intake (`frc.robot.Subsystems.Intake`)**
    *   Articulated ground intake with continuous profiled PID control.
    *   Gravity compensation (`ArmFeedforward`).
    *   Absolute encoder fallback to internal relative encoders.
    *   Current stall jam clearing (auto-reversal).
*   **Vision (`frc.robot.Subsystems.Vision`)**
    *   Dual-vision platform: Front Limelight (MegaTag2), Coprocessor PhotonVision.
    *   Object detection (YOLOv8 pipeline).
*   **LEDs (`frc.robot.Subsystems.LEDs`)**
    *   Addressable LED integration for system status (Strobe Red, Solid Orange, etc.).

## 2. Hardware IO Abstraction (AdvantageKit Pattern)
*   **DriveIO** (`DriveIOSparkMax`, `DriveIOSim`)
*   **ShooterIO** (`ShooterIOSparkMax`, `ShooterIOSim`)
*   **IntakeIO** (`IntakeIOSparkMax`, `IntakeIOSim`)
*   **VisionIO** (`VisionIOLimelight`, `VisionIOPhotonVision`, `VisionIOSim`)

## 3. Autonomous & Planning (Pathing & Auto)
*   **Choreo Integration**
    *   Trajectory tracking & dynamic obstacle avoidance (`DynamicRouter`, `SmartTunnelRouter`).
*   **Autonomous Missions**
    *   `AutoMissionChooser`, `AutoMissionExecutor`.
    *   Mission Base classes and Action structures (`DriveToPoseAction`, `ShootAction`, etc.).
*   **Legal Pinning Watchdog** (`LegalPinningWatchdog.java`)
*   **Collision Detection** (`CollisionDetector.java`)

## 4. AI & Simulation (Jev AI & Sim)
*   **Jev AI Decision Engine** (`JevDecisionEngine.java`)
    *   System 1 (Tactical Reflex) and System 2 (Executive Strategy).
    *   Unified cognitive architecture for live matches and sim sparring.
*   **Simulation Sparring Engine**
    *   `AIRobotSim`, `AIRobotInstance`.
    *   Rigid-body 2D simulation via `IronMaple`.
    *   Simulated multi-robot interactions with inverse-distance repulsive forces.
*   **Driver Assist / Co-Pilot**
    *   `AutonomousTeleopAgent.java`
    *   `MatchCoach.java` (Real-time HUD recommendations).
*   **Simulated Sensors**
    *   `ShooterSim`, `ArmSim`, `VisionSim`, `LimelightSim`.

## 5. Driver UI, Telemetry, and Diagnostics
*   **Elastic Dashboard**
    *   Feature switches, Pre-Flight Scorecard.
*   **AdvantageScope**
    *   3D field tracking, mechanism poses, and deterministic replay (.wpilog).
*   **Alert & Haptic Feedback System**
    *   `AlertManager` (non-CLI dashboard alerts).
    *   `Controller.java` (non-blocking sequenced rumbles for target locked, acquired, warnings).
*   **Diagnostics** (`frc.robot.Test.Diagnostics`)
    *   15-Second Automated Pit Check (CAN audit, motor tests, vision links).
*   **Alliance Flipping** (`AllianceFlipUtil.java`)
    *   Blue-origin standardized coordinate geometry.

---

# Proposed Architecture Restructuring (High-Level Ideas)

Currently, files related to similar high-level responsibilities are spread out. For example, AI components (`Sim/`), autonomous routines (`Auto/`), and driver assist tools exist as sibling or unrelated structures. The goal of this restructuring is to **reduce complexity** and **group related files/systems together**.

### Specific Focus Areas for Restructuring
1.  **AI vs. Simulation vs. Driver Assist vs. Pathing**
    *   Right now, `Sim/` contains both pure physics simulators (`ShooterSim`, `ArmSim`) AND the Jev AI Decision Engine (`JevDecisionEngine`, `WorldState`, `StrategicObjective`).
    *   `Auto/` contains both static pathing tools (`StaticPathfinder`, Choreo bindings) AND dynamic planning (`DynamicRouter`, `AutonomousTeleopAgent`).
    *   We need to cleanly separate:
        *   **Cognitive/AI Layer**: Where decisions are made (Jev Engine, Strategy, States).
        *   **Navigation/Pathing Layer**: How decisions are executed physically (Choreo, Trajectory Controllers, Obstacle Avoidance).
        *   **Physics Simulation Layer**: Pure virtual hardware representation.
        *   **Human-Machine Interface (HMI)**: Driver Assist, Coaching, Controller Rumbles.

*A detailed proposal will be created in `RESTRUCTURING_PLAN.md`.*
