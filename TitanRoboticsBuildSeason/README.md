# 🚀 Titan Robotics 2026/2027 Robot Platform (Mentor Fork)

Welcome to the **Team 8334 Titan Robotics** advanced exploration repository. This codebase pairs full competition-proven hardware calibrations with modern software innovations: high-fidelity physics simulation, AdvantageKit IO abstraction, dual-camera AprilTag fusion, an automated pre-flight diagnostics suite, and the Jev AI tactical decision engine.

---

## 🏛️ System Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                   DECISION & TACTICAL STRATEGY LAYER                     │
│  - Jev AI Decision Engine (TypeSafe System One: <20ms structured choices)│
│  - Choreo Trajectory Tracking & Dynamic Obstacle Avoidance               │
│  - Autonomous Mission Chooser & Teleop State Machine                     │
└────────────────────────────────────┬─────────────────────────────────────┘
                                     ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         SUBSYSTEM LOGIC LAYER                            │
│  - SwerveBase (Kinematics, Glide Points, Field-Oriented Drive)           │
│  - Shooter (Distance-to-RPM Dual Flywheel Tables, Kicker Control)        │
│  - Intake (Continuous ProfiledPID [0,360], Gravity Feedforward)          │
│  - Vision (Multi-Tag AprilTag Fusion: Limelight MT2 + PhotonVision)      │
│  - Diagnostics (15-Second Pre-Flight Self-Test Sequencer)                │
└────────────────────────────────────┬─────────────────────────────────────┘
                                     ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                   HARDWARE IO ABSTRACTION (AdvantageKit)                 │
│         DriveIO         ShooterIO         IntakeIO         VisionIO      │
│        /      \         /       \         /      \         /      \      │
│    [Spark]  [Sim]   [Spark]   [Sim]   [Spark]  [Sim]   [LL/PV]  [PVSim]  │
└────────────────────────────────────┬─────────────────────────────────────┘
                                     ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                     TELEMETRY & VISUALIZATION LAYER                      │
│  - Elastic Dashboard (Driver UI, Feature Switches, Pre-Flight Scorecard) │
│  - AdvantageScope (3D Field, Robot Poses, Mechanism Visualizer)          │
│  - Deterministic Replay (.wpilog Byte-for-Byte Match Simulation)         │
└──────────────────────────────────────────────────────────────────────────┘
```

For detailed specifications of each layer, see the [Architecture Guide](ARCHITECTURE.md).

---

## 🛠️ Quick Start & Developer Guide

### 1. Compiling the Code (Offline / WPILib JDK)
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
./gradlew compileJava --offline
```

### 2. Launching Desktop Physics Simulation
Launch the WPILib SimGUI with our full `IronMaple` arena, simulated game pieces, and `AIRobotSim` opponent:
```powershell
./gradlew simulateJava
```

### 3. Key Documentation Links
- 📖 [Architecture Specification](ARCHITECTURE.md): Deep-dive into subsystems, vision fusion, and Jev AI.
- 🎮 [Operator's Guide](OPERATORS_GUIDE.md): Driver and operator controls, Glide Mode navigation, and match rules.
- 🧪 [Testing & Diagnostics Guide](src/main/java/frc/robot/Test/README.md): Pre-flight checks, SysId characterization, and tuning routines.
