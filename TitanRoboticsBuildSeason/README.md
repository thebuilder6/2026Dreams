---
title: Platform README
audience: [human, ai]
owner: programming-leads
last_verified: 2026-10-06
status: authoritative
---

# 🚀 Titan Robotics 2026/2027 Robot Platform (Mentor Fork)

Welcome to the **Team 8334 Titan Robotics** advanced exploration repository. This codebase pairs full competition-proven hardware calibrations with modern software innovations: high-fidelity physics simulation, AdvantageKit IO abstraction, dual-camera AprilTag fusion, an automated pre-flight diagnostics suite, and the Jev AI tactical decision engine.

---

## 🏛️ System Architecture Overview

See [ARCHITECTURE.md](ARCHITECTURE.md) for the authoritative 4-layer diagram and subsystem contracts (not duplicated here).

---

## 🛠️ Quick Start & Developer Guide

### 1. Compiling the Code (Offline / WPILib JDK, Java 17, GradleRIO 2026.2.1)
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
./gradlew compileJava --offline
```

### 2. Tests (JUnit 5 — 49 files / 467 tests as of 2026-10-06; green on clean re-run, see `KNOWN_ISSUES.md` §A)
```powershell
./gradlew test --offline
# Single class: ./gradlew test --offline --tests "frc.robot.Intelligence.JevDecisionEngineTest"
```

### 3. Launching Desktop Physics Simulation
Launch the WPILib SimGUI with our full `IronMaple` arena, simulated game pieces, and `AIRobotSim` opponent:
```powershell
./gradlew simulateJava
# Headless 6v6 (no GUI, replayable .wpilog + markdown report):
# ./gradlew simulateJavaRelease --offline -Pheadless [-Pseed=2026] [-PdurationSec=150] [-PautoSec=15]
#   [-PfieldFuelCount=108] [-PdisabledGapSec=6] [-PbootWaitSec=8] [-PlogDir=logs] [-PreportDir=reports]
```

### 4. Deploying to RoboRIO
```powershell
./gradlew deploy   # same WPILib JDK as above
```

### 5. Key Documentation Links
- 📖 [Architecture Specification](ARCHITECTURE.md): Deep-dive into subsystems, vision fusion, and Jev AI.
- 🎮 [Operator's Guide](OPERATORS_GUIDE.md): Driver and operator controls, Glide Mode navigation, and match rules.
- 🕹️ [Simulation Setup & User Guide](SIMULATION_GUIDE.md): Step-by-step setup for SimGUI, Elastic Dashboard, AdvantageScope, and AI sparring.
- 🧪 [Testing & Diagnostics Guide](src/main/java/frc/robot/Test/README.md): Pre-flight checks, SysId characterization, and tuning routines.
- 🎯 [Shooter Calibration & Tuning Guide](docs/SHOOTER_TUNING_GUIDE.md): Step-by-step physical calibration, live PID tuning, SOTF, and ballistics automation tools.
- 📥 [Intake Arm Calibration & Tuning Guide](docs/INTAKE_TUNING_GUIDE.md): Arm Profiled PID, gravity feedforward, jam detection, and trench safety.
- 🛞 [Swerve Drive Calibration & Tuning Guide](docs/SWERVE_TUNING_GUIDE.md): MK4i module zeroing, wheel radius via `tune.py`, and YAGSL PIDF.
- 🤖 [Autonomous & Trajectory Pipeline Guide](docs/AUTONOMOUS_GUIDE.md): Custom action framework, Choreo paths, and event markers.
- 👁️ [Dual-Vision Platform & Calibration Guide](docs/VISION_GUIDE.md): Limelight MegaTag2, Orange Pi 5 PhotonVision, and pose filtering.
- 📋 [Pit Tuning & Pre-Match Checklist](docs/PIT_TUNING_CHECKLIST.md): 5-station rapid pit check, 15-second diagnostics scorecard, and carpet benchmarks.
- 🎒 [Developer & Student Onboarding Guide](docs/ONBOARDING.md): Getting started guide for new student programmers and contributors.

