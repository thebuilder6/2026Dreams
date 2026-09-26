---
title: Platform README
audience: [human, ai]
owner: programming-leads
last_verified: 2026-09-26
status: authoritative
---

# 🚀 Titan Robotics 2026/2027 Robot Platform (Mentor Fork)

Welcome to the **Team 8334 Titan Robotics** advanced exploration repository. This codebase pairs full competition-proven hardware calibrations with modern software innovations: high-fidelity physics simulation, AdvantageKit IO abstraction, dual-camera AprilTag fusion, an automated pre-flight diagnostics suite, and the Jev AI tactical decision engine.

---

## 🏛️ System Architecture Overview

See [ARCHITECTURE.md](ARCHITECTURE.md) for the authoritative 4-layer diagram and subsystem contracts (not duplicated here).

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
- 🕹️ [Simulation Setup & User Guide](SIMULATION_GUIDE.md): Step-by-step setup for SimGUI, Elastic Dashboard, AdvantageScope, and AI sparring.
- 🧪 [Testing & Diagnostics Guide](src/main/java/frc/robot/Test/README.md): Pre-flight checks, SysId characterization, and tuning routines.
