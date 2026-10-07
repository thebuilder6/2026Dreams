---
title: Software Developer & Student Onboarding Guide
audience: [human, programmers, students, ai]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Software Developer & Student Onboarding Guide — FRC Team 8334

Welcome to the Team 8334 Titan Robotics software development team! This guide introduces new student programmers, mentors, and contributors to our software architecture, development workflow, coding standards, and testing protocols.

---

## Scope

### What this document covers
- Development workstation setup and WPILib 2026 toolchain configuration.
- Repository structure and the distinction between the git root and `TitanRoboticsBuildSeason/`.
- The 4-layer architectural mental model (Decision, Subsystems, AdvantageKit IO, Telemetry).
- Team conventions that differ from textbook WPILib (Subsystem singletons, Blue-origin geometry, Alert banners, TunableNumbers).
- Day-to-day development workflow: compiling, running tests, desktop simulation with Elastic and AdvantageScope.
- Multi-agent resource coordination lock protocol.

### What this document does NOT cover
- Low-level motor PID tuning math (see [`docs/SHOOTER_TUNING_GUIDE.md`](SHOOTER_TUNING_GUIDE.md), [`docs/INTAKE_TUNING_GUIDE.md`](INTAKE_TUNING_GUIDE.md), and [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md)).
- Competition pit procedures (see [`docs/PIT_TUNING_CHECKLIST.md`](PIT_TUNING_CHECKLIST.md)).
- Machine learning and Jev AI game theoretical formulation (see [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3J).

---

## Content

### 1. Workstation Setup & Environment

All builds require the official **WPILib 2026 Suite** (WPILib 2026.2.1 and Java 17). Standard system JDKs or Eclipse Temurin builds will fail due to native JNI library bindings.

#### PowerShell Environment Setup
In every new Windows PowerShell terminal, initialize your environment:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
```

#### Repository Rule: Where to Run Commands
The GradleRIO project resides in the subfolder **`TitanRoboticsBuildSeason/`**.
- **Always** run `./gradlew` from `TitanRoboticsBuildSeason/`, never from the git repository root.
- **Always** use the wrapper script (`.\gradlew` or `./gradlew`), never your computer's global `gradle`.
- **Always** append `--offline` for local builds to prevent slow network fetches against external maven repositories.

```powershell
# Navigate to the project directory
cd TitanRoboticsBuildSeason

# Fast offline compile check
.\gradlew compileJava --offline
```

---

### 2. High-Level Architectural Mental Model

Our software is structured in four distinct layers. Understanding where your feature belongs prevents cross-layer coupling:

```
┌────────────────────────────────────────────────────────┐
│  Layer 1: DECISION & AUTONOMOUS LAYER                  │
│  - Jev AI Decision Engine (Intelligence/)              │
│  - Autonomous Mission Chooser & Action Framework (Auto/)│
│  - Driver Assist & Co-Pilot (AutonomousTeleopAgent.java)│
└───────────────────────────┬────────────────────────────┘
                            │ Commands desired states (e.g. INTAKING, SHOOTING)
                            ▼
┌────────────────────────────────────────────────────────┐
│  Layer 2: SUBSYSTEM LOGIC LAYER                        │
│  - SwerveBase, Shooter, Intake, Vision, LEDs           │
│  - Closed-loop PID controllers, feedforward math       │
│  - State machines, safety interlocks, jam recovery     │
└───────────────────────────┬────────────────────────────┘
                            │ Reads inputs / writes outputs
                            ▼
┌────────────────────────────────────────────────────────┐
│  Layer 3: HARDWARE IO ABSTRACTION (AdvantageKit)       │
│  - Interfaces: DriveIO, ShooterIO, IntakeIO, VisionIO   │
│  - Real Hardware: SparkMax, Limelight, NavX, CANcoder  │
│  - Desktop Sim: IronMaple, ArmSim, FlywheelSim, VisionSim│
└───────────────────────────┬────────────────────────────┘
                            │ Publishes telemetry & logs
                            ▼
┌────────────────────────────────────────────────────────┐
│  Layer 4: TELEMETRY & VISUALIZATION LAYER              │
│  - Elastic Dashboard (Driver HUD on port 5800)         │
│  - AdvantageScope (3D field, robot poses, wpilog replay)│
│  - AlertManager (Persistent driver alerts, no console) │
└────────────────────────────────────────────────────────┘
```

---

### 3. Conventions That Differ from Default WPILib

Please pay close attention to these rules. They are strictly enforced across the codebase:

1. **Subsystems are Singletons, NOT WPILib Commands**:
   - Subsystems implement [`frc.robot.Interfaces.Subsystem`](../src/main/java/frc/robot/Interfaces/Subsystem.java) and manage themselves via [`SubsystemManager`](../src/main/java/frc/robot/Subsystems/SubsystemManager.java).
   - Do **not** convert our code to WPILib `SubsystemBase` or `Command` patterns.
2. **AdvantageKit IO Abstraction**:
   - Hardware calls (e.g. `CANSparkMax`, `LimelightHelpers`) never live directly inside the subsystem logic class.
   - All hardware calls are isolated inside an `IOSparkMax` or `IOLimelight` class behind an `@AutoLog` input struct. This allows the identical robot code to run in simulation with zero hardware present.
3. **Strict Blue-Origin Field Coordinates**:
   - Every coordinate, waypoint, target, and autonomous path is defined once for the **Blue Alliance** ($X=0$ at the Blue wall).
   - If running on Red, transform poses dynamically using [`Utils/AllianceFlipUtil.java`](../src/main/java/frc/robot/Utils/AllianceFlipUtil.java) ($X_{\text{red}} = \text{FIELD\_LENGTH} - X_{\text{blue}}$, $Y$ unchanged). Never hardcode Red coordinates.
4. **No Driver Station Console Prints (`System.out.println`)**:
   - Driver Station console spam obscures vital field communication.
   - Use [`Telemetry/Alert.java`](../src/main/java/frc/robot/Telemetry/Alert.java) and `AlertManager` to show alerts on the Elastic Dashboard banner, or use [`Subsystems/LEDs.java`](../src/main/java/frc/robot/Subsystems/LEDs.java) for driver signals.
5. **Tunable Constants via `TunableNumber`**:
   - Do not create raw `SmartDashboard.putNumber` fields for PID gains or setpoints.
   - Use [`Telemetry/TunableNumber.java`](../src/main/java/frc/robot/Telemetry/TunableNumber.java) to enable live tuning when `Constants.TUNING_MODE = true`.

---

### 4. Daily Developer Workflow

#### A. Running Unit Tests
We maintain an extensive automated JUnit 5 test suite (59 files, 539 tests):
```powershell
# Run the entire test suite
.\gradlew test --offline --no-daemon

# Run only a specific test class (much faster for quick iteration!)
.\gradlew test --offline --tests "frc.robot.Subsystems.ShooterTuningTest"
```

#### B. Running Desktop Simulation
You do not need a physical robot to test your code:
```powershell
.\gradlew simulateJava
```
- **WPILib SimGUI**: Opens automatically with the 2D field, motor outputs, and joystick inputs.
- **Elastic Dashboard**: Open [Elastic](https://github.com/Gold872/elastic-dashboard) and connect to `127.0.0.1`. The layout automatically mirrors `src/main/deploy/elastic-layout.json` (served on port 5800).
- **AdvantageScope**: Open AdvantageScope and connect to `127.0.0.1` on NetworkTables 4 to see real-time 3D robot pose, intake arm articulation, and shooting vectors.

#### C. Deploying to the RoboRIO
Connect to the robot via USB tether or radio WiFi:
```powershell
.\gradlew deploy
```

---

### 5. Multi-Agent Resource Coordination

Because multiple developers and AI agents collaborate within this repository, shared resources (`build/`, `.gradle/`, and WPILib network ports) are protected by advisory locks in `tools/lock/`:

| Resource | Guarded Tasks |
|---|---|
| `gradle-build` | `compileJava`, `test`, `dumpSimLaunch` |
| `sim-gui` | `simulateJava`, SimGUI |
| `sweep` | Headless rig sweeps (`tools/score/sweep.ps1`) |
| `deploy` | `gradlew deploy` |

Check lock status anytime:
```powershell
powershell -File tools/lock/status.ps1
```

If a build fails complaining that a `.dll` or `.jar` is locked by another process, check who owns the lock. **Never run `taskkill /IM java.exe` or `gradlew --stop`**, as this terminates active simulations and tests being run by your teammates!

---

## Verification

- **Prerequisites verified**: WPILib 2026.2.1 JDK compiles cleanly offline.
- **Test Suite**: 59 files / 539 tests pass cleanly (`0 FAILURES`, 2026-10-07).
- **Next review due**: 2026-11-06.

---

## Related

- [`AGENTS.md`](../../AGENTS.md): The authoritative rulebook for all developers and agents.
- [`docs/INDEX.md`](INDEX.md): The central directory of all guides and documents.
- [`ARCHITECTURE.md`](../ARCHITECTURE.md): Deep-dive into technical contracts and subsystem state machines.
- [`OPERATORS_GUIDE.md`](../OPERATORS_GUIDE.md): Driver and operator controller button bindings.
- [`docs/PIT_TUNING_CHECKLIST.md`](PIT_TUNING_CHECKLIST.md): Rapid 5-station pre-match pit checklist.
