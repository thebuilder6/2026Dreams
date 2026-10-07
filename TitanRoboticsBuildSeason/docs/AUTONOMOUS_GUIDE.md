---
title: Autonomous & Trajectory Pipeline Guide
audience: [human, ai, programming-leads]
owner: programming-leads
last_verified: 2026-10-07
status: authoritative
---

# Autonomous & Trajectory Pipeline Guide — FRC Team 8334

Complete architectural manual for Team 8334's autonomous routine framework, Choreo trajectory integration, action sequencing, event markers, and dynamic mission discovery.

---

## Scope

### What this document covers
- Architecture of the custom Action/Mission framework in `frc.robot.Auto` ([`AutoMissionChooser`](../src/main/java/frc/robot/Auto/AutoMissionChooser.java), [`AutoMissionExecutor`](../src/main/java/frc/robot/Auto/AutoMissionExecutor.java), [`MissionBase`](../src/main/java/frc/robot/Auto/Missions/MissionBase.java), [`Actions`](../src/main/java/frc/robot/Interfaces/Actions.java)).
- Creating, generating, and exporting Choreo trajectories (`deploy/choreo/*.traj`).
- Autonomous trajectory execution via [`FollowChoreoPath`](../src/main/java/frc/robot/Auto/Actions/FollowChoreoPath.java) with automatic Red-alliance mirroring and continuous heading PID.
- Parallel and sequential action coordination ([`SeriesAction`](../src/main/java/frc/robot/Auto/Actions/SeriesAction.java), [`ParallelAction`](../src/main/java/frc/robot/Auto/Actions/ParallelAction.java), [`ParallelRaceAction`](../src/main/java/frc/robot/Auto/Actions/ParallelRaceAction.java)).
- Choreo event markers and inline triggers using [`WaitUntilMarkerAction`](../src/main/java/frc/robot/Auto/Actions/WaitUntilMarkerAction.java) and `FollowChoreoPath.bind(...)`.
- Automatic trajectory discovery into the dashboard chooser.
- Step-by-step tutorial for writing, testing, and verifying a new autonomous mission.

### What this document does NOT cover
- WPILib Command-based auto routines (Team 8334 uses custom `MissionBase` / `Actions`; see [`AGENTS.md`](../../AGENTS.md) architectural rules).
- Swerve drive kinematic configuration and physical module constants (see [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md)).
- Dynamic real-time obstacle avoidance and Jev AI decision engine during teleop (see [`ARCHITECTURE.md`](../ARCHITECTURE.md) §3A/§3J).

---

## Content

### 1. Framework Architecture & Design Philosophy

Team 8334 does not use WPILib `Command` / `Subsystem` structures. Autonomous routines execute within a dedicated background thread managed by [`AutoMissionExecutor`](../src/main/java/frc/robot/Auto/AutoMissionExecutor.java):

```
┌────────────────────────────────────────────────────────┐
│                   Robot.java Lifecycle                 │
│  autonomousInit() ───► AutoMissionExecutor.start()     │
│  teleopInit() / disabledInit() / testInit() ──► stop() │
└───────────────────────────┬────────────────────────────┘
                            │ Spawns Thread
                            ▼
┌────────────────────────────────────────────────────────┐
│             AutoMissionExecutor Worker Thread          │
│  1. Respects user-configured Start Delay (seconds)     │
│  2. Calls selected MissionBase.run()                   │
│  3. Catches AutoMissionEndedException upon disable/stop│
└───────────────────────────┬────────────────────────────┘
                            │ Executes
                            ▼
┌────────────────────────────────────────────────────────┐
│              MissionBase (e.g. AdvancedChoreoMission)  │
│  runAction(Action action) ──► 50 Hz loop               │
│    - action.start()                                    │
│    - while (!action.isFinished()) { action.update(); } │
│    - action.done()                                     │
└────────────────────────────────────────────────────────┘
```

#### Why Custom Actions?
1. **Deterministic Sequential Logic**: Standard commands often obscure sequential step ordering behind nested decorator compositions. Custom `MissionBase.runAction(...)` reads strictly sequentially top-to-bottom.
2. **Explicit Concurrency & Resource Conflict Protection**: [`ParallelAction`](../src/main/java/frc/robot/Auto/Actions/ParallelAction.java) and [`ParallelRaceAction`](../src/main/java/frc/robot/Auto/Actions/ParallelRaceAction.java) validate subsystem requirements contracts (`Actions.getSubsystemRequirements()`). If two parallel branches attempt to command the same subsystem (e.g., two actions commanding `"Intake"` simultaneously), the runner emits an immediate `DriverStation.reportWarning(...)` warning to catch race conditions during testing.
3. **50.0 Hz Microsecond Loop Pacing**: `MissionBase.runAction` paces action updates using monotonic FPGA hardware timestamps (`Timer.getFPGATimestamp()`) targeting exact $20.0\text{ ms}$ intervals, eliminating 15–30 ms operating system thread sleep jitter.
4. **Clean Interruption**: Mode transitions ([`AutoMissionExecutor.stop()`](../src/main/java/frc/robot/Auto/AutoMissionExecutor.java)) immediately invoke `mThread.interrupt()`, restoring thread interrupt flags and evaluating `isActiveWithThrow()` immediately after `action.done()` to ensure zero thread hang into teleop.

---

### 2. Choreo Trajectory Pipeline

Trajectories are designed in **[Choreo](https://choreo.autos)**, an open-source time-optimal trajectory generator for swerve robots.

#### Static Trajectory Pre-Caching
Disk I/O and JSON parsing during the first tick of autonomous cause loop overruns. [`AutoMissionChooser.registerChoreoMissions()`](../src/main/java/frc/robot/Auto/AutoMissionChooser.java) warms the trajectory cache (`FollowChoreoPath.warmCache`) during robot boot, guaranteeing trajectories are pre-deserialized in memory before the match starts.

#### Exporting & File Structure
- Save your Choreo project file (`ChoreoPlanner.chor` / `AutoMissions.chor`) in `src/main/deploy/choreo/`.
- Export generated trajectories (`.traj` files) directly into `src/main/deploy/choreo/`:
  - `AdvancedLeftStart.traj`
  - `DepotPath.traj`
  - `DepotToShootPath.traj`
  - `MoveForward.traj`
  - `ShootPath.traj`

#### Coordinate Geometry & Alliance Mirroring
Choreo trajectories are generated using standard WPILib field dimensions:
- Trajectory coordinates are Blue-origin.
- When running on Red Alliance, [`FollowChoreoPath`](../src/main/java/frc/robot/Auto/Actions/FollowChoreoPath.java#L116) samples via `trajectory.sampleAt(clampedTime, isRedAlliance())`. Choreo automatically mirrors $X$ position and inverts $Y$ velocity and heading to provide symmetric execution from the Red driver station.

---

### 3. Core Action Library

All actions implement [`frc.robot.Interfaces.Actions`](../src/main/java/frc/robot/Interfaces/Actions.java):

| Action Class | Constructor / Usage | Description |
|---|---|---|
| [`FollowChoreoPath`](../src/main/java/frc/robot/Auto/Actions/FollowChoreoPath.java) | `new FollowChoreoPath(String trajName, boolean resetOdometry)` | Follows Choreo trajectory using Cartesian PID ($kP=10.0$) and continuous heading PID. Features static trajectory pre-caching and dual-gated (time + spatial Euclidean distance) marker triggers. |
| [`WaitUntilMarkerAction`](../src/main/java/frc/robot/Auto/Actions/WaitUntilMarkerAction.java) | `new WaitUntilMarkerAction(FollowChoreoPath path, String marker [, double toleranceMeters])` | Blocks execution until the referenced trajectory triggers or reaches a named marker (default 0.45 m spatial tolerance with monotonic latching). |
| [`AutoAimAction`](../src/main/java/frc/robot/Auto/Actions/AutoAimAction.java) | `new AutoAimAction(FollowChoreoPath path, double timeoutSec)` | Dynamically overrides trajectory rotation to aim swerve at the Hub while spooled. |
| [`ShootAction`](../src/main/java/frc/robot/Auto/Actions/ShootAction.java) | `new ShootAction(double timeoutSec)` | Spools flywheels, checks RPM tolerance (<150 RPM) and heading (<3°), pulses kicker. |
| [`IntakeAction`](../src/main/java/frc/robot/Auto/Actions/IntakeAction.java) | `new IntakeAction(IntakeState state)` | Commands intake arm position (Ground / Standby) and roller state. |
| [`BranchAction`](../src/main/java/frc/robot/Auto/Actions/BranchAction.java) | `new BranchAction(Supplier<Boolean> condition, Action trueAction, Action falseAction)` | Evaluates a dynamic condition at launch and executes the chosen branch for adaptive decision-making. |
| [`WaitForBallAction`](../src/main/java/frc/robot/Auto/Actions/WaitForBallAction.java) | `new WaitForBallAction([double timeoutSec])` | Gated sensor action that finishes when game piece acquisition is detected (ideal for race sweeps). |
| [`WaitAction`](../src/main/java/frc/robot/Auto/Actions/WaitAction.java) | `new WaitAction(double seconds)` | Pauses the calling sequence for the specified duration. |
| [`LambdaAction`](../src/main/java/frc/robot/Auto/Actions/LambdaAction.java) | `new LambdaAction(() -> { ... })` | Executes an instantaneous one-cycle Java lambda function. |
| [`SeriesAction`](../src/main/java/frc/robot/Auto/Actions/SeriesAction.java) | `new SeriesAction(Action... actions)` | Runs provided actions in strict sequential order. |
| [`ParallelAction`](../src/main/java/frc/robot/Auto/Actions/ParallelAction.java) | `new ParallelAction(Action... actions)` | Runs all actions simultaneously; finishes when **all** actions finish. Guards against subsystem resource conflicts. |
| [`ParallelRaceAction`](../src/main/java/frc/robot/Auto/Actions/ParallelRaceAction.java) | `new ParallelRaceAction(Action... actions)` | Runs all actions simultaneously; finishes as soon as **any** action finishes. Guards against subsystem resource conflicts. |

---

### 4. Creating a New Autonomous Mission

Follow this step-by-step walkthrough to author a custom routine:

#### Step 1: Create the Mission Class
Create a new file in `src/main/java/frc/robot/Auto/Missions/`:

```java
package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.*;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

@AutoMission(name = "Two Ball Depot Sweep")
public class TwoBallDepotMission extends MissionBase {

    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Initialize trajectory and reset odometry to start pose
        FollowChoreoPath driveToDepot = new FollowChoreoPath("DepotPath", true);

        // 2. Drive to depot while deploying intake at the 'DeployIntake' marker
        runAction(new ParallelAction(
            driveToDepot,
            new SeriesAction(
                new WaitUntilMarkerAction(driveToDepot, "DeployIntake"),
                new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.INTAKING))
            )
        ));

        // 3. Drive back to scoring position and shoot
        FollowChoreoPath driveToHub = new FollowChoreoPath("DepotToShootPath", false);
        runAction(new ParallelAction(
            driveToHub,
            new SeriesAction(
                new WaitUntilMarkerAction(driveToHub, "SpoolFlywheels"),
                new LambdaAction(() -> Shooter.getInstance().prepareToShoot()),
                new WaitUntilMarkerAction(driveToHub, "Fire"),
                new ShootAction(2.0)
            )
        ));

        // 4. Ensure actuators are cleanly safely stopped
        runAction(new LambdaAction(() -> {
            Shooter.getInstance().stop();
            Intake.getInstance().setState(Intake.IntakeState.STANDBY);
            SwerveBase.getInstance().stop();
        }));
    }
}
```

#### Step 2: Register the Mission
Open [`AutoMissionChooser.java`](../src/main/java/frc/robot/Auto/AutoMissionChooser.java) and register your class:
```java
// In AutoMissionChooser constructor:
registerMission(TwoBallDepotMission.class);
```
The `@AutoMission(name = "...")` annotation supplies the human-readable display string that appears in Elastic Dashboard.

#### Step 3: Dynamic Choreo Trajectory Auto-Discovery
If you export a trajectory `.traj` to `deploy/choreo/` and do *not* write a custom Java class for it, [`AutoMissionChooser.registerChoreoMissions()`](../src/main/java/frc/robot/Auto/AutoMissionChooser.java#L82-L96) automatically discovers it at boot time and registers it as a [`DynamicChoreoMission`](../src/main/java/frc/robot/Auto/Missions/DynamicChoreoMission.java) driving that path without mechanism triggers.

---

#### Mission Catalog

| Dashboard name | Trajectories | Behavior |
|---|---|---|
| Mobility (Drive Forward) | `MoveForward` | Taxi across the auto line only |
| Subwoofer Shoot & Leave | `MoveForward` | Shoot preloads (3.5 s), then taxi |
| Fast Depot Cycle | `DepotPath`, `DepotToShootPath` | Intake at depot, wait ≤1.5 s for ball, return, shoot |
| Adaptive Depot Sweep | `DepotPath`, `DepotToShootPath` | Same, but `BranchAction` on `hasGamePiece()`: shoot if ball acquired, else stow and stop |
| Delayed Partner Shoot | `ShootPath` | Wait 4 s (yield Hub to partner), drive, shoot |
| Shoot & Trench Disruption | `TrenchSweep` | Shoot preloads from Blue zone, then traverse South trench to midfield intaking without mid-auto odometry reset |
| Trench Midfield Disruptor | `TrenchSweep` | Traverse South trench from legal Blue alliance line (X=3.40 m, Y=0.65 m) to midfield with intake down |
| Centerline Sweep & Leave | `MoveForward` | Taxi with intake down |
| DepotShootMission, ShooterMission, ExampleMission, Advanced Choreo Shot | — | Original missions (names unchanged) |

All paths are Blue-origin and mirrored for Red by Choreo/`AllianceFlipUtil`. `TrenchSweep` begins legally in the Blue Alliance Zone (X=3.40 m, Y=0.65 m) and sweeps east through the South trench corridor to neutral midfield (X=7.60 m) safely before the FRC G201 centerline boundary, eliminating the illegal cross-field odometry reset previously inherited from `OpponentPath`.

### 5. Driver Dashboard Controls & Tuning

On the Elastic Dashboard (and SmartDashboard):
- `Auto Mission`: Dropdown chooser populated with registered missions.
- `Auto Delay (seconds)`: Configurable wait time (0.0 to 10.0 s) executed by `AutoMissionExecutor` before `routine()` begins (ideal for coordinating with alliance partners).
- `Current Action System`: Real-time telemetry reporting the active action class name.

---

## Verification

- **Unit Tests**:
  - [`AutoMissionChooserTest.java`](../src/test/java/frc/robot/Auto/AutoMissionChooserTest.java) pins registry discovery, delay clamping, and mission resolution.
  - [`AutoMissionExecutorTest.java`](../src/test/java/frc/robot/Auto/AutoMissionExecutorTest.java) pins worker thread lifecycle, mode transition safety, and interruption handling.
  - [`AutoEnhancementsTest.java`](../src/test/java/frc/robot/Auto/AutoEnhancementsTest.java) pins spatial marker gating, trajectory cache lifecycle, resource conflict warnings, `BranchAction`, and `WaitForBallAction`.
- **Simulation Validation**:
  - Run `.\gradlew simulateJava` from `TitanRoboticsBuildSeason/`.
  - In SimGUI, set Autonomous Mode and observe the virtual robot execute the selected Choreo trajectory on the AdvantageScope 2D/3D field.
- **Verified against**: JUnit 5 full test suite clean with `--rerun-tasks` (53 result files / 503 tests, 0 failures, 2026-10-07). Mission behaviour itself is only SimGUI-validated, not unit-tested.
- **Next review due**: 2026-11-06.

---

## Related

- [`ARCHITECTURE.md`](../ARCHITECTURE.md): System design, 4-layer architecture, subsystem singletons.
- [`OPERATORS_GUIDE.md`](../OPERATORS_GUIDE.md): Driver station control layout and teleop state machines.
- [`docs/SWERVE_TUNING_GUIDE.md`](SWERVE_TUNING_GUIDE.md): Swerve kinematics, module offsets, and trajectory tracking PID.
- [`docs/INDEX.md`](INDEX.md): Central documentation directory.
