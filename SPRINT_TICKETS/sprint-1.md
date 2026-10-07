# Sprint 1 Tickets — Framework + geometry + drive (goal: robot drives field-relative in sim)

Demo: push left stick in sim → robot drives; double-tap A re-zeroes. Total: 9 pts.

---

## S1-1 Main + Subsystem contract (1 pt) — depends: S0-3

**Tasks:**
1. `src/main/java/frc/robot/Main.java` (verbatim, never add statics):
   ```java
   package frc.robot;
   import edu.wpi.first.wpilibj.RobotBase;
   public final class Main {
     private Main() {}
     public static void main(String... args) { RobotBase.startRobot(Robot::new); }
   }
   ```
2. `src/main/java/frc/robot/Interfaces/Subsystem.java`: own contract (NOT WPILib's): `update()`, `initialize()`, `log()`, `isEnabled()`, `getName()`, extending `edu.wpi.first.wpilibj2.command.Subsystem` for the default `idle()`.

**Acceptance:** compiles; `Main` has zero fields/methods besides `main`. **Verify:** `.\gradlew compileJava --offline`

---

## S1-2 SubsystemManager + PortMap + Constants (2 pts) — depends: S1-1

**Tasks:**
1. `Subsystems/SubsystemManager.java`: static `List<Subsystem>` + `registerSubsystem`; `initializeSubsystems` (iterate a COPY to dodge concurrent-modification); `updateSubsystems` / `logSubsystems`. Wrap EVERY call in try/catch → `DriverStation.reportWarning("SubsystemManager isolated " + name + "." + phase, false)` so one throwing subsystem is skipped, not fatal. `Error` still propagates.
2. `Hardware/PortMap.java` (exact, single owner of all numbers): driver 0, operator 1; arm 10, wheels 14, hopper 9, encoder DIO 0; shooter L 12 / R 11, kicker 13. All SparkMax/NEO.
3. Trimmed `Data/Constants.java`: `MAX_SPEED = feetToMeters(15)` (≈4.57 m/s), `MAX_ROTATION_SPEED = 8.0` rad/s, `TRANSLATION_SLEW_RATE = 16`, `ROTATION_SLEW_RATE = 10`, `AUTO_DRIVE_KP = 10.0`, `AUTO_TURN_KP = 7.5` (I/D = 0). Skip vision/LED/delegate blocks.

**Acceptance:** compiles; review shows guarded loop + no duplicated CAN IDs. **Verify:** `.\gradlew compileJava --offline`

---

## S1-3 Alliance geometry (1 pt) — depends: S1-2

**Tasks:**
1. Hardcode meters: `FIELD_LENGTH 16.541`, `FIELD_WIDTH 8.069`, `HUB_Y 4.0346`, `BLUE_HUB_X 4.6256` (= `BLUE_ZONE_MAX_X`), `RED_HUB_X = LEN − BLUE` (= 11.9154 = `RED_ZONE_MIN_X`), `GOAL_HEIGHT 1.575`, `SHOOTING_MAX_DISTANCE 4.20`.
2. `Utils/AllianceFlipUtil.java`: `isRedAlliance()` (from `DriverStation.getAlliance()`); `apply(T/Pose/Rotation)` mirroring `X_red = LEN − X_blue` (rotation: `new Rotation2d(-cos, sin)`); `isPoseInAllianceZone(pose)` (Blue `x ≤ 4.6256`, Red `x ≥ 11.9154`, null-safe); `getDriverRelativeHeading(h)` (+180° on Red).
3. Tests: Blue `(2,4)` in-zone / `(6,4)` out; Red `(14.5,4)` in-zone / `(10,4)` out; pose-mirror of `(1,2,0°)` on Red = `(15.541,2,180°)`.

**Acceptance:** all 5 assertions pass. **Verify:** `.\gradlew test --offline --tests "frc.robot.Utils.AllianceFlipUtilTest"`

---

## S1-4 SwerveBase wrapper (5 pts) — depends: S1-2, S1-3

**Tasks:**
1. `Subsystems/SwerveBase.java` singleton (`getInstance`, synchronized), self-registers in constructor.
2. Build ONE YAGSL `SwerveDrive`: `new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(MAX_SPEED, startingPose)` — start Blue `(1,4,0°)` / Red `(16,4,180°)` from `DriverStation.getAlliance()`. `setHeadingCorrection(true)`; telemetry verbosity NONE.
3. Public API ONLY: `drive(Translation2d mps, double rotRadS, boolean fieldRelative)`, `driveFieldOriented(ChassisSpeeds)`, `stop()`, `zeroGyroWithAlliance()` (zero + snap 0°/180°), `getPose()`, `getHeading()`. `update()` = odometry refresh; `log()` = Field2d + pose to SmartDashboard.
4. Skip: vision fusion, brownout scaling, watchdogs, GlidePoints, path following.

**Acceptance:** sim teleop — left stick drives field-relative (away = away on both alliances); double-tap A re-zeroes; disable stops motors. **Verify:** `.\gradlew compileJava --offline`, then `.\gradlew simulateJava --offline` + joystick.
