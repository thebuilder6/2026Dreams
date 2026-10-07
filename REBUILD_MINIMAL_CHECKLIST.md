# Minimal Robot Rebuild — Detailed Self-Contained Checklist

Goal: rebuild a **drivable FRC robot** (swerve + intake + shooter + simple auto + simple teleop + AdvantageKit logging + desktop sim) from an **empty folder**, with **no access to the old code**. Every number, file path, API name, and code skeleton you need is in this file. If a line of code is not in this file, you do not need it.

**Skipped on purpose (do not rebuild):** AI decision engine, co-pilot, coach, vision (Limelight/Photon), GameSim fuel/score, multi-bot opponents, headless rig, advanced navigation (potential fields, stuck watchdogs, pathfinder), dashboard/LEDs/alerts UI, test mode, tuning-table UI.

**Conventions used below:** `CAN 12` = CAN bus device ID 12. `DIO 0` = roboRIO digital input port 0. Speeds in m/s, angles in the units stated (convert with `Math.toRadians` at boundaries — never mix). All field coordinates are **Blue-origin** (`X=0` at the Blue wall; Step 2).

---

## Prerequisites (install once)

1. Install **WPILib 2026** (VS Code extension + toolchain from wpilib.org). It creates two folders you depend on:
   - `C:\Users\Public\wpilib\2026\jdk` — the ONLY JDK you build with.
   - `C:\Users\Public\wpilib\2026\maven` — local library cache; what makes `--offline` builds work.
2. Every PowerShell shell, first command:
   ```powershell
   $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
   $env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
   ```
3. **Java 17** (comes from the JDK above). **GradleRIO `2026.2.1`** (declared in `build.gradle`, Step 0 — nothing to install, the wrapper downloads it once).
4. Rules: always the wrapper (`.\gradlew.bat`, never system `gradle`); always `--offline`; all Gradle commands run from the project folder (where `build.gradle` lives), never the repo root.
5. Install **AdvantageScope** (mechanical-advantage.org) for log viewing — needed in Step 9.

## Step 0 — Empty project scaffold

**0.1 Create the project.** In VS Code with the WPILib extension: Command Palette → "WPILib: Create a new robot project" → Template `Java - TimedRobot skeleton` → base package `frc.robot` → team `8334`. This generates the folder tree below; you will replace almost every file. Name the folder `MinimalRobot/`.

Expected tree ( memorize it — every later step writes into it):
```
MinimalRobot/
  build.gradle  settings.gradle  gradlew  gradlew.bat
  .wpilib/wpilib_preferences.json
  vendordeps/*.json
  src/main/java/frc/robot/
    Main.java  Robot.java  Teleop.java
    Data/Constants.java  Hardware/PortMap.java  Hardware/NeoSparkMaxMotor.java
    Interfaces/Subsystem.java  Interfaces/Actions.java
    Subsystems/SubsystemManager.java  Subsystems/SwerveBase.java
    Subsystems/Intake.java  Subsystems/Shooter.java
    Subsystems/intake/{IntakeConstants,IntakeIO,IntakeIOSparkMax,IntakeIOSim}.java
    Subsystems/shooter/{ShooterConstants,ShooterIO,ShooterIOSparkMax,ShooterIOSim}.java
    Subsystems/drive/{DriveIO,DriveIOSparkMax,DriveIOSim}.java
    Utils/AllianceFlipUtil.java
    Auto/{AutoMissionExecutor,AutoMissionChooser}.java
    Auto/Missions/MissionBase.java  Auto/Missions/Drive2mAndShoot.java
    Auto/Actions/{WaitAction,DriveDistanceAction}.java
  src/main/deploy/swerve/*.json   (8 YAGSL files)
```

**0.2 `.wpilib/wpilib_preferences.json`** (written by the creator; confirm contents):
```json
{ "enableCppIntellisense": false, "currentLanguage": "java", "projectYear": "2026", "teamNumber": 8334 }
```

**0.3 `settings.gradle`.** Plugin resolution MUST check the local WPILib maven first (this is what lets `--offline` work on a fresh machine):
```gradle
pluginManagement {
  repositories {
    mavenLocal(); gradlePluginPortal()
    // Windows: <PUBLIC>/wpilib/2026/maven  (PUBLIC=C:\Users\Public)
    // macOS/Linux: ~/wpilib/2026/maven
    maven { name = 'frcHome'; url = <that folder> }
  }
}
```

**0.4 `build.gradle`** (minimal; AdvantageKit processor line arrives in Step 9):
```gradle
plugins { id "java"; id "edu.wpi.first.GradleRIO" version "2026.2.1" }
java { sourceCompatibility = JavaVersion.VERSION_17; targetCompatibility = JavaVersion.VERSION_17 }
def ROBOT_MAIN_CLASS = "frc.robot.Main"
deploy { targets { roborio(getTargetTypeClass('Roborio')) {
  team = 8334
  artifacts {
    frcJava(getArtifactTypeClass('FRCJavaArtifact')) {}
    frcStaticFileDeploy(getArtifactTypeClass('FileTreeArtifact')) {
      files = project.fileTree('src/main/deploy'); directory = '/home/lvuser/deploy' }
  } } } }
dependencies {
  implementation wpi.java.deps.wpilib()       // WPILib
  implementation wpi.java.vendor.java()       // ALL vendordeps/*.json at once
  testImplementation 'org.junit.jupiter:junit-jupiter:5.10.1'
  testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
}
test { useJUnitPlatform() }
```

**0.5 `vendordeps/`.** WPILib extension → "Manage Vendor Libraries → Install new libraries (offline)": **YAGSL `2026.1.14`**, **REVLib `2026.0.5`**, **AdvantageKit `26.0.2`**, **WPILibNewCommands**. Do NOT install Choreo/Photon/Limelight/CTRE. Trap: YAGSL's JSON `requires` REVLib + Phoenix + Redux + Thrifty JSONs to exist — keep those stub JSON files in the folder even though no code calls them.

**0.6 `src/main/deploy/swerve/` (8 JSON files).** Generate with the YAGSL configurator against YOUR chassis (module CAN IDs, CANCoder offsets, wheel size, gear ratios). Never hand-guess the `absoluteEncoderOffset` — a wrong offset drives sideways. Anatomy of one module file (IDs below are EXAMPLES — yours come from the configurator):
```json
{ "drive": { "type": "sparkmax_neo", "id": 6 }, "angle": { "type": "sparkmax_neo", "id": 5 },
  "encoder": { "type": "cancoder", "id": 16 },
  "inverted": { "drive": true, "angle": true },
  "absoluteEncoderOffset": 350.59572,
  "location": { "front": 11.125, "left": 11.125 } }
```
`location` is inches from robot center (positive front/left). Shared `modules/physicalproperties.json`: drive gear `6.75`, wheel diameter `4` (inches), angle gear `21.4285714286`; current limits drive `40 A` / angle `20 A`; ramp `0.25 s`. Root `swervedrive.json`: `{ "imu": { "type": "navx" }, "invertedIMU": false, "modules": ["frontleft.json","frontright.json","backleft.json","backright.json"] }`.

**Verify:** `.\gradlew compileJava --offline` green on the untouched template before writing robot code.

## Step 1 — Core framework (5 small files, in order)

**1a. `Main.java`** — entry point; never add anything to it:
```java
package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;
public final class Main {
  private Main() {}
  public static void main(String... args) { RobotBase.startRobot(Robot::new); }
}
```

**1b. `Interfaces/Subsystem.java`** — the project's own contract (NOT WPILib's `Subsystem` class; ours adds lifecycle methods):
```java
package frc.robot.Interfaces;
public interface Subsystem extends edu.wpi.first.wpilibj2.command.Subsystem {
  void update();        // every 20 ms, called from Robot.robotPeriodic
  void initialize();    // once at startup
  void log();           // SmartDashboard / Logger output
  void simulationUpdate();            // default: {} (only sim IO overrides)
  default double getSimulationCurrentDraw() { return 0.0; }
  boolean isEnabled();
  String getName();
}
```

**1c. `Subsystems/SubsystemManager.java`** — static registry; subsystems self-register in their constructors so `Robot` never lists them:
```java
package frc.robot.Subsystems;
import java.util.*; import edu.wpi.first.wpilibj.DriverStation; import frc.robot.Interfaces.Subsystem;
public class SubsystemManager {
  private static final List<Subsystem> subsystems = new ArrayList<>();
  public static void registerSubsystem(Subsystem s) { subsystems.add(s); }
  public static void initializeSubsystems() { for (Subsystem s : new ArrayList<>(subsystems)) s.initialize(); }
  public static void updateSubsystems() { for (Subsystem s : subsystems) guarded(s, () -> s.update()); }
  public static void logSubsystems()    { for (Subsystem s : subsystems) guarded(s, () -> s.log()); }
  public static void simulationUpdateSubsystems() { for (Subsystem s : subsystems) guarded(s, () -> s.simulationUpdate()); }
  public static List<Subsystem> getSubsystems() { return subsystems; }
  private static void guarded(Subsystem s, Runnable r) {
    try { r.run(); } catch (Exception e) {
      DriverStation.reportWarning("SubsystemManager isolated " + s.getName() + ": " + e, false); }
  } // one bad subsystem is skipped, never fatal. Error still propagates.
}
```

**1d. `Hardware/PortMap.java`** — single owner of every port number. When wiring changes, ONLY this file changes:
```java
package frc.robot.Hardware;
public final class PortMap {
  private PortMap() {}
  public static final int DRIVER_CONTROLLER = 0, OPERATOR_CONTROLLER = 1; // USB ports on driver station
  public static final int INTAKE_ARM_MOTOR_ID = 10, INTAKE_WHEELS_MOTOR_ID = 14; // CAN
  public static final int HOPPER_MOTOR_CANID = 9;                                // CAN
  public static final int INTAKE_ENCODER_ID = 0;                                 // DIO 0 (throughbore absolute)
  public static final int SHOOTER_MOTOR_LEFT_ID = 12, SHOOTER_MOTOR_RIGHT_ID = 11; // CAN
  public static final int KICKER_MOTOR_ID = 13;                                  // CAN
}
```
All motors are REV NEO brushless on SparkMax controllers.

**1e. `Hardware/NeoSparkMaxMotor.java`** — one thin wrapper so IO classes never touch REV API directly. Constructor: `new SparkMax(CANID, MotorType.kBrushless)` inside try/catch (null on failure + console note, so sim/CI without hardware still constructs). Methods to provide: `configure(SparkMaxConfig)`, `setVoltage(double)`, `set(double speed)`, `stop()`, `getSpeed()` (RPM), `getPosition()` (rotations), `getAppliedVoltage()`, `getOutputCurrent()`, `getBusVoltage()`. Standard config per motor:
```java
SparkMaxConfig cfg = new SparkMaxConfig();
cfg.inverted(false);                                        // arm=true, right flywheel=true, else false
cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);              // kBrake: arm, hopper, kicker. kCoast: wheels, flywheels
cfg.smartCurrentLimit(40);                                  // flywheels 40, kicker 30, arm 40, rollers 30
motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

**1f. `Data/Constants.java` (trimmed)** — only what the minimal robot reads:
```java
package frc.robot.Data;
import edu.wpi.first.math.util.Units;
public class Constants {
  public enum Mode { REAL, SIM, REPLAY }
  public static final Mode currentMode = edu.wpi.first.wpilibj.RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static Mode getMode() { return currentMode; }
  public static final double MAX_SPEED = Units.feetToMeters(15); // ~4.57 m/s
  public static final double MAX_ROTATION_SPEED = 8.0;           // rad/s
  public static class OperatorConstants {
    public static final double TRANSLATION_SLEW_RATE = 16; // m/s^2
    public static final double ROTATION_SLEW_RATE = 10;    // rad/s^2
  }
  public static class AutonConstants {
    public static final double AUTO_DRIVE_KP = 10.0, AUTO_TURN_KP = 7.5; // I and D = 0
  }
}
```

**Verify:** `.\gradlew compileJava --offline`.

## Step 2 — Field geometry (numbers + one rule)

**The rule (entire codebase obeys it):** define every point in **Blue coordinates** (`X=0` at Blue wall). For Red, mirror: `X_red = FIELD_LENGTH − X_blue` (Y unchanged, heading + 180°). Never keep two hardcoded coordinate sets.

**Hardcode (meters):**
```java
FIELD_LENGTH = 16.541;  FIELD_WIDTH = 8.069;
HUB_Y = 4.0346;                       // hub centerline across field width
BLUE_HUB_X = 4.6256;                  // ALSO the Blue zone edge (zone edge IS the hub plane)
RED_HUB_X = FIELD_LENGTH - BLUE_HUB_X; // 11.9154, ALSO the Red zone edge
GOAL_HEIGHT = 1.575;                  // hub rim height
SHOOTER_MOUNT_HEIGHT = 0.53;  HEIGHT_DIFFERENCE = GOAL_HEIGHT - 0.53; // ball flight math
SHOOTING_MAX_DISTANCE = 4.20;         // validated reliable envelope (not the 6.5 m geometric limit)
```
Zone test: Blue in-zone ⇔ `pose.x ≤ 4.6256`; Red ⇔ `pose.x ≥ 11.9154`. The shooter refuses to fire outside its own zone.

**`Utils/AllianceFlipUtil.java`** — copy this whole file; it is the only place that reads `DriverStation.getAlliance()`:
```java
package frc.robot.Utils;
import edu.wpi.first.math.geometry.*; import edu.wpi.first.wpilibj.DriverStation;
public class AllianceFlipUtil {
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red; }
  public static Translation2d apply(Translation2d t, boolean isRed) {
    return isRed ? new Translation2d(16.541 - t.getX(), t.getY()) : t; }
  public static Rotation2d apply(Rotation2d r, boolean isRed) {   // mirror across field midline
    return isRed ? new Rotation2d(-r.getCos(), r.getSin()) : r; }
  public static Pose2d apply(Pose2d p, boolean isRed) {
    return isRed ? new Pose2d(apply(p.getTranslation(), true), apply(p.getRotation(), true)) : p; }
  public static Rotation2d getDriverRelativeHeading(Rotation2d driverHeading) {
    return isRedAlliance() ? driverHeading.plus(Rotation2d.fromDegrees(180)) : driverHeading; }
  public static boolean isPoseInAllianceZone(Pose2d pose) {
    if (pose == null) return false;
    double x = pose.getX();
    return isRedAlliance() ? (x >= 11.9154) : (x <= 4.6256); }
}
```
Why `new Rotation2d(-cos, sin)`: mirroring X negates the cosine component (= 180° − θ). Why `getDriverRelativeHeading`: on Red, "away from driver" is −X (180° in field coords), so driver intents rotate half a turn.

**Verify:** unit tests — Blue `(2,4)` in / `(6,4)` out; Red `(14.5,4)` in / `(10,4)` out; mirror of `(1,2,0°)` on Red = `(15.541,2,180°)`: `.\gradlew test --offline --tests "frc.robot.Utils.*"`.

## Step 3 — Swerve drive (YAGSL wrapper)

**`Subsystems/SwerveBase.java`** — singleton, self-registers. Constructor builds ONE YAGSL drive:
```java
import swervelib.parser.SwerveParser; import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.Filesystem; import java.io.File;
boolean blue = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(true);
Pose2d start = blue ? new Pose2d(1.0, 4.0, Rotation2d.fromDegrees(0))
                   : new Pose2d(16.0, 4.0, Rotation2d.fromDegrees(180));
swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
    .createSwerveDrive(Constants.MAX_SPEED, start);
swerveDrive.setHeadingCorrection(true);
```
Public API (only these — everything else stays inside):
- `drive(Translation2d mps, double rotRadS, boolean fieldRelative)` → `swerveDrive.drive(mps, rotRadS, fieldRelative, false)` (`false` = closed-loop velocity; never open-loop percent).
- `driveFieldOriented(ChassisSpeeds s)` → `swerveDrive.driveFieldOriented(s)`.
- `stop()` → zero speeds. `getPose()` → `swerveDrive.getPose()`. `getHeading()` → `swerveDrive.getYaw()`.
- `zeroGyroWithAlliance()` → `swerveDrive.zeroGyro()` then reset heading expectation to 0° Blue / 180° Red.
- `update()` = nothing extra needed (YAGSL odometry runs internally); `log()` = Field2d + pose to SmartDashboard. `setMotorBrake(true/false)` sets all modules idle mode (brake when disabled).
- IO split (Step 9): constructor picks `RobotBase.isSimulation() ? new DriveIOSim(swerveDrive) : new DriveIOSparkMax(...)`; YAGSL simulates its own modules so `DriveIOSim` just reads module velocity/position + gyro yaw into `@AutoLog` inputs.

Skip: vision fusion, brownout scaling, watchdogs, GlidePoints, path following.

**Verify:** `compileJava`, then `simulateJava` + joystick — left stick drives field-relative on both alliances; double-tap A re-zeroes.

## Step 4 — Intake (arm + rollers)

**4a. `Subsystems/intake/IntakeConstants.java`** (exact values — do not tune yet):
```java
up/stow 347°, down/ground 250°, horizontal reference 250° (gravity FF measured from here)
MAX_ARM_VELOCITY 400 °/s, MAX_ARM_ACCELERATION 400 °/s²
arm PID kP 0.1 / kI 0 / kD 0.01 ;  FF kS 0.2 / kG 0.34 / kV 0 / kA 0
roller duty 0.7, hopper duty 0.5 ;  jam: current >30 A for >0.5 s → auto-reverse 1.0 s
soft stops: cut arm volts below 240° / above 355°
```

**4b. `Subsystems/Intake.java`** — singleton. Arm control = `ProfiledPIDController` (constraints above, `enableContinuousInput(0, 360)` so 359°→0° wraps) + `ArmFeedforward`, both in DEGREES-land converted at the FF boundary:
```java
// each update():
io.updateInputs(inputs); Logger.processInputs("Intake", inputs);
currentPosition = inputs.armPositionDeg;               // from absolute encoder (sim: ArmSim)
pivot.setGoal(goal);                                   // goal = 347 or 250
double pid = pivot.calculate(currentPosition);
double ff = feedforward.calculate(Math.toRadians(currentPosition - 250.0),
                                  Math.toRadians(pivot.getSetpoint().velocity));
io.setArmVoltage(pid + ff);
// rollers per state:
Intaking:        io.setRollerSpeed(0.7);  io.setHopperSpeed(0.5);
Reversed etc:    io.setRollerSpeed(-0.7); io.setHopperSpeed(-0.5);
else:            io.setRollerSpeed(0);    io.setHopperSpeed(0);
```
States: `Standby` (347, off), `Down` (250, off), `Intaking` (250, on), `Reversed`/`StandbyReversed` (reversed), `Disabled`. `setState(String)` parses case-insensitively, defaults `Standby`; leaving `Disabled` resets the profiled PID to the current position (avoids a lurch).
Jam: `Debouncer(0.5 s, kRising)` fed by `rollerCurrent > 30` → start 1 s eject timer (both reversed), then resume previous state.

**4c. `Subsystems/intake/IntakeIOSparkMax.java`.** Arm SparkMax CAN 10 (inverted, brake, 40 A limit); wheels CAN 14 (inverted, coast, 30 A); hopper CAN 9 (brake, 30 A); `DutyCycleEncoder` on DIO 0:
```java
pivotEncoder = new DutyCycleEncoder(PortMap.INTAKE_ENCODER_ID); // .get() returns 0..1 rotations
double unmodified = pivotEncoder.get() * 360.0;
inputs.armPositionDeg = MathUtil.inputModulus(INTAKE_POSITION_OFFSET /*276*/ - unmodified, 0, 360);
```
`INTAKE_POSITION_OFFSET` (276°) is YOUR rig's mounting offset — measure once (arm at known angle, solve offset = known + reading), then never touch. `setRollerSpeed(speed)` writes duty (`set(speed)`); `setArmVoltage` writes volts. Sim twin `IntakeIOSim` = `SingleJointedArmSim` + duty roller model (Step 9).

**Verify:** X toggles 347↔250 smoothly (<1 s settle); LT spins both forward; B reverses; held stall (>30 A, 0.5 s) auto-reverses exactly 1 s.

## Step 5 — Shooter (flywheels + kicker)

**5a. `Subsystems/shooter/ShooterConstants.java`:** kicker `12 V` full pulse; flywheel FF `kS 0 / kV 0.0022 V-per-RPM / kA 0`; PID `kP 0.0007 / kI 0 / kD 0`, integrator clamp ±1.5; idle `60 RPM`.

**5b. `Subsystems/Shooter.java`** — singleton. Two `PIDController` + `SimpleMotorFeedforward` loops (L CAN 12 normal, R CAN 11 inverted — inversion lives in the Spark config, Step 1e):
```java
// each update():
io.updateInputs(inputs); Logger.processInputs("Shooter", inputs);
voltsL = ffL.calculate(targetRpmL) + pidL.calculate(inputs.leftVelocityRPM, targetRpmL);
voltsR = ffR.calculate(targetRpmR) + pidR.calculate(inputs.rightVelocityRPM, targetRpmR);
io.setFlywheelVoltages(voltsL, voltsR);   // zero both when targets are 0
```
Distance→RPM table — interpolate LINEARLY between these points (right ≈ +50 for spin); `InterpolatingDoubleTreeMap` does this for free:
```
1.20 m → 2400/2450   1.92 → 2700/2750   2.47 → 2900/2950   3.05 → 3500/3550
3.48 → 3550/3600     4.18 → 3750/3800   5.00 → 4100/4150   6.00 → 4500/4550
```
`calculateShootingSolution(pose)`: hub = Blue `(4.6256,4.0346)` or Red mirror; shooter point = robot translation + `SHOOTER_OFFSET −0.2032 m` rotated by heading; `angle = (hub − shooter).getAngle()`; RPMs from table; `possible = inZone && 1.2 ≤ dist ≤ 6.5 && dist·tan(70°) − 1.045 > 0`.
**Firing gate (ALL must hold):** `possible && isAtCorrectSpeed()`. `isAtCorrectSpeed` = both errors `<150 RPM` (ignore targets ≤100), LATCHED — stays true until either error `>750`. Then `io.setKickerVoltage(12)`, else 0. API: `setTargetRPM(l,r)`, `shoot()` (kicker on), `prepareToShoot()` (spool only), `stop()`, `manualFire()` (3200/3200 fallback when no solution).

**Verify:** spool steps settle ±150 RPM ≤2 s at 2–5 m; kicker NEVER fires out-of-zone or under-speed (test both negatives); fires in-zone at speed.

## Step 6 — Teleop (drivers + input math)

**`Teleop.java`** owns both `XboxController`s (ports 0/1 from `PortMap`) and calls the three subsystems. Method map: `init()` (reset all state) → `teleopPeriodic()` = `readControllers()` → e-stop check → `driveBaseControl()` + `intakeControl()` + `shooterControl()`.

**Input cleanup (copy verbatim — order matters):**
```java
public static double shapeInput(double x) { // cubic: precision at center, 100% at full throw
  double a = Math.abs(x); return Math.signum(x) * (0.7*a*a*a + 0.3*a); }
// translation: circular (not square!) deadband 0.08, then shape along the same angle:
double mag = Math.hypot(x, y);
Translation2d out = (mag < 0.08) ? new Translation2d()
  : new Translation2d(x/mag*shapeInput(Math.min(1,(mag-0.08)/0.92)),
                      y/mag*shapeInput(Math.min(1,(mag-0.08)/0.92)));
// rotation: 1D deadband 0.06, then shapeInput.
// then: slew limit (16 m/s² vector + 10 rad/s²), scale by MAX_SPEED / MAX_ROTATION_SPEED,
// on Red negate X and Y ("push away" always = away from driver).
```
Why circular: a square deadband clips stick corners and crawls motors at tiny inputs.

**Bindings:** left stick drive · right stick rotate · **X** (edge-toggle: `if (x && !lastX) deployed = !deplo
...[truncated 6154 chars]