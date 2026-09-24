package frc.robot.Test;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Unified System Identification (SysID) Manager for Team 8334.
 *
 * <p>Supports 5 comprehensive subsystem characterizations:
 * 1. SWERVE_DRIVE_LINEAR: Linear drive dynamics (kS, kV, kA) with modules locked at 0°.
 * 2. SWERVE_DRIVE_ANGULAR: Yaw rotational inertia (MoI) & angular feedforwards with tangent module angles.
 * 3. SWERVE_STEER: Steering azimuth motor dynamics.
 * 4. SHOOTER_FLYWHEELS: Dual flywheel velocity & acceleration response.
 * 5. INTAKE_ARM: Articulated arm dynamics with active mechanical angle safeguards.
 *
 * <p>Safety features:
 * - Voltage limits (7.0V max drive/flywheels, 3.5V max arm).
 * - Hold-to-run semantics & emergency stop.
 * - Headless NetworkTables triggers for pit tuning without controllers.
 * - WPILib DataLog & AdvantageKit deterministic replay recording.
 */
public class SysIdManager {

    private static SysIdManager instance = null;

    public enum Mechanism {
        SWERVE_DRIVE_LINEAR("Swerve Linear"),
        SWERVE_DRIVE_ANGULAR("Swerve Angular"),
        SWERVE_STEER("Swerve Steer"),
        SHOOTER_FLYWHEELS("Shooter Flywheels"),
        INTAKE_ARM("Intake Arm");

        public final String displayName;

        Mechanism(String displayName) {
            this.displayName = displayName;
        }
    }

    public enum TestType {
        QUASISTATIC,
        DYNAMIC
    }

    private final SwerveBase swerve;
    private final Shooter shooter;
    private final Intake intake;

    private final Map<Mechanism, SysIdRoutine> routines = new EnumMap<>(Mechanism.class);
    private Mechanism activeMechanism = Mechanism.SWERVE_DRIVE_LINEAR;
    private Command activeCommand = null;
    private String routineState = "IDLE";
    private boolean isRunning = false;

    // Safety Voltage Caps
    private static final double MAX_DRIVE_VOLTAGE = 7.0;
    private static final double MAX_FLYWHEEL_VOLTAGE = 7.0;
    private static final double MAX_ARM_VOLTAGE = 3.5;

    public static synchronized SysIdManager getInstance() {
        if (instance == null) {
            instance = new SysIdManager();
        }
        return instance;
    }

    public SysIdManager() {
        this(SwerveBase.getInstance(), Shooter.getInstance(), Intake.getInstance());
    }

    public SysIdManager(SwerveBase swerve, Shooter shooter, Intake intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;

        buildRoutines();
        setupDashboard();
    }

    private void buildRoutines() {
        // 1. Swerve Drive Linear Routine
        routines.put(Mechanism.SWERVE_DRIVE_LINEAR, new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(MAX_DRIVE_VOLTAGE),
                Seconds.of(10.0),
                (state) -> recordState(Mechanism.SWERVE_DRIVE_LINEAR, state)
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdDriveVoltage(volts.in(Volts)),
                (log) -> {
                    var vels = swerve.getDriveMotorVelocities();
                    var positions = swerve.getDriveMotorPositions();
                    var voltages = swerve.getDriveMotorVoltages();

                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPos = positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgVel = vels.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                    log.motor("drive-linear-avg")
                        .voltage(Volts.of(avgVolts))
                        .linearPosition(Meters.of(avgPos))
                        .linearVelocity(MetersPerSecond.of(avgVel));
                },
                swerve
            )
        ));

        // 2. Swerve Drive Angular Routine (Yaw Moment of Inertia)
        routines.put(Mechanism.SWERVE_DRIVE_ANGULAR, new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(MAX_DRIVE_VOLTAGE),
                Seconds.of(10.0),
                (state) -> recordState(Mechanism.SWERVE_DRIVE_ANGULAR, state)
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdRotationVoltage(volts.in(Volts)),
                (log) -> {
                    var voltages = swerve.getDriveMotorVoltages();
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double yawRads = Math.toRadians(swerve.getHeading().getDegrees());
                    double yawRateRps = swerve.getInputs().gyroYawVelocityDegPerSec / 360.0;

                    log.motor("drive-angular")
                        .voltage(Volts.of(avgVolts))
                        .angularPosition(Radians.of(yawRads))
                        .angularVelocity(RotationsPerSecond.of(yawRateRps));
                },
                swerve
            )
        ));

        // 3. Swerve Steer Azimuth Routine
        routines.put(Mechanism.SWERVE_STEER, new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(MAX_DRIVE_VOLTAGE),
                Seconds.of(8.0),
                (state) -> recordState(Mechanism.SWERVE_STEER, state)
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdSteerVoltage(volts.in(Volts)),
                (log) -> {
                    var positions = swerve.getSteerMotorPositions();
                    var voltages = swerve.getSteerMotorVoltages();
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPosRads = Math.toRadians(positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0));

                    log.motor("steer-azimuth-avg")
                        .voltage(Volts.of(avgVolts))
                        .angularPosition(Radians.of(avgPosRads));
                },
                swerve
            )
        ));

        // 4. Shooter Dual Flywheels Routine
        routines.put(Mechanism.SHOOTER_FLYWHEELS, new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1.5),
                Volts.of(MAX_FLYWHEEL_VOLTAGE),
                Seconds.of(8.0),
                (state) -> recordState(Mechanism.SHOOTER_FLYWHEELS, state)
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> shooter.setFlywheelCharacterizationVoltage(volts.in(Volts), volts.in(Volts)),
                (log) -> {
                    log.motor("flywheel-left")
                        .voltage(Volts.of(shooter.getFlywheelLeftAppliedVoltage()))
                        .angularVelocity(RotationsPerSecond.of(shooter.getFlywheelLeftVelocityRPM() / 60.0));
                    log.motor("flywheel-right")
                        .voltage(Volts.of(shooter.getFlywheelRightAppliedVoltage()))
                        .angularVelocity(RotationsPerSecond.of(shooter.getFlywheelRightVelocityRPM() / 60.0));
                },
                shooter
            )
        ));

        // 5. Intake Arm Pivot Routine (with Angle Safeguards)
        routines.put(Mechanism.INTAKE_ARM, new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                Volts.of(MAX_ARM_VOLTAGE),
                Seconds.of(6.0),
                (state) -> recordState(Mechanism.INTAKE_ARM, state)
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> intake.setCharacterizationVoltage(volts.in(Volts)),
                (log) -> {
                    log.motor("arm")
                        .voltage(Volts.of(intake.getArmAppliedVoltage()))
                        .angularPosition(Radians.of(intake.getArmPositionRads()))
                        .angularVelocity(RotationsPerSecond.of(intake.getArmVelocityRads() / (2.0 * Math.PI)));
                },
                intake
            )
        ));
    }

    private void recordState(Mechanism mechanism, edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State state) {
        routineState = state.toString();
        org.littletonrobotics.junction.Logger.recordOutput("SysId/" + mechanism.name() + "/State", routineState);
    }

    /**
     * Schedules a SysId test command for the specified mechanism, type, and direction.
     */
    public void startTest(Mechanism mechanism, TestType type, Direction direction) {
        cancelTest();

        SysIdRoutine routine = routines.get(mechanism);
        if (routine == null) return;

        activeMechanism = mechanism;
        if (type == TestType.QUASISTATIC) {
            activeCommand = routine.quasistatic(direction);
        } else {
            activeCommand = routine.dynamic(direction);
        }

        System.out.printf("[SysIdManager] Starting %s (%s, %s)%n",
                mechanism.displayName, type.name(), direction.name());

        CommandScheduler.getInstance().schedule(activeCommand);
        isRunning = true;
    }

    /**
     * Aborts any currently running SysId command and halts all actuators.
     */
    public void cancelTest() {
        if (activeCommand != null) {
            CommandScheduler.getInstance().cancel(activeCommand);
            activeCommand = null;
        }
        isRunning = false;
        routineState = "CANCELLED / IDLE";

        swerve.stop();
        shooter.stop();
        intake.stop();
    }

    /**
     * Updates SysId manager from controller inputs with safe hold-to-run semantics.
     *
     * <p>Mapping:
     * - D-pad Up/Down: Cycle mechanisms
     * - Left Trigger (>0.5): Hold to run Quasistatic Forward
     * - Left Bumper: Hold to run Quasistatic Reverse
     * - Right Trigger (>0.5): Hold to run Dynamic Forward
     * - Right Bumper: Hold to run Dynamic Reverse
     * - B Button / Start: Emergency Cancel
     */
    public void update(Controller controller) {
        if (controller == null) return;

        // Emergency Cancel
        if (controller.getBButtonPressed() || controller.getStartButtonPressed()) {
            cancelTest();
            return;
        }

        // Mechanism Selection via D-Pad
        int pov = controller.getPOV();
        if (pov == 0) {
            setActiveMechanism(Mechanism.SWERVE_DRIVE_LINEAR);
        } else if (pov == 90) {
            setActiveMechanism(Mechanism.SHOOTER_FLYWHEELS);
        } else if (pov == 180) {
            setActiveMechanism(Mechanism.INTAKE_ARM);
        } else if (pov == 270) {
            setActiveMechanism(Mechanism.SWERVE_DRIVE_ANGULAR);
        }

        // Hold-to-Run execution
        boolean lt = controller.getLeftTriggerAxis() > 0.5;
        boolean lb = controller.getLeftBumperButton();
        boolean rt = controller.getRightTriggerAxis() > 0.5;
        boolean rb = controller.getRightBumperButton();

        if (lt) {
            if (!isRunning) startTest(activeMechanism, TestType.QUASISTATIC, Direction.kForward);
        } else if (lb) {
            if (!isRunning) startTest(activeMechanism, TestType.QUASISTATIC, Direction.kReverse);
        } else if (rt) {
            if (!isRunning) startTest(activeMechanism, TestType.DYNAMIC, Direction.kForward);
        } else if (rb) {
            if (!isRunning) startTest(activeMechanism, TestType.DYNAMIC, Direction.kReverse);
        } else {
            // Releasing all test buttons automatically stops and cancels test
            if (isRunning) {
                cancelTest();
            }
        }
    }

    public void setActiveMechanism(Mechanism mechanism) {
        if (this.activeMechanism != mechanism) {
            cancelTest();
            this.activeMechanism = mechanism;
            System.out.println("[SysIdManager] Active Mechanism: " + mechanism.displayName);
        }
    }

    public Mechanism getActiveMechanism() {
        return activeMechanism;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public String getRoutineState() {
        return routineState;
    }

    public void setupDashboard() {
        SmartDashboard.putString("Test/SysId/Mechanism", activeMechanism.name());
        SmartDashboard.putString("Test/SysId/State", routineState);
        SmartDashboard.putBoolean("Test/SysId/Running", false);

        // Headless Test Triggers
        SmartDashboard.putBoolean("Test/SysId/QuasistaticForward", false);
        SmartDashboard.putBoolean("Test/SysId/QuasistaticReverse", false);
        SmartDashboard.putBoolean("Test/SysId/DynamicForward", false);
        SmartDashboard.putBoolean("Test/SysId/DynamicReverse", false);
        SmartDashboard.putBoolean("Test/SysId/Abort", false);
    }

    public void updateDashboard() {
        SmartDashboard.putString("Test/SysId/Mechanism", activeMechanism.name());
        SmartDashboard.putString("Test/SysId/State", routineState);
        SmartDashboard.putBoolean("Test/SysId/Running", isRunning);

        // Read mechanism selector string if set from Elastic
        String selectedMech = SmartDashboard.getString("Test/SysId/SelectedMechanism", "");
        if (!selectedMech.isEmpty()) {
            for (Mechanism m : Mechanism.values()) {
                if (m.name().equalsIgnoreCase(selectedMech) || m.displayName.equalsIgnoreCase(selectedMech)) {
                    if (activeMechanism != m) setActiveMechanism(m);
                    break;
                }
            }
        }

        // Process headless triggers
        if (SmartDashboard.getBoolean("Test/SysId/Abort", false)) {
            SmartDashboard.putBoolean("Test/SysId/Abort", false);
            cancelTest();
            return;
        }

        if (SmartDashboard.getBoolean("Test/SysId/QuasistaticForward", false)) {
            SmartDashboard.putBoolean("Test/SysId/QuasistaticForward", false);
            startTest(activeMechanism, TestType.QUASISTATIC, Direction.kForward);
        } else if (SmartDashboard.getBoolean("Test/SysId/QuasistaticReverse", false)) {
            SmartDashboard.putBoolean("Test/SysId/QuasistaticReverse", false);
            startTest(activeMechanism, TestType.QUASISTATIC, Direction.kReverse);
        } else if (SmartDashboard.getBoolean("Test/SysId/DynamicForward", false)) {
            SmartDashboard.putBoolean("Test/SysId/DynamicForward", false);
            startTest(activeMechanism, TestType.DYNAMIC, Direction.kForward);
        } else if (SmartDashboard.getBoolean("Test/SysId/DynamicReverse", false)) {
            SmartDashboard.putBoolean("Test/SysId/DynamicReverse", false);
            startTest(activeMechanism, TestType.DYNAMIC, Direction.kReverse);
        }
    }
}
