package frc.robot.Test;

import frc.robot.Telemetry.Dashboard;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.HMI.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Unified System Identification (SysID) Manager for Team 8334.
 * 
 * Provides rock-solid, hold-to-run characterization routines for:
 * 1. Swerve Drive Linear (Translational velocity & friction with wheels locked at 0 deg)
 * 2. Swerve Drive Angular (Rotational Moment of Inertia with wheels oriented tangentially)
 * 3. Swerve Steer Azimuth (Steering motor dynamics)
 * 4. Shooter Flywheels (Dual flywheel speed & acceleration feedforwards)
 * 5. Intake Arm Pivot (Gravity and friction modeling with software angle bounds)
 */
public class SysIdManager {

    private static SysIdManager instance = null;

    public enum MechanismType {
        SWERVE_DRIVE_LINEAR("Swerve Drive Linear"),
        SWERVE_DRIVE_ANGULAR("Swerve Drive Angular"),
        SWERVE_STEER("Swerve Steer"),
        SHOOTER_FLYWHEELS("Shooter Flywheels"),
        INTAKE_ARM("Intake Arm Pivot");

        public final String displayName;

        MechanismType(String displayName) {
            this.displayName = displayName;
        }
    }

    private final SwerveBase swerve;
    private final Shooter shooter;
    private final Intake intake;

    private MechanismType activeMechanism = MechanismType.SWERVE_DRIVE_LINEAR;

    // SysId Routines
    private final SysIdRoutine driveLinearRoutine;
    private final SysIdRoutine driveAngularRoutine;
    private final SysIdRoutine steerRoutine;
    private final SysIdRoutine shooterRoutine;
    private final SysIdRoutine armRoutine;

    // Active command tracking
    private Command activeCommand = null;
    private String routineState = "IDLE";

    public static SysIdManager getInstance() {
        if (instance == null) {
            instance = new SysIdManager(Shooter.getInstance(), Intake.getInstance(), SwerveBase.getInstance());
        }
        return instance;
    }

    public SysIdManager(Shooter shooter, Intake intake, SwerveBase swerve) {
        this.shooter = shooter;
        this.intake = intake;
        this.swerve = swerve;

        // 1. Swerve Drive Linear Routine
        driveLinearRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(1.0),
                Volts.of(7.0),
                Seconds.of(10.0),
                state -> {
                    routineState = "DriveLinear: " + state.toString();
                    SmartDashboard.putString("Test/SysId/State", routineState);
                    org.littletonrobotics.junction.Logger.recordOutput("SysId/State", routineState);
                }
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdDriveVoltage(volts.in(Volts)),
                log -> {
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
        );

        // 2. Swerve Drive Angular Routine (Rotational Moment of Inertia)
        driveAngularRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(1.0),
                Volts.of(6.0),
                Seconds.of(8.0),
                state -> {
                    routineState = "DriveAngular: " + state.toString();
                    SmartDashboard.putString("Test/SysId/State", routineState);
                    org.littletonrobotics.junction.Logger.recordOutput("SysId/State", routineState);
                }
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdRotationVoltage(volts.in(Volts)),
                log -> {
                    var voltages = swerve.getDriveMotorVoltages();
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double yawRateRadPerSec = swerve.getGyroYawRateRadsPerSec();
                    double headingRad = swerve.getHeading().getRadians();

                    log.motor("drive-angular")
                        .voltage(Volts.of(avgVolts))
                        .angularPosition(Radians.of(headingRad))
                        .angularVelocity(RadiansPerSecond.of(yawRateRadPerSec));
                },
                swerve
            )
        );

        // 3. Swerve Steer Azimuth Routine
        steerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(1.5),
                Volts.of(5.0),
                Seconds.of(6.0),
                state -> {
                    routineState = "Steer: " + state.toString();
                    SmartDashboard.putString("Test/SysId/State", routineState);
                    org.littletonrobotics.junction.Logger.recordOutput("SysId/State", routineState);
                }
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSysIdSteerVoltage(volts.in(Volts)),
                log -> {
                    var positions = swerve.getSteerMotorPositions();
                    var voltages = swerve.getSteerMotorVoltages();

                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPosRad = Math.toRadians(positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0));

                    log.motor("steer-azimuth")
                        .voltage(Volts.of(avgVolts))
                        .angularPosition(Radians.of(avgPosRad));
                },
                swerve
            )
        );

        // 4. Shooter Flywheels Routine
        shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(1.5),
                Volts.of(7.0),
                Seconds.of(8.0),
                state -> {
                    routineState = "Shooter: " + state.toString();
                    SmartDashboard.putString("Test/SysId/State", routineState);
                    org.littletonrobotics.junction.Logger.recordOutput("SysId/State", routineState);
                }
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> shooter.setFlywheelCharacterizationVoltage(volts.in(Volts), volts.in(Volts)),
                log -> {
                    log.motor("flywheel-left")
                        .voltage(Volts.of(shooter.getFlywheelLeftAppliedVoltage()))
                        .angularVelocity(RotationsPerSecond.of(shooter.getFlywheelLeftVelocityRPM() / 60.0));
                    log.motor("flywheel-right")
                        .voltage(Volts.of(shooter.getFlywheelRightAppliedVoltage()))
                        .angularVelocity(RotationsPerSecond.of(shooter.getFlywheelRightVelocityRPM() / 60.0));
                },
                shooter
            )
        );

        // 5. Intake Arm Routine
        armRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.75),
                Volts.of(3.5),
                Seconds.of(5.0),
                state -> {
                    routineState = "Arm: " + state.toString();
                    SmartDashboard.putString("Test/SysId/State", routineState);
                    org.littletonrobotics.junction.Logger.recordOutput("SysId/State", routineState);
                }
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> intake.setCharacterizationVoltage(volts.in(Volts)),
                log -> {
                    log.motor("arm")
                        .voltage(Volts.of(intake.getArmAppliedVoltage()))
                        .angularPosition(Radians.of(intake.getArmPositionRads()))
                        .angularVelocity(RadiansPerSecond.of(intake.getArmVelocityRads()));
                },
                intake
            )
        );

        setupDashboard();
    }

    public SysIdRoutine getActiveRoutine() {
        switch (activeMechanism) {
            case SWERVE_DRIVE_LINEAR: return driveLinearRoutine;
            case SWERVE_DRIVE_ANGULAR: return driveAngularRoutine;
            case SWERVE_STEER: return steerRoutine;
            case SHOOTER_FLYWHEELS: return shooterRoutine;
            case INTAKE_ARM: return armRoutine;
            default: return driveLinearRoutine;
        }
    }

    public void setActiveMechanism(MechanismType mechanism) {
        if (this.activeMechanism != mechanism) {
            abort();
            this.activeMechanism = mechanism;
            System.out.println("[SysIdManager] Switched active mechanism to: " + mechanism.displayName);
        }
    }

    public MechanismType getActiveMechanism() {
        return activeMechanism;
    }

    private boolean isTestRunning = false;

    public boolean isRunning() {
        return isTestRunning;
    }

    public String getRoutineState() {
        return routineState;
    }

    /**
     * Start a SysId routine command with hold-to-run semantics.
     */
    public void startQuasistatic(Direction direction) {
        abort();
        isTestRunning = true;
        activeCommand = getActiveRoutine().quasistatic(direction);
        CommandScheduler.getInstance().schedule(activeCommand);
        System.out.println("[SysIdManager] Scheduled Quasistatic " + direction.name() + " for " + activeMechanism.displayName);
    }

    public void startDynamic(Direction direction) {
        abort();
        isTestRunning = true;
        activeCommand = getActiveRoutine().dynamic(direction);
        CommandScheduler.getInstance().schedule(activeCommand);
        System.out.println("[SysIdManager] Scheduled Dynamic " + direction.name() + " for " + activeMechanism.displayName);
    }

    /**
     * Abort any active SysId routine immediately and safely zero actuator voltages.
     */
    public void abort() {
        isTestRunning = false;
        if (activeCommand != null) {
            CommandScheduler.getInstance().cancel(activeCommand);
            activeCommand = null;
        }
        routineState = "IDLE";
        SmartDashboard.putString("Test/SysId/State", routineState);

        // Safely stop all actuators
        swerve.stop();
        shooter.stop();
        intake.stop();
    }

    /**
     * Update controller hold-to-run handling in Test mode.
     */
    public void updateController(Controller controller) {
        // D-Pad for mechanism selection
        int pov = controller.getPOV();
        if (pov == 0) {
            setActiveMechanism(MechanismType.SWERVE_DRIVE_LINEAR);
        } else if (pov == 45) {
            setActiveMechanism(MechanismType.SWERVE_DRIVE_ANGULAR);
        } else if (pov == 90) {
            setActiveMechanism(MechanismType.SWERVE_STEER);
        } else if (pov == 180) {
            setActiveMechanism(MechanismType.SHOOTER_FLYWHEELS);
        } else if (pov == 270) {
            setActiveMechanism(MechanismType.INTAKE_ARM);
        }

        // B Button / Back: Emergency Stop
        if (controller.getBButtonPressed() || controller.getBackButton()) {
            abort();
            return;
        }

        // Triggers / Face Buttons for hold-to-run execution
        boolean runQuasiFwd = controller.getAButton();
        boolean runQuasiRev = controller.getXButton();
        boolean runDynFwd = controller.getYButton();
        boolean runDynRev = controller.getRightBumperButton();

        if (runQuasiFwd) {
            if (!isRunning()) startQuasistatic(Direction.kForward);
        } else if (runQuasiRev) {
            if (!isRunning()) startQuasistatic(Direction.kReverse);
        } else if (runDynFwd) {
            if (!isRunning()) startDynamic(Direction.kForward);
        } else if (runDynRev) {
            if (!isRunning()) startDynamic(Direction.kReverse);
        } else {
            // Releasing buttons cancels test immediately (hold-to-run)
            if (isRunning()) {
                abort();
            }
        }
    }

    public void setupDashboard() {
        SmartDashboard.putString("Test/SysId/ActiveMechanism", activeMechanism.name());
        SmartDashboard.putString("Test/SysId/State", "IDLE");
        SmartDashboard.putBoolean("Test/SysId/IsRunning", false);

        // Headless Trigger buttons
        SmartDashboard.putBoolean("Test/SysId/QuasistaticForward", false);
        SmartDashboard.putBoolean("Test/SysId/QuasistaticReverse", false);
        SmartDashboard.putBoolean("Test/SysId/DynamicForward", false);
        SmartDashboard.putBoolean("Test/SysId/DynamicReverse", false);
        SmartDashboard.putBoolean("Test/SysId/Abort", false);
    }

    public void log() {
        boolean running = isRunning();
        SmartDashboard.putBoolean("Test/SysId/IsRunning", running);
        SmartDashboard.putString("Test/SysId/ActiveMechanism", activeMechanism.name());
        SmartDashboard.putString("Test/SysId/State", routineState);

        // Check mechanism switcher from dashboard
        String mechFromDash = SmartDashboard.getString("Test/SysId/SelectMechanism", "");
        if (!mechFromDash.isEmpty() && !mechFromDash.equals(activeMechanism.name())) {
            try {
                setActiveMechanism(MechanismType.valueOf(mechFromDash));
                SmartDashboard.putString("Test/SysId/SelectMechanism", "");
            } catch (IllegalArgumentException ignored) {}
        }

        // Headless dashboard trigger processing
        if (SmartDashboard.getBoolean("Test/SysId/Abort", false)) {
            SmartDashboard.putBoolean("Test/SysId/Abort", false);
            abort();
        } else if (SmartDashboard.getBoolean("Test/SysId/QuasistaticForward", false)) {
            SmartDashboard.putBoolean("Test/SysId/QuasistaticForward", false);
            startQuasistatic(Direction.kForward);
        } else if (SmartDashboard.getBoolean("Test/SysId/QuasistaticReverse", false)) {
            SmartDashboard.putBoolean("Test/SysId/QuasistaticReverse", false);
            startQuasistatic(Direction.kReverse);
        } else if (SmartDashboard.getBoolean("Test/SysId/DynamicForward", false)) {
            SmartDashboard.putBoolean("Test/SysId/DynamicForward", false);
            startDynamic(Direction.kForward);
        } else if (SmartDashboard.getBoolean("Test/SysId/DynamicReverse", false)) {
            SmartDashboard.putBoolean("Test/SysId/DynamicReverse", false);
            startDynamic(Direction.kReverse);
        }

        // Live telemetry
        switch (activeMechanism) {
            case SWERVE_DRIVE_LINEAR:
                SmartDashboard.putNumber("Test/SysId/LiveVelocity", swerve.getDriveMotorVelocities().stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
                SmartDashboard.putNumber("Test/SysId/LiveVoltage", swerve.getDriveMotorVoltages().stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
                break;
            case SWERVE_DRIVE_ANGULAR:
                SmartDashboard.putNumber("Test/SysId/LiveVelocity", swerve.getGyroYawRateRadsPerSec());
                SmartDashboard.putNumber("Test/SysId/LiveVoltage", swerve.getDriveMotorVoltages().stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
                break;
            case SWERVE_STEER:
                SmartDashboard.putNumber("Test/SysId/LiveVelocity", 0.0);
                SmartDashboard.putNumber("Test/SysId/LiveVoltage", swerve.getSteerMotorVoltages().stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
                break;
            case SHOOTER_FLYWHEELS:
                SmartDashboard.putNumber("Test/SysId/LiveVelocity", (shooter.getFlywheelLeftVelocityRPM() + shooter.getFlywheelRightVelocityRPM()) / 2.0);
                SmartDashboard.putNumber("Test/SysId/LiveVoltage", shooter.getFlywheelLeftAppliedVoltage());
                break;
            case INTAKE_ARM:
                SmartDashboard.putNumber("Test/SysId/LiveVelocity", intake.getArmVelocityRads());
                SmartDashboard.putNumber("Test/SysId/LiveVoltage", intake.getArmAppliedVoltage());
                break;
        }
    }
}
