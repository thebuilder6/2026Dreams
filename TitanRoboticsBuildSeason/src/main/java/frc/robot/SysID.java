package frc.robot;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.SwerveBase;

public class SysID {
    private final Shooter m_shooter;
    private final Intake m_intake;
    private final SwerveBase m_swerve;

    private final SysIdRoutine m_shooterRoutine;
    private final SysIdRoutine m_armRoutine;
    private final SysIdRoutine m_driveRoutine;

    private enum ActiveMechanism {
        SHOOTER,
        ARM,
        DRIVE
    }
    
    private ActiveMechanism activeMechanism = ActiveMechanism.SHOOTER;

    public SysID(Shooter shooter, Intake intake, SwerveBase swerve) {
        this.m_shooter = shooter;
        this.m_intake = intake;
        this.m_swerve = swerve;

        // 1. Shooter Routine (Flywheels)
        m_shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> m_shooter.setVoltages(volts.in(Volts), 0), // characterize flywheels only
                (log) -> {
                    log.motor("flywheel-left")
                        .voltage(Volts.of(m_shooter.getFlywheelLeftAppliedVoltage()))
                        .angularVelocity(RotationsPerSecond.of(m_shooter.getFlywheelLeftVelocityRPM() / 60.0));
                },
                null
            )
        );

        // 2. Arm Routine
        m_armRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> m_intake.setArmVoltage(volts.in(Volts)),
                (log) -> {
                    log.motor("arm")
                        .voltage(Volts.of(m_intake.getArmAppliedVoltage()))
                        .angularPosition(Radians.of(m_intake.getArmPositionRads()))
                        .angularVelocity(RotationsPerSecond.of(m_intake.getArmVelocityRads() / (2.0 * Math.PI)));
                },
                null
            )
        );

        // 3. Drive Routine (Linear characterization)
        m_driveRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> m_swerve.setDriveVoltage(volts.in(Volts)),
                (log) -> {
                    var vels = m_swerve.getDriveMotorVelocities();
                    var positions = m_swerve.getDriveMotorPositions();
                    var voltages = m_swerve.getDriveMotorVoltages();

                    // Log average of all 4 drive motors for a simple linear model
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPos = positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgVel = vels.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                    log.motor("drive-linear-avg")
                        .voltage(Volts.of(avgVolts))
                        .linearPosition(Meters.of(avgPos))
                        .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(avgVel));
                },
                null
            )
        );
    }

    public void runTest(XboxController controller) {
        // --- Mechanism Switching (D-pad) ---
        if (controller.getPOV() == 0) {
            activeMechanism = ActiveMechanism.SHOOTER;
            System.out.println("[SysID] Active Mechanism: SHOOTER");
        } else if (controller.getPOV() == 180) {
            activeMechanism = ActiveMechanism.ARM;
            System.out.println("[SysID] Active Mechanism: ARM");
        } else if (controller.getPOV() == 270) {
            activeMechanism = ActiveMechanism.DRIVE;
            System.out.println("[SysID] Active Mechanism: DRIVE");
        }

        // --- Command Scheduling (Face Buttons) ---
        if (controller.getAButtonPressed()) {
            System.out.println("[SysID] Scheduling Quasistatic Forward for " + activeMechanism.name());
            getRoutineFor(activeMechanism).quasistatic(Direction.kForward).schedule();
        } else if (controller.getBButtonPressed()) {
            System.out.println("[SysID] Scheduling Quasistatic Reverse for " + activeMechanism.name());
            getRoutineFor(activeMechanism).quasistatic(Direction.kReverse).schedule();
        } else if (controller.getYButtonPressed()) {
            System.out.println("[SysID] Scheduling Dynamic Forward for " + activeMechanism.name());
            getRoutineFor(activeMechanism).dynamic(Direction.kForward).schedule();
        } else if (controller.getXButtonPressed()) {
            System.out.println("[SysID] Scheduling Dynamic Reverse for " + activeMechanism.name());
            getRoutineFor(activeMechanism).dynamic(Direction.kReverse).schedule();
        }
    }

    private SysIdRoutine getRoutineFor(ActiveMechanism mechanism) {
        switch (mechanism) {
            case SHOOTER: return m_shooterRoutine;
            case ARM: return m_armRoutine;
            case DRIVE: return m_driveRoutine;
            default: return m_shooterRoutine;
        }
    }
}


