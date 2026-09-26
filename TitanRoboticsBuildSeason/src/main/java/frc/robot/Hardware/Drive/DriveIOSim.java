package frc.robot.Hardware.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.SwerveDrive;

/**
 * Desktop simulation implementation of DriveIO.
 * Operates over YAGSL / IronMaple simulated physics or standalone test benches.
 */
public class DriveIOSim implements DriveIO {

    private final SwerveDrive swerveDrive;
    private Pose2d simPose = new Pose2d();
    private double simYawDeg = 0.0;
    private double[] moduleVolts = new double[4];
    private double[] moduleAngleVolts = new double[4];

    public DriveIOSim(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public DriveIOSim() {
        this(null);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        if (swerveDrive != null) {
            var modules = swerveDrive.getModules();
            for (int i = 0; i < Math.min(4, modules.length); i++) {
                inputs.driveVelocitiesMetersPerSec[i] = modules[i].getDriveMotor().getVelocity();
                inputs.drivePositionsMeters[i] = modules[i].getDriveMotor().getPosition();
                inputs.driveAppliedVolts[i] = moduleVolts[i];

                inputs.steerPositionsDeg[i] = modules[i].getAngleMotor().getPosition();
                inputs.steerAppliedVolts[i] = moduleAngleVolts[i];
            }
            inputs.gyroYawDeg = swerveDrive.getYaw().getDegrees();
            swerveDrive.getAccel().ifPresent(accel -> {
                inputs.accelXG = accel.getX();
                inputs.accelYG = accel.getY();
                inputs.accelZG = accel.getZ();
            });
            inputs.odometryPose = swerveDrive.getPose();
        } else {
            inputs.gyroYawDeg = simYawDeg;
            inputs.odometryPose = simPose;
            for (int i = 0; i < 4; i++) {
                inputs.driveAppliedVolts[i] = moduleVolts[i];
                inputs.steerAppliedVolts[i] = moduleAngleVolts[i];
            }
        }
    }

    @Override
    public void setModuleDriveVoltage(int index, double volts) {
        if (index >= 0 && index < 4) {
            moduleVolts[index] = volts;
        }
        if (swerveDrive != null && index < swerveDrive.getModules().length) {
            swerveDrive.getModules()[index].getDriveMotor().setVoltage(volts);
        }
    }

    @Override
    public void setModuleAngleVoltage(int index, double volts) {
        if (index >= 0 && index < 4) {
            moduleAngleVolts[index] = volts;
        }
        if (swerveDrive != null && index < swerveDrive.getModules().length) {
            swerveDrive.getModules()[index].getAngleMotor().setVoltage(volts);
        }
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        if (swerveDrive != null) {
            swerveDrive.drive(speeds);
        }
    }

    @Override
    public void zeroGyro() {
        simYawDeg = 0.0;
        if (swerveDrive != null) {
            swerveDrive.zeroGyro();
        }
    }

    @Override
    public void setPose(Pose2d pose) {
        this.simPose = pose;
        if (swerveDrive != null) {
            swerveDrive.resetOdometry(pose);
        }
    }
}
