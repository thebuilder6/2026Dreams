package frc.robot.Subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

/**
 * Physical hardware implementation of DriveIO interfacing with REV SparkMax controllers
 * and CANcoder absolute encoders via YAGSL.
 */
public class DriveIOSparkMax implements DriveIO {

    private final SwerveDrive swerveDrive;

    public DriveIOSparkMax(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveModule[] modules = swerveDrive.getModules();
        for (int i = 0; i < Math.min(4, modules.length); i++) {
            SwerveModule mod = modules[i];
            inputs.driveVelocitiesMetersPerSec[i] = mod.getDriveMotor().getVelocity();
            inputs.drivePositionsMeters[i] = mod.getDriveMotor().getPosition();
            inputs.driveAppliedVolts[i] = mod.getDriveMotor().getVoltage();

            inputs.steerPositionsDeg[i] = mod.getAngleMotor().getPosition();
            inputs.steerAppliedVolts[i] = mod.getAngleMotor().getVoltage();
        }

        inputs.gyroYawDeg = swerveDrive.getYaw().getDegrees();
        inputs.gyroPitchDeg = swerveDrive.getPitch().getDegrees();
        inputs.gyroRollDeg = swerveDrive.getRoll().getDegrees();
        inputs.gyroYawVelocityDegPerSec = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        inputs.odometryPose = swerveDrive.getPose();
    }

    @Override
    public void setModuleDriveVoltage(int index, double volts) {
        SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            modules[index].getDriveMotor().setVoltage(volts);
        }
    }

    @Override
    public void setModuleAngleVoltage(int index, double volts) {
        SwerveModule[] modules = swerveDrive.getModules();
        if (index >= 0 && index < modules.length) {
            modules[index].getAngleMotor().setVoltage(volts);
        }
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    @Override
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void setPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }
}
