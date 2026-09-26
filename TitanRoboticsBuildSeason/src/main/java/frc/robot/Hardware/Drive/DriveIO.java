package frc.robot.Hardware.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware IO abstraction interface for the Swerve Drivebase.
 * Follows the AdvantageKit pattern to cleanly isolate physical SparkMax motor controllers,
 * encoders, and gyros from simulation kinematics.
 */
public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs {
        // Module Drive data (FL, FR, BL, BR)
        public double[] drivePositionsMeters = new double[4];
        public double[] driveVelocitiesMetersPerSec = new double[4];
        public double[] driveAppliedVolts = new double[4];
        public double[] driveCurrentAmps = new double[4];

        // Module Steer data (FL, FR, BL, BR)
        public double[] steerPositionsDeg = new double[4];
        public double[] steerVelocitiesDegPerSec = new double[4];
        public double[] steerAppliedVolts = new double[4];
        public double[] steerCurrentAmps = new double[4];

        // Gyroscope data
        public double gyroYawDeg = 0.0;
        public double gyroPitchDeg = 0.0;
        public double gyroRollDeg = 0.0;
        public double gyroYawVelocityDegPerSec = 0.0;

        // Accelerometer data (in Gs)
        public double accelXG = 0.0;
        public double accelYG = 0.0;
        public double accelZG = 1.0;

        // Odometry pose
        public Pose2d odometryPose = new Pose2d();
    }

    /** Updates input telemetry struct from physical hardware or simulation. */
    public default void updateInputs(DriveIOInputs inputs) {}

    /** Sets raw voltage to a single drive motor (0=FL, 1=FR, 2=BL, 3=BR). */
    public default void setModuleDriveVoltage(int index, double volts) {}

    /** Sets raw voltage to a single steer motor (0=FL, 1=FR, 2=BL, 3=BR). */
    public default void setModuleAngleVoltage(int index, double volts) {}

    /** Sets target chassis speeds for field or robot oriented drive. */
    public default void setChassisSpeeds(ChassisSpeeds speeds) {}

    /** Resets the gyro heading to zero. */
    public default void zeroGyro() {}

    /** Sets the current odometry pose. */
    public default void setPose(Pose2d pose) {}

    /** Stops all drivebase motors. */
    public default void stop() {
        for (int i = 0; i < 4; i++) {
            setModuleDriveVoltage(i, 0.0);
            setModuleAngleVoltage(i, 0.0);
        }
    }
}
