package frc.robot.Auto.Actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.SwerveBase;

/**
 * Action to drive the robot to a specific pose using YAGSL's internal PID
 * controllers.
 * This is useful for both autonomous and teleop "gliding" to a target.
 */
public class DriveToPoseAction implements Actions {
    private final SwerveBase swerveBase;
    private final Pose2d targetPose;
    private final double translationTolerance = 0.05; // meters
    private final double rotationTolerance = 2.0; // degrees

    public DriveToPoseAction(Pose2d targetPose) {
        this.swerveBase = SwerveBase.getInstance();
        this.targetPose = targetPose;
    }

    @Override
    public void start() {
        // Initialization if needed
    }

    @Override
    public void update() {
        // Use YAGSL's getTargetSpeeds which uses the configured PID controllers (Theta,
        // X, Y)
        // We pass 0 for joystick inputs because we want the PID to handle the motion

        // Note: getTargetSpeeds handles the heading PID.
        // For X and Y, we might need to manually calculate if YAGSL doesn't provide a
        // direct "drive to Pose" PID helper.
        // However, we can leverage the existing logic in Teleop that uses
        // getTargetSpeeds(x, y, angle).

        // Since getTargetSpeeds(0, 0, targetPose.getRotation()) only does rotation,
        // a full DriveToPose would ideally use a ProfiledPIDController for X and Y too.

        // For now, let's implement a basic version that uses YAGSL's heading control
        // and a simple P-controller for translation to match the user's "glide"
        // request.

        Pose2d currentPose = swerveBase.getPose();
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();

        // Simple P control for translation (could be moved to constants)
        double kP = 2.0;
        double vx = xError * kP;
        double vy = yError * kP;

        ChassisSpeeds targetSpeeds = swerveBase.getTargetSpeeds(vx, vy, targetPose.getRotation());
        swerveBase.driveFieldOriented(targetSpeeds);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerveBase.getPose();
        double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        return translationError < translationTolerance && rotationError < rotationTolerance;
    }

    @Override
    public void done() {
        swerveBase.driveFieldOriented(new ChassisSpeeds());
    }
}
