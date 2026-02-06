package frc.robot.Auto.Actions;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.Constants.ShooterConstants;
import java.util.Optional;

/**
 * Action that continuously calculates the best shooting heading and RPM while
 * the robot is moving.
 * It integrates with FollowChoreoPath to override the robot's orientation.
 */
public class AutoAimAction implements Actions {
    private final Shooter shooter = Shooter.getInstance();
    private final SwerveBase swerve = SwerveBase.getInstance();
    private final FollowChoreoPath path;
    private final double duration;
    private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

    public AutoAimAction(FollowChoreoPath path, double duration) {
        this.path = path;
        this.duration = duration;
    }

    @Override
    public void start() {
        timer.restart();
        if (path != null) {
            // Set the path to use our calculated heading instead of the one baked in
            path.setRotationOverride(() -> {
                double lookAhead = frc.robot.Data.Constants.LOOP_TIME;
                Optional<SwerveSample> sample = path.getSampleAtRelativeTime(lookAhead);
                if (sample.isPresent()) {
                    SwerveSample s = sample.get();
                    var solution = shooter.calculateShootingSolution(s.getPose(), s.getChassisSpeeds(), 0);
                    return solution.turretAngle();
                }
                return swerve.getHeading();
            });
        }
    }

    @Override
    public void update() {
        double lookAhead = frc.robot.Data.Constants.LOOP_TIME;
        Optional<SwerveSample> sample = (path != null) ? path.getSampleAtRelativeTime(lookAhead) : Optional.empty();

        Pose2d pose;
        ChassisSpeeds speeds;

        if (sample.isPresent()) {
            SwerveSample s = sample.get();
            pose = s.getPose();
            speeds = s.getChassisSpeeds();
        } else {
            pose = swerve.getPose();
            speeds = swerve.getFieldVelocity();
        }

        var solution = shooter.calculateShootingSolution(pose, speeds, 0);

        if (solution.possible()) {
            shooter.setFlywheelVelocity(solution.flywheelRPM());

            // Check if we are aimed and spun up
            double headingError = Math.abs(swerve.getHeading().minus(solution.turretAngle()).getDegrees());
            if (headingError < 3.0 && shooter.isAtTargetVelocity()) {
                shooter.setFeederSpeed(ShooterConstants.FEED_SPEED);
            } else {
                shooter.setFeederSpeed(0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void done() {
        if (path != null) {
            path.setRotationOverride(null); // Return control to the path
        }
        shooter.stop();
        timer.stop();
    }
}
