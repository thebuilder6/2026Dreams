package frc.robot.Auto.Actions;

import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.Constants.ShooterConstants;

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
                var solution = shooter.calculateShootingSolution(swerve.getPose(), swerve.getFieldVelocity());
                return solution.turretAngle();
            });
        }
    }

    @Override
    public void update() {
        var solution = shooter.calculateShootingSolution(swerve.getPose(), swerve.getFieldVelocity());

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
