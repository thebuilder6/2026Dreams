package frc.robot.Auto.Actions;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import java.util.Optional;

public class AutoAimAction implements Actions {
    private final Shooter shooter = Shooter.getInstance();
    private final SwerveBase swerve = SwerveBase.getInstance();
    private final FollowChoreoPath path;
    private final double duration;
    private final Timer timer = new Timer();

    // PID for aiming when the path is finished
    private final PIDController stationaryAimPID = new PIDController(5.0, 0.0, 0.0);

    public AutoAimAction(FollowChoreoPath path, double duration) {
        this.path = path;
        this.duration = duration;
        stationaryAimPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void start() {
        timer.restart();
        stationaryAimPID.reset();

        if (path != null) {
            path.setRotationOverride(() -> {
                double lookAhead = Constants.SHOOTER_PREDICTIVE_LOOK_AHEAD;
                Optional<SwerveSample> sample = path.getSampleAtRelativeTime(lookAhead);

                if (sample.isPresent()) {
                    SwerveSample s = sample.get();
                    var solution = shooter.calculateShootingSolution(s.getPose(), s.getChassisSpeeds(), 0);
                    if (solution.possible()) {
                        return solution.turretAngle();
                    }
                }
                // If path has no sample (finished) or shot impossible, keep current heading
                return swerve.getHeading();
            });
        }
    }

    @Override
    public void update() {
        double currentTime = Timer.getTimestamp();
        double lookAhead = Constants.SHOOTER_PREDICTIVE_LOOK_AHEAD;
        Optional<SwerveSample> sample = (path != null) ? path.getSampleAtRelativeTime(lookAhead) : Optional.empty();

        Pose2d pose;
        ChassisSpeeds speeds;
        boolean isPathActive = sample.isPresent();

        // 1. Determine Robot State (Predictive vs Real)
        if (isPathActive) {
            SwerveSample s = sample.get();
            pose = s.getPose();
            speeds = s.getChassisSpeeds();
        } else {
            // Path is done, fallback to real-time sensors
            pose = swerve.getPose();
            speeds = swerve.getFieldVelocity();
        }

        // 2. Calculate Solution
        var solution = shooter.calculateShootingSolution(pose, speeds, 0);

        // Debugging to SmartDashboard
        SmartDashboard.putBoolean("AutoAim/Possible", solution.possible());
        SmartDashboard.putNumber("AutoAim/TargetRPM", solution.flywheelRPM());
        SmartDashboard.putNumber("AutoAim/TargetYaw", solution.turretAngle().getDegrees());

        if (solution.possible()) {
            shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());

            // 3. Handle Aiming (Path Override vs Manual Drive)
            double headingErrorDegrees = Math.abs(swerve.getHeading().minus(solution.turretAngle()).getDegrees());
            SmartDashboard.putNumber("AutoAim/HeadingError", headingErrorDegrees);
            SmartDashboard.putNumber("AutoAim/FlywheelError",
                    Math.abs(shooter.getActualRPM() - solution.flywheelRPM()));

            if (!isPathActive) {
                // FALLBACK: If the path is finished, we must turn the robot manually!
                double rotationOutput = stationaryAimPID.calculate(
                        swerve.getHeading().getRadians(),
                        solution.turretAngle().getRadians());
                swerve.drive(new ChassisSpeeds(0, 0, rotationOutput));
            }

            // 4. Fire Check
            // Check if aimed (< 5.0 deg), Flywheel Ready, and Solution Valid
            boolean aimed = headingErrorDegrees < 5.0;
            boolean ready = shooter.isAtTargetVelocity();
            if (aimed && ready) {
                shooter.shoot();
                SmartDashboard.putString("AutoAim/Status", "FIRING");
            } else {
                shooter.prepareToShoot();

                // Detailed Status for Debugging
                StringBuilder status = new StringBuilder("Wait: ");
                if (!aimed)
                    status.append(String.format("Aim Err %.1f > 5.0; ", headingErrorDegrees));
                if (!ready)
                    status.append("Spooling;");
                SmartDashboard.putString("AutoAim/Status", status.toString());
            }

        } else {
            // Shot Impossible (e.g. too close/far)
            shooter.prepareToShoot();
            shooter.setTargetRPM(Constants.ShooterConstants.IDLE_RPM);
            SmartDashboard.putString("AutoAim/Status", "Solution Impossible");

            // If path is done and we can't shoot, stop moving
            if (!isPathActive) {
                swerve.stop();
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
            path.setRotationOverride(null);
        }

        // Return shooter to PREPARING with IDLE speed so feeder is stopped,
        // but flywheels stay warm for subsequent actions
        shooter.prepareToShoot();
        shooter.setTargetRPM(Constants.ShooterConstants.IDLE_RPM);

        timer.stop();
        if (path == null || path.isFinished()) {
            swerve.stop();
        }
    }
}