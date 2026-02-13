package frc.robot.Auto.Actions;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Auto.StaticPathfinder;

/**
 * Action to drive the robot to a specific pose (or series of waypoints) using
 * Holonomic Pure Pursuit for smooth, continuous movement.
 */
public class DriveToPoseAction implements Actions {
    private final SwerveBase swerveBase;
    private final List<Pose2d> waypoints;

    private final double translationTolerance = 0.10; // meters
    private final double rotationTolerance = 3.0; // degrees

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private static final double LOOKAHEAD_DIST = 0.6; // Meters
    private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5,
            2.5);

    // FIX: Use Degrees for Rotation Constraints (was Radians)
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 500); // Deg/s,
                                                                                                                         // Deg/s^2

    private int currentWaypointIndex = 0;

    /**
     * Drives to a single target, with automatic pathfinding around static
     * obstacles.
     */
    public DriveToPoseAction(Pose2d targetPose) {
        this(StaticPathfinder.findPath(SwerveBase.getInstance().getPose(), targetPose), true);
    }

    /**
     * Drives through a specific list of waypoints.
     */
    public DriveToPoseAction(List<Pose2d> waypoints, boolean isPath) {
        this.swerveBase = SwerveBase.getInstance();
        this.waypoints = waypoints;

        // Translation controller tracks distance progress along the path
        // FIX: Lower P gain to prevent wheel slip
        this.translationController = new ProfiledPIDController(
                2.0, 0, 0, TRANSLATION_CONSTRAINTS);

        // Rotation remains on its own profile
        // FIX: Tuned P gain for degrees
        this.rotationController = new ProfiledPIDController(
                4.0, 0, 0, ROTATION_CONSTRAINTS);

        translationController.setTolerance(translationTolerance);
        rotationController.setTolerance(rotationTolerance);
        rotationController.enableContinuousInput(-180, 180);
    }

    @Override
    public void start() {
        Pose2d currentPose = swerveBase.getPose();
        ChassisSpeeds currentSpeeds = swerveBase.getRobotVelocity();

        // FIX: Reset with actual distance error (negative) to avoid step input
        // We want to go from -Distance to 0.
        double distToFinal = 0;
        if (!waypoints.isEmpty()) {
            distToFinal = currentPose.getTranslation()
                    .getDistance(waypoints.get(waypoints.size() - 1).getTranslation());
        }

        // Project current velocity onto the path direction would be ideal,
        // but simple linear magnitude is a safe approximation for "forward" motion.
        double currentLinearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        translationController.reset(-distToFinal, currentLinearVel);

        rotationController.reset(currentPose.getRotation().getDegrees(),
                Math.toDegrees(currentSpeeds.omegaRadiansPerSecond));

        currentWaypointIndex = 0;
        swerveBase.setPathVisualization(waypoints);
    }

    @Override
    public void update() {
        if (waypoints.isEmpty())
            return;

        Pose2d currentPose = swerveBase.getPose();
        Pose2d finalTarget = waypoints.get(waypoints.size() - 1);

        // 1. Find the Lookahead Point on the path (Index-Aware)
        Translation2d lookaheadPoint = getLookaheadPoint(currentPose.getTranslation());

        // 2. Calculate Distance to Final Target for velocity scaling
        double distanceToFinal = currentPose.getTranslation().getDistance(finalTarget.getTranslation());

        // 3. Calculate Speed using ProfiledPID
        // We want to go to 0 distance. Current measurement is -distanceToFinal.
        double speed = translationController.calculate(-distanceToFinal, 0);

        // 4. Calculate Vector towards Lookahead
        Translation2d driveDir = lookaheadPoint.minus(currentPose.getTranslation());
        if (driveDir.getNorm() > 1e-6) {
            driveDir = driveDir.div(driveDir.getNorm());
        } else {
            driveDir = new Translation2d();
        }

        double vx = driveDir.getX() * speed;
        double vy = driveDir.getY() * speed;

        // 5. Calculate Rotation
        double rotationSpeed = rotationController.calculate(
                currentPose.getRotation().getDegrees(),
                finalTarget.getRotation().getDegrees());

        swerveBase.driveFieldOriented(new ChassisSpeeds(vx, vy, Math.toRadians(rotationSpeed)));
    }

    private Translation2d getLookaheadPoint(Translation2d currentPos) {
        if (waypoints.isEmpty())
            return currentPos;
        Translation2d finalTarget = waypoints.get(waypoints.size() - 1).getTranslation();

        // If we are close to the end, just drive to the end
        if (currentPos.getDistance(finalTarget) < LOOKAHEAD_DIST) {
            return finalTarget;
        }

        // FIX: Index-Aware Lookahead
        // Only search segments starting from currentWaypointIndex
        for (int i = currentWaypointIndex; i < waypoints.size(); i++) {
            Translation2d p = waypoints.get(i).getTranslation();

            // If the point is outside the lookahead distance, we might have an intersection
            // on this segment
            if (currentPos.getDistance(p) > LOOKAHEAD_DIST) {
                // Determine the start of this segment
                Translation2d segmentStart = (i == 0) ? currentPos : waypoints.get(i - 1).getTranslation();

                // Special check: If we are effectively "past" this segment start (i.e., closer
                // to next point),
                // we should update our index to avoid checking old segments next time.
                if (i > currentWaypointIndex) {
                    currentWaypointIndex = i - 1;
                }

                // Return intersection of segment (segmentStart -> p) and circle
                return interpolateLookahead(currentPos, segmentStart, p, LOOKAHEAD_DIST);
            }
        }

        // If all points are within distance (should stay caught by first check), go to
        // last
        return finalTarget;
    }

    private Translation2d interpolateLookahead(Translation2d center, Translation2d p1, Translation2d p2, double r) {
        Translation2d d = p2.minus(p1);
        Translation2d f = p1.minus(center);

        double a = d.dot(d);
        double b = 2 * f.dot(d);
        double c = f.dot(f) - r * r;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
            return p2;

        discriminant = Math.sqrt(discriminant);
        double t = (-b + discriminant) / (2 * a);

        if (t < 0 || t > 1)
            return p2;
        return p1.plus(d.times(t));
    }

    @Override
    public boolean isFinished() {
        if (waypoints.isEmpty())
            return true;

        Pose2d currentPose = swerveBase.getPose();
        Pose2d lastTarget = waypoints.get(waypoints.size() - 1);

        double translationError = currentPose.getTranslation().getDistance(lastTarget.getTranslation());
        double rotationError = Math.abs(currentPose.getRotation().minus(lastTarget.getRotation()).getDegrees());

        return translationError < translationTolerance && rotationError < rotationTolerance;
    }

    @Override
    public void done() {
        swerveBase.setPathVisualization(java.util.Collections.emptyList());
        swerveBase.driveFieldOriented(new ChassisSpeeds());
    }
}
