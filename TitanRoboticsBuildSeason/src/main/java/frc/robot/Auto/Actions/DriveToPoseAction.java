package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Auto.LegalPinningWatchdog;
import frc.robot.Auto.SmartTunnelRouter;
import frc.robot.Auto.StaticPathfinder;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.SwerveBase;
import org.littletonrobotics.junction.Logger;

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

    // Proprioceptive stall / pin detection state
    private double stallStartTime = -1.0;
    private boolean isPirouetteActive = false;
    private double pirouetteEndTime = -1.0;
    private static final double STALL_SPEED_CMD_THRESHOLD = 1.20; // m/s
    private static final double STALL_MEASURED_VEL_THRESHOLD = 0.20; // m/s
    private static final double STALL_CURRENT_THRESHOLD_AMPS = 30.0; // Amperes
    private static final double STALL_MIN_DURATION_SEC = 0.20; // 200 ms
    private static final double PIROUETTE_SPIN_RATE_RAD_PER_SEC = 8.0; // 720 deg/s spin slip

    // Automated Trench Tunneling Transit state
    private boolean isTunnelTransit = false;
    private Rotation2d tunnelHeading = new Rotation2d();

    /**
     * Drives to a single target, with automatic pathfinding around static obstacles
     * and intelligent tunnel sequencing if approaching a trench entrance.
     */
    public DriveToPoseAction(Pose2d targetPose) {
        this(buildWaypointsForTarget(SwerveBase.getInstance().getPose(), targetPose), true);
        GlideConstants.GlidePoint tunnel = GlideConstants.getMatchingTunnelEntrance(targetPose);
        if (tunnel != null) {
            boolean preferTop = tunnel.name().contains("Top") || targetPose.getY() > 4.0;
            SmartTunnelRouter.TunnelRoute route = SmartTunnelRouter.planTunnelRoute(SwerveBase.getInstance().getPose(), preferTop);
            this.isTunnelTransit = true;
            this.tunnelHeading = route.corridorHeading;
        }
    }

    /**
     * Drives to a designated GlidePoint with intelligent tunnel sequencing if applicable.
     */
    public DriveToPoseAction(GlideConstants.GlidePoint glidePoint) {
        this(buildWaypointsForGlidePoint(SwerveBase.getInstance().getPose(), glidePoint), true);
        if (glidePoint != null && glidePoint.isTunnelEntrance) {
            boolean preferTop = glidePoint.name().contains("Top") || glidePoint.pose().getY() > 4.0;
            SmartTunnelRouter.TunnelRoute route = SmartTunnelRouter.planTunnelRoute(SwerveBase.getInstance().getPose(), preferTop);
            this.isTunnelTransit = true;
            this.tunnelHeading = route.corridorHeading;
        }
    }

    private static List<Pose2d> buildWaypointsForGlidePoint(Pose2d currentPose, GlideConstants.GlidePoint glidePoint) {
        if (glidePoint == null) {
            return List.of(currentPose);
        }
        if (glidePoint.isTunnelEntrance) {
            boolean preferTop = glidePoint.name().contains("Top") || glidePoint.pose().getY() > 4.0;
            SmartTunnelRouter.TunnelRoute route = SmartTunnelRouter.planTunnelRoute(currentPose, preferTop);
            return route.getWaypoints();
        }
        return StaticPathfinder.findPath(currentPose, glidePoint.pose());
    }

    private static List<Pose2d> buildWaypointsForTarget(Pose2d currentPose, Pose2d targetPose) {
        GlideConstants.GlidePoint tunnel = GlideConstants.getMatchingTunnelEntrance(targetPose);
        if (tunnel != null) {
            return buildWaypointsForGlidePoint(currentPose, tunnel);
        }
        return StaticPathfinder.findPath(currentPose, targetPose);
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
        double targetDeg = finalTarget.getRotation().getDegrees();
        if (isTunnelTransit) {
            boolean inTrenchZone = Intake.isPoseInTrenchLowClearanceZone(currentPose);
            boolean nearEntrance = !waypoints.isEmpty()
                    && currentPose.getTranslation().getDistance(waypoints.get(0).getTranslation()) < 1.2;
            if (inTrenchZone || nearEntrance) {
                targetDeg = tunnelHeading.getDegrees();
                Intake.getInstance().setArmPosition(Constants.INTAKE_HORIZONTAL_POSITION);
            }
            if (inTrenchZone && !waypoints.isEmpty()) {
                // High-gain cross-track centering constraint inside the 53-inch corridor
                double corridorY = waypoints.get(waypoints.size() - 1).getY();
                double crossTrackError = corridorY - currentPose.getY();
                vy += crossTrackError * 2.0; // Stiff centering bias
            }
        }

        double rotationSpeed = rotationController.calculate(
                currentPose.getRotation().getDegrees(),
                targetDeg);

        // 6. Proprioceptive Stall / Pin Check
        double commandedSpeed = Math.hypot(vx, vy);
        ChassisSpeeds actualSpeeds = swerveBase.getRobotVelocity();
        double measuredSpeed = Math.hypot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond);
        double driveCurrent = swerveBase.getAverageDriveCurrent();
        double now = Timer.getFPGATimestamp();

        boolean stallCondition = (commandedSpeed > STALL_SPEED_CMD_THRESHOLD)
                && (measuredSpeed < STALL_MEASURED_VEL_THRESHOLD)
                && (driveCurrent > STALL_CURRENT_THRESHOLD_AMPS);

        if (stallCondition) {
            if (stallStartTime < 0) {
                stallStartTime = now;
            } else if (now - stallStartTime > STALL_MIN_DURATION_SEC) {
                // Pin / Stall confirmed: Insert virtual obstacle 0.65m in front of robot along drive direction
                Translation2d virtualObstaclePos = currentPose.getTranslation().plus(driveDir.times(0.65));
                DynamicRouter.registerObstacle(virtualObstaclePos, new Translation2d(), 0.55, 0.60, true);

                // Trigger Swerve Pirouette Slip for 0.5s to break cloth bumper friction
                // (Disabled during tunnel transit to avoid wedging against trench walls)
                if (!isTunnelTransit) {
                    isPirouetteActive = true;
                    pirouetteEndTime = now + 0.50;
                }
            }
        } else {
            stallStartTime = -1.0;
        }

        if (isPirouetteActive && now > pirouetteEndTime) {
            isPirouetteActive = false;
        }
        Logger.recordOutput("DynamicAvoidance/PirouetteActive", isPirouetteActive);
        Logger.recordOutput("DynamicAvoidance/TunnelTransitActive", isTunnelTransit);

        // 7. Dynamic Obstacle Avoidance Routing
        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(vx, vy, Math.toRadians(rotationSpeed));
        ChassisSpeeds avoidanceSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, lookaheadPoint);

        // 8. Legal Pinning Watchdog Backoff Override
        LegalPinningWatchdog watchdog = LegalPinningWatchdog.getInstance();
        watchdog.update(stallCondition, currentPose, null, 0.02);
        if (watchdog.isForcedBackoffActive()) {
            Pose2d backoffPose = watchdog.getBackOffTarget(currentPose, null);
            Translation2d backoffDir = backoffPose.getTranslation().minus(currentPose.getTranslation());
            if (backoffDir.getNorm() > 1e-4) {
                backoffDir = backoffDir.div(backoffDir.getNorm());
            }
            avoidanceSpeeds = new ChassisSpeeds(backoffDir.getX() * 1.5, backoffDir.getY() * 1.5, 0.0);
        } else if (isPirouetteActive) {
            // If Pirouette is active, inject high-rate spin (8.0 rad/s) to break contact
            avoidanceSpeeds = new ChassisSpeeds(
                    avoidanceSpeeds.vxMetersPerSecond,
                    avoidanceSpeeds.vyMetersPerSecond,
                    PIROUETTE_SPIN_RATE_RAD_PER_SEC);
        }

        swerveBase.driveFieldOriented(avoidanceSpeeds);
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

    public boolean isTunnelTransit() {
        return isTunnelTransit;
    }

    public Rotation2d getTunnelHeading() {
        return tunnelHeading;
    }

    public List<Pose2d> getWaypoints() {
        return Collections.unmodifiableList(waypoints);
    }
}
