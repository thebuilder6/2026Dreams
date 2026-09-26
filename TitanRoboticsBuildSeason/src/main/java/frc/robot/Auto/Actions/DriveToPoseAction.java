package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.HMI.Watchdogs.LegalPinningWatchdog;
import frc.robot.Auto.SmartTunnelRouter;
import frc.robot.Auto.SmartTunnelRouter.TunnelRoute;
import frc.robot.Auto.TrajectoryController;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.SwerveBase;

public class DriveToPoseAction implements Actions {
    private final SwerveBase swerveBase;
    private final TrajectoryController controller;
    private Pose2d targetPose;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private final boolean isTunnelTransit;
    private final Rotation2d tunnelHeading;
    private boolean holdPosition = false;

    // Shared Driver Authority & Blending
    private double driverForward = 0.0;
    private double driverStrafe = 0.0;
    private double driverRotation = 0.0;
    private boolean breakoutRequested = false;

    public DriveToPoseAction(Pose2d targetPose) {
        this.swerveBase = SwerveBase.getInstance();
        this.targetPose = targetPose;

        var config = swerveBase.getSwerveController().config;
        this.controller = new TrajectoryController(
                new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d));

        // Evaluate if destination represents an auto-tunnel corridor
        boolean isTunnel = SmartTunnelRouter.isTunnelTarget(targetPose);
        if (isTunnel) {
            this.isTunnelTransit = true;
            boolean preferTop = targetPose.getY() >= 4.0;
            TunnelRoute route = SmartTunnelRouter.planTunnelRoute(swerveBase.getPose(), preferTop);
            this.tunnelHeading = route.corridorHeading;
            this.waypoints.addAll(route.getWaypoints());
        } else {
            this.isTunnelTransit = false;
            this.tunnelHeading = targetPose.getRotation();
            // Free-pathfinding mode: do NOT add single targetPose to waypoints!
            // Leaving waypoints empty allows TrajectoryController to dynamically invoke StaticPathfinder.
        }
    }

    public DriveToPoseAction(GlideConstants.GlidePoint glidePoint) {
        this(glidePoint.pose());
    }

    public DriveToPoseAction(List<Pose2d> explicitWaypoints, boolean isPath) {
        this.swerveBase = SwerveBase.getInstance();
        this.targetPose = explicitWaypoints.isEmpty() ? new Pose2d() : explicitWaypoints.get(explicitWaypoints.size() - 1);
        this.isTunnelTransit = false;
        this.tunnelHeading = targetPose.getRotation();
        this.waypoints.addAll(explicitWaypoints);

        var config = swerveBase.getSwerveController().config;
        this.controller = new TrajectoryController(
                new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d));
    }

    public void setDriverInput(double forwardField, double strafeField, double rotationCmd) {
        this.driverForward = forwardField;
        this.driverStrafe = strafeField;
        this.driverRotation = rotationCmd;

        double drvSpeed = Math.hypot(driverForward, driverStrafe);
        double maxSpeed = Math.max(0.1, Constants.MAX_SPEED);
        double normDriverMag = drvSpeed / maxSpeed;
        double maxRotSpeed = Math.max(0.1, Constants.MAX_ROTATION_SPEED);
        double normRotMag = Math.abs(driverRotation) / maxRotSpeed;

        if (normDriverMag > 0.65 || normRotMag > 0.60) {
            this.breakoutRequested = true;
        }
    }

    public boolean isBreakoutRequested() {
        return breakoutRequested;
    }

    @Override
    public void start() {
        controller.reset();
        breakoutRequested = false;
        driverForward = 0.0;
        driverStrafe = 0.0;
        driverRotation = 0.0;
        if (!waypoints.isEmpty()) {
            controller.setExplicitWaypoints(waypoints);
        }
    }

    @Override
    public void update() {
        Pose2d currentPose = swerveBase.getPose();
        ChassisSpeeds currentSpeeds = swerveBase.getFieldVelocity();

        // 1. Evaluate Driver Authority & Breakout Thresholds
        double drvSpeed = Math.hypot(driverForward, driverStrafe);
        double maxSpeed = Math.max(0.1, Constants.MAX_SPEED);
        double normDriverMag = drvSpeed / maxSpeed;
        double maxRotSpeed = Math.max(0.1, Constants.MAX_ROTATION_SPEED);
        double normRotMag = Math.abs(driverRotation) / maxRotSpeed;

        if (normDriverMag > 0.65 || normRotMag > 0.60) {
            breakoutRequested = true;
            return;
        }

        boolean isStalled = swerveBase.getCollisionDetector().isStalled();
        LegalPinningWatchdog.getInstance().update(isStalled, currentPose, null, 0.02);

        ChassisSpeeds speeds;
        if (LegalPinningWatchdog.getInstance().isForcedBackoffActive()) {
            Pose2d backoff = LegalPinningWatchdog.getInstance().getBackOffTarget(currentPose, null);
            var dir = backoff.getTranslation().minus(currentPose.getTranslation());
            if (dir.getNorm() > 1e-4) dir = dir.div(dir.getNorm());
            speeds = new ChassisSpeeds(dir.getX() * 1.5, dir.getY() * 1.5, 0.0);
        } else {
            Pose2d finalGoal = isTunnelTransit && !waypoints.isEmpty()
                    ? waypoints.get(waypoints.size() - 1)
                    : targetPose;

            speeds = controller.calculate(
                    currentPose,
                    currentSpeeds,
                    finalGoal,
                    Constants.MAX_SPEED,
                    isStalled,
                    true);
        }

        // 2. Apply Shared Authority Nudge Blending (0.10 <= normDriverMag <= 0.65)
        if (normDriverMag >= 0.10) {
            double alpha = Math.min(1.0, Math.max(0.0, (normDriverMag - 0.10) / (0.65 - 0.10)));
            double blendedVx = (1.0 - 0.5 * alpha) * speeds.vxMetersPerSecond + alpha * driverForward;
            double blendedVy = (1.0 - 0.5 * alpha) * speeds.vyMetersPerSecond + alpha * driverStrafe;
            speeds = new ChassisSpeeds(blendedVx, blendedVy, speeds.omegaRadiansPerSecond);
        }

        if (normRotMag >= 0.10) {
            double alphaRot = Math.min(1.0, Math.max(0.0, (normRotMag - 0.10) / (0.60 - 0.10)));
            double blendedOmega = (1.0 - alphaRot) * speeds.omegaRadiansPerSecond + alphaRot * driverRotation;
            speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, blendedOmega);
        }

        swerveBase.setPathVisualization(controller.getWaypoints());
        swerveBase.driveFieldOriented(speeds);
    }

    public void setTargetPose(Pose2d newTarget) {
        if (newTarget != null && !isTunnelTransit) {
            this.targetPose = newTarget;
        }
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setHoldPosition(boolean hold) {
        this.holdPosition = hold;
    }

    public boolean isHoldingPosition() {
        return holdPosition;
    }

    public void setRotationOverride(java.util.function.Supplier<Rotation2d> override) {
        controller.setRotationOverride(override);
    }

    @Override
    public boolean isFinished() {
        if (breakoutRequested) {
            return true;
        }
        if (holdPosition) {
            return false;
        }
        Pose2d finalGoal = isTunnelTransit && !waypoints.isEmpty()
                ? waypoints.get(waypoints.size() - 1)
                : targetPose;
        return controller.isFinished(swerveBase.getPose(), finalGoal, 0.12, 4.0);
    }

    @Override
    public void done() {
        swerveBase.setPathVisualization(Collections.emptyList());
        if (!breakoutRequested) {
            swerveBase.stop();
        }
    }

    public boolean isTunnelTransit() {
        return isTunnelTransit;
    }

    public Rotation2d getTunnelHeading() {
        return tunnelHeading;
    }

    public List<Pose2d> getWaypoints() {
        if (controller != null && !controller.getWaypoints().isEmpty()) {
            return controller.getWaypoints();
        }
        if (!waypoints.isEmpty()) {
            return Collections.unmodifiableList(waypoints);
        }
        return Collections.singletonList(targetPose);
    }
}