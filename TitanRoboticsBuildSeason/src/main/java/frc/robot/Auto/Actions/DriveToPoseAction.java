package frc.robot.Auto.Actions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Auto.LegalPinningWatchdog;
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
    private final Pose2d targetPose;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private final boolean isTunnelTransit;
    private final Rotation2d tunnelHeading;

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
            this.waypoints.add(targetPose);
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

    @Override
    public void start() {
        controller.reset();
        if (!waypoints.isEmpty()) {
            controller.setExplicitWaypoints(waypoints);
        }
    }

    @Override
    public void update() {
        Pose2d currentPose = swerveBase.getPose();
        ChassisSpeeds currentSpeeds = swerveBase.getRobotVelocity();

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
                    !isTunnelTransit);
        }

        swerveBase.setPathVisualization(controller.getWaypoints());
        swerveBase.driveFieldOriented(speeds);
    }

    @Override
    public boolean isFinished() {
        Pose2d finalGoal = isTunnelTransit && !waypoints.isEmpty()
                ? waypoints.get(waypoints.size() - 1)
                : targetPose;
        return controller.isFinished(swerveBase.getPose(), finalGoal, 0.12, 4.0);
    }

    @Override
    public void done() {
        swerveBase.setPathVisualization(Collections.emptyList());
        swerveBase.stop();
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