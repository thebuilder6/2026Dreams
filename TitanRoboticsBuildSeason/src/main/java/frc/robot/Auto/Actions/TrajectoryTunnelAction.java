package frc.robot.Auto.Actions;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.SwerveBase;

/**
 * Trajectory-based action for traversing tunnels.
 * Uses HolonomicDriveController to rigidly follow a straight line, minimizing
 * cross-track error to prevent wall collisions.
 * 
 * See Chapter 17: Trajectory Optimization.
 */
public class TrajectoryTunnelAction implements Actions {

    private final SwerveBase swerveBase;
    private final Pose2d exitPose;
    private Trajectory trajectory; // Generated at runtime
    private final HolonomicDriveController controller;
    private final Timer timer = new Timer();

    // Constraints for Tunnel Traversal
    // We want to go fast, but controlled.
    private static final double MAX_VEL = 3.5; // m/s
    private static final double MAX_ACCEL = 2.5; // m/s^2

    public TrajectoryTunnelAction(Pose2d entrancePose, Pose2d exitPose) {
        this.swerveBase = SwerveBase.getInstance();
        this.exitPose = exitPose;

        // Setup Controller
        // X and Y controllers for Translation error
        // Tune 'Y' (Cross-track) aggressively
        PIDController xController = new PIDController(2.0, 0, 0);
        PIDController yController = new PIDController(4.0, 0, 0); // Higher gain for cross-track stiffness

        // Rotation Controller
        ProfiledPIDController thetaController = new ProfiledPIDController(
                4.0, 0, 0,
                new TrapezoidProfile.Constraints(360, 360)); // Degrees
        thetaController.enableContinuousInput(-180, 180);

        this.controller = new HolonomicDriveController(xController, yController, thetaController);

        // Disable rotation controller's tolerance check inside HolonomicDriveController
        // because we manage finish conditions separately.
    }

    @Override
    public void start() {
        Pose2d currentPose = swerveBase.getPose();
        ChassisSpeeds currentSpeeds = swerveBase.getRobotVelocity();
        double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // 1. Generate Trajectory
        // Dynamic Start: From Current Pose to Exit Pose.
        TrajectoryConfig config = new TrajectoryConfig(MAX_VEL, MAX_ACCEL)
                .setKinematics(swerveBase.getKinematics())
                .setStartVelocity(currentVel) // Maintain momentum ("Glide")
                .setEndVelocity(MAX_VEL); // Exit fast

        // Force start path-heading to match tunnel axis (Exit Rotation)
        // This ensures smooth merging ("S-curve") if we are offset, rather than a sharp
        // cut.
        Pose2d startPose = new Pose2d(currentPose.getTranslation(), exitPose.getRotation());

        this.trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(), // No internal waypoints
                exitPose,
                config);

        timer.restart();
        // Visualize the path
        swerveBase.setTrajectoryVisualization(trajectory);
    }

    @Override
    public void update() {
        if (trajectory == null)
            return;

        double currentTime = timer.get();
        Pose2d currentPose = swerveBase.getPose();

        // Calculate Target State
        Trajectory.State desiredState = trajectory.sample(currentTime);

        // Calculate Output
        // User "doesn't care" about heading, but aligning with the tunnel (exit
        // rotation) is safest.
        Rotation2d desiredRotation = exitPose.getRotation();

        ChassisSpeeds targetSpeeds = controller.calculate(
                currentPose,
                desiredState,
                desiredRotation);

        swerveBase.driveFieldOriented(targetSpeeds);
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null)
            return true;
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void done() {
        swerveBase.driveFieldOriented(new ChassisSpeeds());
        swerveBase.setTrajectoryVisualization(null);
    }
}
