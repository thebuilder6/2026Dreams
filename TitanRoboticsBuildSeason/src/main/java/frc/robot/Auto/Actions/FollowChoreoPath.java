package frc.robot.Auto.Actions;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.HashSet;
import choreo.Choreo;
import choreo.trajectory.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.Constants.AutonConstants;
import frc.robot.Interfaces.Actions;

/*  Class: Move Swerve Action
    Description: Ties our swerve base to the Choreo Trajectory platform so that we can make autos way easier
    Author: Rhea Sneller

    helpful links:  https://choreo.autos/choreolib/getting-started/ this is how to connect the swerve to choreo
                    https://github.com/SleipnirGroup/Choreo/releases/tag/v2026.0.1 this is where to download choreo
*/

public class FollowChoreoPath implements Actions {
    private final Optional<Trajectory<SwerveSample>> trajectory;
    private final boolean resetOdometry;
    SwerveBase swerveBase;
    Timer timer;
    private final Map<String, Runnable> eventBindings = new HashMap<>();
    private final Set<String> triggeredMarkers = new HashSet<>();
    private java.util.function.Supplier<edu.wpi.first.math.geometry.Rotation2d> rotationOverride = null;
    private boolean isPaused = false;
    private double totalPausedTime = 0;
    private double pauseStartTimestamp = 0;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    public FollowChoreoPath(String trajectoryName, boolean resetOdometry) {
        swerveBase = SwerveBase.getInstance();
        this.trajectory = Choreo.loadTrajectory(trajectoryName);
        this.timer = new Timer();
        this.resetOdometry = resetOdometry;

        // Pull Heading PID constants directly from YAGSL SwerveController config
        var config = swerveBase.getSwerveController().config;
        this.xController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
                AutonConstants.kAutoDriveD);
        this.yController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
                AutonConstants.kAutoDriveD);
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);

        // Rotation2d.getRadians() returns -PI to PI, so continuous input must match
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Binds an event name from Choreo to a Runnable action
     * 
     * @param eventName Name of the event marker in Choreo
     * @param action    Code to execute when the marker is reached
     * @return this for chaining
     */
    public FollowChoreoPath bind(String eventName, Runnable action) {
        eventBindings.put(eventName, action);
        return this;
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == Alliance.Red : false;
    }

    @Override
    public void start() {
        timer.restart();
        triggeredMarkers.clear();

        if (resetOdometry && trajectory.isPresent()) {
            Optional<Pose2d> startPose = trajectory.get().getInitialPose(isRedAlliance());
            if (startPose != null && startPose.isPresent()) {
                swerveBase.resetOdometry(startPose.get());
            }
        }
    }

    public FollowChoreoPath setRotationOverride(
            java.util.function.Supplier<edu.wpi.first.math.geometry.Rotation2d> override) {
        this.rotationOverride = override;
        return this;
    }

    public boolean hasMarkerBeenPassed(String markerName) {
        return triggeredMarkers.contains(markerName);
    }

    public void update() {
        if (!trajectory.isPresent())
            return;

        double time = timer.get() - totalPausedTime;
        if (isPaused) {
            time = pauseStartTimestamp - totalPausedTime;
        }

        SwerveSample sample = trajectory.get().sampleAt(time, isRedAlliance()).get();
        Pose2d currentRobotPose = swerveBase.getPose();
        Pose2d targetPose = sample.getPose();

        // Target Heading: Use override if provided, otherwise use trajectory value
        Rotation2d targetHeading = (rotationOverride != null) ? rotationOverride.get() : targetPose.getRotation();

        // Generate the next speeds for the robot
        ChassisSpeeds autoSpeeds = new ChassisSpeeds(
                sample.vx + xController.calculate(currentRobotPose.getX(), sample.x),
                sample.vy + yController.calculate(currentRobotPose.getY(), sample.y),
                sample.omega + headingController.calculate(currentRobotPose.getRotation().getRadians(),
                        targetHeading.getRadians()));

        // Apply the generated speeds into swerve
        swerveBase.driveFieldOriented(autoSpeeds);

        // Handle events
        for (EventMarker event : trajectory.get().events()) {
            double timestamp = event.timestamp;
            // If we passed the timestamp and haven't triggered it yet
            if (time >= timestamp && !triggeredMarkers.contains(event.event)) {
                Runnable action = eventBindings.get(event.event);
                if (action != null) {
                    action.run();
                }
                triggeredMarkers.add(event.event);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // the timer is done, so we reached end of trajectory
        return timer.hasElapsed(trajectory.get().getTotalTime());
    }

    @Override
    public void done() {
        // Completely stop robot and timer
        timer.stop();
        swerveBase.driveFieldOriented(new ChassisSpeeds());
    }

    public void setPaused(boolean paused) {
        if (paused && !isPaused) {
            pauseStartTimestamp = timer.get();
            isPaused = true;
        } else if (!paused && isPaused) {
            totalPausedTime += (timer.get() - pauseStartTimestamp);
            isPaused = false;
        }
    }

    public boolean isPaused() {
        return isPaused;
    }
}
