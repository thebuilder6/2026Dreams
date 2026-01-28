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
    private final Set<EventMarker> triggeredEvents = new HashSet<>();
    private boolean isPaused = false;
    private double totalPausedTime = 0;
    private double pauseStartTimestamp = 0;

    private final PIDController xController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
            AutonConstants.kAutoDriveD);
    private final PIDController yController = new PIDController(AutonConstants.kAutoDriveP, AutonConstants.kAutoDriveI,
            AutonConstants.kAutoDriveD);
    // heading controller lets us go in a full circle
    private final PIDController headingController = new PIDController(AutonConstants.kAutoTurnP,
            AutonConstants.kAutoTurnI, AutonConstants.kAutoTurnD);

    public FollowChoreoPath(String trajectoryName, boolean resetOdometry) {
        swerveBase = SwerveBase.getInstance();
        this.trajectory = Choreo.loadTrajectory(trajectoryName);
        this.timer = new Timer();
        this.resetOdometry = resetOdometry;

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
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return true;
        }
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void start() {
        timer.restart();
        triggeredEvents.clear();

        if (resetOdometry) {
            Optional<Pose2d> startPose = trajectory.get().getInitialPose(isRedAlliance());
            if (startPose != null) {
                swerveBase.resetOdometry(startPose.get());
            }
        }
    }

    public void update() {
        // autoDrive.followTrajectory(SwerveSample); Tried to use to call this stuff
        // from auto drive

        double time = timer.get() - totalPausedTime;
        if (isPaused) {
            time = pauseStartTimestamp - totalPausedTime;
        }

        SwerveSample sample = trajectory.get().sampleAt(time, resetOdometry).get();
        // Get the current currentRobotPose the robot
        Pose2d currentRobotPose = swerveBase.getPose();
        Pose2d targetPose = sample.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds autoSpeeds = new ChassisSpeeds(
                sample.vx + xController.calculate(currentRobotPose.getX(), sample.x),
                sample.vy + yController.calculate(currentRobotPose.getY(), sample.y),
                sample.omega + headingController.calculate(currentRobotPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()));

        // Apply the generated speeds into swerve
        swerveBase.driveFieldOriented(autoSpeeds);

        // Handle events
        if (trajectory.isPresent()) {
            for (EventMarker event : trajectory.get().events()) {
                double timestamp = event.timestamp;
                // If we passed the timestamp and haven't triggered it yet
                if (time >= timestamp && !triggeredEvents.contains(event)) {
                    Runnable action = eventBindings.get(event.event);
                    if (action != null) {
                        action.run();
                    }
                    triggeredEvents.add(event);
                }
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
