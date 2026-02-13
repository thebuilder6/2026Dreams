package frc.robot.Auto.Actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Interfaces.Actions;

public class TunnelAction implements Actions {
    private final DriveToPoseAction action;

    public TunnelAction(Pose2d entrancePose, Pose2d exitPose) {
        // Use the multi-pose constructor of DriveToPoseAction to enable
        // smooth waypoint switching between the entrance and exit.
        this.action = new DriveToPoseAction(java.util.List.of(entrancePose, exitPose), true);
    }

    @Override
    public void start() {
        action.start();
    }

    @Override
    public void update() {
        action.update();
    }

    @Override
    public boolean isFinished() {
        return action.isFinished();
    }

    @Override
    public void done() {
        action.done();
    }
}
