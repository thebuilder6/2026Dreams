package frc.robot.Auto.Actions;

import frc.robot.Interfaces.Actions;

/**
 * Action that waits until a specific marker has been passed in a
 * FollowChoreoPath action.
 * This is meant to be used in a ParallelAction alongside the path.
 */
public class WaitUntilMarkerAction implements Actions {
    private final FollowChoreoPath path;
    private final String markerName;

    public WaitUntilMarkerAction(FollowChoreoPath path, String markerName) {
        this.path = path;
        this.markerName = markerName;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return path.hasMarkerBeenPassed(markerName);
    }

    @Override
    public void done() {
    }
}
