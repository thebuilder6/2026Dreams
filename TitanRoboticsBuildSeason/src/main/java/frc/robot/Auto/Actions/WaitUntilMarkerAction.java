package frc.robot.Auto.Actions;

import frc.robot.Interfaces.Actions;

/**
 * Action that waits until a specific marker has been passed in a
 * FollowChoreoPath action.
 * This is meant to be used in a ParallelAction alongside the path.
 */
public class WaitUntilMarkerAction implements Actions {
    public static final double DEFAULT_TOLERANCE_METERS = 0.45;

    private final FollowChoreoPath path;
    private final String markerName;
    private final double spatialToleranceMeters;

    private boolean markerTriggered = false;

    public WaitUntilMarkerAction(FollowChoreoPath path, String markerName) {
        this(path, markerName, DEFAULT_TOLERANCE_METERS);
    }

    public WaitUntilMarkerAction(FollowChoreoPath path, String markerName, double spatialToleranceMeters) {
        this.path = path;
        this.markerName = markerName;
        this.spatialToleranceMeters = spatialToleranceMeters;
    }

    @Override
    public void start() {
        markerTriggered = false;
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (markerTriggered || path == null || path.isFinished()) {
            return true;
        }
        if (path.hasMarkerBeenPassed(markerName) || path.isWithinMarkerDistance(markerName, spatialToleranceMeters)) {
            markerTriggered = true;
            return true;
        }
        return false;
    }

    @Override
    public void done() {
    }
}
