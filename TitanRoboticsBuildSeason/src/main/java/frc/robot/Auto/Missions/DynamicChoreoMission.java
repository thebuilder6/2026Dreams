package frc.robot.Auto.Missions;

import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.AutoMissionEndedException;

/**
 * A generic autonomous mission that simply follows a Choreo trajectory.
 */
public class DynamicChoreoMission extends MissionBase {
    private final String trajectoryName;

    public DynamicChoreoMission(String trajectoryName) {
        this.trajectoryName = trajectoryName;
    }

    @Override
    protected void routine() throws AutoMissionEndedException {
        // Follow the specified Choreo path
        runAction(new FollowChoreoPath(trajectoryName, true));
    }
}
