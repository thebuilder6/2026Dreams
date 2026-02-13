package frc.robot.Auto.Missions;

import frc.robot.Auto.Actions.ClimbAction;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.AutoMissionEndedException;

public class BlueLeftShootClimbMission extends MissionBase {

    @Override
    protected void routine() throws AutoMissionEndedException {

        // Follow Choreo Path "BlueLeftShootClimb"
        FollowChoreoPath followPath = new FollowChoreoPath("BlueLeftShootClimb", true);

        runAction(followPath);

        // Climb at the end
        runAction(new ClimbAction());
    }
}
