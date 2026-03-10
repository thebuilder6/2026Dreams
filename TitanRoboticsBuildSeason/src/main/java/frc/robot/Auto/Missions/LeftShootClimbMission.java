package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.Actions.ClimbAction;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.AutoMissionEndedException;

@AutoMission(name = "Left Shoot Climb")
public class LeftShootClimbMission extends MissionBase {

    @Override
    protected void routine() throws AutoMissionEndedException {

        // Follow Choreo Path "LeftShootClimb"
        // Choreo handles mirroring automatically based on alliance
        FollowChoreoPath followPath = new FollowChoreoPath("LeftShootClimb", true);

        runAction(followPath);

        // Climb at the end
        runAction(new ClimbAction());
    }
}
