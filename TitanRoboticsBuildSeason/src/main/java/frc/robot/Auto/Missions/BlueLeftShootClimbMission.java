package frc.robot.Auto.Missions;

import frc.robot.Auto.Actions.ClimbAction;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.Constants;
import edu.wpi.first.wpilibj.Timer;

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
