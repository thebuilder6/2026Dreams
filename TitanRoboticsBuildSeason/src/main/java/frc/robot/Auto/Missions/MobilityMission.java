package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Subsystems.SwerveBase;

/**
 * Autonomous routine that drives forward past the starting line to earn Mobility points/RP.
 * Low-risk, dependable auto when alliance partners occupy the scoring zones.
 */
@AutoMission(name = "Mobility (Drive Forward)")
public class MobilityMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // Follow the forward mobility trajectory and reset odometry to start pose
        runAction(new FollowChoreoPath("MoveForward", true));

        // Ensure swerve chassis comes to a complete controlled stop
        runAction(new LambdaAction(() -> SwerveBase.getInstance().stop()));
    }
}
