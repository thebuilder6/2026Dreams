package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Autonomous routine that immediately scores all preloaded fuel pieces into the Hub,
 * then drives past the autonomous line to earn Mobility points.
 */
@AutoMission(name = "Subwoofer Shoot & Leave")
public class SubwooferShootAndLeaveMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Fire preloaded fuel pieces into the active Hub
        runAction(new ShootAction(3.5));

        // 2. Shut down shooter flywheels safely
        runAction(new LambdaAction(() -> Shooter.getInstance().stop()));

        // 3. Drive forward across the starting line for mobility (don't reset odometry)
        runAction(new FollowChoreoPath("MoveForward", false));

        // 4. Clean chassis stop
        runAction(new LambdaAction(() -> SwerveBase.getInstance().stop()));
    }
}
