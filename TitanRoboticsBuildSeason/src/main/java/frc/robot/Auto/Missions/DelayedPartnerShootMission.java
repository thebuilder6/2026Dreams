package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Auto.Actions.WaitAction;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Alliance-coordinated autonomous mission:
 * Pauses for 4.0 seconds at the start of auto to yield the Hub area to an alliance
 * partner with immediate scoring capability, then navigates to the scoring zone and
 * discharges preloads into the Hub.
 */
@AutoMission(name = "Delayed Partner Shoot")
public class DelayedPartnerShootMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Wait for partner to execute primary shot and clear the Hub area
        runAction(new WaitAction(4.0));

        // 2. Drive to Hub shooting location
        runAction(new FollowChoreoPath("ShootPath", true));

        // 3. Fire preloaded fuel into the Hub
        runAction(new ShootAction(4.0));

        // 4. Clean shutdown of mechanisms and drive
        runAction(new LambdaAction(() -> {
            Shooter.getInstance().stop();
            SwerveBase.getInstance().stop();
        }));
    }
}
