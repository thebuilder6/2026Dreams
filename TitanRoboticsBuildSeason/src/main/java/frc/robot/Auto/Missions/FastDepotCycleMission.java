package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ParallelRaceAction;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Auto.Actions.WaitForBallAction;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * High-tempo autonomous cycle:
 * 1. Drives to human player depot while intaking.
 * 2. Waits briefly for fuel capture verification.
 * 3. Stows intake mechanism.
 * 4. Drives to high-efficiency Hub shooting range.
 * 5. Fires preloaded and acquired fuel into the Hub.
 */
@AutoMission(name = "Fast Depot Cycle")
public class FastDepotCycleMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Move to depot while deploying intake
        runAction(new ParallelRaceAction(
            new FollowChoreoPath("DepotPath", true),
            new IntakeAction(999, Intake.IntakeState.INTAKING)
        ));

        // 2. Sensor-gated wait to confirm fuel acquisition at depot while actively running rollers (max 1.5s failsafe)
        runAction(new ParallelRaceAction(
            new WaitForBallAction(1.5),
            new IntakeAction(1.5, Intake.IntakeState.INTAKING)
        ));

        // 3. Stow intake safely before transit
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)));

        // 4. Transit to scoring range in front of Hub (maintain odometry)
        runAction(new FollowChoreoPath("DepotToShootPath", false));

        // 5. Score all fuel pieces into Hub with auto-aim alignment
        runAction(new ShootAction(3.5));

        // 6. Clean subsystem shutdown
        runAction(new LambdaAction(() -> {
            Shooter.getInstance().stop();
            SwerveBase.getInstance().stop();
            Intake.getInstance().stop();
        }));
    }
}
