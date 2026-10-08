package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ParallelRaceAction;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.SwerveBase;

/**
 * Defensive & midfield disruption autonomous mission:
 * Traverses the trench and neutral zone boundary along TrenchSweep while intaking
 * floor pieces to starve opponent autonomous cycles and collect neutral game pieces.
 */
@AutoMission(name = "Trench Midfield Disruptor")
public class TrenchDisruptorMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Follow trench trajectory starting from legal Blue alliance line while actively intaking fuel
        runAction(new ParallelRaceAction(
            new FollowChoreoPath("TrenchSweep", true),
            new IntakeAction(999, Intake.IntakeState.INTAKING)
        ));

        // 2. Stow intake mechanism to standby position
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)));

        // 3. Bring chassis to a complete stop
        runAction(new LambdaAction(() -> SwerveBase.getInstance().stop()));
    }
}
