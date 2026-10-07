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
 * Midfield sweep and mobility routine:
 * Drives forward past the starting line along MoveForward while intaking
 * floor pieces, earning Mobility RP while vacuuming corridor fuel.
 */
@AutoMission(name = "Centerline Sweep & Leave")
public class CenterlineSweepMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Move forward across starting line while actively intaking
        runAction(new ParallelRaceAction(
            new FollowChoreoPath("MoveForward", true),
            new IntakeAction(999, Intake.IntakeState.INTAKING)
        ));

        // 2. Stow intake safely
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)));

        // 3. Controlled chassis stop
        runAction(new LambdaAction(() -> SwerveBase.getInstance().stop()));
    }
}
