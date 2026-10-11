package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.ParallelAction;
import frc.robot.Auto.Actions.ParallelRaceAction;
import frc.robot.Auto.Actions.SeriesAction;
import frc.robot.Auto.Actions.ShootAction;

@AutoMission(name = "Right Half Center Shoot")
public class RightHalfCenterShootMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // Drive RightHalfCenterPath while staging the intake (standby, then intaking).
        // SeriesAction (not two parallel IntakeActions): ParallelAction keeps updating
        // finished actions, so parallel intake branches would fight over the Intake subsystem.
        runAction(new ParallelAction(
            new FollowChoreoPath("RightHalfCenterPath", true),
            new SeriesAction(
                new IntakeAction(0.5, "Standby"),
                new IntakeAction(4, "Intaking"))));
        runAction(new ParallelRaceAction(
            new ShootAction(99),
            new SeriesAction(
                new IntakeAction(0.5, "Down"),
                new IntakeAction(0.5, "Standby"),
                new IntakeAction(0.5, "Down"),
                new IntakeAction(0.5, "Standby"),
                new IntakeAction(0.5, "Down"),
                new IntakeAction(0.5, "Standby"),
                new IntakeAction(0.5, "Down"),
                new IntakeAction(0.5, "Standby"))));
    }
}
