package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ParallelRaceAction;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * High-value combination autonomous mission:
 * 1. Immediately scores 8 preloaded fuel pieces into the Hub.
 * 2. Powers down shooter flywheels.
 * 3. Traverses through the low-clearance trench corridor via TrenchSweep while intaking
 *    neutral field fuel to starve opponent intake cycles.
 * 4. Stows intake and cleanly stops.
 */
@AutoMission(name = "Shoot & Trench Disruption")
public class ShootAndTrenchMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Immediately fire preloaded fuel pieces into Hub
        runAction(new ShootAction(3.5));

        // 2. Shut down shooter flywheels
        runAction(new LambdaAction(() -> Shooter.getInstance().stop()));

        // 3. Drive through trench corridor while intaking field fuel without mid-auto odometry reset
        runAction(new ParallelRaceAction(
            new FollowChoreoPath("TrenchSweep", false),
            new IntakeAction(999, Intake.IntakeState.INTAKING)
        ));

        // 4. Stow intake safely to standby
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)));

        // 5. Clean chassis stop
        runAction(new LambdaAction(() -> SwerveBase.getInstance().stop()));
    }
}
