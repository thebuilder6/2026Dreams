package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMission;
import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.BranchAction;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.ParallelRaceAction;
import frc.robot.Auto.Actions.SeriesAction;
import frc.robot.Auto.Actions.ShootAction;
import frc.robot.Auto.Actions.WaitForBallAction;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Intelligent branching autonomous mission:
 * 1. Drives to human player depot with intake active.
 * 2. Uses sensor feedback to check if game piece was successfully acquired.
 * 3. Dynamically branches:
 *    - If ball acquired: drives to shoot position and executes scoring routine.
 *    - If ball missed: aborts transit to conserve time and stows intake to standby.
 */
@AutoMission(name = "Adaptive Depot Sweep")
public class AdaptiveDepotMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Move to depot while deploying intake
        runAction(new ParallelRaceAction(
            new FollowChoreoPath("DepotPath", true),
            new IntakeAction(999, Intake.IntakeState.INTAKING)
        ));

        // 2. Sensor-gated wait to ingest game piece (up to 1.5 seconds)
        runAction(new WaitForBallAction(1.5));

        // 3. Sensor-conditioned dynamic branch
        runAction(new BranchAction(
            () -> Intake.getInstance().hasGamePiece(),
            // Branch A: Ball secured -> stow intake, drive to shooting position, and fire
            new SeriesAction(
                new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)),
                new FollowChoreoPath("DepotToShootPath", false),
                new ShootAction(3.5)
            ),
            // Branch B: Ball missed -> stow intake and hold defensive position
            new SeriesAction(
                new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.STANDBY)),
                new LambdaAction(() -> SwerveBase.getInstance().stop())
            )
        ));

        // 4. Subsystems clean stop
        runAction(new LambdaAction(() -> {
            Shooter.getInstance().stop();
            SwerveBase.getInstance().stop();
            Intake.getInstance().stop();
        }));
    }
}
