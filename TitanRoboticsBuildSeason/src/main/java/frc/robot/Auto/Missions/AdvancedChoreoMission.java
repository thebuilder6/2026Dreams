package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.*;
import frc.robot.Subsystems.Intake;

/**
 * Example of a high-performance Choreo mission.
 * Features:
 * - Starting intake on a marker while driving.
 * - Shooting while moving (Auto-Aim) triggered by a marker.
 */
public class AdvancedChoreoMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // 1. Initialize path
        FollowChoreoPath path = new FollowChoreoPath("AdvancedLeftStart", true);

        // 2. High-level parallel execution
        runAction(new ParallelAction(
                path,

                // Intake sequence: Wait for 'IntakeOn' marker, then run intake for 2s
                new SeriesAction(
                        new WaitUntilMarkerAction(path, "IntakeOn"),
                        new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.INTAKING)),
                        new WaitAction(2.0),
                        new LambdaAction(() -> Intake.getInstance().setState(Intake.IntakeState.IDLE))),

                // Shooting sequence: Wait for 'StartAim' marker, then initiate Auto-Aim while
                // moving
                new SeriesAction(
                        new WaitUntilMarkerAction(path, "StartAim"),
                        new AutoAimAction(path, 8.0) // Aim and shoot for 1.5 seconds while driving
                )));
    }
}
