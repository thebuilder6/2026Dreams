package frc.robot.Auto.Missions;

//import these so that the mission is an option when testing
import frc.robot.Auto.AutoMissionChooser;
import frc.robot.Auto.AutoMissionEndedException;

// import the actions from the auto.actions folder
import frc.robot.Auto.Actions.WaitAction;
import frc.robot.Auto.Actions.FollowChoreoPath;

/**
 * Class: ExampleMission
 *
 * Description:
 * This file serves as a TEMPLATE and documentation reference for creating new autonomous
 * missions within the codebase.
 *
 * HOW TO USE THIS FILE:
 * 1. Copy this file and rename the class to match your new strategy (e.g., `ThreeNoteAuto.java`).
 * 2. Ensure it extends `MissionBase`.
 * 3. Override the `routine()` method.
 * 4. Use `runAction(new YourActionHere())` to queue sequential commands.
 *    For parallel commands, use `ParallelAction` or `ParallelRaceAction`.
 * 5. Important: Ensure you register your new mission in `AutoMissionChooser.java` so it appears
 *    on the dashboard!
 *
 * Author: Rhea
 */
public class ExampleMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // Put the actions you want to do here in order of execution
        runAction(new FollowChoreoPath("ExamplePath", true));
    }
}
