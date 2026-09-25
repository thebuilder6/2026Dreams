package frc.robot.Auto.Missions;

//import these so that the mission is an option when testing
import frc.robot.Auto.AutoMissionChooser;
import frc.robot.Auto.AutoMissionEndedException;

// import the actions from the auto.actions folder
import frc.robot.Auto.Actions.WaitAction;
import frc.robot.Auto.Actions.FollowChoreoPath;

/*
 * Class: ExampleMission
 * Description: This mission is an example mission.
 *              It has examples of a normal action and a choreo path action
 * Author: Rhea
 */

public class ExampleMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
        // Put the actions you want to do here in order of execution
        runAction(new FollowChoreoPath("ExamplePath", true));
    }
}
