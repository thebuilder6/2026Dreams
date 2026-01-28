package frc.robot.Auto.Missions;

//import these so that the mission is an option when testing
import frc.robot.Auto.AutoMissionChooser;
import frc.robot.Auto.AutoMissionEndedException;

// import the actions from the auto.actions folder
import frc.robot.Auto.Actions.WaitAction;


/*
 * Class: ExampleMission
 * Description: This mission is an example mission.
 *              It has examples of a normal action and a parallel action.
 * Notes: This is the normal mission format that does not use Choreo.
 * Author: Rhea Sneller
 */

public class ExampleMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
       
        //put the actions you want to do here in order of execution
        
        // String options for MoveElevatorAction: passive, ramp, Score L1, Score L2, Score L3, Score L4, ejecting coral
        runAction(new WaitAction(AutoMissionChooser.delay));
       // runAction(new TurnDegreesAction(90, 3.0)); 
       // runAction(new DriveForTimeAction (.5, 0));;
       // runAction(new ParallelAction(new DriveForTimeAction( 0, 0) , new TurnDegreesAction(20, 1)));
       // runAction(new MoveElevatorAction(2, state.SCOREL3));
       // runAction(new EffectorAction(0.5,1));
    }
}