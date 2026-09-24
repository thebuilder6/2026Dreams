package frc.robot.Auto.Missions;

//import these so that the mission is an option when testing
import frc.robot.Auto.AutoMissionChooser;
import frc.robot.Auto.AutoMissionEndedException;

// import the actions from the auto.actions folder
import frc.robot.Auto.Actions.WaitAction;
import frc.robot.Auto.Actions.IntakeAction;
import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.Actions.ShootAction;

/*
 * Class: ShooterMission
 * Description: This mission is to shoot our eight fuel into the hub using a
 *              Choreo movement and then a shoot action.
 * Notes: Choreo (the path) can be changed any time, just remember to generate
 *        the code and deploy the new code
 * Author: Rhea
 */

public class ShooterMission extends MissionBase {
    @Override
    protected void routine() throws AutoMissionEndedException {
       
        runAction(new FollowChoreoPath("ShootPath", true));
        //runAction(new IntakeAction(5, "Intaking"));
        
        runAction(new ShootAction(10));
        
    }
}
