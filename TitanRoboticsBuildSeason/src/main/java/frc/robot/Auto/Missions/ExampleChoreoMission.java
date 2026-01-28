package frc.robot.Auto.Missions;

import frc.robot.Auto.Actions.FollowChoreoPath;
import frc.robot.Auto.AutoMissionEndedException;

public class ExampleChoreoMission extends MissionBase {

    @Override
    protected void routine() throws AutoMissionEndedException {
        // Load the "TestPath" trajectory from Choreo and reset odometry to start
        FollowChoreoPath followPath = new FollowChoreoPath("TestPath", true);
        // Bind event markers to actions
        followPath.bind("marker1", () -> {
            System.out.println("Marker 1 reached!");
            // Add subsystem actions here, e.g., intaking, shooting
        });
        followPath.bind("stopIntake", () -> {
            System.out.println("Stopping Intake");
        });

        // Add the action to the mission queue
        runAction(followPath);
    }
}
