package frc.robot.Auto;

import frc.robot.Auto.Missions.*;
//import frc.robot.Auto.Missions.BlueMissions.BlueScoreL4;
//import frc.robot.Auto.Missions.RedMissions.RedScoreL4;

import java.util.Optional;

import com.fasterxml.jackson.databind.deser.NullValueProvider;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Data.Debug;

/*
    Class: AutoMissionChooser
    Description: This lets the person choose which mission is executed
    Author: Unknown
 */

public class AutoMissionChooser {
    enum DesiredMission {
        //these are the options you will see in smart dashboard.
        DoSomething,
        exampleMission,
        // general missions that use alliance to determine the actual missions
        ScoringL4Mission,
        // actual missions
        MoveAcrossLineMission,
        doNothing,
        RedScoreL4,
        BlueScoreL4,
    }

    private DesiredMission cachedDesiredMission = DesiredMission.doNothing;

    private final SendableChooser<DesiredMission> missionChooser;

    private Optional<MissionBase> autoMission = Optional.empty();

    public static double delay;

    String alliance;

    public AutoMissionChooser() {
        missionChooser = new SendableChooser<>();

        // add more here as needed, is what is seen when choosing a mission
        missionChooser.addOption("Do Nothing", DesiredMission.doNothing);
        missionChooser.addOption("Do Something", DesiredMission.DoSomething);
        missionChooser.addOption("Leave Community", DesiredMission.MoveAcrossLineMission);
        missionChooser.addOption("Scoring L4", DesiredMission.ScoringL4Mission);

        SmartDashboard.putNumber("Auto Delay (seconds)", 0);

        SmartDashboard.putData("Auto Mission", missionChooser);
        SmartDashboard.putString("Current Action System", "None");

        try {
            alliance = DriverStation.getAlliance().orElseThrow(() -> new Exception("No alliance")).toString();
        }
        catch (Exception e) {
            // Handle the exception, for example:
            System.out.println("Exception occurred: " + e.getMessage());
        }
    }

    public void updateMissionCreator() {
        try {
            alliance = DriverStation.getAlliance().orElseThrow(() -> new Exception("No alliance")).toString();
        }
        catch (Exception e) {
            
        }
        delay = SmartDashboard.getNumber("Auto Delay", 0);
        DesiredMission desiredMission = missionChooser.getSelected();

        if (desiredMission == null) {
            desiredMission = DesiredMission.doNothing;
        }

        if (cachedDesiredMission != desiredMission) {
            autoMission = getAutoMissionForParams(desiredMission);
        }

        cachedDesiredMission = desiredMission;
    }

    private Optional<MissionBase> getAutoMissionForParams(DesiredMission mission) {
        switch (mission) {
            // do nothing mission
            case doNothing:
                return null;
            // if no auto mission is found
            default:
                System.err.println("No valid autonomous mission found for" + mission);
                return Optional.empty();
        }
    }

    public void reset() {
        autoMission = Optional.empty();
        cachedDesiredMission = DesiredMission.doNothing;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoMissionSelected", cachedDesiredMission.name());
    }

    public Optional<MissionBase> getAutoMission() {
        return autoMission;
    }
}