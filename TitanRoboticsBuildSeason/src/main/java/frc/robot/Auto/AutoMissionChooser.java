package frc.robot.Auto;

import frc.robot.Auto.Missions.*;
//import frc.robot.Auto.Missions.BlueMissions.BlueScoreL4;
//import frc.robot.Auto.Missions.RedMissions.RedScoreL4;

import java.util.Optional;

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
        doNothing,
        DoSomething,
        exampleMission,
        MoveAcrossLineMission,
        ScoringL4Mission,
        LeftShootClimb,
        AdvancedChoreoMission,
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
        missionChooser.addOption("Left Shoot Climb", DesiredMission.LeftShootClimb);
        missionChooser.addOption("Advanced Choreo Shot", DesiredMission.AdvancedChoreoMission);

        SmartDashboard.putNumber("Auto Delay (seconds)", 0);

        SmartDashboard.putData("Auto Mission", missionChooser);
        SmartDashboard.putString("Current Action System", "None");

        try {
            alliance = DriverStation.getAlliance().orElseThrow(() -> new Exception("No alliance")).toString();
        } catch (Exception e) {
            // Handle the exception, for example:
            System.out.println("Exception occurred: " + e.getMessage());
        }
    }

    public void updateMissionCreator() {
        try {
            alliance = DriverStation.getAlliance().orElseThrow(() -> new Exception("No alliance")).toString();
        } catch (Exception e) {

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

    public Optional<MissionBase> getAutoMissionForParams(String missionName) {
        try {
            return getAutoMissionForParams(DesiredMission.valueOf(missionName));
        } catch (IllegalArgumentException e) {
            return Optional.empty();
        }
    }

    private Optional<MissionBase> getAutoMissionForParams(DesiredMission mission) {
        switch (mission) {
            case LeftShootClimb:
                return Optional.of(new LeftShootClimbMission());
            case AdvancedChoreoMission:
                return Optional.of(new AdvancedChoreoMission());
            case DoSomething:
                return Optional.of(new DoSomething());
            case exampleMission:
                return Optional.of(new ExampleMission());
            case doNothing:
            default:
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

    public SendableChooser<DesiredMission> getRawChooser() {
        return missionChooser;
    }

    public String getSelected() {
        DesiredMission selected = missionChooser.getSelected();
        return selected == null ? DesiredMission.doNothing.name() : selected.name();
    }

    public Optional<MissionBase> getAutoMission() {
        return autoMission;
    }
}