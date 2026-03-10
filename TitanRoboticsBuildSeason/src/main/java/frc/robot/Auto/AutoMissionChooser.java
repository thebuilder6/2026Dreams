package frc.robot.Auto;

import frc.robot.Auto.Missions.*;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
    Class: AutoMissionChooser
    Description: This lets the person choose which mission is executed.
                 Now supports dynamic discovery of Choreo trajectories and simplified mission registration.
    Author: Unknown
 */

public class AutoMissionChooser {
    private final SendableChooser<String> missionChooser;
    private final Map<String, Supplier<MissionBase>> missionRegistry = new HashMap<>();

    public static double delay;
    private String cachedSelected = "Do Nothing";
    private Optional<MissionBase> autoMission = Optional.empty();

    public AutoMissionChooser() {
        missionChooser = new SendableChooser<>();

        // 1. Register specialized Java missions
        registerMission(LeftShootClimbMission.class);
        registerMission(AdvancedChoreoMission.class);

        // 2. Automatically register Choreo trajectories from the deploy directory
        registerChoreoMissions();

        // 3. Setup the chooser
        missionChooser.setDefaultOption("Do Nothing", "Do Nothing");
        for (String name : missionRegistry.keySet()) {
            missionChooser.addOption(name, name);
        }

        SmartDashboard.putNumber("Auto Delay (seconds)", 0);
        SmartDashboard.putData("Auto Mission", missionChooser);
        SmartDashboard.putString("Current Action System", "None");
    }

    /**
     * Registers a mission class by reading its @AutoMission annotation for the display name.
     */
    private void registerMission(Class<? extends MissionBase> missionClass) {
        AutoMission annotation = missionClass.getAnnotation(AutoMission.class);
        String name = (annotation != null) ? annotation.name() : missionClass.getSimpleName();
        missionRegistry.put(name, () -> {
            try {
                return missionClass.getDeclaredConstructor().newInstance();
            } catch (Exception e) {
                DriverStation.reportError("Failed to instantiate mission: " + name, e.getStackTrace());
                return null;
            }
        });
    }

    /**
     * Scans the deploy/choreo directory for .traj files and adds them as simple missions.
     */
    private void registerChoreoMissions() {
        File choreoDir = new File(Filesystem.getDeployDirectory(), "choreo");
        if (choreoDir.exists() && choreoDir.isDirectory()) {
            File[] files = choreoDir.listFiles((dir, name) -> name.endsWith(".traj"));
            if (files != null) {
                for (File file : files) {
                    String trajName = file.getName().replace(".traj", "");
                    // Only add if not already registered by a specialized mission
                    if (!missionRegistry.containsKey(trajName)) {
                        missionRegistry.put(trajName, () -> new DynamicChoreoMission(trajName));
                    }
                }
            }
        }
    }

    public void updateMissionCreator() {
        delay = SmartDashboard.getNumber("Auto Delay", 0);
        String selected = missionChooser.getSelected();

        if (selected == null) {
            selected = "Do Nothing";
        }

        if (!selected.equals(cachedSelected)) {
            autoMission = getAutoMissionForParams(selected);
        }

        cachedSelected = selected;
    }

    public Optional<MissionBase> getAutoMissionForParams(String missionName) {
        if (missionName == null || missionName.equals("Do Nothing")) {
            return Optional.empty();
        }

        Supplier<MissionBase> supplier = missionRegistry.get(missionName);
        if (supplier != null) {
            return Optional.ofNullable(supplier.get());
        }

        return Optional.empty();
    }

    public void reset() {
        autoMission = Optional.empty();
        cachedSelected = "Do Nothing";
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoMissionSelected", cachedSelected);
    }

    public SendableChooser<String> getRawChooser() {
        return missionChooser;
    }

    public String getSelected() {
        String selected = missionChooser.getSelected();
        return selected == null ? "Do Nothing" : selected;
    }

    public Optional<MissionBase> getAutoMission() {
        return autoMission;
    }
}