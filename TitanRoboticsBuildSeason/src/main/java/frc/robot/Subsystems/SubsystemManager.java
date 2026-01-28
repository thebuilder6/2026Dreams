package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import frc.robot.Interfaces.Subsystem;

public class SubsystemManager {
    public static List<Subsystem> subsystems = new ArrayList<Subsystem>();

    /**
     * Registers a subsystem for periodic updating and execution.
     *
     * @param subsystem Subsystem to register
     */
    public static void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    /**
     * Initializes all registered subsystems by calling their initialize method.
     * This method should
     * be called once at the beginning of the program to set up all of the robot's
     * subsystems.
     */
    public static void initializeSubsystems() {
        for (Subsystem subsystem : subsystems) {
            subsystem.initialize();
        }
    }

    public static void updateSubsystems() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public static void simulationUpdateSubsystems() {
        for (Subsystem subsystem : subsystems) {
            subsystem.simulationUpdate();
        }
    }

    /**
     * Returns the list of registered subsystems. This list is unmodifiable, as the
     * purpose is to
     * allow access to the list for the purpose of iterating over the list to
     * perform actions on
     * each subsystem, such as performing periodic updates.
     *
     * @return List of registered subsystems
     */
    public static List<Subsystem> getSubsystems() {
        return subsystems;
    }
}