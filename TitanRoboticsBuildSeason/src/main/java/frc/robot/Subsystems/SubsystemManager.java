package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Interfaces.Subsystem;

public class SubsystemManager {
    private static final List<Subsystem> subsystems = new ArrayList<Subsystem>();

    /**
     * Minimum interval between repeat failure reports for the same subsystem
     * (nanoseconds). Failures are isolated per subsystem; reporting is
     * throttled so a persistently throwing subsystem cannot spam the console.
     */
    private static final long REPORT_THROTTLE_NANOS = 10_000_000_000L;
    private static final Map<String, Long> lastReportNanos = new HashMap<String, Long>();

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
        // Snapshot to avoid ConcurrentModificationException if initialize() registers new subsystems
        List<Subsystem> snapshot = new ArrayList<>(subsystems);
        for (Subsystem subsystem : snapshot) {
            subsystem.initialize();
        }
    }

    public static void updateSubsystems() {
        for (Subsystem subsystem : subsystems) {
            runGuarded(subsystem, "update", () -> subsystem.update());
        }
    }

    public static void simulationUpdateSubsystems() {
        for (Subsystem subsystem : subsystems) {
            runGuarded(subsystem, "simulationUpdate", () -> subsystem.simulationUpdate());
        }
    }

    public static void logSubsystems() {
        for (Subsystem subsystem : subsystems) {
            runGuarded(subsystem, "log", () -> subsystem.log());
        }
    }

    /**
     * Runs one subsystem callback in isolation: a throwing subsystem is
     * reported (throttled) and skipped instead of aborting the remaining
     * subsystems' ticks. Only {@link Exception} is contained; {@link Error}
     * still propagates.
     */
    private static void runGuarded(Subsystem subsystem, String phase, Runnable call) {
        try {
            call.run();
        } catch (Exception e) {
            long now = System.nanoTime();
            String name;
            try {
                name = subsystem.getName();
            } catch (Exception ignored) {
                name = subsystem.getClass().getSimpleName();
            }
            Long last = lastReportNanos.get(name);
            if (last == null || now - last > REPORT_THROTTLE_NANOS) {
                lastReportNanos.put(name, now);
                DriverStation.reportWarning(
                        "SubsystemManager isolated " + name + "." + phase + " failure: " + e, false);
            }
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