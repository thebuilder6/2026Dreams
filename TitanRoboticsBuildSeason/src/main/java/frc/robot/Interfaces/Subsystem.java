package frc.robot.Interfaces;

public interface Subsystem {
    public void update();

    public void initialize();

    public void log();

    public boolean isEnabled();

    public default void simulationUpdate() {
    }

    public String getName();
}