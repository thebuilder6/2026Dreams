package frc.robot.Interfaces;

public interface Subsystem extends edu.wpi.first.wpilibj2.command.Subsystem {
    public void update();

    public void initialize();

    public void log();

    public boolean isEnabled();

    public default void simulationUpdate() {
    }

    public default double getSimulationCurrentDraw() {
        return 0.0;
    }

    public String getName();
}