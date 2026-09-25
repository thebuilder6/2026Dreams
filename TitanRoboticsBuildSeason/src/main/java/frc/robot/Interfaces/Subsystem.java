package frc.robot.Interfaces;

import edu.wpi.first.wpilibj2.command.Command;

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

    /**
     * WPILib Commands v2 Subsystem.idle():
     * Returns a command that continuously requires this subsystem and holds it in an idle / safe standby state.
     */
    @Override
    public default Command idle() {
        return edu.wpi.first.wpilibj2.command.Subsystem.super.idle();
    }
}