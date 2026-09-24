package frc.robot.Subsystems.shooter;

/**
 * Hardware IO abstraction interface for the dual-flywheel shooter subsystem.
 * Follows the AdvantageKit pattern to cleanly decouple control logic from physical motor hardware.
 */
public interface ShooterIO {

    public static class ShooterIOInputs {
        public double leftVelocityRPM = 0.0;
        public double rightVelocityRPM = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double kickerAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
        public double kickerCurrentAmps = 0.0;
        public double kickerVelocityRPM = 0.0;
        public double leftBusVolts = 12.0;
        public double rightBusVolts = 12.0;
    }

    /** Updates the inputs struct with latest sensor/telemetry data from hardware or simulation. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets voltages to the left and right flywheel motors. */
    public default void setFlywheelVoltages(double leftVolts, double rightVolts) {}

    /** Sets voltage to the kicker motor. */
    public default void setKickerVoltage(double kickerVolts) {}

    /** Stops all shooter motors. */
    public default void stop() {
        setFlywheelVoltages(0.0, 0.0);
        setKickerVoltage(0.0);
    }
}
