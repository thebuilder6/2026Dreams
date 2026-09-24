package frc.robot.Subsystems.intake;

/**
 * Hardware IO abstraction interface for the articulated ground intake subsystem.
 * Follows the AdvantageKit pattern to cleanly isolate motor controllers and encoders.
 */
public interface IntakeIO {

    public static class IntakeIOInputs {
        public double armPositionDeg = 0.0;
        public double armVelocityDegPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double armMotorRotations = 0.0;
        public boolean encoderConnected = true;

        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerVelocityRPM = 0.0;

        public double hopperAppliedVolts = 0.0;
        public double hopperCurrentAmps = 0.0;
        public double hopperVelocityRPM = 0.0;
    }

    /** Updates inputs from hardware sensors or simulation. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Sets voltage to the arm pivot motor. */
    public default void setArmVoltage(double volts) {}

    /** Sets voltage to the roller intake motor. */
    public default void setRollerVoltage(double volts) {}

    /** Sets speed [-1.0, 1.0] to the roller intake motor. */
    public default void setRollerSpeed(double speed) {}

    /** Sets voltage to the hopper centering motor. */
    public default void setHopperVoltage(double volts) {}

    /** Sets speed [-1.0, 1.0] to the hopper centering motor. */
    public default void setHopperSpeed(double speed) {}

    /** Stops all intake motors. */
    public default void stop() {
        setArmVoltage(0.0);
        setRollerSpeed(0.0);
        setHopperSpeed(0.0);
    }
}
