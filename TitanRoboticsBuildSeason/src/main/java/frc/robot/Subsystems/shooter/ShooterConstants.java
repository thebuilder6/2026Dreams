package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.Data.FieldMap;
import frc.robot.Data.TunableNumber;

/**
 * Physical geometry, tuning parameters, and simulation metrics for the Dual-Flywheel Shooter mechanism.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // Shooter Physical Geometry
    public static final double SHOOTER_OFFSET = -0.2032; // meters horizontal mounting offset from center
    public static final double FIRING_ANGLE = Units.degreesToRadians(70); // fixed hood angle relative to horizontal
    public static final double SHOOTER_ANGLE_RAD = FIRING_ANGLE;
    public static final double SHOOTER_MOUNT_HEIGHT_METERS = 0.53;
    public static final double HEIGHT_DIFFERENCE = FieldMap.Hubs.GOAL_HEIGHT - SHOOTER_MOUNT_HEIGHT_METERS;

    public static final TunableNumber SHOOTER_HEIGHT_METERS = new TunableNumber("Shooter/HeightMeters", SHOOTER_MOUNT_HEIGHT_METERS);
    public static final TunableNumber SHOOTER_OFFSET_METERS = new TunableNumber("Shooter/OffsetMeters", SHOOTER_OFFSET);

    // Speed & Tolerances
    public static final double FEED_SPEED = 0.5;
    public static final double IDLE_RPM = 60.0;
    public static final double RPM_TOLERANCE = 50.0;
    public static final double ALIGNMENT_HEADING_TOLERANCE_DEG = 3.0;
    public static final double LIMELIGHT_TX_TOLERANCE_DEG = 2.0;

    /** Flywheel static friction voltage feedforward (volts). */
    public static final double FLYWHEEL_KS_VAL = 0.0;
    /** Flywheel velocity feedforward gain (volts / RPM). */
    public static final double FLYWHEEL_KV_VAL = 0.0022;
    /** Flywheel acceleration feedforward gain (volts / (RPM/s)). */
    public static final double FLYWHEEL_KA_VAL = 0.0;
    /** Flywheel proportional feedback gain (volts / RPM error). */
    public static final double FLYWHEEL_KP_VAL = 0.0007;
    /** Flywheel integral feedback gain. */
    public static final double FLYWHEEL_KI_VAL = 0.000;
    /** Flywheel derivative feedback gain. */
    public static final double FLYWHEEL_KD_VAL = 0.000;

    /** Full nominal voltage applied to kicker feed motor (volts). */
    public static final double KICKER_VOLTAGE = 12.0;
    public static final double SHOOTER_PREDICTIVE_LOOK_AHEAD = 0.13;

    // Flywheel Feedback & Feedforward Tunables
    public static final TunableNumber FLYWHEEL_KP = new TunableNumber("Shooter/kP", FLYWHEEL_KP_VAL);
    public static final TunableNumber FLYWHEEL_KI = new TunableNumber("Shooter/kI", FLYWHEEL_KI_VAL);
    public static final TunableNumber FLYWHEEL_KD = new TunableNumber("Shooter/kD", FLYWHEEL_KD_VAL);
    public static final TunableNumber FLYWHEEL_KS = new TunableNumber("Shooter/kS", FLYWHEEL_KS_VAL);
    public static final TunableNumber FLYWHEEL_KV = new TunableNumber("Shooter/kV", FLYWHEEL_KV_VAL);
    public static final TunableNumber FLYWHEEL_KA = new TunableNumber("Shooter/kA", FLYWHEEL_KA_VAL);

    // Current Limits
    public static final double FLYWHEEL_CURRENT_LIMIT = 40.0; // Amps
    public static final double KICKER_CURRENT_LIMIT = 30.0;   // Amps

    // Java Units Type-Safe Configuration Constants
    public static final edu.wpi.first.units.measure.Voltage KICKER_VOLTAGE_MEASURE = edu.wpi.first.units.Units.Volts.of(KICKER_VOLTAGE);
    public static final edu.wpi.first.units.measure.Current FLYWHEEL_CURRENT_LIMIT_MEASURE = edu.wpi.first.units.Units.Amps.of(FLYWHEEL_CURRENT_LIMIT);
    public static final edu.wpi.first.units.measure.Current KICKER_CURRENT_LIMIT_MEASURE = edu.wpi.first.units.Units.Amps.of(KICKER_CURRENT_LIMIT);
    public static final edu.wpi.first.units.measure.AngularVelocity IDLE_RPM_MEASURE = edu.wpi.first.units.Units.RPM.of(IDLE_RPM);

    // Simulation
    public static final double SIM_GEARING = 1.0;
    public static final double SIM_MOI = 0.001; // Estimate
    public static final double BALL_SPAWN_INTERVAL = 0.12; // seconds
    public static final double SHOOTER_WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI;
    public static final TunableNumber BALL_LAUNCH_EFFICIENCY = new TunableNumber("Shooter/SimLaunchEfficiency", 0.42);
}
