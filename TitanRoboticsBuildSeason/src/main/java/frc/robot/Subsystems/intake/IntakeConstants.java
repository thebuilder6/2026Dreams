package frc.robot.Subsystems.intake;

import frc.robot.Telemetry.TunableNumber;

/**
 * Physical geometry, tuning parameters, and motion limits for the Articulated Ground Intake.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // Motor and Encoder Config
    public static final boolean INTAKE_ARM_INVERTED = true;
    public static final boolean INTAKE_WHEELS_INVERTED = true;
    public static final double INTAKE_POSITION_OFFSET = 276.0;

    // Setpoints (Degrees)
    public static final double INTAKE_UP_POSITION = 347.0;
    public static final double INTAKE_DOWN_POSITION = 250.0;
    public static final double INTAKE_HORIZONTAL_POSITION = 250.0;

    public static final double ARM_INTAKE_POS = INTAKE_DOWN_POSITION;
    public static final double ARM_IDLE_POS = INTAKE_UP_POSITION;

    // Safety and Current
    public static final double STALL_CURRENT_LIMIT = 30.0; // Amps
    public static final double STALL_TIME = 0.5; // Seconds to trigger unjam
    public static final double EJECT_TIME = 1.0; // Seconds to eject

    // Java Units Type-Safe Configuration Constants
    public static final edu.wpi.first.units.measure.Angle INTAKE_UP_POSITION_MEASURE = edu.wpi.first.units.Units.Degrees.of(INTAKE_UP_POSITION);
    public static final edu.wpi.first.units.measure.Angle INTAKE_DOWN_POSITION_MEASURE = edu.wpi.first.units.Units.Degrees.of(INTAKE_DOWN_POSITION);
    public static final edu.wpi.first.units.measure.Angle INTAKE_HORIZONTAL_POSITION_MEASURE = edu.wpi.first.units.Units.Degrees.of(INTAKE_HORIZONTAL_POSITION);
    public static final edu.wpi.first.units.measure.Current STALL_CURRENT_LIMIT_MEASURE = edu.wpi.first.units.Units.Amps.of(STALL_CURRENT_LIMIT);
    public static final edu.wpi.first.units.measure.Time STALL_TIME_MEASURE = edu.wpi.first.units.Units.Seconds.of(STALL_TIME);
    public static final edu.wpi.first.units.measure.Time EJECT_TIME_MEASURE = edu.wpi.first.units.Units.Seconds.of(EJECT_TIME);

    // Roller Speeds
    public static final double INTAKE_SPEED = 0.7;
    public static final double HOPPER_SPEED = 0.5;

    // Arm Motion Profiling Limits
    public static final double MAX_ARM_VELOCITY = 400.0;     // deg/s
    public static final double MAX_ARM_ACCELERATION = 400.0; // deg/s^2

    // Arm Gains (Degrees based)
    public static final double INTAKE_ARM_KP_VAL = 0.1;
    public static final double INTAKE_ARM_KI_VAL = 0.0;
    public static final double INTAKE_ARM_KD_VAL = 0.01;
    public static final double INTAKE_ARM_KS_VAL = 0.2;
    public static final double INTAKE_ARM_KG_VAL = 0.34;
    public static final double INTAKE_ARM_KV_VAL = 0.0;
    public static final double INTAKE_ARM_KA_VAL = 0.0;

    public static final TunableNumber ARM_KP = new TunableNumber("Intake/kArmP", INTAKE_ARM_KP_VAL);
    public static final TunableNumber ARM_KI = new TunableNumber("Intake/kArmI", INTAKE_ARM_KI_VAL);
    public static final TunableNumber ARM_KD = new TunableNumber("Intake/kArmD", INTAKE_ARM_KD_VAL);
    public static final TunableNumber ARM_KS = new TunableNumber("Intake/kArmS", INTAKE_ARM_KS_VAL);
    public static final TunableNumber ARM_KG = new TunableNumber("Intake/kArmG", INTAKE_ARM_KG_VAL);
    public static final TunableNumber ARM_KV = new TunableNumber("Intake/kArmV", INTAKE_ARM_KV_VAL);
    public static final TunableNumber ARM_KA = new TunableNumber("Intake/kArmA", INTAKE_ARM_KA_VAL);

    // Simulation
    public static final int MAX_HELD_BALLS = 30;
    public static final double SIM_ARM_GEARING = 100.0;
    public static final double SIM_ARM_LENGTH = 0.4; // meters
    public static final double SIM_ARM_MASS = 3.0; // kg
}
