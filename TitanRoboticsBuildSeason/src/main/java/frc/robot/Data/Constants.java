package frc.robot.Data;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    // Global flag for enabling live tuning of PID values/setpoints via
    // NetworkTables.
    // Set to false for competition to save loop time.
    public static final boolean TUNING_MODE = true;

    public static final class FieldConstants {
        public static final Translation3d RED_GOAL_LOCATION = new Translation3d(11.938, 4.035, 1.575);
        public static final Translation3d BLUE_GOAL_LOCATION = new Translation3d(4.597, 4.035, 1.575);

        public static final double GOAL_HEIGHT_METERS = 1.575;
    }

    // Root-level constants from physical robot
    public static final Translation3d RED_HUB_LOCATION = FieldConstants.RED_GOAL_LOCATION;
    public static final Translation3d BLUE_HUB_LOCATION = FieldConstants.BLUE_GOAL_LOCATION;

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    /**
     * Seconds to look ahead for projectile predictive math (accounts for
     * mechanical/CAN latency).
     */
    public static final double SHOOTER_PREDICTIVE_LOOK_AHEAD = 0.13;
    public static final double MAX_SPEED = Units.feetToMeters(15);
    public static final double MAX_ROTATION_SPEED = 8.0;

    // Shooter physical parameters
    /** Shooter horizontal mounting offset from robot center (meters). */
    public static final double SHOOTER_OFFSET = -0.2032;
    /**
     * Fixed shooter hood/flywheel firing angle relative to horizontal (radians).
     */
    public static final double FIRING_ANGLE = Units.degreesToRadians(70);
    /**
     * Height differential between hub goal opening and shooter exit point (meters).
     */
    public static final double HEIGHT_DIFFERENCE = RED_HUB_LOCATION.getZ() - 0.53;

    // Flywheel Feedforward and Feedback Gains
    /** Flywheel static friction voltage feedforward (volts). */
    public static final double FLYWHEEL_KS = 0.0;
    /** Flywheel velocity feedforward gain (volts / RPM). */
    public static final double FLYWHEEL_KV = 0.0022;
    /** Flywheel acceleration feedforward gain (volts / (RPM/s)). */
    public static final double FLYWHEEL_KA = 0.0;
    /** Flywheel proportional feedback gain (volts / RPM error). */
    public static final double FLYWHEEL_KP = 0.0007;
    /** Flywheel integral feedback gain. */
    public static final double FLYWHEEL_KI = 0.000;
    /** Flywheel derivative feedback gain. */
    public static final double FLYWHEEL_KD = 0.000;
    /** Full nominal voltage applied to kicker feed motor (volts). */
    public static final double KICKER_VOLTAGE = 12.0;

    // Legacy aliases for backwards compatibility
    public static final double kFLYWHEELs = FLYWHEEL_KS;
    public static final double kFLYWHEELv = FLYWHEEL_KV;
    public static final double kFLYWHEELa = FLYWHEEL_KA;
    public static final double kFLYWHEELp = FLYWHEEL_KP;
    public static final double kFLYWHEELi = FLYWHEEL_KI;
    public static final double kFLYWHEELd = FLYWHEEL_KD;
    public static final double KICKERMOTOR = KICKER_VOLTAGE;

    // Intake physical parameters
    /** Intake pivot proportional gain. */
    public static final double INTAKE_ARM_KP = 0.1;
    /** Intake pivot integral gain. */
    public static final double INTAKE_ARM_KI = 0.0;
    /** Intake pivot derivative gain. */
    public static final double INTAKE_ARM_KD = 0.01;
    /** Intake pivot static friction voltage (volts). */
    public static final double INTAKE_ARM_KS = 0.2;
    /** Intake pivot gravity compensation feedforward (volts). */
    public static final double INTAKE_ARM_KG = 0.34;
    /** Intake pivot velocity feedforward gain (volts / (rad/s)). */
    public static final double INTAKE_ARM_KV = 0.0;
    /** Intake pivot acceleration feedforward gain (volts / (rad/s^2)). */
    public static final double INTAKE_ARM_KA = 0.0;

    /**
     * Maximum allowed angular velocity for intake pivot trapezoidal motion
     * profiling (deg/s).
     */
    public static final double MAX_ARM_VELOCITY = 400.0;
    /**
     * Maximum allowed angular acceleration for intake pivot trapezoidal motion
     * profiling (deg/s^2).
     */
    public static final double MAX_ARM_ACCELERATION = 400.0;

    /** Intake pivot stowed position setpoint (degrees). */
    public static final double INTAKE_UP_POSITION = 347.0;
    /** Intake pivot ground deployed position setpoint (degrees). */
    public static final double INTAKE_DOWN_POSITION = 250.0;
    /**
     * Intake pivot horizontal level position used for gravity cosine calculation
     * (degrees).
     */
    public static final double INTAKE_HORIZONTAL_POSITION = 250.0;
    /** Direction inversion flag for intake pivot motor. */
    public static final boolean INTAKE_ARM_INVERTED = true;
    /** Direction inversion flag for intake roller motor. */
    public static final boolean INTAKE_WHEELS_INVERTED = true;
    /** Absolute encoder zero-offset calibration (degrees). */
    public static final double INTAKE_POSITION_OFFSET = 276.0;

    public static final class AutonConstants {
        // PID constants for X, Y translation
        public static final double AUTO_DRIVE_KP = 10.0;
        public static final double AUTO_DRIVE_KI = 0.0;
        public static final double AUTO_DRIVE_KD = 0.0;

        // PID constants for holonomic heading rotation
        public static final double AUTO_TURN_KP = 7.5;
        public static final double AUTO_TURN_KI = 0.0;
        public static final double AUTO_TURN_KD = 0.0;

        // Dynamic Auton PID Tunables
        public static final TunableNumber DRIVE_KP = new TunableNumber("Auton/Drive_kP", AUTO_DRIVE_KP);
        public static final TunableNumber DRIVE_KI = new TunableNumber("Auton/Drive_kI", AUTO_DRIVE_KI);
        public static final TunableNumber DRIVE_KD = new TunableNumber("Auton/Drive_kD", AUTO_DRIVE_KD);

        public static final TunableNumber TURN_KP = new TunableNumber("Auton/Turn_kP", AUTO_TURN_KP);
        public static final TunableNumber TURN_KI = new TunableNumber("Auton/Turn_kI", AUTO_TURN_KI);
        public static final TunableNumber TURN_KD = new TunableNumber("Auton/Turn_kD", AUTO_TURN_KD);
    }

    public static final class DrivebaseConstants {
        // Vision Rejection Thresholds
        public static final double VISION_MAX_YAW_RATE = 360.0; // deg/s
        public static final double VISION_MAX_TAG_DIST = 4.0; // meters
        public static final double VISION_SINGLE_TAG_MAX_DIST = 3.0; // meters
        public static final double VISION_MAX_AMBIGUITY = 0.4;

        // Vision Trust (Std Dev) Coefficients
        public static final double VISION_BASE_STD_DEV = 0.1;
        public static final double VISION_SINGLE_TAG_PENALTY = 0.4;
        public static final double VISION_DIST_PENALTY_DIVISOR = 20.0;

        // Rubik Pi 3 (Qualcomm QCS6490) Camera Physical Mounting Geometry
        public static final double RUBIK_PI_CAMERA_HEIGHT_METERS = 0.45; // 45 cm from carpet
        public static final double RUBIK_PI_CAMERA_PITCH_DEG = -15.0; // 15 degrees down-tilt toward carpet
        public static final double RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS = 0.25; // 25 cm forward from robot center
        public static final double FUEL_TARGET_HEIGHT_METERS = 0.075; // Fuel radius ~3 inches (center of sphere)
    }

    public static final class ShooterConstants {
        public static final double FEED_SPEED = 0.5;

        // Physics Constants
        public static final double SHOOTER_ANGLE_RAD = FIRING_ANGLE;
        public static final TunableNumber SHOOTER_HEIGHT_METERS = new TunableNumber("Shooter/HeightMeters", 0.53);
        public static final TunableNumber SHOOTER_OFFSET_METERS = new TunableNumber("Shooter/OffsetMeters",
                SHOOTER_OFFSET);
        public static final double IDLE_RPM = 60;

        // Tolerances
        public static final double RPM_TOLERANCE = 50.0;
        public static final double ALIGNMENT_HEADING_TOLERANCE_DEG = 3.0;
        public static final double LIMELIGHT_TX_TOLERANCE_DEG = 2.0;

        // Flywheel Feedback & Feedforward Tunables
        public static final TunableNumber FLYWHEEL_KP = new TunableNumber("Shooter/kP", Constants.FLYWHEEL_KP);
        public static final TunableNumber FLYWHEEL_KI = new TunableNumber("Shooter/kI", Constants.FLYWHEEL_KI);
        public static final TunableNumber FLYWHEEL_KD = new TunableNumber("Shooter/kD", Constants.FLYWHEEL_KD);
        public static final TunableNumber FLYWHEEL_KS = new TunableNumber("Shooter/kS", Constants.FLYWHEEL_KS);
        public static final TunableNumber FLYWHEEL_KV = new TunableNumber("Shooter/kV", Constants.FLYWHEEL_KV);
        public static final TunableNumber FLYWHEEL_KA = new TunableNumber("Shooter/kA", Constants.FLYWHEEL_KA);

        // Safety
        public static final double FLYWHEEL_CURRENT_LIMIT = 40.0; // Amps

        // Simulation
        public static final double SIM_GEARING = 1.0;
        public static final double SIM_MOI = 0.001; // Estimate
        public static final double BALL_SPAWN_INTERVAL = 0.3; // seconds
        public static final double SHOOTER_WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI;
        /**
         * Energy transfer and slip efficiency from flywheel surface to ball exit
         * velocity (~0.42 for dual flywheels).
         */
        public static final TunableNumber BALL_LAUNCH_EFFICIENCY = new TunableNumber("Shooter/SimLaunchEfficiency",
                0.42);
    }

    public static final class IntakeConstants {
        public static final boolean INTAKE_ARM_INVERTED = Constants.INTAKE_ARM_INVERTED;
        public static final boolean INTAKE_WHEELS_INVERTED = Constants.INTAKE_WHEELS_INVERTED;
        public static final double INTAKE_POSITION_OFFSET = Constants.INTAKE_POSITION_OFFSET;

        public static final double STALL_CURRENT_LIMIT = 30.0; // Amps
        public static final double STALL_TIME = 0.5; // Seconds to trigger unjam
        public static final double EJECT_TIME = 1.0; // Seconds to eject

        public static final double INTAKE_SPEED = 0.7;
        public static final double HOPPER_SPEED = 0.5;

        // Arm Gains (Degrees based) - Physical tuning
        public static final TunableNumber ARM_KP = new TunableNumber("Intake/kArmP", INTAKE_ARM_KP);
        public static final TunableNumber ARM_KI = new TunableNumber("Intake/kArmI", INTAKE_ARM_KI);
        public static final TunableNumber ARM_KD = new TunableNumber("Intake/kArmD", INTAKE_ARM_KD);
        public static final TunableNumber ARM_KS = new TunableNumber("Intake/kArmS", INTAKE_ARM_KS);
        public static final TunableNumber ARM_KG = new TunableNumber("Intake/kArmG", INTAKE_ARM_KG);
        public static final TunableNumber ARM_KV = new TunableNumber("Intake/kArmV", INTAKE_ARM_KV);
        public static final TunableNumber ARM_KA = new TunableNumber("Intake/kArmA", INTAKE_ARM_KA);

        public static final double MAX_ARM_VELOCITY = Constants.MAX_ARM_VELOCITY;
        public static final double MAX_ARM_ACCELERATION = Constants.MAX_ARM_ACCELERATION;

        public static final double ARM_INTAKE_POS = INTAKE_DOWN_POSITION;
        public static final double ARM_IDLE_POS = INTAKE_UP_POSITION;

        // Simulation
        public static final double SIM_ARM_GEARING = 100.0;
        public static final double SIM_ARM_LENGTH = 0.4; // meters
        public static final double SIM_ARM_MASS = 3.0; // kg
    }

    public static final class LEDConstants {
        public static final int BLINKIN_PWM_PORT = 0;

        // Blinkin Patterns (-1.0 to 1.0)
        public static final double RAINBOW = -0.99;
        public static final double RAINBOW_PARTY = -0.97;
        public static final double CONFETTI = -0.87;
        public static final double LARSON_SCAN_RED = -0.35;
        public static final double HEARTBEAT_RED = -0.25;
        public static final double HEARTBEAT_BLUE = -0.23;
        public static final double BREATH_RED = -0.17;
        public static final double BREATH_BLUE = -0.15;
        public static final double STROBE_RED = -0.11;
        public static final double STROBE_BLUE = -0.09;
        public static final double STROBE_GOLD = -0.07;
        public static final double STROBE_WHITE = -0.05;

        // Solid Colors (Color 1)
        public static final double SOLID_RED = 0.61;
        public static final double SOLID_ORANGE = 0.65;
        public static final double SOLID_YELLOW = 0.69;
        public static final double SOLID_GREEN = 0.77;
        public static final double SOLID_BLUE = 0.87;
        public static final double SOLID_PURPLE = 0.91;
        public static final double SOLID_WHITE = 0.93;
        public static final double SOLID_BLACK = 0.99;
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;

        // Driver Slew Rate Limiters (m/s^2 for translation, rad/s^2 for rotation)
        public static final TunableNumber TRANSLATION_SLEW_RATE = new TunableNumber("Operator/TranslationSlewRate", 16); // m/s^2
        public static final TunableNumber ROTATION_SLEW_RATE = new TunableNumber("Operator/RotationSlewRate", 10); // rad/s^2
    }
}
