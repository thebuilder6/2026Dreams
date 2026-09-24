package frc.robot.Data;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    // Global flag for enabling live tuning of PID values/setpoints via NetworkTables.
    // Set to false for competition to save loop time.
    public static final boolean TUNING_MODE = true;

    public static final class FieldConstants {
        public static final Translation3d RED_GOAL_LOCATION = new Translation3d(11.938, 4.035, 1.829);
        public static final Translation3d BLUE_GOAL_LOCATION = new Translation3d(4.597, 4.035, 1.829);

        public static final double GOAL_HEIGHT_METERS = 1.829;
    }

    // Root-level constants from physical robot
    public static final Translation3d RED_HUB_LOCATION = FieldConstants.RED_GOAL_LOCATION;
    public static final Translation3d BLUE_HUB_LOCATION = FieldConstants.BLUE_GOAL_LOCATION;

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    /** Seconds to look ahead for projectile predictive math (accounts for mechanical/CAN latency). */
    public static final double SHOOTER_PREDICTIVE_LOOK_AHEAD = 0.13; 
    public static final double MAX_SPEED = Units.feetToMeters(15);
    public static final double MAX_ROTATION_SPEED = 8.0;

    // Shooter physical parameters
    public static final double SHOOTER_OFFSET = -0.2032;
    public static final double FIRING_ANGLE = Units.degreesToRadians(70);
    public static final double HEIGHT_DIFFERENCE = RED_HUB_LOCATION.getZ() - 0.53;
    public static final double kFLYWHEELs = 0.0;
    public static final double kFLYWHEELv = 0.0022;
    public static final double kFLYWHEELa = 0.0;
    public static final double kFLYWHEELp = 0.0007;
    public static final double kFLYWHEELi = 0.000;
    public static final double kFLYWHEELd = 0.000;
    public static final double KICKERMOTOR = 12.0;

    // Intake physical parameters
    public static final double INTAKE_ARM_KP = 0.1;
    public static final double INTAKE_ARM_KI = 0.0;
    public static final double INTAKE_ARM_KD = 0.01;
    public static final double INTAKE_ARM_KS = 0.2;
    public static final double INTAKE_ARM_KG = 0.34;
    public static final double INTAKE_ARM_KV = 0.0;
    public static final double INTAKE_ARM_KA = 0.0;

    public static final double MAX_ARM_VELOCITY = 400.0; // degrees per second
    public static final double MAX_ARM_ACCELERATION = 400.0; // degrees per second squared

    public static final double INTAKE_UP_POSITION = 347.0;
    public static final double INTAKE_DOWN_POSITION = 250.0;
    public static final double INTAKE_HORIZONTAL_POSITION = 250.0;
    public static final boolean INTAKE_ARM_INVERTED = true;
    public static final boolean INTAKE_WHEELS_INVERTED = true;
    public static final double INTAKE_POSITION_OFFSET = 276.0;

    public static final class AutonConstants {
        // PID constants for X, Y, and Rotation
        public static final double kAutoDriveP = 10.0;
        public static final double kAutoDriveI = 0.0;
        public static final double kAutoDriveD = 0.0;

        public static final double kAutoTurnP = 7.5;
        public static final double kAutoTurnI = 0.0;
        public static final double kAutoTurnD = 0.0;
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
        public static final TunableNumber SHOOTER_OFFSET_METERS = new TunableNumber("Shooter/OffsetMeters", SHOOTER_OFFSET);
        public static final double IDLE_RPM = 60;

        // Tolerances
        public static final double RPM_TOLERANCE = 50.0;
        public static final double ALIGNMENT_HEADING_TOLERANCE_DEG = 3.0;
        public static final double LIMELIGHT_TX_TOLERANCE_DEG = 2.0;

        // Safety
        public static final double FLYWHEEL_CURRENT_LIMIT = 40.0; // Amps

        // Simulation
        public static final double SIM_GEARING = 1.0;
        public static final double SIM_MOI = 0.001; // Estimate
        public static final double BALL_SPAWN_INTERVAL = 0.3; // seconds
        public static final double SHOOTER_WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI;
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
        public static final TunableNumber kArmP = new TunableNumber("Intake/kArmP", INTAKE_ARM_KP);
        public static final TunableNumber kArmI = new TunableNumber("Intake/kArmI", INTAKE_ARM_KI);
        public static final TunableNumber kArmD = new TunableNumber("Intake/kArmD", INTAKE_ARM_KD);
        public static final TunableNumber kArmS = new TunableNumber("Intake/kArmS", INTAKE_ARM_KS);
        public static final TunableNumber kArmG = new TunableNumber("Intake/kArmG", INTAKE_ARM_KG);
        public static final TunableNumber kArmV = new TunableNumber("Intake/kArmV", INTAKE_ARM_KV);
        public static final TunableNumber kArmA = new TunableNumber("Intake/kArmA", INTAKE_ARM_KA);

        public static final double kMaxArmVelocity = MAX_ARM_VELOCITY;
        public static final double kMaxArmAcceleration = MAX_ARM_ACCELERATION;

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
    }
}
