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

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    /** Seconds to look ahead for projectile predictive math (accounts for mechanical/CAN latency). */
    public static final double SHOOTER_PREDICTIVE_LOOK_AHEAD = 0.13; 
    public static final double MAX_SPEED = Units.feetToMeters(15);
    public static final double MAX_ROTATION_SPEED = 8.0;
    // Maximum speed of the robot in meters per second, used to limit acceleration.

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
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        // Vision Rejection Thresholds
        public static final double VISION_MAX_YAW_RATE = 360.0; // deg/s
        public static final double VISION_MAX_TAG_DIST = 4.0; // meters
        public static final double VISION_SINGLE_TAG_MAX_DIST = 3.0; // meters
        public static final double VISION_MAX_AMBIGUITY = 0.4;

        // Vision Trust (Std Dev) Coefficients
        public static final double VISION_BASE_STD_DEV = 0.1;
        public static final double VISION_SINGLE_TAG_PENALTY = 0.4;
        public static final double VISION_DIST_PENALTY_DIVISOR = 20.0;
    }

    public static final class ShooterConstants {
        public static final int FLYWHEEL_MOTOR_LEFT_ID = PortMap.SHOOTER_MOTOR_LEFT_ID;
        public static final int FLYWHEEL_MOTOR_RIGHT_ID = PortMap.SHOOTER_MOTOR_RIGHT_ID;
        public static final int KICKER_MOTOR_ID = PortMap.KICKER_MOTOR_ID;

        public static final double SHOOT_SPEED = 0.8;
        public static final double KICK_VOLTAGE = 12.0;
        public static final double FEED_SPEED = 0.5;

        // Flywheel Gains (RPM based) - TUNE THESE
        public static final TunableNumber kFlywheelP = new TunableNumber("Shooter/kFlywheelP", 0.0001);
        public static final TunableNumber kFlywheelS = new TunableNumber("Shooter/kFlywheelS", 0.1); // Volts
        public static final TunableNumber kFlywheelV = new TunableNumber("Shooter/kFlywheelV", 0.002); // Volts per RPM
        public static final TunableNumber kFlywheelA = new TunableNumber("Shooter/kFlywheelA", 0.0001); // Volts per RPM^2

        // Physics Constants (Old values kept as defaults)
        public static final double SHOOTER_ANGLE_RAD = Units.degreesToRadians(75.0); // Fixed angle from floor
        public static final TunableNumber SHOOTER_HEIGHT_METERS = new TunableNumber("Shooter/HeightMeters", 0.5);
        public static final TunableNumber SHOOTER_OFFSET_METERS = new TunableNumber("Shooter/OffsetMeters", 0.3);
        public static final double SHOOT_MAX_DISTANCE = 5.0; // Meters
        public static final double IDLE_RPM = 60;

        // Tolerances
        public static final double RPM_TOLERANCE = 50.0;
        public static final double ALIGNMENT_HEADING_TOLERANCE_DEG = 2.5;
        public static final double LIMELIGHT_TX_TOLERANCE_DEG = 2.0;

        // Safety
        public static final double FLYWHEEL_CURRENT_LIMIT = 40.0; // Amps

        // Simulation
        public static final double SIM_GEARING = 1.0;
        public static final double SIM_MOI = 0.001; // Estimate
        public static final double BALL_SPAWN_INTERVAL = 0.3; // seconds
        public static final double SHOOTER_WHEEL_DIAMETER = 0.1016; // meters (4 inch)
        public static final double SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER * Math.PI;
    }

    public static final class IntakeConstants {
        public static final int ARM_MOTOR_ID = PortMap.INTAKE_ARM_MOTOR_ID;
        public static final int ROLLER_MOTOR_ID = PortMap.INTAKE_WHEELS_MOTOR_ID;
        public static final int HOPPER_MOTOR_ID = PortMap.HOPPER_MOTOR_CANID;

        public static final boolean INTAKE_ARM_INVERTED = false;
        public static final boolean INTAKE_WHEELS_INVERTED = true;
        public static final double INTAKE_POSITION_OFFSET = 276.0;

        public static final double STALL_CURRENT_LIMIT = 30.0; // Amps
        public static final double STALL_TIME = 0.5; // Seconds to trigger unjam
        public static final double EJECT_TIME = 1.0; // Seconds to eject

        public static final double INTAKE_SPEED = 0.7;
        public static final double ARM_UP_SPEED = 0.3;
        public static final double ARM_DOWN_SPEED = -0.3;
        public static final double HOPPER_SPEED = 0.5;

        // Arm Gains (Degrees based) - Retained physical tuning
        public static final TunableNumber kArmP = new TunableNumber("Intake/kArmP", 0.05);
        public static final TunableNumber kArmI = new TunableNumber("Intake/kArmI", 0.0);
        public static final TunableNumber kArmD = new TunableNumber("Intake/kArmD", 0.0);
        public static final TunableNumber kArmS = new TunableNumber("Intake/kArmS", 0.0);
        public static final TunableNumber kArmG = new TunableNumber("Intake/kArmG", 0.0);
        public static final TunableNumber kArmV = new TunableNumber("Intake/kArmV", 0.0);
        public static final TunableNumber kArmA = new TunableNumber("Intake/kArmA", 0.0);

        public static final double kMaxArmVelocity = 10.0; // degrees/s
        public static final double kMaxArmAcceleration = 10.0; // degrees/s^2

        public static final double ARM_INTAKE_POS = 1.0; // degrees
        public static final double ARM_IDLE_POS = 95.0; // degrees

        // Simulation
        public static final double SIM_ARM_GEARING = 100.0;
        public static final double SIM_ARM_LENGTH = 0.4; // meters
        public static final double SIM_ARM_MASS = 3.0; // kg
    }

    public static final class ClimberConstants {
        public static final int PNEUMATICS_MODULE_ID = 0;

        public static final int LEFT_FORWARD_CHANNEL = 7;
        public static final int LEFT_REVERSE_CHANNEL = 6;

        public static final int RIGHT_FORWARD_CHANNEL = 5;
        public static final int RIGHT_REVERSE_CHANNEL = 4;
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
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }
}
