package frc.robot.Data;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final class FieldConstants {
        public static final Translation3d RED_GOAL_LOCATION = new Translation3d(11.938, 4.035, 1.829);
        public static final Translation3d BLUE_GOAL_LOCATION = new Translation3d(4.597, 4.035, 1.829);

        public static final double GOAL_HEIGHT_METERS = 1.829;

    }

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(20);
    public static final double MAX_ROTATION_SPEED = 20;
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
    }

    public static final class ShooterConstants {
        public static final int FLYWHEEL_MOTOR_ID = 10;
        public static final int FEEDER_MOTOR_ID = 11;

        public static final double SHOOT_SPEED = 0.8;
        public static final double FEED_SPEED = 0.5;

        // Flywheel Gains (RPM based) - TUNE THESE
        public static final double kFlywheelP = 0.0001;
        public static final double kFlywheelS = 0.1; // Volts
        public static final double kFlywheelV = 0.002; // Volts per RPM
        public static final double kFlywheelA = 0.0001; // Volts per RPM^2

        // Physics Constants
        public static final double SHOOTER_ANGLE_RAD = Units.degreesToRadians(75.0); // Fixed angle from floor
        public static final double SHOOTER_HEIGHT_METERS = 0.5; // Height from floor
        public static final double SHOOTER_OFFSET_METERS = 0.3; // Distance forward from robot center
        public static final double SHOOT_MAX_DISTANCE = 5.0; // Meters
    }

    public static final class IntakeConstants {
        public static final int ARM_MOTOR_ID = 20;
        public static final int ROLLER_MOTOR_ID = 21;
        public static final int HOPPER_MOTOR_ID = 22;

        public static final double STALL_CURRENT_LIMIT = 30.0; // Amps
        public static final double STALL_TIME = 0.5; // Seconds to trigger unjam
        public static final double EJECT_TIME = 1.0; // Seconds to eject

        public static final double INTAKE_SPEED = 0.7;
        public static final double ARM_UP_SPEED = 0.3;
        public static final double ARM_DOWN_SPEED = -0.3;
        public static final double HOPPER_SPEED = 0.5;

        // Arm Gains (Radians based) - TUNE THESE
        public static final double kArmP = 1.0;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.0;
        public static final double kArmS = 0.1;
        public static final double kArmG = 0.2;
        public static final double kArmV = 0.5;
        public static final double kArmA = 0.1;

        public static final double kMaxArmVelocity = 2.0; // rad/s
        public static final double kMaxArmAcceleration = 1.0; // rad/s^2

        // Encoder conversion: motor rotations to arm radians
        // Formula: (2 * PI) / gear_ratio - adjust gear ratio for your hardware
        public static final double ARM_GEAR_RATIO = 100.0; // Example: 100:1 reduction
        public static final double ARM_POSITION_CONVERSION = (2 * Math.PI) / ARM_GEAR_RATIO; // rotations -> rad
        public static final double ARM_VELOCITY_CONVERSION = ARM_POSITION_CONVERSION / 60.0; // RPM -> rad/s

        public static final double ARM_INTAKE_POS = 0.5; // rad
        public static final double ARM_IDLE_POS = 0.0; // rad
    }

    public static final class ClimberConstants {
        public static final int PNEUMATICS_MODULE_ID = 1;
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 1;
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }
}
