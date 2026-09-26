package frc.robot.Data;

import frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import swervelib.math.Matter;

/**
 * Global Robot & Platform Constants.
 * 
 * DESIGN PRINCIPLE:
 * - This file contains core robot framework, chassis physics, and operator configuration.
 * - Seasonal field geometry lives in {@link FieldMap}.
 * - Mechanism-specific constants (tuning, setpoints, geometry) live directly within their subsystem
 *   packages (e.g. {@link MechanismConstants.Shooter},
 *   {@link MechanismConstants.Intake}).
 */
public class Constants {

    // =========================================================================
    // 1. CORE SYSTEM & EXECUTION ENVIRONMENT (Evergreen)
    // =========================================================================

    public static final Mode currentMode = edu.wpi.first.wpilibj.RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running in desktop simulation. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static Mode getMode() {
        return currentMode;
    }

    /**
     * Global flag for enabling live tuning of PID values/setpoints via NetworkTables.
     * Set to false for official competition matches to save loop time.
     */
    public static final boolean TUNING_MODE = true;

    // =========================================================================
    // 2. ROBOT CHASSIS & DRIVETRAIN DYNAMICS (Chassis-Specific)
    // =========================================================================

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32 lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double MAX_SPEED = Units.feetToMeters(15);
    public static final double MAX_ROTATION_SPEED = 8.0;

    // Java Units Type-Safe Configuration Constants
    public static final LinearVelocity MAX_SPEED_MEASURE = edu.wpi.first.units.Units.MetersPerSecond.of(MAX_SPEED);
    public static final AngularVelocity MAX_ROTATION_SPEED_MEASURE = edu.wpi.first.units.Units.RadiansPerSecond.of(MAX_ROTATION_SPEED);

    // =========================================================================
    // 3. AUTONOMOUS PATH FOLLOWING & HEADING PID (Evergreen)
    // =========================================================================

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

    // =========================================================================
    // 4. DRIVEBASE VISION ESTIMATION & FILTERING (Chassis / Sensor Mounts)
    // =========================================================================

    public static final class DrivebaseConstants {
        // Vision Rejection Thresholds
        public static final double VISION_MAX_YAW_RATE = 360.0; // deg/s
        public static final double VISION_MAX_TAG_DIST = 4.0;   // meters
        public static final double VISION_SINGLE_TAG_MAX_DIST = 3.0; // meters
        public static final double VISION_MAX_AMBIGUITY = 0.4;

        // Java Units measures for vision thresholds
        public static final AngularVelocity VISION_MAX_YAW_RATE_MEASURE = edu.wpi.first.units.Units.DegreesPerSecond.of(VISION_MAX_YAW_RATE);
        public static final Distance VISION_MAX_TAG_DIST_MEASURE = edu.wpi.first.units.Units.Meters.of(VISION_MAX_TAG_DIST);
        public static final Distance VISION_SINGLE_TAG_MAX_DIST_MEASURE = edu.wpi.first.units.Units.Meters.of(VISION_SINGLE_TAG_MAX_DIST);

        // Vision Trust (Std Dev) Coefficients
        public static final double VISION_BASE_STD_DEV = 0.1;
        public static final double VISION_SINGLE_TAG_PENALTY = 0.4;
        public static final double VISION_DIST_PENALTY_DIVISOR = 20.0;

        // Rubik Pi 3 (Qualcomm QCS6490) Camera Physical Mounting Geometry
        public static final double RUBIK_PI_CAMERA_HEIGHT_METERS = 0.45; // 45 cm from carpet
        public static final double RUBIK_PI_CAMERA_PITCH_DEG = -15.0;     // 15 degrees down-tilt toward carpet
        public static final double RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS = 0.25; // 25 cm forward from robot center
        public static final double FUEL_TARGET_HEIGHT_METERS = 0.075;    // Center of sphere for object detection

        public static final Distance RUBIK_PI_CAMERA_HEIGHT_MEASURE = edu.wpi.first.units.Units.Meters.of(RUBIK_PI_CAMERA_HEIGHT_METERS);
        public static final Angle RUBIK_PI_CAMERA_PITCH_MEASURE = edu.wpi.first.units.Units.Degrees.of(RUBIK_PI_CAMERA_PITCH_DEG);
        public static final Distance RUBIK_PI_CAMERA_FORWARD_OFFSET_MEASURE = edu.wpi.first.units.Units.Meters.of(RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS);
        public static final Distance FUEL_TARGET_HEIGHT_MEASURE = edu.wpi.first.units.Units.Meters.of(FUEL_TARGET_HEIGHT_METERS);
    }

    // =========================================================================
    // 5. OPERATOR CONTROLS & INPUT FILTERING (Evergreen)
    // =========================================================================

    public static class OperatorConstants {
        // Joystick Deadband
        public static final double DEADBAND = 0.1;

        // Driver Slew Rate Limiters (m/s^2 for translation, rad/s^2 for rotation)
        public static final TunableNumber TRANSLATION_SLEW_RATE = new TunableNumber("Operator/TranslationSlewRate", 16); // m/s^2
        public static final TunableNumber ROTATION_SLEW_RATE = new TunableNumber("Operator/RotationSlewRate", 10);       // rad/s^2
    }

    // =========================================================================
    // 6. ADDRESSABLE LED PATTERNS (Evergreen)
    // =========================================================================

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

    // =========================================================================
    // 7. MECHANISM & SEASONAL DELEGATES (Backward Compatibility)
    // =========================================================================
    // These delegates allow existing subsystem code to continue compiling without
    // disruption while new code imports mechanism constants directly from their packages.

    public static final class FieldConstants {
        public static final Translation3d RED_GOAL_LOCATION = FieldMap.Hubs.RED_HUB_3D;
        public static final Translation3d BLUE_GOAL_LOCATION = FieldMap.Hubs.BLUE_HUB_3D;
        public static final double GOAL_HEIGHT_METERS = FieldMap.Hubs.GOAL_HEIGHT;
    }

    public static final Translation3d RED_HUB_LOCATION = FieldMap.Hubs.RED_HUB_3D;
    public static final Translation3d BLUE_HUB_LOCATION = FieldMap.Hubs.BLUE_HUB_3D;

    // Shooter Delegates -> MechanismConstants.Shooter
    public static final double SHOOTER_PREDICTIVE_LOOK_AHEAD = MechanismConstants.Shooter.SHOOTER_PREDICTIVE_LOOK_AHEAD;
    public static final double SHOOTER_OFFSET = MechanismConstants.Shooter.SHOOTER_OFFSET;
    public static final double FIRING_ANGLE = MechanismConstants.Shooter.FIRING_ANGLE;
    public static final double HEIGHT_DIFFERENCE = MechanismConstants.Shooter.HEIGHT_DIFFERENCE;
    public static final double FLYWHEEL_KS = MechanismConstants.Shooter.FLYWHEEL_KS_VAL;
    public static final double FLYWHEEL_KV = MechanismConstants.Shooter.FLYWHEEL_KV_VAL;
    public static final double FLYWHEEL_KA = MechanismConstants.Shooter.FLYWHEEL_KA_VAL;
    public static final double FLYWHEEL_KP = MechanismConstants.Shooter.FLYWHEEL_KP_VAL;
    public static final double FLYWHEEL_KI = MechanismConstants.Shooter.FLYWHEEL_KI_VAL;
    public static final double FLYWHEEL_KD = MechanismConstants.Shooter.FLYWHEEL_KD_VAL;
    public static final double KICKER_VOLTAGE = MechanismConstants.Shooter.KICKER_VOLTAGE;
    public static final Voltage KICKER_VOLTAGE_MEASURE = MechanismConstants.Shooter.KICKER_VOLTAGE_MEASURE;

    public static final class ShooterConstants {
        public static final double FEED_SPEED = MechanismConstants.Shooter.FEED_SPEED;
        public static final double SHOOTER_ANGLE_RAD = MechanismConstants.Shooter.SHOOTER_ANGLE_RAD;
        public static final TunableNumber SHOOTER_HEIGHT_METERS = MechanismConstants.Shooter.SHOOTER_HEIGHT_METERS;
        public static final TunableNumber SHOOTER_OFFSET_METERS = MechanismConstants.Shooter.SHOOTER_OFFSET_METERS;
        public static final double IDLE_RPM = MechanismConstants.Shooter.IDLE_RPM;
        public static final AngularVelocity IDLE_RPM_MEASURE = MechanismConstants.Shooter.IDLE_RPM_MEASURE;
        public static final double RPM_TOLERANCE = MechanismConstants.Shooter.RPM_TOLERANCE;
        public static final double ALIGNMENT_HEADING_TOLERANCE_DEG = MechanismConstants.Shooter.ALIGNMENT_HEADING_TOLERANCE_DEG;
        public static final double LIMELIGHT_TX_TOLERANCE_DEG = MechanismConstants.Shooter.LIMELIGHT_TX_TOLERANCE_DEG;
        public static final TunableNumber FLYWHEEL_KP = MechanismConstants.Shooter.FLYWHEEL_KP;
        public static final TunableNumber FLYWHEEL_KI = MechanismConstants.Shooter.FLYWHEEL_KI;
        public static final TunableNumber FLYWHEEL_KD = MechanismConstants.Shooter.FLYWHEEL_KD;
        public static final TunableNumber FLYWHEEL_KS = MechanismConstants.Shooter.FLYWHEEL_KS;
        public static final TunableNumber FLYWHEEL_KV = MechanismConstants.Shooter.FLYWHEEL_KV;
        public static final TunableNumber FLYWHEEL_KA = MechanismConstants.Shooter.FLYWHEEL_KA;
        public static final double FLYWHEEL_CURRENT_LIMIT = MechanismConstants.Shooter.FLYWHEEL_CURRENT_LIMIT;
        public static final double KICKER_CURRENT_LIMIT = MechanismConstants.Shooter.KICKER_CURRENT_LIMIT;
        public static final Current FLYWHEEL_CURRENT_LIMIT_MEASURE = MechanismConstants.Shooter.FLYWHEEL_CURRENT_LIMIT_MEASURE;
        public static final Current KICKER_CURRENT_LIMIT_MEASURE = MechanismConstants.Shooter.KICKER_CURRENT_LIMIT_MEASURE;
        public static final double SIM_GEARING = MechanismConstants.Shooter.SIM_GEARING;
        public static final double SIM_MOI = MechanismConstants.Shooter.SIM_MOI;
        public static final double BALL_SPAWN_INTERVAL = MechanismConstants.Shooter.BALL_SPAWN_INTERVAL;
        public static final double SHOOTER_WHEEL_CIRCUMFERENCE = MechanismConstants.Shooter.SHOOTER_WHEEL_CIRCUMFERENCE;
        public static final TunableNumber BALL_LAUNCH_EFFICIENCY = MechanismConstants.Shooter.BALL_LAUNCH_EFFICIENCY;
    }

    // Intake Delegates -> MechanismConstants.Intake
    public static final double INTAKE_ARM_KP = MechanismConstants.Intake.INTAKE_ARM_KP_VAL;
    public static final double INTAKE_ARM_KI = MechanismConstants.Intake.INTAKE_ARM_KI_VAL;
    public static final double INTAKE_ARM_KD = MechanismConstants.Intake.INTAKE_ARM_KD_VAL;
    public static final double INTAKE_ARM_KS = MechanismConstants.Intake.INTAKE_ARM_KS_VAL;
    public static final double INTAKE_ARM_KG = MechanismConstants.Intake.INTAKE_ARM_KG_VAL;
    public static final double INTAKE_ARM_KV = MechanismConstants.Intake.INTAKE_ARM_KV_VAL;
    public static final double INTAKE_ARM_KA = MechanismConstants.Intake.INTAKE_ARM_KA_VAL;
    public static final double MAX_ARM_VELOCITY = MechanismConstants.Intake.MAX_ARM_VELOCITY;
    public static final double MAX_ARM_ACCELERATION = MechanismConstants.Intake.MAX_ARM_ACCELERATION;
    public static final double INTAKE_UP_POSITION = MechanismConstants.Intake.INTAKE_UP_POSITION;
    public static final double INTAKE_DOWN_POSITION = MechanismConstants.Intake.INTAKE_DOWN_POSITION;
    public static final double INTAKE_HORIZONTAL_POSITION = MechanismConstants.Intake.INTAKE_HORIZONTAL_POSITION;
    public static final Angle INTAKE_UP_POSITION_MEASURE = MechanismConstants.Intake.INTAKE_UP_POSITION_MEASURE;
    public static final Angle INTAKE_DOWN_POSITION_MEASURE = MechanismConstants.Intake.INTAKE_DOWN_POSITION_MEASURE;
    public static final Angle INTAKE_HORIZONTAL_POSITION_MEASURE = MechanismConstants.Intake.INTAKE_HORIZONTAL_POSITION_MEASURE;
    public static final boolean INTAKE_ARM_INVERTED = MechanismConstants.Intake.INTAKE_ARM_INVERTED;
    public static final boolean INTAKE_WHEELS_INVERTED = MechanismConstants.Intake.INTAKE_WHEELS_INVERTED;
    public static final double INTAKE_POSITION_OFFSET = MechanismConstants.Intake.INTAKE_POSITION_OFFSET;

    public static final class IntakeConstants {
        public static final boolean INTAKE_ARM_INVERTED = MechanismConstants.Intake.INTAKE_ARM_INVERTED;
        public static final boolean INTAKE_WHEELS_INVERTED = MechanismConstants.Intake.INTAKE_WHEELS_INVERTED;
        public static final double INTAKE_POSITION_OFFSET = MechanismConstants.Intake.INTAKE_POSITION_OFFSET;
        public static final double STALL_CURRENT_LIMIT = MechanismConstants.Intake.STALL_CURRENT_LIMIT;
        public static final double STALL_TIME = MechanismConstants.Intake.STALL_TIME;
        public static final double EJECT_TIME = MechanismConstants.Intake.EJECT_TIME;
        public static final Current STALL_CURRENT_LIMIT_MEASURE = MechanismConstants.Intake.STALL_CURRENT_LIMIT_MEASURE;
        public static final Time STALL_TIME_MEASURE = MechanismConstants.Intake.STALL_TIME_MEASURE;
        public static final Time EJECT_TIME_MEASURE = MechanismConstants.Intake.EJECT_TIME_MEASURE;
        public static final double INTAKE_SPEED = MechanismConstants.Intake.INTAKE_SPEED;
        public static final double HOPPER_SPEED = MechanismConstants.Intake.HOPPER_SPEED;
        public static final TunableNumber ARM_KP = MechanismConstants.Intake.ARM_KP;
        public static final TunableNumber ARM_KI = MechanismConstants.Intake.ARM_KI;
        public static final TunableNumber ARM_KD = MechanismConstants.Intake.ARM_KD;
        public static final TunableNumber ARM_KS = MechanismConstants.Intake.ARM_KS;
        public static final TunableNumber ARM_KG = MechanismConstants.Intake.ARM_KG;
        public static final TunableNumber ARM_KV = MechanismConstants.Intake.ARM_KV;
        public static final TunableNumber ARM_KA = MechanismConstants.Intake.ARM_KA;
        public static final double MAX_ARM_VELOCITY = MechanismConstants.Intake.MAX_ARM_VELOCITY;
        public static final double MAX_ARM_ACCELERATION = MechanismConstants.Intake.MAX_ARM_ACCELERATION;
        public static final double ARM_INTAKE_POS = MechanismConstants.Intake.ARM_INTAKE_POS;
        public static final double ARM_IDLE_POS = MechanismConstants.Intake.ARM_IDLE_POS;
        public static final int MAX_HELD_BALLS = MechanismConstants.Intake.MAX_HELD_BALLS;
        public static final double SIM_ARM_GEARING = MechanismConstants.Intake.SIM_ARM_GEARING;
        public static final double SIM_ARM_LENGTH = MechanismConstants.Intake.SIM_ARM_LENGTH;
        public static final double SIM_ARM_MASS = MechanismConstants.Intake.SIM_ARM_MASS;
    }
}
