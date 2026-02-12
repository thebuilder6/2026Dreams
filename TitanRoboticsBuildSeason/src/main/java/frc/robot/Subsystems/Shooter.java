package frc.robot.Subsystems;

import edu.wpi.first.units.Units;
import java.util.Optional;

import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.Interfaces.Subsystem;
import frc.robot.ThirdParty.LimelightHelpers;

public class Shooter implements Subsystem {

    private static Shooter instance = null;

    private final NeoSparkMaxMotor flywheelMotor;
    private final NeoSparkMaxMotor feederMotor;

    private final SimpleMotorFeedforward flywheelFeedforward;
    private final PIDController flywheelPID;

    private FlywheelSim flywheelSim;

    private double targetVelocityRPM = 0;
    private double lastBallSpawnTime = 0;
    private long simShotCount = 0;
    private long simScoreCount = 0;

    // Record to hold the calculated shooting solution
    public record ShootingSolution(Rotation2d turretAngle, double flywheelRPM, boolean possible) {
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        flywheelMotor = new NeoSparkMaxMotor(ShooterConstants.FLYWHEEL_MOTOR_ID);
        feederMotor = new NeoSparkMaxMotor(ShooterConstants.FEEDER_MOTOR_ID);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.inverted(false);
        flywheelMotor.configure(flywheelConfig);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.inverted(false);
        feederMotor.configure(feederConfig);

        flywheelFeedforward = new SimpleMotorFeedforward(
                ShooterConstants.kFlywheelS,
                ShooterConstants.kFlywheelV,
                ShooterConstants.kFlywheelA);

        flywheelPID = new PIDController(ShooterConstants.kFlywheelP, 0, 0);

        if (RobotBase.isSimulation()) {
            flywheelSim = new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(
                            ShooterConstants.kFlywheelV * 60 / (2 * Math.PI),
                            ShooterConstants.kFlywheelA * 60 / (2 * Math.PI)),
                    DCMotor.getNEO(1),
                    1.0 // Gear ratio 1:1 for now as it's directly on spark max relative encoder usually
            );
        }

        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Set the target velocity for the flywheel.
     * 
     * @param velocityRPM Target velocity in RPM.
     */
    public void setFlywheelVelocity(double velocityRPM) {
        this.targetVelocityRPM = velocityRPM;
    }

    public void setFeederSpeed(double speed) {
        feederMotor.setSpeed(speed);
    }

    public void stop() {
        targetVelocityRPM = 0;
        flywheelMotor.stop();
        feederMotor.stop();
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(flywheelMotor.getVelocity() - targetVelocityRPM) < 50; // 50 RPM tolerance
    }

    /**
     * Checks if the robot is aligned with the goal using the Limelight.
     * Uses tx (horizontal offset) from LimelightHelpers.
     */
    public boolean isLinedUp() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        return hasTarget && Math.abs(tx) < 2.0; // +/- 2 degrees tolerance
    }

    /**
     * Gets the goal location based on the current alliance.
     * Defaults to Blue Goal if alliance is not found.
     * 
     * @return Translation3d of the target goal.
     */
    private Translation3d getGoalLocation() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.FieldConstants.RED_GOAL_LOCATION;
        }
        return Constants.FieldConstants.BLUE_GOAL_LOCATION;
    }

    /**
     * Calculates the shooting solution (heading and RPM) to hit the goal while
     * moving.
     * Incorporates predictive look-ahead to compensate for control/sensor latency.
     * 
     * @param robotPose Current robot pose
     * @param robotVel  Current robot field-relative velocity
     * @return ShootingSolution containing target heading and RPM
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel) {
        return calculateShootingSolution(robotPose, robotVel, Constants.LOOP_TIME);
    }

    /**
     * Calculates the shooting solution (heading and RPM) to hit the goal while
     * moving.
     * 
     * @param robotPose     Current robot pose
     * @param robotVel      Current robot field-relative velocity (used for leading
     *                      target)
     * @param lookAheadTime Seconds to predict forward for the robot's pose
     * @return ShootingSolution containing target heading and RPM
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel, double lookAheadTime) {
        // 1. Predictive Look-ahead (Latency Compensation)
        // Adjust robot pose based on expected delay (sensor lag + loop time)
        Pose2d predictedPose = new Pose2d(
                robotPose.getX() + robotVel.vxMetersPerSecond * lookAheadTime,
                robotPose.getY() + robotVel.vyMetersPerSecond * lookAheadTime,
                robotPose.getRotation().plus(Rotation2d.fromRadians(robotVel.omegaRadiansPerSecond * lookAheadTime)));

        Translation2d goalLoc = getGoalLocation().toTranslation2d();
        Translation2d robotTrans = predictedPose.getTranslation();

        // 2. Compensate for physical shooter offset from robot center
        Translation2d shooterLoc = robotTrans.plus(
                new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS, 0).rotateBy(predictedPose.getRotation()));

        Translation2d diff = goalLoc.minus(shooterLoc);
        double distance = diff.getNorm();

        double heightDiff = Constants.FieldConstants.GOAL_HEIGHT_METERS - ShooterConstants.SHOOTER_HEIGHT_METERS;
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // 3. Calculate initial estimate for required horizontal velocity (vIdealMag)
        // Projectile motion: v = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta)
        // - y)) )
        double term = distance * tanTheta - heightDiff;
        if (term <= 0)
            return new ShootingSolution(new Rotation2d(), 0, false); // Impossible shot

        double vIdealMag = Math.sqrt((g * distance * distance) / (2 * cosTheta * cosTheta * term));

        // 4. Refine for robot velocity
        // vBallHorizontal = vShotHorizontal + vRobot
        Translation2d vRobot = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);
        Translation2d vShotHorizontal = diff.div(distance).times(vIdealMag * cosTheta).minus(vRobot);

        // Calculate new heading and RPM
        Rotation2d targetYaw = vShotHorizontal.getAngle();
        double targetHorizontalSpeed = vShotHorizontal.getNorm();
        double targetTotalSpeed = (targetHorizontalSpeed / cosTheta);

        // Convert m/s to RPM. Assumed 4 inch wheel (0.1016 m) -> Circumference approx
        // 0.319m
        double wheelCircumference = 0.1016 * Math.PI;
        double targetRPM = (targetTotalSpeed / wheelCircumference) * 60.0;

        return new ShootingSolution(targetYaw, targetRPM, true);
    }

    @Override
    public void update() {
        if (targetVelocityRPM > 0) {
            double ff = flywheelFeedforward.calculate(targetVelocityRPM);
            double feedback = flywheelPID.calculate(flywheelMotor.getVelocity(), targetVelocityRPM);
            flywheelMotor.setVoltage(ff + feedback);
        } else {
            flywheelMotor.stop();
        }
    }

    @Override
    public void simulationUpdate() {
        if (flywheelSim != null) {
            double voltage = (targetVelocityRPM > 0)
                    ? flywheelFeedforward.calculate(targetVelocityRPM)
                            + flywheelPID.calculate(flywheelMotor.getVelocity(), targetVelocityRPM)
                    : 0;

            flywheelSim.setInput(voltage);
            flywheelSim.update(0.02); // 20ms loop

            // Update the motor's simulated encoder
            flywheelMotor.setSimState(flywheelSim.getAngularVelocityRPM(), 0);

            // Ball Simulation Logic
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            // Spawn a ball if feeder is running and flywheel is near target
            if (feederMotor.getSpeed() > 0.1 && isAtTargetVelocity() && (currentTime - lastBallSpawnTime) > 0.2) {
                if (GameSim.getInstance().consumeHeldBallForShot()) {
                    Pose2d robotPose = SwerveBase.getInstance().getPose();
                    ChassisSpeeds robotVel = SwerveBase.getInstance().getFieldVelocity();

                    // Create Maple Sim projectile
                    // Note: The constructor args are based on the documentation example.
                    // We map our constants to the expected parameters.

                    double exitVelocity = (flywheelMotor.getVelocity() / 60.0) * (0.1016 * Math.PI);

                    var fuelOnFly = new RebuiltFuelOnFly(
                            robotPose.getTranslation(),
                            new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS, 0),
                            robotVel,
                            robotPose.getRotation(),
                            Units.Meters.of(ShooterConstants.SHOOTER_HEIGHT_METERS),
                            Units.MetersPerSecond.of(exitVelocity),
                            Units.Radians.of(ShooterConstants.SHOOTER_ANGLE_RAD));

                    // Configure target based on alliance
                    Translation3d targetLoc = getGoalLocation();

                    // Maple Sim utilities for mirroring might be needed if the library expects
                    // blue-relative always,
                    // but since we are providing the absolute field location, it might be fine.
                    // The docs example used: FieldMirroringUtils.toCurrentAllianceTranslation(...)
                    // We will trust our `getGoalLocation()` returns the correct field coordinates.

                    fuelOnFly.withTargetPosition(() -> targetLoc)
                            .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3)) // Tolerance from docs
                            .withHitTargetCallBack(() -> {
                                boolean isBlueGoal = targetLoc.equals(Constants.FieldConstants.BLUE_GOAL_LOCATION);
                                if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt) {
                                    Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
                                    if (arena.isActive(isBlueGoal)) {
                                        simScoreCount++;
                                    }
                                } else {
                                    // Fallback for generic arena
                                    simScoreCount++;
                                }
                            });

                    swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance()
                            .addGamePieceProjectile(fuelOnFly);

                    simShotCount++;
                    lastBallSpawnTime = currentTime;
                }
            }
        }
    }

    @Override
    public void initialize() {
        stop();
    }

    public long getSimShotCount() {
        return simShotCount;
    }

    public long getSimScoreCount() {
        return simScoreCount;
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Subsystems/Shooter/Flywheel Velocity", flywheelMotor.getVelocity());
        SmartDashboard.putNumber("Subsystems/Shooter/Target Velocity", targetVelocityRPM);
        SmartDashboard.putNumber("Subsystems/Shooter/Feeder Speed", feederMotor.getVelocity());
        SmartDashboard.putBoolean("Subsystems/Shooter/Is At Target", isAtTargetVelocity());
        SmartDashboard.putBoolean("Subsystems/Shooter/Is Lined Up", isLinedUp());

        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("Simulation/Shooter/Shot Count", simShotCount);
            SmartDashboard.putNumber("Simulation/Shooter/Score Count", simScoreCount);
        }
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
