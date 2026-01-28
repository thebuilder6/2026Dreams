package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.Interfaces.Subsystem;
import frc.robot.ThirdParty.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;

public class Shooter implements Subsystem {

    private static Shooter instance = null;

    private final NeoSparkMaxMotor flywheelMotor;
    private final NeoSparkMaxMotor feederMotor;

    private final SimpleMotorFeedforward flywheelFeedforward;
    private final PIDController flywheelPID;

    private FlywheelSim flywheelSim;

    private double targetVelocityRPM = 0;
    private final List<SimBall> activeBalls = new ArrayList<>();
    private double lastBallSpawnTime = 0;

    private static class SimBall {
        double x, y, vx, vy, timeAlive;

        public SimBall(double x, double y, double vx, double vy) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.timeAlive = 0;
        }
    }

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
     * Calculates the distance from the robot to the goal.
     * 
     * @param robotPose Current robot pose.
     * @return Distance in meters.
     */
    public double calculateDistanceToGoal(Pose2d robotPose) {
        Translation3d robotPos3d = new Translation3d(robotPose.getX(), robotPose.getY(),
                ShooterConstants.SHOOTER_HEIGHT_METERS);
        return robotPos3d.getDistance(getGoalLocation());
    }

    /**
     * Calculates the shooting solution (heading and RPM) to hit the goal while
     * moving.
     * 
     * @param robotPose Current robot pose
     * @param robotVel  Current robot field-relative velocity
     * @return ShootingSolution containing target heading and RPM
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel) {
        Translation2d goalLoc = getGoalLocation().toTranslation2d();
        Translation2d robotLoc = robotPose.getTranslation();
        Translation2d diff = goalLoc.minus(robotLoc);
        double distance = diff.getNorm();

        double heightDiff = Constants.FieldConstants.GOAL_HEIGHT_METERS - ShooterConstants.SHOOTER_HEIGHT_METERS;

        // 1. Calculate ideal exit velocity (V_total) required if stationary
        // Using projectile motion equation: v = sqrt( (g * x^2) / (2 * cos^2(theta) *
        // (x * tan(theta) - y)) )
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        double term = distance * tanTheta - heightDiff;
        if (term <= 0)
            return new ShootingSolution(new Rotation2d(), 0, false); // Impossible shot

        double vIdealMag = Math.sqrt((g * distance * distance) / (2 * cosTheta * cosTheta * term));

        // Vector of the ideal shot in the horizontal plane
        Translation2d vIdealHorizontal = diff.div(distance).times(vIdealMag * cosTheta);

        // 2. Compensate for robot velocity
        // vShot_horizontal = vIdeal_horizontal - vRobot
        Translation2d vRobot = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);
        Translation2d vShotHorizontal = vIdealHorizontal.minus(vRobot);

        // 3. Calculate new heading and RPM
        Rotation2d targetYaw = vShotHorizontal.getAngle();
        double targetHorizontalSpeed = vShotHorizontal.getNorm();
        double targetTotalSpeed = targetHorizontalSpeed / cosTheta;

        // Convert m/s to RPM. This depends on flywheel radius and gear ratio
        // Approximation: RPM = (Speed / Circumference) * 60 * GearRatio
        // Assumed 4 inch wheel (0.1016 m) -> Circumference approx 0.319m
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
                Pose2d robotPose = SwerveBase.getInstance().getPose();
                Rotation2d shootHeading = robotPose.getRotation(); // Default to robot heading if no turret

                // Use a more accurate heading if snap-to-angle or auto-aim is in effect
                // For simplicity in sim, we use the robot's current rotation
                double exitVelocity = (flywheelMotor.getVelocity() / 60.0) * (0.1016 * Math.PI);
                double vHorizontal = exitVelocity * Math.cos(ShooterConstants.SHOOTER_ANGLE_RAD);

                activeBalls.add(new SimBall(
                        robotPose.getX(),
                        robotPose.getY(),
                        vHorizontal * shootHeading.getCos(),
                        vHorizontal * shootHeading.getSin()));
                System.out.println("DEBUG: Ball spawned at " + robotPose.getX() + ", " + robotPose.getY());
                lastBallSpawnTime = currentTime;
            }

            // Update active balls
            Iterator<SimBall> iter = activeBalls.iterator();
            List<Pose2d> ballPoses = new ArrayList<>();
            while (iter.hasNext()) {
                SimBall ball = iter.next();
                ball.x += ball.vx * 0.02;
                ball.y += ball.vy * 0.02;
                ball.timeAlive += 0.02;

                if (ball.timeAlive > 2.0) { // Despawn after 2 seconds
                    iter.remove();
                } else {
                    ballPoses.add(new Pose2d(ball.x, ball.y, new Rotation2d()));
                }
            }

            SwerveBase.getInstance().getField().getObject("Fuel").setPoses(ballPoses);
        }
    }

    @Override
    public void initialize() {
        stop();
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Shooter/Flywheel Velocity", flywheelMotor.getVelocity());
        SmartDashboard.putNumber("Shooter/Target Velocity", targetVelocityRPM);
        SmartDashboard.putNumber("Shooter/Feeder Speed", feederMotor.getVelocity());
        SmartDashboard.putBoolean("Shooter/Is At Target", isAtTargetVelocity());
        SmartDashboard.putBoolean("Shooter/Is Lined Up", isLinedUp());
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
