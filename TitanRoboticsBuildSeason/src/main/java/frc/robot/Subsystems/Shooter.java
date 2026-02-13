package frc.robot.Subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import static edu.wpi.first.units.Units.*;
import java.util.Optional;

import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Nat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.ThirdParty.LimelightHelpers;

public class Shooter implements frc.robot.Interfaces.Subsystem {

    private static Shooter instance = null;

    private final NeoSparkMaxMotor flywheelMotor;
    private final NeoSparkMaxMotor feederMotor;

    private final LinearSystemLoop<N1, N1, N1> flywheelLoop;

    private FlywheelSim flywheelSim;

    private double targetVelocityRPM = 0;
    private double lastBallSpawnTime = 0;
    private long simShotCount = 0;
    private long simScoreCount = 0;

    private final InterpolatingDoubleTreeMap shooterInterpolationMap = new InterpolatingDoubleTreeMap();

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

        // State-Space Control Setup
        // Plant: Models the flywheel physics (Velocity System) in SI units (rad/s)
        double kV_rads = ShooterConstants.kFlywheelV * 60.0 / (2.0 * Math.PI);
        double kA_rads = ShooterConstants.kFlywheelA * 60.0 / (2.0 * Math.PI);
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV_rads, kA_rads);

        // Controller: Linear Quadratic Regulator (LQR)
        // Adjust tolerances to rad/s (e.g., 20 RPM error tolerance)
        double velocityToleranceRads = (20.0 * 2.0 * Math.PI) / 60.0;
        LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(velocityToleranceRads), // q: Velocity error tolerance
                VecBuilder.fill(12.0), // r: Voltage tolerance
                0.02); // dt: 20ms loop time

        // Observer: Kalman Filter
        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                plant,
                VecBuilder.fill(10.0), // Process noise (Model uncertainty) in rad/s
                VecBuilder.fill(0.1), // Measurement noise (Sensor noise) in rad/s
                0.02); // dt: 20ms loop time

        // Combine into Loop
        flywheelLoop = new LinearSystemLoop<>(plant, controller, observer, 12.0, 0.02);

        if (RobotBase.isSimulation()) {
            flywheelSim = new FlywheelSim(
                    plant,
                    DCMotor.getNEO(1),
                    1.0 // Gear ratio
            );
        }

        // Initialize Interpolation Map (Distance in Meters -> RPM Offset)
        shooterInterpolationMap.put(0.0, 0.0);
        shooterInterpolationMap.put(1.0, 0.0);
        shooterInterpolationMap.put(3.0, 50.0);
        shooterInterpolationMap.put(5.0, 150.0);
        shooterInterpolationMap.put(10.0, 300.0);

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
     * Checks if the shooter is ready to fire based on RPM stability and alignment.
     * 
     * @param targetHeading The calculated target heading the robot should be at.
     * @return True if ready to shoot.
     */
    public boolean isReadyToFire(Rotation2d targetHeading) {
        double headingError = Math.abs(SwerveBase.getInstance().getHeading().minus(targetHeading).getDegrees());
        return Dashboard.getInstance().isHubActive() && isAtTargetVelocity() && headingError < 2.5;
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

        // 5. Apply Interpolation Offset
        double wheelCircumference = 0.1016 * Math.PI;
        double rpmOffset = shooterInterpolationMap.get(distance);
        double targetRPM = ((targetTotalSpeed / wheelCircumference) * 60.0) + rpmOffset;

        return new ShootingSolution(targetYaw, targetRPM, true);
    }

    @Override
    public void update() {

        if (targetVelocityRPM > 0) {
            // Convert measurement to SI (rad/s)
            double velocityRads = flywheelMotor.getVelocity() * (2.0 * Math.PI) / 60.0;
            double targetRads = targetVelocityRPM * (2.0 * Math.PI) / 60.0;

            // Correct the loop with the fresh measurement
            flywheelLoop.correct(VecBuilder.fill(velocityRads));

            // Predict the next state (20ms)
            flywheelLoop.predict(0.02);

            // Calculate the next control output
            flywheelLoop.setNextR(VecBuilder.fill(targetRads));

            // Get the calculated voltage
            double voltage = flywheelLoop.getU(0);

            // Add kS (Static Friction) feedforward manually
            voltage += Math.signum(targetVelocityRPM) * ShooterConstants.kFlywheelS;

            flywheelMotor.setVoltage(voltage);
        } else {
            flywheelMotor.stop();
            // Reset loop state (in rad/s)
            double velocityRads = flywheelMotor.getVelocity() * (2.0 * Math.PI) / 60.0;
            flywheelLoop.reset(VecBuilder.fill(velocityRads));
        }
    }

    public void simulationUpdate() {
        if (flywheelSim != null) {
            // In simulation, we rely on the same update() loop running before this
            // But for the physics sim, we need to pass the voltage
            // The motor wrapper usually handles "setVoltage" -> SimState, but let's be
            // explicit if needed
            // Actually, NeoSparkMaxMotor likely handles it. Let's just update the physics.

            // We can retrieve the last set voltage from the motor (if the wrapper supports
            // it)
            // Or we can assume update() ran.

            // For FlywheelSim, it needs the input voltage.
            // Since we set it in update(), let's just make sure flywheelSim gets it.
            // But wait, in the original code, it recalculated voltage here.
            // Let's use the actual applied voltage from the motor object if possible,
            // or just let the motor wrapper handle the sim state integration if it does.

            // Original code:
            // flywheelSim.setInput(voltage);
            // flywheelSim.update(0.02);

            // Since we switched to proper structure, let's just use the loop's calculated U
            // from previous step?
            // Or easier: Just let the loop run in update(), and here we just step the
            // physics.
            // We need to fetch the voltage we *just* asked the motor to run at.
            // Assuming "flywheelMotor.setSimState" does what we expect, we might not need
            // to manually step FlywheelSim
            // IF we were using the REV Physics Sim. But we are using WPILib FlywheelSim.

            // Let's rely on the loop's output.
            double voltage = flywheelLoop.getU(0) + Math.signum(targetVelocityRPM) * ShooterConstants.kFlywheelS;
            if (targetVelocityRPM == 0)
                voltage = 0;

            flywheelSim.setInput(voltage);
            flywheelSim.update(0.02); // 20ms sim step matches control loop

            // Update the motor's simulated encoder
            flywheelMotor.setSimState(flywheelSim.getAngularVelocityRPM(), 0);

            // Ball Simulation Logic
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            // Spawn balls if feeder is running and flywheel is near target
            if (Math.abs(feederMotor.getSpeed()) > 0.1
                    && isReadyToFire(calculateShootingSolution(SwerveBase.getInstance().getPose(),
                            SwerveBase.getInstance().getFieldVelocity()).turretAngle())
                    && (currentTime - lastBallSpawnTime) > 0.3) {
                int ballsToFire = GameSim.getInstance().consumeHeldBallsForShot(2);
                if (ballsToFire > 0) {
                    Pose2d robotPose = SwerveBase.getInstance().getPose();
                    ChassisSpeeds robotVel = SwerveBase.getInstance().getFieldVelocity();
                    double exitVelocity = (flywheelMotor.getVelocity() / 60.0) * (0.1016 * Math.PI);
                    Translation3d targetLoc = getGoalLocation();

                    for (int i = 0; i < ballsToFire; i++) {
                        // Lateral offset for 2-wide shooter (+/- 0.12m)
                        double lateralOffset = (ballsToFire == 2) ? (i == 0 ? -0.12 : 0.12) : 0.0;
                        Translation2d shooterOffset = new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS,
                                lateralOffset);

                        // Introduce Randomness (+/- 2% velocity, +/- 0.5 deg yaw, +/- 1 deg pitch)
                        double randomExitVelocity = exitVelocity * (1.0 + (Math.random() - 0.5) * 0.04);
                        Rotation2d randomYaw = robotPose.getRotation()
                                .plus(Rotation2d.fromDegrees((Math.random() - 0.5) * 1.0));
                        double randomPitch = ShooterConstants.SHOOTER_ANGLE_RAD + (Math.random() - 0.5) * 0.035; // ~2
                                                                                                                 // deg
                                                                                                                 // total
                                                                                                                 // spread

                        var fuelOnFly = new RebuiltFuelOnFly(
                                robotPose.getTranslation(),
                                shooterOffset,
                                robotVel,
                                randomYaw,
                                Meters.of(ShooterConstants.SHOOTER_HEIGHT_METERS),
                                MetersPerSecond.of(randomExitVelocity),
                                Radians.of(randomPitch));

                        // Probabilistic scoring logic: Swish (tight) vs Rim (loose)
                        fuelOnFly.withTargetPosition(() -> targetLoc)
                                .withTargetTolerance(new Translation3d(0.3, 0.4, 0.2)) // Tight swish zone
                                .withHitTargetCallBack(() -> {
                                    boolean isBlueGoal = targetLoc.equals(Constants.FieldConstants.BLUE_GOAL_LOCATION);
                                    if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt) {
                                        Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
                                        if (arena.isActive(isBlueGoal)) {
                                            simScoreCount++;
                                        }
                                    } else {
                                        simScoreCount++;
                                    }
                                });

                        // Secondary "Rim hit" chance (extra wide tolerance but only 40% probability)
                        if (Math.random() < 0.4) {
                            fuelOnFly.withTargetTolerance(new Translation3d(0.8, 1.0, 0.4));
                        }

                        swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance()
                                .addGamePieceProjectile(fuelOnFly);
                    }

                    simShotCount += ballsToFire;
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
    public double getSimulationCurrentDraw() {
        if (flywheelSim != null) {
            // Estimate feeder current (stall current is ~2.6A for NEO 550, free is ~0.4A)
            // Using a simple resistive model matching simulated voltage
            double feederCurrent = Math.abs(feederMotor.getSpeed()) * 2.0;
            return flywheelSim.getCurrentDrawAmps() + feederCurrent;
        }
        return 0.0;
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

    public double getTargetVelocityRPM() {
        return targetVelocityRPM;
    }

}
