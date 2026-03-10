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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Nat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Data.TunableNumber;
import frc.robot.Devices.NeoSparkMaxMotor;
import frc.robot.Sim.GameSim;

public class Shooter implements frc.robot.Interfaces.Subsystem {

    private static Shooter instance = null;

    private final NeoSparkMaxMotor flywheelMotorLeft;
    private final NeoSparkMaxMotor flywheelMotorRight;
    private final NeoSparkMaxMotor kickerMotor;

    /**
     * The Linear System Loop combines the Plant, Controller, and Observer.
     * Implements State-Space Control (Chapter 6) and Discrete Control (Chapter 7).
     */
    private final LinearSystemLoop<N1, N1, N1> flywheelLoop;

    private frc.robot.Sim.ShooterSim flywheelSim;

    private double targetVelocityRPM = 0;
    private double lastBallSpawnTime = 0;
    private long simShotCount = 0;
    private long simScoreCount = 0;

    private final InterpolatingDoubleTreeMap shooterInterpolationMap = new InterpolatingDoubleTreeMap();

    public static class ShootingSolution {
        private final Rotation2d turretAngle;
        private final double flywheelRPM;
        private final boolean possible;

        public ShootingSolution(Rotation2d turretAngle, double flywheelRPM, boolean possible) {
            this.turretAngle = turretAngle;
            this.flywheelRPM = flywheelRPM;
            this.possible = possible;
        }

        public Rotation2d turretAngle() {
            return turretAngle;
        }

        public double flywheelRPM() {
            return flywheelRPM;
        }

        public boolean possible() {
            return possible;
        }
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        flywheelMotorLeft = new NeoSparkMaxMotor(ShooterConstants.FLYWHEEL_MOTOR_LEFT_ID);
        flywheelMotorRight = new NeoSparkMaxMotor(ShooterConstants.FLYWHEEL_MOTOR_RIGHT_ID);
        kickerMotor = new NeoSparkMaxMotor(ShooterConstants.KICKER_MOTOR_ID);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.inverted(false);
        flywheelConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelMotorLeft.configure(flywheelConfig);

        SparkMaxConfig flywheelRightConfig = new SparkMaxConfig();
        flywheelRightConfig.inverted(true);
        flywheelRightConfig.smartCurrentLimit((int) ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelMotorRight.configure(flywheelRightConfig);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(false);
        kickerMotor.configure(kickerConfig);

        // State-Space Control Setup
        // Plant: Models the flywheel physics (Velocity System) in SI units (rad/s)
        double kV_rads = ShooterConstants.kFlywheelV.get() * 60.0 / (2.0 * Math.PI);
        double kA_rads = ShooterConstants.kFlywheelA.get() * 60.0 / (2.0 * Math.PI);
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
            flywheelSim = new frc.robot.Sim.ShooterSim();
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
     * Set the raw voltage for the flywheel and kicker.
     * Used by standalone SysId testing.
     */
    public void setVoltages(double flywheelVolts, double kickerVolts) {
        setFlywheelVoltages(flywheelVolts, flywheelVolts);
        kickerMotor.setVoltage(kickerVolts);
    }

    /** Sets flywheel voltages individually for diagnostics. */
    public void setFlywheelVoltages(double leftVolts, double rightVolts) {
        flywheelMotorLeft.setVoltage(leftVolts);
        flywheelMotorRight.setVoltage(rightVolts);
    }

    /**
     * Get the applied voltage for the left flywheel motor.
     */
    public double getFlywheelLeftAppliedVoltage() {
        return flywheelMotorLeft.getBusVoltage() * flywheelMotorLeft.getAppliedOutput();
    }

    /**
     * Get the velocity of the left flywheel in RPM.
     */
    public double getFlywheelLeftVelocityRPM() {
        return flywheelMotorLeft.getVelocity();
    }

    /**
     * Get the velocity of the right flywheel in RPM.
     */
    public double getFlywheelRightVelocityRPM() {
        return flywheelMotorRight.getVelocity();
    }

    /**
     * Get the applied voltage for the kicker motor.
     */
    public double getKickerAppliedVoltage() {
        return kickerMotor.getBusVoltage() * kickerMotor.getAppliedOutput();
    }

    /**
     * Get the velocity of the kicker in RPM.
     */
    public double getKickerVelocityRPM() {
        return kickerMotor.getVelocity();
    }

    /**
     * Set the target velocity for the flywheel.
     * 
     * @param velocityRPM Target velocity in RPM.
     */
    public void setFlywheelVelocity(double velocityRPM) {
        this.targetVelocityRPM = velocityRPM;
    }

    /**
     * Alias for setFlywheelVelocity to match user's manual controller
     * implementation.
     * 
     * @param rpm Target velocity in RPM.
     */
    public void setTargetRPM(double rpm) {
        setFlywheelVelocity(rpm);
    }

    public void setKickerSpeed(double speed) {
        kickerMotor.setSpeed(speed);
    }

    public void stop() {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        if (stackTrace.length > 2) {
            String caller = stackTrace[2].getClassName() + "." + stackTrace[2].getMethodName();
            // Don't print if called by Loop or Init, only interesting callers
            if (!caller.contains("linearSystem") && targetVelocityRPM > 0) {
                System.out.println("[Shooter] STOP called by: " + caller);
            }
        }

        targetVelocityRPM = 0;
        flywheelMotorLeft.stop();
        flywheelMotorRight.stop();
        kickerMotor.stop();
    }

    public double getActualRPM() {
        return flywheelMotorLeft.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(flywheelMotorLeft.getVelocity() - targetVelocityRPM) < ShooterConstants.RPM_TOLERANCE;
    }

    /**
     * Checks if the shooter is ready to fire based on RPM stability and alignment.
     * 
     * @param targetHeading The calculated target heading the robot should be at.
     * @return True if ready to shoot.
     */
    public boolean isReadyToFire(Rotation2d targetHeading) {
        double headingError = Math.abs(SwerveBase.getInstance().getHeading().minus(targetHeading).getDegrees());
        return Dashboard.getInstance().isHubActive() && isAtTargetVelocity()
                && headingError < ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG;
    }

    /**
     * Checks if the robot is aligned with the goal using the Limelight.
     */
    public boolean isLinedUp() {
        Vision vision = Vision.getInstance();
        boolean hasTarget = vision.hasTarget();
        double tx = vision.getTX();
        return hasTarget && Math.abs(tx) < ShooterConstants.LIMELIGHT_TX_TOLERANCE_DEG;
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
     * Uses 3D projectile motion equations:
     * v = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y)) )
     * where x is horizontal distance and y is height difference.
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
        // 1. Predictive Look-ahead
        Pose2d predictedPose = new Pose2d(
                robotPose.getX() + robotVel.vxMetersPerSecond * lookAheadTime,
                robotPose.getY() + robotVel.vyMetersPerSecond * lookAheadTime,
                robotPose.getRotation().plus(Rotation2d.fromRadians(robotVel.omegaRadiansPerSecond * lookAheadTime)));

        Translation2d goalLoc = getGoalLocation().toTranslation2d();
        Translation2d shooterLoc = predictedPose.getTranslation().plus(
                new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS.get(), 0)
                        .rotateBy(predictedPose.getRotation()));

        Translation2d vRobot = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

        // We will iteratively refine the virtual target location
        Translation2d virtualGoalLoc = goalLoc;
        Rotation2d targetYaw = new Rotation2d();
        double targetTotalSpeed = 0;
        double distance = 0;

        double heightDiff = Constants.FieldConstants.GOAL_HEIGHT_METERS - ShooterConstants.SHOOTER_HEIGHT_METERS.get();
        double g = 9.81;
        double theta = ShooterConstants.SHOOTER_ANGLE_RAD;
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // ITERATIVE CONVERGENCE (Loop 3 times to perfect the math)
        for (int i = 0; i < 3; i++) {
            Translation2d diff = virtualGoalLoc.minus(shooterLoc);
            distance = diff.getNorm();

            double term = distance * tanTheta - heightDiff;
            if (term <= 0)
                return new ShootingSolution(new Rotation2d(), 0, false);

            // Calculate time of flight and ideal velocity for this specific distance
            double vIdealMag = Math.sqrt((g * distance * distance) / (2 * cosTheta * cosTheta * term));

            // Subtract robot velocity to find the new required shot vector
            Translation2d vShotHorizontal = diff.div(distance).times(vIdealMag * cosTheta).minus(vRobot);

            targetYaw = vShotHorizontal.getAngle();
            double targetHorizontalSpeed = vShotHorizontal.getNorm();
            targetTotalSpeed = (targetHorizontalSpeed / cosTheta);

            // Update the virtual goal location for the next loop based on how much the
            // robot's velocity shifted the shot
            // Time of flight = distance / horizontal speed
            double timeOfFlight = distance / (vIdealMag * cosTheta);
            virtualGoalLoc = goalLoc.minus(vRobot.times(timeOfFlight));
        }

        // 5. Apply Interpolation Offset using the final converged distance
        double wheelCircumference = ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE;
        double rpmOffset = shooterInterpolationMap.get(distance);
        double targetRPM = ((targetTotalSpeed / wheelCircumference) * 60.0) + rpmOffset;

        return new ShootingSolution(targetYaw, targetRPM, true);
    }

    @Override
    public void update() {

        if (targetVelocityRPM > 0) {
            // Convert measurement to SI (rad/s)
            double velocityRads = flywheelMotorLeft.getVelocity() * (2.0 * Math.PI) / 60.0;
            double targetRads = targetVelocityRPM * (2.0 * Math.PI) / 60.0;

            // Correct the loop with the fresh measurement
            flywheelLoop.correct(VecBuilder.fill(velocityRads));

            // Predict the next state (20ms)
            flywheelLoop.predict(0.02);

            // Calculate the next control output
            flywheelLoop.setNextR(VecBuilder.fill(targetRads));

            // Get the calculated voltage
            double voltage = flywheelLoop.getU(0);

            // Note: If we wanted to tune kV/kA live, we would need to re-generate the
            // plant/controller/observer here.
            // For now, we allow live tuning of kS (Static Friction) as it is applied
            // outside the LinearSystemLoop.
            double feedforwardS = calculateStaticFriction(targetVelocityRPM);
            voltage += feedforwardS;

            flywheelMotorLeft.setVoltage(voltage);
            flywheelMotorRight.setVoltage(voltage);
        } else {
            flywheelMotorLeft.stop();
            flywheelMotorRight.stop();
            // Reset loop state (in rad/s)
            double velocityRads = flywheelMotorLeft.getVelocity() * (2.0 * Math.PI) / 60.0;
            flywheelLoop.reset(VecBuilder.fill(velocityRads));
        }
    }

    @Override
    public void simulationUpdate() {
        if (flywheelSim != null) {
            // Calculate voltage including static friction feedforward
            double voltage = flywheelLoop.getU(0) + Math.signum(targetVelocityRPM) * ShooterConstants.kFlywheelS.get();
            if (targetVelocityRPM == 0)
                voltage = 0;

            flywheelSim.update(voltage);

            // Update the motor's simulated encoder
            flywheelMotorLeft.setSimState(flywheelSim.getVelocityRPM(), 0);
            flywheelMotorRight.setSimState(flywheelSim.getVelocityRPM(), 0);

            // Ball Simulation Logic
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

            // IF Kicker is running AND we have waited long enough since last ball
            if (Math.abs(kickerMotor.getSpeed()) > 0.1
                    && (currentTime - lastBallSpawnTime) > ShooterConstants.BALL_SPAWN_INTERVAL) {
                int ballsToFire = GameSim.getInstance().consumeHeldBallsForShot(2);
                if (ballsToFire > 0) {
                    Pose2d robotPose = SwerveBase.getInstance().getPose();
                    ChassisSpeeds robotVel = SwerveBase.getInstance().getFieldVelocity();
                    double exitVelocity = (flywheelMotorLeft.getVelocity() / 60.0)
                            * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE;
                    Translation3d targetLoc = getGoalLocation();

                    for (int i = 0; i < ballsToFire; i++) {
                        // Lateral offset for 2-wide shooter (+/- 0.12m)
                        double lateralOffset = (ballsToFire == 2) ? (i == 0 ? -0.12 : 0.12) : 0.0;
                        Translation2d shooterOffset = new Translation2d(ShooterConstants.SHOOTER_OFFSET_METERS.get(),
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
                                Meters.of(ShooterConstants.SHOOTER_HEIGHT_METERS.get()),
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
            // Estimate kicker current (stall current is ~2.6A for NEO 550, free is ~0.4A)
            // Using a simple resistive model matching simulated voltage
            double kickerCurrent = Math.abs(kickerMotor.getSpeed()) * 2.0;
            return flywheelSim.getCurrentDrawAmps() + kickerCurrent;
        }
        return 0.0;
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Subsystems/Shooter/Flywheel Velocity", flywheelMotorLeft.getVelocity());
        SmartDashboard.putNumber("Subsystems/Shooter/Target Velocity", targetVelocityRPM);
        SmartDashboard.putNumber("Subsystems/Shooter/Kicker Speed", kickerMotor.getVelocity());
        SmartDashboard.putBoolean("Subsystems/Shooter/Is At Target", isAtTargetVelocity());
        SmartDashboard.putBoolean("Subsystems/Shooter/Is Lined Up", isLinedUp());

        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("Simulation/Shooter/Shot Count", simShotCount);
            SmartDashboard.putNumber("Simulation/Shooter/Score Count", simScoreCount);
        }

        // 3D Mechanism Visualization
        // Shooter is at a fixed angle and position
        Translation3d shooterRootRobotRelative = new Translation3d(
                ShooterConstants.SHOOTER_OFFSET_METERS.get(),
                0,
                ShooterConstants.SHOOTER_HEIGHT_METERS.get());
        Rotation3d shooterRotation = new Rotation3d(0, -ShooterConstants.SHOOTER_ANGLE_RAD, 0);
        Pose3d shooterPose = new Pose3d(shooterRootRobotRelative, shooterRotation);

        SmartDashboard.putNumberArray("Subsystems/Shooter/ShooterPose3d", new double[] {
                shooterPose.getX(),
                shooterPose.getY(),
                shooterPose.getZ(),
                shooterPose.getRotation().getQuaternion().getW(),
                shooterPose.getRotation().getQuaternion().getX(),
                shooterPose.getRotation().getQuaternion().getY(),
                shooterPose.getRotation().getQuaternion().getZ()
        });
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

    /**
     * Calculates the static friction feedforward.
     * Use this to compensate for the non-linear force required to break static
     * friction.
     * 
     * @param targetRPM The target velocity in RPM.
     * @return The voltage to add to the control output.
     */
    private double calculateStaticFriction(double targetRPM) {
        return Math.signum(targetRPM) * ShooterConstants.kFlywheelS.get();
    }

}
