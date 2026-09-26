package frc.robot.Subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.shooter.ShooterIO;
import frc.robot.Subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.Subsystems.shooter.ShooterIOSim;
import frc.robot.Subsystems.shooter.ShooterIOSparkMax;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Dual-flywheel shooter mechanism with indexer/kicker feed, closed-loop velocity control,
 * distance-to-RPM interpolation lookup tables, predictive pose-based aim targeting,
 * and high-fidelity physics simulation.
 */
public class Shooter implements Subsystem {

    private static Shooter instance = null;

    /**
     * Discrete operational states for shooter state machine.
     */
    public enum ShooterState {
        STOPPED("stop"),
        PREPARING("preparing"),
        SHOOTING("shoot"),
        MANUAL_PREP("manualPrep"),
        MANUAL_FIRE("manualFire"),
        CHARACTERIZATION("characterization");

        private final String stateName;

        ShooterState(String stateName) {
            this.stateName = stateName;
        }

        public String getStateName() {
            return stateName;
        }
    }

    // IO Abstraction (AdvantageKit pattern)
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Feedforward & PID Controllers
    private SimpleMotorFeedforward flywheelFeedForwardLeft;
    private SimpleMotorFeedforward flywheelFeedForwardRight;
    private final PIDController flywheelPidLeft;
    private final PIDController flywheelPidRight;

    // Target RPMs and State
    public double targetRpmLeft = 0;
    public double targetRpmRight = 0;
    private boolean wasAtSpeed = false;
    private ShooterState state = ShooterState.STOPPED;

    // Distance-to-RPM Interpolation Tables
    private final InterpolatingDoubleTreeMap leftRpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rightRpmTable = new InterpolatingDoubleTreeMap();

    // Diagnostics / telemetry
    public double normalDistanceToHub = 0;
    public double leftShooterVoltageCalc = 0;
    public double rightShooterVoltageCalc = 0;
    private ShootingSolution latestShootingSolution = new ShootingSolution(new Rotation2d(), 0, 0, false);

    // Simulation state lives in ShooterIOSim (sole owner of ShooterSim).

    /**
     * Shooting Solution record containing targeting calculations.
     * 
     * @param shootingAngle   Holonomic robot heading angle required to face the goal.
     * @param flywheelRpmLeft Target left flywheel velocity in RPM.
     * @param flywheelRpmRight Target right flywheel velocity in RPM.
     * @param shotPossibility Whether the shot is geometrically reachable from current distance.
     */
    public record ShootingSolution(Rotation2d shootingAngle, double flywheelRpmLeft, double flywheelRpmRight, boolean shotPossibility) {
        public Rotation2d turretAngle() { return shootingAngle; }
        public double flywheelRPM() { return (flywheelRpmLeft + flywheelRpmRight) / 2.0; }
        public boolean possible() { return shotPossibility; }
    }

    /**
     * Gets the singleton instance of Shooter, instantiating the appropriate IO layer.
     */
    public static synchronized Shooter getInstance() {
        if (instance == null) {
            ShooterIO io = RobotBase.isSimulation() ? new ShooterIOSim() : new ShooterIOSparkMax();
            instance = new Shooter(io);
        }
        return instance;
    }

    /**
     * Constructs the Shooter subsystem with the provided IO layer.
     * 
     * @param io Hardware or simulation IO abstraction layer.
     */
    public Shooter(ShooterIO io) {
        this.io = io;

        // Feedforward & PID controllers
        flywheelFeedForwardLeft = new SimpleMotorFeedforward(Constants.FLYWHEEL_KS, Constants.FLYWHEEL_KV, Constants.FLYWHEEL_KA);
        flywheelFeedForwardRight = new SimpleMotorFeedforward(Constants.FLYWHEEL_KS, Constants.FLYWHEEL_KV, Constants.FLYWHEEL_KA);

        flywheelPidLeft = new PIDController(Constants.FLYWHEEL_KP, Constants.FLYWHEEL_KI, Constants.FLYWHEEL_KD);
        flywheelPidRight = new PIDController(Constants.FLYWHEEL_KP, Constants.FLYWHEEL_KI, Constants.FLYWHEEL_KD);

        // Anti-windup clamping on integral term
        flywheelPidLeft.setIntegratorRange(-1.5, 1.5);
        flywheelPidRight.setIntegratorRange(-1.5, 1.5);

        // Hardware-calibrated RPM tables based on distance (meters)
        leftRpmTable.put(1.20, 2400.0);
        leftRpmTable.put(1.92, 2700.0);
        leftRpmTable.put(2.47, 2900.0);
        leftRpmTable.put(3.05, 3500.0);
        leftRpmTable.put(3.48, 3550.0);
        leftRpmTable.put(4.18, 3750.0);
        leftRpmTable.put(5.00, 4100.0);
        leftRpmTable.put(6.00, 4500.0);

        rightRpmTable.put(1.20, 2450.0);
        rightRpmTable.put(1.92, 2750.0);
        rightRpmTable.put(2.47, 2950.0);
        rightRpmTable.put(3.05, 3550.0);
        rightRpmTable.put(3.48, 3600.0);
        rightRpmTable.put(4.18, 3800.0);
        rightRpmTable.put(5.00, 4150.0);
        rightRpmTable.put(6.00, 4550.0);

        initialize();
        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Gets the target alliance goal location translated for current alliance.
     */
    public Translation3d goalLocation() {
        return AllianceFlipUtil.apply(Constants.BLUE_HUB_LOCATION);
    }

    /**
     * Calculates shooting angle and target RPMs based on robot pose.
     * 
     * @param robotPose Current estimated robot pose.
     * @return ShootingSolution with required angle, RPMs, and geometric possibility.
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose) {
        Translation2d goalLoc = goalLocation().toTranslation2d();
        Translation2d robotTranslation = robotPose.getTranslation();

        Translation2d shooterLoc = robotTranslation.plus(
                new Translation2d(Constants.SHOOTER_OFFSET, 0).rotateBy(robotPose.getRotation()));
        Translation2d distanceToHub = goalLoc.minus(shooterLoc);
        normalDistanceToHub = distanceToHub.getNorm();

        double possibilityDeterminator = normalDistanceToHub * Math.tan(Constants.FIRING_ANGLE) - Constants.HEIGHT_DIFFERENCE;
        Rotation2d shootingAngle = distanceToHub.getAngle();

        double rpmLeft = leftRpmTable.get(normalDistanceToHub);
        double rpmRight = rightRpmTable.get(normalDistanceToHub);

        boolean inAllianceZone = AllianceFlipUtil.isPoseInAllianceZone(robotPose);
        boolean possible = inAllianceZone && possibilityDeterminator > 0 && normalDistanceToHub >= 1.2 && normalDistanceToHub <= 6.5;

        Logger.recordOutput("Shooter/InAllianceZone", inAllianceZone);
        SmartDashboard.putBoolean("Shooter/In Alliance Zone", inAllianceZone);

        if (!possible) {
            return new ShootingSolution(shootingAngle, 0, 0, false);
        } else {
            return new ShootingSolution(shootingAngle, rpmLeft, rpmRight, true);
        }
    }

    /**
     * Shooting-on-the-Fly (SOTF) with iterative vector ballistics:
     * Compensates for robot translation velocity by calculating an apparent virtual goal target
     * P_virtual = P_goal - V_chassis * t_tof.
     * Accurately compensates for tangential drift when firing while strafing or sprinting.
     * 
     * @param robotPose Current estimated robot pose.
     * @param robotVel  Current chassis field speeds.
     * @return Motion-compensated ShootingSolution with dynamic lead angle and RPMs.
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel) {
        if (robotVel == null || (Math.abs(robotVel.vxMetersPerSecond) < 0.05 && Math.abs(robotVel.vyMetersPerSecond) < 0.05)) {
            return calculateShootingSolution(robotPose);
        }

        Translation2d goalLoc = goalLocation().toTranslation2d();
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d shooterLoc = robotTranslation.plus(
                new Translation2d(Constants.SHOOTER_OFFSET, 0).rotateBy(robotPose.getRotation()));

        Translation2d vel = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

        // Iterative virtual target solver (2 iterations for sub-millimeter precision)
        double dist = shooterLoc.getDistance(goalLoc);
        normalDistanceToHub = dist;
        double tof = 0.12 + 0.18 * dist; // Empirical time-of-flight curve

        Translation2d virtualGoal = goalLoc.minus(vel.times(tof));
        dist = shooterLoc.getDistance(virtualGoal);
        tof = 0.12 + 0.18 * dist;
        virtualGoal = goalLoc.minus(vel.times(tof));

        Translation2d distanceToVirtualGoal = virtualGoal.minus(shooterLoc);
        double effectiveDist = distanceToVirtualGoal.getNorm();

        double possibility = effectiveDist * Math.tan(Constants.FIRING_ANGLE) - Constants.HEIGHT_DIFFERENCE;
        Rotation2d compensatedAngle = distanceToVirtualGoal.getAngle();

        double rpmLeft = leftRpmTable.get(effectiveDist);
        double rpmRight = rightRpmTable.get(effectiveDist);
        boolean inAllianceZone = AllianceFlipUtil.isPoseInAllianceZone(robotPose);
        boolean possible = inAllianceZone && possibility > 0 && effectiveDist >= 1.2 && effectiveDist <= 6.5;

        Logger.recordOutput("Shooter/InAllianceZone", inAllianceZone);
        SmartDashboard.putBoolean("Shooter/In Alliance Zone", inAllianceZone);
        Logger.recordOutput("Shooter/SOTF/VirtualGoal", virtualGoal);
        Logger.recordOutput("Shooter/SOTF/EffectiveDistance", effectiveDist);
        Logger.recordOutput("Shooter/SOTF/CompensatedAngleDeg", compensatedAngle.getDegrees());

        if (!possible) {
            return new ShootingSolution(compensatedAngle, 0, 0, false);
        } else {
            return new ShootingSolution(compensatedAngle, rpmLeft, rpmRight, true);
        }
    }

    /**
     * Predictive lookahead shooting solution with custom lookahead time.
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel, double lookAheadTime) {
        Pose2d predictedPose = new Pose2d(
                robotPose.getX() + robotVel.vxMetersPerSecond * lookAheadTime,
                robotPose.getY() + robotVel.vyMetersPerSecond * lookAheadTime,
                robotPose.getRotation().plus(Rotation2d.fromRadians(robotVel.omegaRadiansPerSecond * lookAheadTime)));
        return calculateShootingSolution(predictedPose, robotVel);
    }

    /**
     * Sets target RPM setpoints for left and right flywheels.
     */
    public void setTargetRPM(double leftRpm, double rightRpm) {
        if (Math.abs(this.targetRpmLeft) == 0 && Math.abs(leftRpm) > 0) {
            flywheelPidLeft.reset();
        }
        if (Math.abs(this.targetRpmRight) == 0 && Math.abs(rightRpm) > 0) {
            flywheelPidRight.reset();
        }
        this.targetRpmLeft = leftRpm;
        this.targetRpmRight = rightRpm;
    }

    /**
     * Sets symmetric target RPM setpoint for both flywheels.
     */
    public void setTargetRPM(double rpm) {
        setTargetRPM(rpm, rpm);
    }

    /**
     * Sets target flywheel velocity using Java Units {@link AngularVelocity} measures.
     */
    public void setTargetVelocity(AngularVelocity targetLeft, AngularVelocity targetRight) {
        setTargetRPM(targetLeft.in(Units.RPM), targetRight.in(Units.RPM));
    }

    /**
     * Sets symmetric target flywheel velocity using Java Units {@link AngularVelocity} measure.
     */
    public void setTargetVelocity(AngularVelocity target) {
        setTargetVelocity(target, target);
    }

    /**
     * Computes and applies closed-loop feedforward + feedback voltages to flywheels.
     */
    public void updateFlywheelVoltages() {
        // Live Tunable Gains Check
        if (ShooterConstants.FLYWHEEL_KP.hasChanged(hashCode())
                || ShooterConstants.FLYWHEEL_KI.hasChanged(hashCode())
                || ShooterConstants.FLYWHEEL_KD.hasChanged(hashCode())) {
            double p = ShooterConstants.FLYWHEEL_KP.get();
            double i = ShooterConstants.FLYWHEEL_KI.get();
            double d = ShooterConstants.FLYWHEEL_KD.get();
            flywheelPidLeft.setPID(p, i, d);
            flywheelPidRight.setPID(p, i, d);
        }
        if (ShooterConstants.FLYWHEEL_KS.hasChanged(hashCode())
                || ShooterConstants.FLYWHEEL_KV.hasChanged(hashCode())
                || ShooterConstants.FLYWHEEL_KA.hasChanged(hashCode())) {
            double s = ShooterConstants.FLYWHEEL_KS.get();
            double v = ShooterConstants.FLYWHEEL_KV.get();
            double a = ShooterConstants.FLYWHEEL_KA.get();
            flywheelFeedForwardLeft = new SimpleMotorFeedforward(s, v, a);
            flywheelFeedForwardRight = new SimpleMotorFeedforward(s, v, a);
        }

        if (Math.abs(targetRpmLeft) > 0 || Math.abs(targetRpmRight) > 0) {
            leftShooterVoltageCalc = flywheelFeedForwardLeft.calculate(targetRpmLeft)
                    + flywheelPidLeft.calculate(inputs.leftVelocityRPM, targetRpmLeft);
            rightShooterVoltageCalc = flywheelFeedForwardRight.calculate(targetRpmRight)
                    + flywheelPidRight.calculate(inputs.rightVelocityRPM, targetRpmRight);

            io.setFlywheelVoltages(leftShooterVoltageCalc, rightShooterVoltageCalc);
        } else {
            io.setFlywheelVoltages(0, 0);
        }
    }

    /**
     * Legacy alias for {@link #updateFlywheelVoltages()}.
     */
    public void setFlyWheelVelocity() {
        updateFlywheelVoltages();
    }

    /**
     * Checks if both flywheels are within RPM tolerance with hysteresis.
     */
    public boolean isAtCorrectSpeed() {
        if (targetRpmLeft <= 100 || targetRpmRight <= 100) {
            wasAtSpeed = false;
            return false;
        }
        double leftError = Math.abs(inputs.leftVelocityRPM - targetRpmLeft);
        double rightError = Math.abs(inputs.rightVelocityRPM - targetRpmRight);

        if (!wasAtSpeed && leftError < 150 && rightError < 150) {
            wasAtSpeed = true;
        } else if (wasAtSpeed && (leftError > 750 || rightError > 750)) {
            wasAtSpeed = false;
        }
        return wasAtSpeed;
    }

    /**
     * Alias for {@link #isAtCorrectSpeed()}.
     */
    public boolean isAtTargetVelocity() {
        return isAtCorrectSpeed();
    }

    /**
     * Checks if the shooter is spun up and the robot chassis is aligned to target heading.
     */
    public boolean isReadyToFire(Rotation2d targetHeading) {
        double headingError = Math.abs(SwerveBase.getInstance().getHeading().minus(targetHeading).getDegrees());
        return isAtCorrectSpeed() && headingError < Constants.ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG;
    }

    /**
     * Checks if Limelight vision has target centered within tolerance.
     */
    public boolean isLinedUp() {
        Vision vision = Vision.getInstance();
        return vision.hasTarget() && Math.abs(vision.getTX()) < Constants.ShooterConstants.LIMELIGHT_TX_TOLERANCE_DEG;
    }

    public double getTargetVelocityRPM() {
        return (targetRpmLeft + targetRpmRight) / 2.0;
    }

    public double getSpeed() {
        return inputs.leftVelocityRPM;
    }

    public double getActualRPM() {
        return (inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0;
    }

    public double getFlywheelLeftVelocityRPM() {
        return inputs.leftVelocityRPM;
    }

    public double getFlywheelRightVelocityRPM() {
        return inputs.rightVelocityRPM;
    }

    /**
     * Gets target flywheel velocity as an {@link AngularVelocity} measure.
     */
    public AngularVelocity getTargetVelocityMeasure() {
        return Units.RPM.of(getTargetVelocityRPM());
    }

    /**
     * Gets left flywheel velocity as an {@link AngularVelocity} measure.
     */
    public AngularVelocity getLeftFlywheelVelocityMeasure() {
        return Units.RPM.of(getFlywheelLeftVelocityRPM());
    }

    /**
     * Gets right flywheel velocity as an {@link AngularVelocity} measure.
     */
    public AngularVelocity getRightFlywheelVelocityMeasure() {
        return Units.RPM.of(getFlywheelRightVelocityRPM());
    }

    /**
     * Gets average actual flywheel velocity as an {@link AngularVelocity} measure.
     */
    public AngularVelocity getActualVelocityMeasure() {
        return Units.RPM.of(getActualRPM());
    }

    /**
     * Gets left flywheel motor electrical current draw as a {@link Current} measure.
     */
    public Current getLeftCurrentMeasure() {
        return Units.Amps.of(inputs.leftCurrentAmps);
    }

    /**
     * Gets right flywheel motor electrical current draw as a {@link Current} measure.
     */
    public Current getRightCurrentMeasure() {
        return Units.Amps.of(inputs.rightCurrentAmps);
    }

    public ShooterIO getIO() {
        return io;
    }

    public ShooterIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void manualFire(double triggerValue) {
        state = ShooterState.MANUAL_FIRE;
        double manualTarget = SmartDashboard.getNumber("Shooter/Manual RPM Setpoint", 3000.0);
        setTargetRPM(manualTarget, manualTarget);
    }

    public void manualPrep() {
        state = ShooterState.MANUAL_PREP;
    }

    public void stop() {
        state = ShooterState.STOPPED;
        targetRpmLeft = 0;
        targetRpmRight = 0;
        io.stop();
    }

    /**
     * WPILib Commands v2 Subsystem.idle():
     * Returns a command that stops flywheels and feed mechanisms, holding the shooter
     * in a safe STOPPED standby state.
     */
    @Override
    public Command idle() {
        return Commands.run(this::stop, this)
                .withName("Shooter.idle");
    }

    public void shoot() {
        state = ShooterState.SHOOTING;
    }

    public void prepareToShoot() {
        state = ShooterState.PREPARING;
    }

    public String getState() {
        return state.getStateName();
    }

    public String getShooterState() {
        return state.getStateName();
    }

    public ShooterState getStateEnum() {
        return state;
    }

    public boolean isShooting() {
        return state == ShooterState.SHOOTING || state == ShooterState.MANUAL_FIRE;
    }

    /**
     * High-speed state machine processing executed at 50 Hz.
     */
    public void processShooterState() {
        switch (state) {
            case PREPARING:
                updateFlywheelVoltages();
                io.setKickerVoltage(0);
                break;

            case MANUAL_PREP:
                updateFlywheelVoltages();
                if (isAtCorrectSpeed() || targetRpmLeft < 0 || targetRpmRight < 0) {
                    io.setKickerVoltage(Constants.KICKER_VOLTAGE);
                } else {
                    io.setKickerVoltage(0);
                }
                break;

            case MANUAL_FIRE:
            case SHOOTING:
                updateFlywheelVoltages();
                if (isAtCorrectSpeed()) {
                    io.setKickerVoltage(Constants.KICKER_VOLTAGE);
                } else {
                    io.setKickerVoltage(0);
                }
                break;

            case CHARACTERIZATION:
                // Preserve voltages commanded by SysId or diagnostic testing
                break;

            case STOPPED:
            default:
                stop();
                break;
        }
    }

    /**
     * Legacy alias for {@link #processShooterState()}.
     */
    public void ShooterStateProcessing() {
        processShooterState();
    }

    @Override
    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        processShooterState();
        latestShootingSolution = calculateShootingSolution(
                SwerveBase.getInstance().getPose(),
                SwerveBase.getInstance().getFieldVelocity()
        );
    }

    /**
     * Gets the latest cached shooting solution computed during the current control loop.
     */
    public ShootingSolution getLatestShootingSolution() {
        return latestShootingSolution;
    }

    @Override
    public void simulationUpdate() {
        // No-op: ShooterIOSim owns ShooterSim ball simulation (updated in updateInputs).
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Shooter/Manual RPM Setpoint", 3000.0);
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Shooter/Shooter Left Motor Speed", inputs.leftVelocityRPM);
        SmartDashboard.putNumber("Shooter/Shooter Right Motor Speed", inputs.rightVelocityRPM);
        SmartDashboard.putNumber("Shooter/Shooter Target RPM Left", targetRpmLeft);
        SmartDashboard.putNumber("Shooter/Shooter Target RPM Right", targetRpmRight);
        SmartDashboard.putNumber("Shooter/Kicker Motor Speed", inputs.kickerVelocityRPM);
        SmartDashboard.putString("Shooter/Shooter State", state.getStateName());
        SmartDashboard.putBoolean("Shooter/Shooter At Target Speed", isAtCorrectSpeed());
        SmartDashboard.putNumber("Shooter/distance to Shooter", normalDistanceToHub);
        SmartDashboard.putNumber("Shooter/Left Motor voltage calc", leftShooterVoltageCalc);

        // 3D Visualizer for AdvantageScope / Elastic
        Translation3d shooterRoot = new Translation3d(Constants.SHOOTER_OFFSET, 0, 0.53);
        Rotation3d shooterRot = new Rotation3d(0, -Constants.FIRING_ANGLE, 0);
        Pose3d shooterPose = new Pose3d(shooterRoot, shooterRot);

        SmartDashboard.putNumberArray("Subsystems/Shooter/ShooterPose3d", new double[] {
                shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
                shooterPose.getRotation().getQuaternion().getW(),
                shooterPose.getRotation().getQuaternion().getX(),
                shooterPose.getRotation().getQuaternion().getY(),
                shooterPose.getRotation().getQuaternion().getZ()
        });
        org.littletonrobotics.junction.Logger.recordOutput("Subsystems/Shooter/ShooterPose3d", shooterPose);
    }

    public void setVoltages(double flywheelVolts, double kickerVolts) {
        state = ShooterState.CHARACTERIZATION;
        setFlywheelVoltages(flywheelVolts, flywheelVolts);
        io.setKickerVoltage(kickerVolts);
    }

    public void setFlywheelVoltages(double leftVolts, double rightVolts) {
        state = ShooterState.CHARACTERIZATION;
        io.setFlywheelVoltages(leftVolts, rightVolts);
    }

    public void setFlywheelCharacterizationVoltage(double leftVolts, double rightVolts) {
        setFlywheelVoltages(leftVolts, rightVolts);
    }

    public double getFlywheelLeftAppliedVoltage() {
        return inputs.leftAppliedVolts;
    }

    public double getFlywheelRightAppliedVoltage() {
        return inputs.rightAppliedVolts;
    }

    public double getKickerAppliedVoltage() {
        return inputs.kickerAppliedVolts;
    }

    public double getKickerVelocityRPM() {
        return inputs.kickerVelocityRPM;
    }

    public void setKickerSpeed(double speed) {
        io.setKickerVoltage(speed * 12.0);
    }

    public long getSimShotCount() {
        if (io instanceof ShooterIOSim simIO) {
            return simIO.getShooterSim().getSimShotCount();
        }
        return 0;
    }

    public long getSimScoreCount() {
        if (io instanceof ShooterIOSim simIO) {
            return simIO.getShooterSim().getSimScoreCount();
        }
        return 0;
    }

    @Override
    public double getSimulationCurrentDraw() {
        if (io instanceof ShooterIOSim simIO) {
            return simIO.getShooterSim().getTotalCurrentDraw(inputs.kickerAppliedVolts / 12.0);
        }
        return 0.0;
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
