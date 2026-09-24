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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.ShooterConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.shooter.ShooterIO;
import frc.robot.Subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.Subsystems.shooter.ShooterIOSim;
import frc.robot.Subsystems.shooter.ShooterIOSparkMax;
import frc.robot.Utils.AllianceFlipUtil;

/*
 * Class: Shooter
 * Description: Dual-flywheel shooter with kicker feed, distance-to-RPM interpolation tables,
 *              closed-loop velocity control, and comprehensive physics simulation.
 * Authors: Sarah, Trevor, InfiniteQuery
 */
public class Shooter implements Subsystem {

    private static Shooter instance = null;

    // IO Abstraction (AdvantageKit pattern)
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // Feedforward & PID Controllers
    private final SimpleMotorFeedforward flyWheelFeedForwardLeft;
    private final SimpleMotorFeedforward flyWheelFeedForwardRight;
    private final PIDController flyWheelPIDLeft;
    private final PIDController flyWheelPIDRight;

    // Target RPMs
    public double targetRpmLeft = 0;
    public double targetRpmRight = 0;
    private boolean wasAtSpeed = false;
    private String state = "stop";

    // Distance-to-RPM Interpolation Tables
    private final InterpolatingDoubleTreeMap leftRpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rightRpmTable = new InterpolatingDoubleTreeMap();

    // Diagnostics / telemetry
    public double normalDistanceToHub = 0;
    public double leftShooterVoltageCalc = 0;
    public double rightShooterVoltageCalc = 0;
    private ShootingSolution latestShootingSolution = new ShootingSolution(new Rotation2d(), 0, 0, false);

    // Simulation
    private frc.robot.Sim.ShooterSim flywheelSim;

    /**
     * Shooting Solution record. Provides getters for both student parity and mentor compatibility.
     */
    public record ShootingSolution(Rotation2d shootingAngle, double flywheelRpmLeft, double flywheelRpmRight, boolean shotPossibility) {
        public Rotation2d turretAngle() { return shootingAngle; }
        public double flywheelRPM() { return (flywheelRpmLeft + flywheelRpmRight) / 2.0; }
        public boolean possible() { return shotPossibility; }
    }

    public static Shooter getInstance() {
        if (instance == null) {
            ShooterIO io = RobotBase.isSimulation() ? new ShooterIOSim() : new ShooterIOSparkMax();
            instance = new Shooter(io);
        }
        return instance;
    }

    public Shooter(ShooterIO io) {
        this.io = io;

        // Feedforward & PID controllers
        flyWheelFeedForwardLeft = new SimpleMotorFeedforward(Constants.kFLYWHEELs, Constants.kFLYWHEELv, Constants.kFLYWHEELa);
        flyWheelFeedForwardRight = new SimpleMotorFeedforward(Constants.kFLYWHEELs, Constants.kFLYWHEELv, Constants.kFLYWHEELa);

        flyWheelPIDLeft = new PIDController(Constants.kFLYWHEELp, Constants.kFLYWHEELi, Constants.kFLYWHEELd);
        flyWheelPIDRight = new PIDController(Constants.kFLYWHEELp, Constants.kFLYWHEELi, Constants.kFLYWHEELd);

        // Anti-windup clamping on integral term
        flyWheelPIDLeft.setIntegratorRange(-1.5, 1.5);
        flyWheelPIDRight.setIntegratorRange(-1.5, 1.5);

        // Hardware-calibrated RPM tables based on distance (meters)
        leftRpmTable.put(1.92, 2700.0);
        leftRpmTable.put(2.47, 2900.0);
        leftRpmTable.put(3.05, 3500.0);
        leftRpmTable.put(3.48, 3550.0);
        leftRpmTable.put(4.18, 3750.0);

        rightRpmTable.put(1.92, 2750.0);
        rightRpmTable.put(2.47, 2950.0);
        rightRpmTable.put(3.05, 3550.0);
        rightRpmTable.put(3.48, 3600.0);
        rightRpmTable.put(4.18, 3800.0);

        if (RobotBase.isSimulation() && io instanceof ShooterIOSim simIO) {
            flywheelSim = simIO.getShooterSim();
        } else if (RobotBase.isSimulation()) {
            flywheelSim = new frc.robot.Sim.ShooterSim();
        }

        initialize();
        SubsystemManager.registerSubsystem(this);
    }

    public Translation3d goalLocation() {
        return AllianceFlipUtil.apply(Constants.BLUE_HUB_LOCATION);
    }

    /**
     * Calculates shooting angle and target RPMs based on robot pose.
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

        if (possibilityDeterminator <= 0) {
            return new ShootingSolution(new Rotation2d(), 0, 0, false);
        } else {
            return new ShootingSolution(shootingAngle, rpmLeft, rpmRight, true);
        }
    }

    /**
     * Predictive lookahead shooting solution for shooting on the move.
     */
    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel) {
        return calculateShootingSolution(robotPose, robotVel, Constants.SHOOTER_PREDICTIVE_LOOK_AHEAD);
    }

    public ShootingSolution calculateShootingSolution(Pose2d robotPose, ChassisSpeeds robotVel, double lookAheadTime) {
        Pose2d predictedPose = new Pose2d(
                robotPose.getX() + robotVel.vxMetersPerSecond * lookAheadTime,
                robotPose.getY() + robotVel.vyMetersPerSecond * lookAheadTime,
                robotPose.getRotation().plus(Rotation2d.fromRadians(robotVel.omegaRadiansPerSecond * lookAheadTime)));
        return calculateShootingSolution(predictedPose);
    }

    public void setTargetRPM(double leftRpm, double rightRpm) {
        if (Math.abs(this.targetRpmLeft) == 0 && Math.abs(leftRpm) > 0) {
            flyWheelPIDLeft.reset();
        }
        if (Math.abs(this.targetRpmRight) == 0 && Math.abs(rightRpm) > 0) {
            flyWheelPIDRight.reset();
        }
        this.targetRpmLeft = leftRpm;
        this.targetRpmRight = rightRpm;
    }

    public void setTargetRPM(double rpm) {
        setTargetRPM(rpm, rpm);
    }

    public void setFlywheelVelocity(double rpm) {
        setTargetRPM(rpm, rpm);
    }

    public void setFlyWheelVelocity() {
        if (Math.abs(targetRpmLeft) > 0 || Math.abs(targetRpmRight) > 0) {
            leftShooterVoltageCalc = flyWheelFeedForwardLeft.calculate(targetRpmLeft)
                    + flyWheelPIDLeft.calculate(inputs.leftVelocityRPM, targetRpmLeft);
            rightShooterVoltageCalc = flyWheelFeedForwardRight.calculate(targetRpmRight)
                    + flyWheelPIDRight.calculate(inputs.rightVelocityRPM, targetRpmRight);

            io.setFlywheelVoltages(leftShooterVoltageCalc, rightShooterVoltageCalc);
        } else {
            io.setFlywheelVoltages(0, 0);
        }
    }

    public boolean isAtCorrectSpeed() {
        double leftError = Math.abs(inputs.leftVelocityRPM - targetRpmLeft);
        double rightError = Math.abs(inputs.rightVelocityRPM - targetRpmRight);

        if (!wasAtSpeed && leftError < 150 && rightError < 150) {
            wasAtSpeed = true;
        } else if (wasAtSpeed && (leftError > 750 || rightError > 750)) {
            wasAtSpeed = false;
        }
        return wasAtSpeed;
    }

    public boolean isAtTargetVelocity() {
        return isAtCorrectSpeed();
    }

    public boolean isReadyToFire(Rotation2d targetHeading) {
        double headingError = Math.abs(SwerveBase.getInstance().getHeading().minus(targetHeading).getDegrees());
        return isAtCorrectSpeed() && headingError < Constants.ShooterConstants.ALIGNMENT_HEADING_TOLERANCE_DEG;
    }

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

    public ShooterIO getIO() {
        return io;
    }

    public ShooterIOInputs getInputs() {
        return inputs;
    }

    public void manualFire(double triggerValue) {
        state = "manualFire";
        double manualTarget = SmartDashboard.getNumber("Shooter/Manual RPM Setpoint", 3000.0);
        setTargetRPM(manualTarget, manualTarget);
    }

    public void manualPrep() {
        state = "manualPrep";
    }

    public void stop() {
        state = "stop";
        targetRpmLeft = 0;
        targetRpmRight = 0;
        io.stop();
    }

    public void shoot() {
        state = "shoot";
    }

    public void prepareToShoot() {
        state = "preparing";
    }

    public void ShooterStateProcessing() {
        switch (state) {
            case "preparing":
                setFlyWheelVelocity();
                io.setKickerVoltage(0);
                break;

            case "manualPrep":
                setFlyWheelVelocity();
                if (isAtCorrectSpeed() || targetRpmLeft < 0 || targetRpmRight < 0) {
                    io.setKickerVoltage(Constants.KICKERMOTOR);
                } else {
                    io.setKickerVoltage(0);
                }
                break;

            case "manualFire":
            case "shoot":
                setFlyWheelVelocity();
                if (isAtCorrectSpeed()) {
                    io.setKickerVoltage(Constants.KICKERMOTOR);
                } else {
                    io.setKickerVoltage(0);
                }
                break;

            case "characterization":
                // In characterization/test mode, voltages are controlled directly by SysId/Diagnostics
                break;

            case "stop":
            default:
                stop();
                break;
        }
    }

    @Override
    public void update() {
        io.updateInputs(inputs);
        ShooterStateProcessing();
        latestShootingSolution = calculateShootingSolution(
                SwerveBase.getInstance().getPose(),
                SwerveBase.getInstance().getFieldVelocity()
        );
    }

    public ShootingSolution getLatestShootingSolution() {
        return latestShootingSolution;
    }

    @Override
    public void simulationUpdate() {
        if (flywheelSim != null) {
            double avgVoltage = (leftShooterVoltageCalc + rightShooterVoltageCalc) / 2.0;
            if (targetRpmLeft == 0 && targetRpmRight == 0) avgVoltage = 0;

            flywheelSim.updateBallSimulation(
                    inputs.kickerAppliedVolts / 12.0,
                    inputs.leftVelocityRPM,
                    (targetRpmLeft + targetRpmRight) / 2.0,
                    avgVoltage
            );
        }
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
        SmartDashboard.putString("Shooter/Shooter State", state);
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
        state = "characterization";
        leftShooterVoltageCalc = flywheelVolts;
        rightShooterVoltageCalc = flywheelVolts;
        setFlywheelVoltages(flywheelVolts, flywheelVolts);
        io.setKickerVoltage(kickerVolts);
    }

    public void setFlywheelVoltages(double leftVolts, double rightVolts) {
        state = "characterization";
        leftShooterVoltageCalc = leftVolts;
        rightShooterVoltageCalc = rightVolts;
        io.setFlywheelVoltages(leftVolts, rightVolts);
    }

    public void setFlywheelCharacterizationVoltage(double leftVolts, double rightVolts) {
        setFlywheelVoltages(leftVolts, rightVolts);
    }

    public double getTargetRPMLeft() {
        return targetRpmLeft;
    }

    public double getTargetRPMRight() {
        return targetRpmRight;
    }

    public double getTargetRPM() {
        return (targetRpmLeft + targetRpmRight) / 2.0;
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
        return flywheelSim != null ? flywheelSim.getSimShotCount() : 0;
    }

    public long getSimScoreCount() {
        return flywheelSim != null ? flywheelSim.getSimScoreCount() : 0;
    }

    @Override
    public double getSimulationCurrentDraw() {
        return flywheelSim != null ? flywheelSim.getTotalCurrentDraw(inputs.kickerAppliedVolts / 12.0) : 0.0;
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
