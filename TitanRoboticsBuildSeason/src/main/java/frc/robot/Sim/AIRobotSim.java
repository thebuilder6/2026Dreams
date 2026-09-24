package frc.robot.Sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Auto.StaticPathfinder;
import frc.robot.Auto.TrajectoryController;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.AutonConstants;
import frc.robot.Data.FieldMap;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class AIRobotSim implements Subsystem {

    public enum AIMode {
        TACTICAL_DEFENSE("Tactical Defense (Jev AI)"),
        LEAD_PURSUIT_INTERCEPT("Lead Pursuit Intercept"),
        PINNING_BULLY("Aggressive Pinning Bully"),
        AUTONOMOUS_CYCLER("Autonomous Fuel Cycler"),
        CHOREO_PATH("Choreo Path Following"),
        MANUAL_2_PLAYER("Manual 2-Player (Port 2)");

        public final String displayName;

        AIMode(String displayName) {
            this.displayName = displayName;
        }

        public static AIMode fromString(String name) {
            if (name == null) return AUTONOMOUS_CYCLER;
            for (AIMode m : values()) {
                if (m.name().equalsIgnoreCase(name) || m.displayName.equalsIgnoreCase(name)) {
                    return m;
                }
            }
            return AUTONOMOUS_CYCLER;
        }
    }

    public enum CyclerPhase {
        HUNT_FUEL,
        SCORE_HUB,
        SHOOTING
    }

    private static AIRobotSim instance;

    public static final Pose2d[] ROBOT_QUEUING_POSITIONS = new Pose2d[] {
            new Pose2d(1.0, -5, new Rotation2d()),
            new Pose2d(1.5, -5, new Rotation2d()),
            new Pose2d(2.0, -5, new Rotation2d())
    };

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queuingPose;
    private IntakeSimulation intakeSimulation;

    private Optional<Trajectory<SwerveSample>> trajectory = Optional.empty();
    private final Timer pathTimer = new Timer();
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final TrajectoryController aiTrajectoryController;

    private final XboxController defenseController;

    private boolean wasOpponentEnabled = false;
    private boolean lastSpawnedPlayerIsRed = false;
    private ChassisSpeeds currentTargetSpeeds = new ChassisSpeeds();
    private ChassisSpeeds lastRobotRelativeSpeeds = new ChassisSpeeds();
    private Pose2d currentTargetPose = new Pose2d();
    private String currentAIStateDetail = "IDLE";
    private int aiScoreCount = 0;
    private double lastShotTimestamp = 0.0;
    private final SendableChooser<AIMode> aiModeChooser = new SendableChooser<>();

    private CyclerPhase cyclerPhase = CyclerPhase.HUNT_FUEL;
    private final Timer cyclerTimer = new Timer();

    private Pose2d lastActualPose = new Pose2d();
    private double lastPoseTimestamp = -1.0;
    private double stallDuration = 0.0;
    private double lastCommandedSpeed = 0.0;
    private boolean lastStallResult = false;
    private double lastStallEvalTimestamp = -1.0;

    // Immutable latched target during transit/staging
    private Pose2d latchedShootTarget = null;

    // Periodic diagnostic console printer
    private double lastConsoleDumpTime = 0.0;

    public static AIRobotSim getInstance() {
        if (instance == null) {
            instance = new AIRobotSim();
        }
        return instance;
    }

    private AIRobotSim() {
        this.queuingPose = ROBOT_QUEUING_POSITIONS[0];

        this.driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default(),
                        queuingPose));
        try {
            this.driveSimulation.getDriveTrainSimulation().getGyroSimulation().setRotation(queuingPose.getRotation());
            this.driveSimulation.resetOdometry(queuingPose);
        } catch (Exception ignored) {}

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

        try {
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                    "Fuel",
                    driveSimulation.getDriveTrainSimulation(),
                    Meters.of(0.70),
                    Meters.of(0.30),
                    IntakeSimulation.IntakeSide.FRONT,
                    Constants.IntakeConstants.MAX_HELD_BALLS 
            );
        } catch (Exception e) {
            System.err.println("[AIRobotSim] Could not attach IntakeSimulation: " + e.getMessage());
        }

        this.xController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI, AutonConstants.AUTO_DRIVE_KD);
        this.yController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI, AutonConstants.AUTO_DRIVE_KD);

        var config = SwerveBase.getInstance().getSwerveController().config;
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.aiTrajectoryController = new TrajectoryController(
                new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d));

        this.defenseController = new XboxController(2);

        SubsystemManager.registerSubsystem(this);
    }

    public void setTrajectory(String pathName) {
        this.trajectory = Choreo.loadTrajectory(pathName);
        pathTimer.restart();
    }

    public void setRobotPose(Pose2d pose) {
        driveSimulation.setSimulationWorldPose(pose);
        try {
            driveSimulation.getDriveTrainSimulation().getGyroSimulation().setRotation(pose.getRotation());
            driveSimulation.resetOdometry(pose);
        } catch (Exception ignored) {}
    }

    public void reset() {
        wasOpponentEnabled = false;
        lastSpawnedPlayerIsRed = false;
        pathTimer.restart();
        cyclerTimer.restart();
        cyclerPhase = CyclerPhase.HUNT_FUEL;
        aiScoreCount = 0;
        aiTrajectoryController.reset();
        lastPoseTimestamp = -1.0;
        stallDuration = 0.0;
        lastCommandedSpeed = 0.0;
        lastShotTimestamp = 0.0;
        setRobotPose(queuingPose);
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(0);
            intakeSimulation.stopIntake();
        }
        latchedShootTarget = null;
    }

    @Override
    public void simulationUpdate() {
        boolean opponentEnabled = Dashboard.isOpponentRobotEnabled();
        boolean manualDefenseMode = Dashboard.is2PlayerDefenseEnabled();

        if (!opponentEnabled) {
            setRobotPose(queuingPose);
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, true);
            wasOpponentEnabled = false;
            currentAIStateDetail = "DISABLED";
            if (intakeSimulation != null && intakeSimulation.isRunning()) {
                intakeSimulation.stopIntake();
            }
            return;
        }

        AIMode activeMode;
        if (manualDefenseMode) {
            activeMode = AIMode.MANUAL_2_PLAYER;
        } else if (aiModeChooser != null && aiModeChooser.getSelected() != null) {
            activeMode = aiModeChooser.getSelected();
        } else {
            String modeStr = SmartDashboard.getString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name());
            activeMode = AIMode.fromString(modeStr);
        }

        boolean isRedAlliance = AllianceFlipUtil.isRedAlliance();

        if (!wasOpponentEnabled || isRedAlliance != lastSpawnedPlayerIsRed) {
            Pose2d initialPose = getOpponentSpawnPose(isRedAlliance, activeMode);
            setRobotPose(initialPose);
            pathTimer.restart();
            cyclerTimer.restart();
            wasOpponentEnabled = true;
            lastSpawnedPlayerIsRed = isRedAlliance;
            System.out.printf("[AIRobotSim] Spawned at (%.2f, %.2f, %.1f deg), PlayerIsRed: %b%n",
                    initialPose.getX(), initialPose.getY(), initialPose.getRotation().getDegrees(), isRedAlliance);
        }

        try {
            driveSimulation.periodic();
        } catch (Exception ignored) {}

        double speedPercent = SmartDashboard.getNumber("Simulation/OpponentSpeedPercent", 75.0);
        double speedScale = Math.max(0.20, Math.min(1.0, speedPercent / 100.0));
        double maxSpeed = Constants.MAX_SPEED * speedScale;

        Pose2d playerPose = SwerveBase.getInstance().getPose();
        Pose2d currentPose = driveSimulation.getActualPoseInSimulationWorld();
        double matchTime = Timer.getMatchTime();
        if (matchTime < 0) matchTime = 150.0;
        boolean isHubActive = Dashboard.getInstance().isHubActive();

        ChassisSpeeds targetSpeeds;
        Pose2d targetPose = currentPose;

        switch (activeMode) {
            case MANUAL_2_PLAYER:
                double x = 0.0;
                double y = 0.0;
                double rot = 0.0;
                if (DriverStation.isJoystickConnected(2)) {
                    x = -defenseController.getLeftY();
                    y = -defenseController.getLeftX();
                    rot = -defenseController.getRightX();

                    x = Math.abs(x) < 0.1 ? 0 : x;
                    y = Math.abs(y) < 0.1 ? 0 : y;
                    rot = Math.abs(rot) < 0.1 ? 0 : rot;
                }
                targetSpeeds = new ChassisSpeeds(x * maxSpeed, y * maxSpeed, rot * 5.0);
                targetPose = currentPose.plus(new edu.wpi.first.math.geometry.Transform2d(x, y, new Rotation2d(rot)));
                currentAIStateDetail = "MANUAL_2_PLAYER";
                break;

            case LEAD_PURSUIT_INTERCEPT:
                ChassisSpeeds playerSpeeds = SwerveBase.getInstance().getFieldVelocity();
                Translation2d playerVel = new Translation2d(playerSpeeds.vxMetersPerSecond, playerSpeeds.vyMetersPerSecond);
                targetPose = JevDecisionEngine.getInstance().solveLeadPursuitIntercept(
                        currentPose, playerPose, playerVel, maxSpeed);
                targetSpeeds = computeDriveToPoseSpeeds(currentPose, targetPose, maxSpeed);
                currentAIStateDetail = String.format("INTERCEPTING @ (%.1f, %.1f)", targetPose.getX(), targetPose.getY());
                break;

            case PINNING_BULLY:
                Rotation2d angleToPlayer = playerPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
                targetPose = new Pose2d(playerPose.getTranslation(), angleToPlayer);
                targetSpeeds = computeDriveToPoseSpeeds(currentPose, targetPose, maxSpeed);
                currentAIStateDetail = "PINNING_BULLY";
                break;

            case AUTONOMOUS_CYCLER:
                targetSpeeds = updateAutonomousCycler(currentPose, isRedAlliance, maxSpeed);
                targetPose = currentTargetPose;
                break;

            case CHOREO_PATH:
                if (trajectory.isPresent()) {
                    double time = pathTimer.get();
                    if (time > trajectory.get().getTotalTime()) {
                        pathTimer.restart();
                        time = 0;
                    }
                    Optional<SwerveSample> sampleOpt = trajectory.get().sampleAt(time, false);
                    if (sampleOpt.isPresent()) {
                        SwerveSample sample = sampleOpt.get();
                        Pose2d trajPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
                        targetPose = mirrorPoseForOpponent(trajPose, isRedAlliance);

                        targetSpeeds = new ChassisSpeeds(
                                sample.vx + xController.calculate(currentPose.getX(), targetPose.getX()),
                                sample.vy + yController.calculate(currentPose.getY(), targetPose.getY()),
                                sample.omega + headingController.calculate(currentPose.getRotation().getRadians(),
                                        targetPose.getRotation().getRadians()));
                        currentAIStateDetail = String.format("CHOREO (t=%.1fs)", time);
                    } else {
                        targetSpeeds = new ChassisSpeeds();
                        currentAIStateDetail = "CHOREO_EMPTY";
                    }
                } else {
                    targetSpeeds = new ChassisSpeeds();
                    currentAIStateDetail = "NO_CHOREO_TRAJ";
                }
                break;

            case TACTICAL_DEFENSE:
            default:
                JevDecisionEngine.DecisionResult decision = JevDecisionEngine.getInstance().evaluate(
                        playerPose, currentPose, matchTime, isHubActive, isRedAlliance);
                targetPose = decision.targetPose;
                targetSpeeds = computeDriveToPoseSpeeds(currentPose, targetPose, maxSpeed);
                currentAIStateDetail = decision.action.name();
                break;
        }

        if (activeMode != AIMode.AUTONOMOUS_CYCLER && intakeSimulation != null) {
            boolean nearFuel = isNearAnyFuel(currentPose.getTranslation(), 0.85);
            if (nearFuel) {
                if (!intakeSimulation.isRunning()) intakeSimulation.startIntake();
                checkProximityPickup(currentPose);
            } else if (intakeSimulation.isRunning() && activeMode != AIMode.MANUAL_2_PLAYER) {
                intakeSimulation.stopIntake();
            }

            boolean opponentIsRed = !isRedAlliance;
            Translation2d oppHub = opponentIsRed ? Constants.RED_HUB_LOCATION.toTranslation2d() : Constants.BLUE_HUB_LOCATION.toTranslation2d();
            if (canShootNow(currentPose, opponentIsRed)) {
                launchOpponentShot(currentPose, oppHub, opponentIsRed);
                intakeSimulation.obtainGamePieceFromIntake();
                lastShotTimestamp = Timer.getFPGATimestamp();
            }
        }

        currentTargetPose = targetPose;
        currentTargetSpeeds = targetSpeeds;
        lastRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation());
        driveSimulation.runChassisSpeeds(lastRobotRelativeSpeeds, new Translation2d(), false, true);
    }

    public ChassisSpeeds computeDriveToPoseSpeeds(Pose2d currentPose, Pose2d targetPose, double maxSpeed) {
        boolean isStalled = isStalled();
        return aiTrajectoryController.calculate(
                currentPose,
                currentTargetSpeeds,
                targetPose,
                maxSpeed,
                isStalled,
                true);
    }

    public boolean isStalled() {
        double now = Timer.getFPGATimestamp();
        if (now - lastStallEvalTimestamp < 0.015 && lastStallEvalTimestamp > 0.0) {
            return lastStallResult;
        }

        double dt = lastStallEvalTimestamp > 0.0 ? (now - lastStallEvalTimestamp) : 0.02;
        lastStallEvalTimestamp = now;

        Pose2d currentWorldPose = driveSimulation.getActualPoseInSimulationWorld();
        double actualMoveDist = currentWorldPose.getTranslation().getDistance(lastActualPose.getTranslation());
        double actualSpeed = actualMoveDist / dt;
        double commandedSpeed = Math.hypot(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);

        if (commandedSpeed > 0.80 && actualSpeed < 0.15) {
            stallDuration += dt;
        } else {
            stallDuration = Math.max(0.0, stallDuration - dt * 2.0);
        }

        lastActualPose = currentWorldPose;
        lastStallResult = (stallDuration > 0.25);
        return lastStallResult;
    }

    private ChassisSpeeds updateAutonomousCycler(Pose2d currentPose, boolean playerIsRed, double maxSpeed) {
        boolean opponentIsRed = !playerIsRed;
        Translation2d opponentHub = opponentIsRed ? Constants.RED_HUB_LOCATION.toTranslation2d() : Constants.BLUE_HUB_LOCATION.toTranslation2d();

        int heldPieces = (intakeSimulation != null) ? intakeSimulation.getGamePiecesAmount() : 0;
        boolean hubActive = isOpponentHubActive(opponentIsRed);

        switch (cyclerPhase) {
            case HUNT_FUEL:
                Pose2d huntTargetPose = findBestFuelTarget(currentPose, opponentIsRed);
                currentTargetPose = huntTargetPose;

                if (intakeSimulation != null && !intakeSimulation.isRunning()) {
                    intakeSimulation.startIntake();
                }
                checkProximityPickup(currentPose);

                int targetCapacity = hubActive ? 16 : Constants.IntakeConstants.MAX_HELD_BALLS;
                boolean hopperFull = heldPieces >= targetCapacity;
                boolean huntTimedOut = (heldPieces >= 6 && cyclerTimer.get() > 6.5) || cyclerTimer.get() > 12.0;

                 if (hopperFull || huntTimedOut) {
                    if (hubActive && heldPieces >= 4) {
                        cyclerPhase = CyclerPhase.SCORE_HUB;
                        cyclerTimer.restart();
                        latchedShootTarget = null; // Clear so SCORE_HUB computes fresh target
                        if (intakeSimulation != null) intakeSimulation.stopIntake();
                    } else if (heldPieces >= 10 || cyclerTimer.get() > 12.0) {
                        // Hub inactive: compute staging target ONCE and latch it
                        if (latchedShootTarget == null) {
                            latchedShootTarget = getOptimalShootingPose(currentPose, opponentIsRed);
                        }
                        currentTargetPose = latchedShootTarget;
                        if (intakeSimulation != null) intakeSimulation.stopIntake();
                        currentAIStateDetail = String.format("CYCLER_STAGING (%d/30, Hub Inactive)", heldPieces);
                        return computeDriveToPoseSpeeds(currentPose, latchedShootTarget, maxSpeed);
                    }
                }

                currentAIStateDetail = String.format("CYCLER_HUNTING (%d/%d, Hub: %s)", 
                        heldPieces, targetCapacity, hubActive ? "ACT" : "INACT");
                return computeDriveToPoseSpeeds(currentPose, huntTargetPose, maxSpeed);

            case SCORE_HUB:
                // Latch shooting target once on entry so it never jumps during transit
                if (latchedShootTarget == null) {
                    latchedShootTarget = getOptimalShootingPose(currentPose, opponentIsRed);
                }
                currentTargetPose = latchedShootTarget;

                if (intakeSimulation != null && intakeSimulation.isRunning()) {
                    intakeSimulation.stopIntake();
                }

                boolean inShootingZone = isValidShootingLocation(currentPose, opponentIsRed);
                boolean laneClear = !isShootingLaneBlocked(currentPose, opponentHub);
                double distToTarget = currentPose.getTranslation().getDistance(latchedShootTarget.getTranslation());

                if ((inShootingZone && laneClear && hubActive) || distToTarget < 0.65 || cyclerTimer.get() > 8.0) {
                    cyclerPhase = CyclerPhase.SHOOTING;
                    cyclerTimer.restart();
                    lastShotTimestamp = 0.0;
                    latchedShootTarget = null; // Clear latch for next cycle
                }

                currentAIStateDetail = String.format("CYCLER_TRANSIT (Fuel: %d, Hub: %s)", heldPieces, hubActive ? "ACT" : "INACT");
                return computeDriveToPoseSpeeds(currentPose, latchedShootTarget, maxSpeed);

            case SHOOTING:
                if (!hubActive && heldPieces > 0) {
                    cyclerPhase = CyclerPhase.HUNT_FUEL;
                    cyclerTimer.restart();
                    currentAIStateDetail = "CYCLER_HUB_INACTIVE_ABORT";
                    return new ChassisSpeeds();
                }

                Rotation2d targetYaw = opponentHub.minus(currentPose.getTranslation()).getAngle();
                currentTargetPose = new Pose2d(currentPose.getTranslation(), targetYaw);

                double omega = headingController.calculate(currentPose.getRotation().getRadians(), targetYaw.getRadians());
                omega = Math.max(-4.5, Math.min(4.5, omega));

                if (isShootingLaneBlocked(currentPose, opponentHub)) {
                    Translation2d toHub = opponentHub.minus(currentPose.getTranslation());
                    Translation2d lateral = new Translation2d(-toHub.getY(), toHub.getX());
                    if (lateral.getNorm() > 0.1) lateral = lateral.div(lateral.getNorm());
                    double strafeSign = (currentPose.getY() > 4.035) ? -1.4 : 1.4;
                    currentAIStateDetail = "CYCLER_EVADING_DEFENDER";
                    return new ChassisSpeeds(lateral.getX() * strafeSign, lateral.getY() * strafeSign, omega);
                }

                if (canShootNow(currentPose, opponentIsRed)) {
                    launchOpponentShot(currentPose, opponentHub, opponentIsRed);
                    if (intakeSimulation != null) intakeSimulation.obtainGamePieceFromIntake();
                    lastShotTimestamp = Timer.getFPGATimestamp();
                } else if (heldPieces == 0 && cyclerTimer.get() > 0.4) {
                    cyclerPhase = CyclerPhase.HUNT_FUEL;
                    cyclerTimer.restart();
                    latchedShootTarget = null;
                }

                if (cyclerTimer.get() > 8.5) {
                    cyclerPhase = CyclerPhase.HUNT_FUEL;
                    cyclerTimer.restart();
                    latchedShootTarget = null;
                }

                currentAIStateDetail = String.format("CYCLER_SHOOTING (%d/30)", heldPieces);
                return new ChassisSpeeds(0, 0, omega);

            default:
                cyclerPhase = CyclerPhase.HUNT_FUEL;
                return new ChassisSpeeds();
        }
    }

    public boolean isOpponentHubActive(boolean opponentIsRed) {
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena instanceof Arena2026Rebuilt arena2026) {
            boolean isBlueGoal = !opponentIsRed;
            return arena2026.isActive(isBlueGoal);
        }

        double matchTime = Timer.getMatchTime();
        if (matchTime < 0 || matchTime > 130.0 || matchTime <= 30.0) {
            return true;
        }

        boolean playerHubActive = Dashboard.getInstance().isHubActive();
        return !playerHubActive;
    }

    public boolean isPoseInLowClearanceZone(Pose2d pose) {
        return FieldMap.Trenches.isLowClearance(pose);
    }

    public boolean isShootingLaneBlocked(Pose2d shooterPose, Translation2d targetHub) {
        Pose2d playerPose = SwerveBase.getInstance().getPose();
        Translation2d botPos = shooterPose.getTranslation();
        Translation2d toHub = targetHub.minus(botPos);
        Translation2d toPlayer = playerPose.getTranslation().minus(botPos);

        double distToPlayer = toPlayer.getNorm();
        if (distToPlayer < 1.60 && distToPlayer > 0.05) {
            double angleDiff = Math.abs(toHub.getAngle().minus(toPlayer.getAngle()).getRadians());
            if (angleDiff < Math.toRadians(25.0)) {
                return true;
            }
        }
        return false;
    }

    public boolean isValidShootingLocation(Pose2d pose, boolean opponentIsRed) {
        if (pose == null) return false;
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(opponentIsRed);
        double dist = pose.getTranslation().getDistance(hub);
        if (dist < 1.40 || dist > 4.00) return false;
        if (isPoseInLowClearanceZone(pose)) return false;

        double x = pose.getX();
        double y = pose.getY();
        if (x < 0.6 || x > 15.9 || y < 1.60 || y > 6.45) return false;
        return FieldMap.AllianceZones.isInAllianceZone(pose, opponentIsRed);
    }

    public boolean canShootNow(Pose2d currentPose, boolean opponentIsRed) {
        if (!isOpponentHubActive(opponentIsRed)) return false;
        if (getFuelCount() <= 0) return false;
        if (!isValidShootingLocation(currentPose, opponentIsRed)) return false;

        Translation2d hub = opponentIsRed ? Constants.RED_HUB_LOCATION.toTranslation2d() : Constants.BLUE_HUB_LOCATION.toTranslation2d();
        if (isShootingLaneBlocked(currentPose, hub)) return false;

        Rotation2d aimAngle = hub.minus(currentPose.getTranslation()).getAngle();
        double headingErr = Math.abs(currentPose.getRotation().minus(aimAngle).getRadians());
        if (headingErr > Math.toRadians(8.0)) return false;

        double now = Timer.getFPGATimestamp();
        return (now - lastShotTimestamp >= 0.15);
    }

    /**
     * Calculates an optimal standoff pose using a polar sector projection around the Hub.
     * Guarantees target is strictly outside the Hub ramps and inside the Alliance Zone.
     */
    public Pose2d calculatePolarStandoffPose(Pose2d robotPose, boolean opponentIsRed) {
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(opponentIsRed);
        Translation2d toRobot = robotPose.getTranslation().minus(hub);

        double approachAngleRad = Math.atan2(toRobot.getY(), toRobot.getX());
        double clampedAngleRad;
        if (opponentIsRed) {
            clampedAngleRad = Math.max(-Math.PI / 4.0, Math.min(Math.PI / 4.0, approachAngleRad));
        } else {
            double angleFromWest = Math.IEEEremainder(approachAngleRad - Math.PI, 2 * Math.PI);
            double clampedFromWest = Math.max(-Math.PI / 4.0, Math.min(Math.PI / 4.0, angleFromWest));
            clampedAngleRad = Math.PI + clampedFromWest;
        }

        double targetX = hub.getX() + FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE * Math.cos(clampedAngleRad);
        double targetY = hub.getY() + FieldMap.Hubs.OPTIMAL_STANDOFF_DISTANCE * Math.sin(clampedAngleRad);

        targetY = Math.max(2.20, Math.min(5.80, targetY));

        if (opponentIsRed) {
            targetX = Math.max(13.60, Math.min(15.00, targetX));
        } else {
            targetX = Math.max(1.60, Math.min(2.90, targetX));
        }

        Translation2d standoffPos = new Translation2d(targetX, targetY);
        Rotation2d faceHubAngle = hub.minus(standoffPos).getAngle();

        return new Pose2d(standoffPos, faceHubAngle);
    }

    /**
     * Canonical alias for {@link #calculatePolarStandoffPose(Pose2d, boolean)}
     * supporting unit tests and legacy callers.
     */
    public Pose2d getOptimalShootingPose(Pose2d currentPose, boolean opponentIsRed) {
        return calculatePolarStandoffPose(currentPose, opponentIsRed);
    }

    public Pose2d findBestFuelTarget(Pose2d currentPose, boolean opponentIsRed) {
        SimulatedArena arena = SimulatedArena.getInstance();
        Translation2d bestFuel = null;
        double minDistance = Double.MAX_VALUE;

        if (arena != null) {
            try {
                Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
                if (pieces != null && !pieces.isEmpty()) {
                    for (var piece : pieces) {
                        if (piece == null || !"Fuel".equals(piece.getType())) continue;
                        Translation2d pos = piece.getPoseOnField().getTranslation();

                        // Must be on the field carpet
                        if (pos.getX() < 0.05 || pos.getX() > 16.48 || pos.getY() < 0.05 || pos.getY() > 8.00) continue;
                        if (StaticPathfinder.isPointInObstacle(pos)) continue;

                        // Focus on opponent's half + center zone
                        if (opponentIsRed && pos.getX() < 4.0) continue;
                        if (!opponentIsRed && pos.getX() > 12.5) continue;

                        double dist = currentPose.getTranslation().getDistance(pos);
                        if (dist < minDistance) {
                            minDistance = dist;
                            bestFuel = pos;
                        }
                    }
                }
            } catch (Exception ignored) {}
        }

        if (bestFuel != null) {
            // ── WALL STANDOFF OFFSET ─────────────────────────────────────────
            // If the ball is tight against a perimeter wall, position the robot center
            // 0.48m off the ball so the front intake rests right on top of it without ramming.
            double approachX = bestFuel.getX();
            double approachY = bestFuel.getY();
            Rotation2d targetHeading;

            if (bestFuel.getX() > 15.60) {
                // Ball near Red driver wall: face wall (0 deg), stop chassis at X = ballX - 0.48m
                approachX = Math.min(15.98, bestFuel.getX() - 0.48);
                targetHeading = Rotation2d.fromDegrees(0);
            } else if (bestFuel.getX() < 0.90) {
                // Ball near Blue driver wall: face wall (180 deg), stop chassis at X = ballX + 0.48m
                approachX = Math.max(0.55, bestFuel.getX() + 0.48);
                targetHeading = Rotation2d.fromDegrees(180);
            } else if (bestFuel.getY() < 0.90) {
                // Ball near bottom wall: face bottom wall (-90 deg), stop chassis at Y = ballY + 0.48m
                approachY = Math.max(0.55, bestFuel.getY() + 0.48);
                targetHeading = Rotation2d.fromDegrees(-90);
            } else if (bestFuel.getY() > 7.15) {
                // Ball near top wall: face top wall (90 deg), stop chassis at Y = ballY - 0.48m
                approachY = Math.min(7.50, bestFuel.getY() - 0.48);
                targetHeading = Rotation2d.fromDegrees(90);
            } else {
                // Open field: drive front bumper directly toward the ball
                targetHeading = bestFuel.minus(currentPose.getTranslation()).getAngle();
                Translation2d offset = new Translation2d(0.35, 0).rotateBy(targetHeading);
                approachX = bestFuel.getX() - offset.getX();
                approachY = bestFuel.getY() - offset.getY();
            }

            Translation2d targetPos = new Translation2d(approachX, approachY);
            return StaticPathfinder.ensurePoseOutsideObstacles(new Pose2d(targetPos, targetHeading), currentPose.getTranslation());
        }

        // Fallback: Midline patrol
        double midX = JevDecisionEngine.CENTERLINE_X;
        double midY = (currentPose.getY() > 4.0) ? 5.80 : 2.40;
        return StaticPathfinder.ensurePoseOutsideObstacles(
                new Pose2d(midX, midY, Rotation2d.fromDegrees(opponentIsRed ? 180 : 0)), currentPose.getTranslation());
    }

    /**
     * Ingests fuel balls using a rectangular front-roller intake bounding box.
     * Captures balls pressed directly against perimeter walls or in corners.
     */
    public void checkProximityPickup(Pose2d robotPose) {
        if (intakeSimulation == null || !intakeSimulation.isRunning()) return;
        if (intakeSimulation.getGamePiecesAmount() >= Constants.IntakeConstants.MAX_HELD_BALLS) return;

        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null) return;

        try {
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
            if (pieces == null) return;

            Translation2d botPos = robotPose.getTranslation();
            Rotation2d botHeading = robotPose.getRotation();

            int collectedThisTick = 0;
            for (var piece : pieces) {
                if (piece == null || !"Fuel".equals(piece.getType())) continue;
                Translation2d ball = piece.getPoseOnField().getTranslation();

                // Transform ball to robot-relative frame (+X forward, +Y left)
                Translation2d rel = ball.minus(botPos).rotateBy(botHeading.unaryMinus());

                // Rectangular intake zone:
                // X: 0.20m to 0.75m in front of robot center (bumper is at ~0.45m)
                // Y: +/- 0.45m lateral width (full bumper opening)
                boolean inIntakeBox = (rel.getX() >= 0.20 && rel.getX() <= 0.78) 
                                   && (Math.abs(rel.getY()) <= 0.45);

                if (inIntakeBox) {
                    arena.removeGamePiece(piece);
                    intakeSimulation.addGamePieceToIntake();
                    collectedThisTick++;

                    if (intakeSimulation.getGamePiecesAmount() >= Constants.IntakeConstants.MAX_HELD_BALLS 
                            || collectedThisTick >= 4) {
                        break;
                    }
                }
            }
        } catch (Exception ignored) {}
    }

    public boolean isNearAnyFuel(Translation2d botPos, double radius) {
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null) return false;
        try {
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
            if (pieces != null) {
                for (var piece : pieces) {
                    if (piece != null && "Fuel".equals(piece.getType())) {
                        if (piece.getPoseOnField().getTranslation().getDistance(botPos) <= radius) return true;
                    }
                }
            }
        } catch (Exception ignored) {}
        return false;
    }

    public void launchOpponentShot(Pose2d robotPose, Translation2d opponentHub, boolean opponentIsRed) {
        Translation3d hub3d = opponentIsRed ? Constants.RED_HUB_LOCATION : Constants.BLUE_HUB_LOCATION;
        Translation3d funnelTarget = new Translation3d(hub3d.getX(), hub3d.getY(), 1.48);

        Translation2d botPos = robotPose.getTranslation();
        Translation2d target2d = new Translation2d(funnelTarget.getX(), funnelTarget.getY());
        Translation2d shooterOffset = new Translation2d(0.20, 0.0);
        ChassisSpeeds robotVel = (driveSimulation != null && driveSimulation.getDriveTrainSimulation() != null)
                ? driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative()
                : new ChassisSpeeds();

        Translation2d vel = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);
        Translation2d compensatedTarget = target2d;
        if (vel.getNorm() > 0.05) {
            double dist = botPos.getDistance(target2d);
            double tof = 0.12 + 0.18 * dist;
            compensatedTarget = target2d.minus(vel.times(tof));
        }

        double distance = botPos.getDistance(compensatedTarget);
        double g = 9.81;
        double theta = Constants.FIRING_ANGLE;
        double h = funnelTarget.getZ() - 0.53;
        double denom = 2.0 * (distance * Math.tan(theta) - h);
        double exitVel = denom > 0.1 ? (distance / Math.cos(theta)) * Math.sqrt(g / denom) : 6.8;

        double randomExitVelocity = exitVel * (1.0 + (Math.random() - 0.5) * 0.04);
        Rotation2d aimAngle = compensatedTarget.minus(botPos).getAngle();
        Rotation2d randomYaw = aimAngle.plus(Rotation2d.fromDegrees((Math.random() - 0.5) * 1.6));
        double randomPitch = theta + (Math.random() - 0.5) * 0.025;

        try {
            var fuelOnFly = new RebuiltFuelOnFly(
                    botPos, shooterOffset, robotVel, randomYaw,
                    Meters.of(0.53), MetersPerSecond.of(randomExitVelocity), Radians.of(randomPitch)
            );
            fuelOnFly.withTargetPosition(() -> funnelTarget)
                    .withTargetTolerance(new Translation3d(0.38, 0.38, 0.20))
                    .withHitTargetCallBack(() -> aiScoreCount++);
            SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
        } catch (Exception e) {
            System.err.println("[AIRobotSim] Error launching fuel projectile: " + e.getMessage());
        }
    }

    @Override
    public void update() {
        if (edu.wpi.first.wpilibj.RobotBase.isReal()) {
            return;
        }

        Pose2d pose = driveSimulation.getActualPoseInSimulationWorld();
        boolean active = Dashboard.isOpponentRobotEnabled();

        Logger.recordOutput("AI_Telemetry/ActualPose", pose);
        Logger.recordOutput("AI_Telemetry/TargetPose", currentTargetPose);
        Logger.recordOutput("AI_Telemetry/StateDetail", currentAIStateDetail);
        Logger.recordOutput("AI_Telemetry/CyclerPhase", cyclerPhase.name());
        Logger.recordOutput("AI_Telemetry/ScoreCount", aiScoreCount);
        Logger.recordOutput("AI_Telemetry/HeldFuel", intakeSimulation != null ? intakeSimulation.getGamePiecesAmount() : 0);
        Logger.recordOutput("AI_Telemetry/IsStalled", isStalled());
        Logger.recordOutput("AI_Telemetry/StallDurationSec", stallDuration);
        Logger.recordOutput("AI_Telemetry/Waypoints", aiTrajectoryController.getWaypoints().toArray(new Pose2d[0]));
        Logger.recordOutput("AI_Telemetry/CurrentWaypointIndex", aiTrajectoryController.getCurrentWaypointIndex());

        Logger.recordOutput("AI_Telemetry/CommandedVxField", currentTargetSpeeds.vxMetersPerSecond);
        Logger.recordOutput("AI_Telemetry/CommandedVyField", currentTargetSpeeds.vyMetersPerSecond);
        Logger.recordOutput("AI_Telemetry/CommandedOmega", currentTargetSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("AI_Telemetry/CommandedVxRobot", lastRobotRelativeSpeeds.vxMetersPerSecond);
        Logger.recordOutput("AI_Telemetry/CommandedVyRobot", lastRobotRelativeSpeeds.vyMetersPerSecond);

        ChassisSpeeds actualPhysicsSpeeds = driveSimulation.getDriveTrainSimulation() != null
                ? driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative()
                : new ChassisSpeeds();
        Logger.recordOutput("AI_Telemetry/ActualVxField", actualPhysicsSpeeds.vxMetersPerSecond);
        Logger.recordOutput("AI_Telemetry/ActualVyField", actualPhysicsSpeeds.vyMetersPerSecond);
        Logger.recordOutput("AI_Telemetry/ActualOmega", actualPhysicsSpeeds.omegaRadiansPerSecond);

        Pose2d playerPose = SwerveBase.getInstance().getPose();
        double distToPlayer = pose.getTranslation().getDistance(playerPose.getTranslation());
        Logger.recordOutput("AI_Telemetry/DistanceToPlayer", distToPlayer);

        SmartDashboard.putBoolean("Simulation/OpponentActive", active);
        SmartDashboard.putNumberArray("Simulation/OpponentPose", new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
        SmartDashboard.putNumberArray("Simulation/OpponentTargetPose", new double[] { currentTargetPose.getX(), currentTargetPose.getY(), currentTargetPose.getRotation().getDegrees() });
        SmartDashboard.putString("Simulation/OpponentAIState", currentAIStateDetail);
        SmartDashboard.putNumber("Simulation/OpponentScoreCount", aiScoreCount);
        SmartDashboard.putNumber("Simulation/OpponentFuelCount", intakeSimulation != null ? intakeSimulation.getGamePiecesAmount() : 0);
        SmartDashboard.putBoolean("Simulation/OpponentStalled", isStalled());

        double now = Timer.getFPGATimestamp();
        double commandedMag = Math.hypot(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
        double actualMag = Math.hypot(actualPhysicsSpeeds.vxMetersPerSecond, actualPhysicsSpeeds.vyMetersPerSecond);

        if (active && commandedMag > 0.5 && actualMag < 0.15 && (now - lastConsoleDumpTime > 1.5)) {
            lastConsoleDumpTime = now;
            System.out.printf(
                    "[AI DIAGNOSTIC] STUCK DETECTED! Mode: %s | Phase: %s | State: %s%n" +
                    "  -> Actual Pose: (%.2f, %.2f, %.1f deg) | Target: (%.2f, %.2f)%n" +
                    "  -> DistToPlayer: %.2fm | StallDuration: %.2fs%n" +
                    "  -> Cmd Speeds: [Vx=%.2f, Vy=%.2f, Omega=%.2f] | Act Speeds: [Vx=%.2f, Vy=%.2f]%n" +
                    "  -> Waypoints Count: %d | Current WP Index: %d%n",
                    getAIMode().name(), cyclerPhase.name(), currentAIStateDetail,
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
                    currentTargetPose.getX(), currentTargetPose.getY(),
                    distToPlayer, stallDuration,
                    currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond, currentTargetSpeeds.omegaRadiansPerSecond,
                    actualPhysicsSpeeds.vxMetersPerSecond, actualPhysicsSpeeds.vyMetersPerSecond,
                    aiTrajectoryController.getWaypoints().size(), aiTrajectoryController.getCurrentWaypointIndex()
            );
        }

        if (active && pose.getY() > 0.0 && pose.getX() > 0.0) {
            Translation2d vel = new Translation2d(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
            boolean isContacting = distToPlayer < 0.85;
            DynamicRouter.registerObstacle(pose.getTranslation(), vel, 0.55, 0.35, isContacting);
        }
    }

    @Override
    public void initialize() {
        setTrajectory("OpponentPath");
        for (AIMode mode : AIMode.values()) {
            aiModeChooser.addOption(mode.displayName, mode);
        }
        aiModeChooser.setDefaultOption(AIMode.AUTONOMOUS_CYCLER.displayName, AIMode.AUTONOMOUS_CYCLER);
        SmartDashboard.putData("Simulation/AIModeChooser", aiModeChooser);
        SmartDashboard.setDefaultString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name());
        SmartDashboard.setDefaultNumber("Simulation/OpponentSpeedPercent", 75.0);
    }

    public static Pose2d getOpponentSpawnPose(boolean playerIsRed) {
        if (playerIsRed) {
            return new Pose2d(2.00, 4.035, Rotation2d.fromDegrees(0));
        } else {
            return new Pose2d(AllianceFlipUtil.FIELD_LENGTH - 2.00, 4.035, Rotation2d.fromDegrees(180));
        }
    }

    public Pose2d getOpponentSpawnPose(boolean playerIsRed, AIMode activeMode) {
        if (activeMode == AIMode.CHOREO_PATH && trajectory.isPresent()) {
            Optional<SwerveSample> initialSample = trajectory.get().sampleAt(0, false);
            if (initialSample.isPresent()) {
                SwerveSample sample = initialSample.get();
                Pose2d startPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
                return mirrorPoseForOpponent(startPose, playerIsRed);
            }
        }
        return getOpponentSpawnPose(playerIsRed);
    }

    public static Pose2d mirrorPoseForOpponent(Pose2d pose, boolean playerIsRed) {
        boolean opponentIsRed = !playerIsRed;
        boolean poseIsOnRed = pose.getX() > (AllianceFlipUtil.FIELD_LENGTH / 2.0);

        if (poseIsOnRed == opponentIsRed) {
            return pose;
        } else {
            double mirroredX = AllianceFlipUtil.FIELD_LENGTH - pose.getX();
            Rotation2d mirroredRotation = pose.getRotation().plus(Rotation2d.fromDegrees(180));
            return new Pose2d(mirroredX, pose.getY(), mirroredRotation);
        }
    }

    @Override
    public void log() {
    }

    @Override
    public boolean isEnabled() {
        return edu.wpi.first.wpilibj.RobotBase.isSimulation();
    }

    @Override
    public String getName() {
        return "AIRobotSim";
    }

    public AIMode getAIMode() {
        if (aiModeChooser != null && aiModeChooser.getSelected() != null) {
            return aiModeChooser.getSelected();
        }
        String modeStr = SmartDashboard.getString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name());
        return AIMode.fromString(modeStr);
    }

    public CyclerPhase getCyclerPhase() {
        return cyclerPhase;
    }

    public void setCyclerPhase(CyclerPhase phase) {
        this.cyclerPhase = phase;
        this.cyclerTimer.restart();
    }

    public int getAiScoreCount() {
        return aiScoreCount;
    }

    public int getFuelCount() {
        return (intakeSimulation != null) ? intakeSimulation.getGamePiecesAmount() : 0;
    }

    public void setFuelCount(int count) {
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(count);
        }
    }

    public SelfControlledSwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public List<Pose2d> getCurrentPath() {
        return aiTrajectoryController.getWaypoints();
    }

    public int getCurrentPathIndex() {
        return aiTrajectoryController.getCurrentWaypointIndex();
    }
}