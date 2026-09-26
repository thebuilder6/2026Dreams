package frc.robot.Intelligence.Sparring;

import frc.robot.Sim.MatchKnowledge;
import frc.robot.Sim.RefereeSim;
import frc.robot.Sim.ShotTracker;

import frc.robot.Sim.DeadlockResolver;

import frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.Shooter;

import frc.robot.Telemetry.Dashboard;

import frc.robot.Intelligence.AIActionIntent;

import frc.robot.Intelligence.JevDecisionEngine;

import frc.robot.Intelligence.State.MatchScoreTracker;

import frc.robot.Intelligence.State.WorldStateBuilder;

import frc.robot.Intelligence.State.WorldState;

import frc.robot.Intelligence.Archetype;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Auto.TrajectoryController;
import frc.robot.Data.Constants;
import frc.robot.Data.FieldMap;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.SwerveBase;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Autonomous AI Robot Instance.
 * Encapsulates an independent simulated swerve drive chassis, intake mechanism,
 * PID controllers, and behavior archetype. Can be run in parallel with other
 * AI instances for multi-robot sparring simulation.
 */
public class AIRobotInstance {

    private final int botId;
    private final Pose2d queuingPose;
    private Archetype archetype;
    private final boolean isAlly;

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private IntakeSimulation intakeSimulation;

    private final PIDController headingController;
    private final TrajectoryController trajectoryController;

    private ChassisSpeeds currentTargetSpeeds = new ChassisSpeeds();
    private ChassisSpeeds lastRobotRelativeSpeeds = new ChassisSpeeds();
    private Pose2d currentTargetPose = new Pose2d();
    private String currentAIStateDetail = "IDLE";
    private int scoreCount = 0;
    private double lastShotTimestamp = 0.0;
    private final frc.robot.HMI.Watchdogs.LegalPinningWatchdog pinWatchdog = new frc.robot.HMI.Watchdogs.LegalPinningWatchdog();

    // Stall watchdog
    private Pose2d lastActualPose = new Pose2d();
    private double stallDuration = 0.0;
    private boolean lastStallResult = false;
    private double lastStallEvalTimestamp = -1.0;

    // Deadlock recovery (head-on trench meetings, same-target scrums)
    private final DeadlockResolver deadlockResolver = new DeadlockResolver();

    public AIRobotInstance(int botId, Pose2d queuingPose, Archetype defaultArchetype) {
        this(botId, queuingPose, defaultArchetype, false);
    }

    public AIRobotInstance(int botId, Pose2d queuingPose, Archetype defaultArchetype, boolean isAlly) {
        this.botId = botId;
        this.queuingPose = queuingPose;
        this.archetype = defaultArchetype != null ? defaultArchetype : Archetype.AUTONOMOUS_CYCLER;
        this.isAlly = isAlly;

        String prefix = isAlly ? ("Simulation/Ally" + (botId - 100)) : ("Simulation/Bot" + botId);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultString(
                prefix + "/Archetype", this.archetype.name());

        this.driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(DriveTrainSimulationConfig.Default(), queuingPose));
        try {
            this.driveSimulation.getDriveTrainSimulation().getGyroSimulation().setRotation(queuingPose.getRotation());
            this.driveSimulation.resetOdometry(queuingPose);
        } catch (Exception ignored) {}

        try {
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
        } catch (Exception e) {
            System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Could not register drivetrain with arena: " + e.getMessage());
        }

        try {
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                    "Fuel",
                    driveSimulation.getDriveTrainSimulation(),
                    Meters.of(0.70),
                    Meters.of(0.30),
                    IntakeSimulation.IntakeSide.FRONT,
                    Constants.IntakeConstants.MAX_HELD_BALLS
            );
            if (this.intakeSimulation != null) {
                this.intakeSimulation.setGamePiecesCount(AIRobotSim.INITIAL_HELD_BALLS);
            }
        } catch (Exception e) {
            System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Could not attach IntakeSimulation: " + e.getMessage());
        }

        var config = SwerveBase.getInstance().getSwerveController().config;
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectoryController = new TrajectoryController(
                new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d));
    }

    public int getBotId() {
        return botId;
    }

    public boolean isAlly() {
        return isAlly;
    }

    public Archetype getArchetype() {
        return archetype;
    }

    public void setArchetype(Archetype archetype) {
        if (archetype != null) {
            this.archetype = archetype;
        }
    }

    public SelfControlledSwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public Pose2d getActualPose() {
        return driveSimulation.getActualPoseInSimulationWorld();
    }

    /**
     * Current field-relative chassis velocity (for lead-pursuit mark tracking).
     */
    public ChassisSpeeds getFieldVelocity() {
        try {
            if (driveSimulation.getDriveTrainSimulation() != null) {
                return driveSimulation.getDriveTrainSimulation()
                        .getDriveTrainSimulatedChassisSpeedsFieldRelative();
            }
        } catch (Exception ignored) {}
        return new ChassisSpeeds();
    }

    public void setRobotPose(Pose2d pose) {
        driveSimulation.setSimulationWorldPose(pose);
        try {
            driveSimulation.getDriveTrainSimulation().getGyroSimulation().setRotation(pose.getRotation());
            driveSimulation.resetOdometry(pose);
        } catch (Exception ignored) {}
    }

    public void reset() {
        setRobotPose(queuingPose);
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(AIRobotSim.INITIAL_HELD_BALLS);
            intakeSimulation.stopIntake();
        }
        scoreCount = 0;
        stallDuration = 0.0;
        lastShotTimestamp = 0.0;
        pinWatchdog.reset();
        deadlockResolver.reset();
        trajectoryController.reset();
        try {
            String botName = isAlly ? ("AllyBot" + (botId - 100)) : ("OpponentBot" + botId);
            String targetName = isAlly ? ("AllyTarget" + (botId - 100)) : ("OpponentTarget" + botId);
            SwerveBase.getInstance().getField().getObject(botName).setPoses(new java.util.ArrayList<>());
            SwerveBase.getInstance().getField().getObject(targetName).setPoses(new java.util.ArrayList<>());
        } catch (Exception ignored) {}
    }

    /**
     * Executes one 50Hz cycle of the unified System 1 + System 2 cognitive loop.
     *
     * @param peerRobotPoses Poses of all peer robots on the field (including other AI bots and player)
     * @param playerIsRed True if player robot is Red Alliance (so this opponent is Blue Alliance)
     * @param maxSpeed Maximum chassis speed in m/s
     */
    public void update(List<Pose2d> peerRobotPoses, boolean playerIsRed, double maxSpeed) {
        update(peerRobotPoses, playerIsRed, maxSpeed, null, null);
    }

    /**
     * Same as {@link #update(List, boolean, double)}, but defensive archetypes
     * track the given mark as their opponent instead of defaulting to the player.
     * A null mark falls back to the player pose/velocity.
     *
     * @param markPose Opponent mark pose (null = player)
     * @param markVelocity Opponent mark field-relative velocity (null = player velocity)
     */
    public void update(List<Pose2d> peerRobotPoses, boolean playerIsRed, double maxSpeed,
            Pose2d markPose, ChassisSpeeds markVelocity) {
        try {
            driveSimulation.periodic();
        } catch (Exception ignored) {}

        boolean botAllianceIsRed = isAlly ? playerIsRed : !playerIsRed;
        Pose2d currentPose = driveSimulation.getActualPoseInSimulationWorld();
        ChassisSpeeds currentVel = driveSimulation.getDriveTrainSimulation() != null
                ? driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative()
                : new ChassisSpeeds();

        int heldPieces = (intakeSimulation != null) ? intakeSimulation.getGamePiecesAmount() : 0;
        boolean hubActive = AIRobotSim.getInstance() != null
                ? AIRobotSim.getInstance().isHubActiveForAlliance(botAllianceIsRed)
                : true;

        // 0. Update Archetype from chooser or dashboard if modified
        if (isAlly) {
            int allyIndex = botId - 100;
            if (allyIndex == 1 && AIRobotSim.getInstance() != null) {
                this.archetype = AIRobotSim.getInstance().getAlly1Archetype();
            } else if (allyIndex == 2 && AIRobotSim.getInstance() != null) {
                this.archetype = AIRobotSim.getInstance().getAlly2Archetype();
            } else {
                String archStr = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString(
                        "Simulation/Ally" + allyIndex + "/Archetype", archetype.name());
                Archetype selectedArch = Archetype.fromString(archStr);
                if (selectedArch != null) {
                    this.archetype = selectedArch;
                }
            }
        } else {
            if (botId == 1 && AIRobotSim.getInstance() != null) {
                this.archetype = AIRobotSim.getInstance().getBot1Archetype();
            } else if (botId == 2 && AIRobotSim.getInstance() != null) {
                this.archetype = AIRobotSim.getInstance().getBot2Archetype();
            } else {
                String archStr = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString(
                        "Simulation/Bot" + botId + "/Archetype", archetype.name());
                Archetype selectedArch = Archetype.fromString(archStr);
                if (selectedArch != null) {
                    this.archetype = selectedArch;
                }
            }
        }

        // 1. Build immutable WorldState snapshot. Defensive bots track their
        // assigned mark as the opponent; everyone else defaults to the player.
        WorldState worldState;
        Pose2d opponentPose;
        ChassisSpeeds opponentVel;
        if (markPose != null && archetype.isDefensive()) {
            opponentPose = markPose;
            opponentVel = (markVelocity != null) ? markVelocity : new ChassisSpeeds();
            worldState = WorldStateBuilder.buildForSimBot(
                    currentPose, currentVel, heldPieces, botAllianceIsRed, hubActive,
                    opponentPose, opponentVel);
        } else {
            opponentPose = SwerveBase.getInstance().getPose();
            opponentVel = SwerveBase.getInstance().getFieldVelocity();
            worldState = WorldStateBuilder.buildForSimBot(
                    currentPose, currentVel, heldPieces, botAllianceIsRed, hubActive);
        }
        MatchKnowledge knowledge = WorldStateBuilder.buildMatchKnowledgeForSimBot(botAllianceIsRed);

        // 2. Evaluate unified Jev policy (stateless System 2 + System 1)
        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(
                worldState, knowledge, archetype);

        // 3. Compute drive trajectory speeds
        currentTargetPose = intent.navigationTarget();
        currentAIStateDetail = intent.objective().name() + " (" + intent.rationale() + ")";
        boolean stalled = isStalled();
        currentTargetSpeeds = trajectoryController.calculate(
                currentPose, currentTargetSpeeds, currentTargetPose, maxSpeed, stalled, true);

        // Track pinning against the assigned mark (or the player by default) -
        // only for opponent bots. RefereeSim scores the actual foul; this drives
        // the backoff maneuver.
        if (!isAlly) {
            Pose2d pinReference = (markPose != null && archetype.isDefensive())
                    ? markPose
                    : ((peerRobotPoses != null && !peerRobotPoses.isEmpty()) ? peerRobotPoses.get(0) : null);
            if (pinReference != null) {
                double distToPlayer = currentPose.getTranslation().getDistance(pinReference.getTranslation());
                boolean isContacting = (distToPlayer < 1.05) && (isStalled() || (distToPlayer < 0.95 && Math.hypot(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond) > 0.5));
                pinWatchdog.update(isContacting, currentPose, pinReference, 0.02);
            }

            if (pinWatchdog.isForcedBackoffActive() && archetype.isDefensive() && pinReference != null) {
                Pose2d backoff = pinWatchdog.getBackOffTarget(currentPose, pinReference);
                currentTargetPose = backoff;
                currentTargetSpeeds = trajectoryController.calculate(
                        currentPose, currentTargetSpeeds, currentTargetPose, maxSpeed, false, false);
                currentAIStateDetail = String.format("PIN_RULE_BACKOFF (%.1fs)", pinWatchdog.getBackoffRemainingSec());
            }
        }

        // Soft peer separation (avoids jamming and scrums between multi-bots)
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double dist = currentPose.getTranslation().getDistance(peerPose.getTranslation());
                if (dist > 0.05 && dist < 1.10) {
                    Translation2d diff = currentPose.getTranslation().minus(peerPose.getTranslation());
                    double scale = (1.10 - dist) / 1.10;
                    Translation2d nudge = diff.div(dist).times(scale * 1.5);
                    currentTargetSpeeds.vxMetersPerSecond += nudge.getX();
                    currentTargetSpeeds.vyMetersPerSecond += nudge.getY();
                }
            }
        }

        // Deadlock recovery: sustained stall pressed against a peer means the
        // symmetric separation nudges above have stalemated (trench head-on or
        // shared target). Yield forward drive and jink laterally to break it.
        double nearestPeerDist = Double.MAX_VALUE;
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double d = currentPose.getTranslation().getDistance(peerPose.getTranslation());
                if (d > 0.05 && d < nearestPeerDist) nearestPeerDist = d;
            }
        }
        DeadlockResolver.Resolution deadlock = deadlockResolver.update(stalled, nearestPeerDist, 0.02);
        if (deadlock.recovering()) {
            Translation2d jinkField = new Translation2d(0, deadlock.lateralJink())
                    .rotateBy(currentPose.getRotation());
            currentTargetSpeeds.vxMetersPerSecond =
                    currentTargetSpeeds.vxMetersPerSecond * deadlock.forwardScale() + jinkField.getX();
            currentTargetSpeeds.vyMetersPerSecond =
                    currentTargetSpeeds.vyMetersPerSecond * deadlock.forwardScale() + jinkField.getY();
            currentAIStateDetail = "DEADLOCK_RECOVERY";
        }

        // Heading aim override (e.g. during shooting or tracking target)
        if (intent.aimOverride() != null) {
            double omega = headingController.calculate(
                    currentPose.getRotation().getRadians(), intent.aimOverride().getRadians());
            omega = Math.max(-4.5, Math.min(4.5, omega));
            currentTargetSpeeds.omegaRadiansPerSecond = omega;
        }

        lastRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(currentTargetSpeeds, currentPose.getRotation());
        driveSimulation.runChassisSpeeds(lastRobotRelativeSpeeds, new Translation2d(), false, true);

        // 4. Mechanism Execution (Intake & Shooter)
        if (intakeSimulation != null) {
            if (intent.intakeCommand() == IntakeState.INTAKING) {
                if (!intakeSimulation.isRunning()) intakeSimulation.startIntake();
                checkProximityPickup(currentPose);
            } else {
                if (intakeSimulation.isRunning()) intakeSimulation.stopIntake();
            }
        }

        if (intent.triggerFeedKicker() && canShootNow(currentPose, botAllianceIsRed)) {
            launchShot(currentPose, botAllianceIsRed);
            if (intakeSimulation != null) intakeSimulation.obtainGamePieceFromIntake();
            lastShotTimestamp = Timer.getFPGATimestamp();
        }

        // 5. Register dynamic obstacle for peer robots
        if (currentPose.getX() > 0.0 && currentPose.getY() > 0.0) {
            Translation2d vel = new Translation2d(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
            DynamicRouter.registerObstacle(currentPose.getTranslation(), vel, 0.55, 0.35, false);
        }

        // Publish to Field2d
        String botObjName = isAlly ? ("AllyBot" + (botId - 100)) : ("OpponentBot" + botId);
        String targetObjName = isAlly ? ("AllyTarget" + (botId - 100)) : ("OpponentTarget" + botId);
        try {
            SwerveBase.getInstance().getField().getObject(botObjName).setPose(currentPose);
            SwerveBase.getInstance().getField().getObject(targetObjName).setPose(currentTargetPose);
        } catch (Exception ignored) {}

        // Telemetry logging (AdvantageKit & SmartDashboard)
        String prefix = (isAlly ? "AI_Telemetry/Ally" + (botId - 100) : "AI_Telemetry/Bot" + botId) + "/";
        Logger.recordOutput(prefix + "ActualPose", currentPose);
        Logger.recordOutput(prefix + "TargetPose", currentTargetPose);
        Logger.recordOutput(prefix + "Objective", intent.objective().name());
        Logger.recordOutput(prefix + "StateDetail", currentAIStateDetail);
        Logger.recordOutput(prefix + "HeldFuel", heldPieces);
        Logger.recordOutput(prefix + "Score", scoreCount);
        Logger.recordOutput(prefix + "Confidence", intent.confidence());
        Logger.recordOutput(prefix + "Archetype", archetype.name());

        String dashPrefix = (isAlly ? "Simulation/Ally" + (botId - 100) : "Simulation/Bot" + botId) + "/";
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumberArray(dashPrefix + "Pose", new double[] { currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees() });
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumberArray(dashPrefix + "TargetPose", new double[] { currentTargetPose.getX(), currentTargetPose.getY(), currentTargetPose.getRotation().getDegrees() });
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "Objective", intent.objective().name());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "StateDetail", currentAIStateDetail);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(dashPrefix + "Fuel", heldPieces);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(dashPrefix + "Score", scoreCount);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(dashPrefix + "Stalled", stalled);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "Archetype", archetype.name());
    }

    public boolean canShootNow(Pose2d currentPose, boolean botAllianceIsRed) {
        if (AIRobotSim.getInstance() == null) return false;
        if (!AIRobotSim.getInstance().isHubActiveForAlliance(botAllianceIsRed)) return false;
        if (intakeSimulation == null || intakeSimulation.getGamePiecesAmount() <= 0) return false;
        if (!AIRobotSim.getInstance().isValidShootingLocation(currentPose, botAllianceIsRed)) return false;

        Translation2d hub = FieldMap.Hubs.getHubLocation2d(botAllianceIsRed);
        if (AIRobotSim.getInstance().isShootingLaneBlocked(currentPose, hub)) return false;

        Rotation2d aimAngle = hub.minus(currentPose.getTranslation()).getAngle();
        double headingErr = Math.abs(currentPose.getRotation().minus(aimAngle).getRadians());
        if (headingErr > Math.toRadians(8.0)) return false;

        double now = Timer.getFPGATimestamp();
        return (now - lastShotTimestamp >= 0.08);
    }

    public void launchShot(Pose2d robotPose, boolean botAllianceIsRed) {
        String label = isAlly ? ("Ally " + (botId - 100)) : ("Opponent Bot " + botId);
        RefereeSim.checkShotLegality(robotPose, botAllianceIsRed, label);
        Translation3d hub3d = botAllianceIsRed ? Constants.RED_HUB_LOCATION : Constants.BLUE_HUB_LOCATION;
        Translation3d funnelTarget = new Translation3d(hub3d.getX(), hub3d.getY(), 1.48);

        Translation2d botPos = robotPose.getTranslation();
        Translation2d target2d = new Translation2d(funnelTarget.getX(), funnelTarget.getY());
        Translation2d shooterOffset = new Translation2d(0.20, 0.0);
        ChassisSpeeds robotVel = driveSimulation.getDriveTrainSimulation() != null
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
            // Scoring is resolved by ShotTracker (see class docs): the hub
            // captures balls before the analytic hit-time, so the hit callback
            // alone would silently drop most scores.
            ShotTracker.track(fuelOnFly, funnelTarget, botAllianceIsRed,
                    () -> {
                        noteScoredHit();
                        MatchScoreTracker.getInstance().recordBotScore(botId, botAllianceIsRed);
                    },
                    null);
            fuelOnFly.withTargetPosition(() -> funnelTarget)
                    .withTargetTolerance(new Translation3d(0.38, 0.38, 0.20));
            SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
        } catch (Exception e) {
            System.err.println("[" + (isAlly ? "AllyBot-" : "AIRobotInstance-") + botId + "] Error launching fuel projectile: " + e.getMessage());
        }
    }

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

            Translation2d rollerCenter = botPos.plus(new Translation2d(0.48, 0.0).rotateBy(botHeading));
            double pickupWidth = 0.72;
            double pickupDepth = 0.38;

            int collectedThisTick = 0;
            for (GamePieceOnFieldSimulation piece : pieces) {
                if (piece == null || !"Fuel".equals(piece.getType())) continue;

                Translation2d piecePos = piece.getPoseOnField().getTranslation();
                Translation2d rel = piecePos.minus(rollerCenter).rotateBy(botHeading.unaryMinus());

                if (Math.abs(rel.getY()) <= pickupWidth / 2.0 && Math.abs(rel.getX()) <= pickupDepth / 2.0) {
                    arena.removeGamePiece(piece);
                    intakeSimulation.addGamePieceToIntake();
                    collectedThisTick++;

                    if (intakeSimulation.getGamePiecesAmount() >= Constants.IntakeConstants.MAX_HELD_BALLS
                            || collectedThisTick >= 10) {
                        break;
                    }
                }
            }
        } catch (Exception ignored) {}
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

    public ChassisSpeeds getCurrentTargetSpeeds() {
        return currentTargetSpeeds;
    }

    public Pose2d getCurrentTargetPose() {
        return currentTargetPose;
    }

    public String getCurrentAIStateDetail() {
        return currentAIStateDetail;
    }

    public int getScoreCount() {
        return scoreCount;
    }

    /**
     * Records one scored ball for this bot (invoked by {@link ShotTracker}
     * when a tracked shot resolves as scored).
     */
    public void noteScoredHit() {
        scoreCount++;
    }

    public int getFuelCount() {
        return intakeSimulation != null ? intakeSimulation.getGamePiecesAmount() : 0;
    }

    public double getStallDuration() {
        return stallDuration;
    }

    public TrajectoryController getTrajectoryController() {
        return trajectoryController;
    }
}
