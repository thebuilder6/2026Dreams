package frc.robot.Sim;

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
import frc.robot.Navigation.ContactWatchdog;
import frc.robot.Navigation.DynamicRouter;
import frc.robot.Navigation.TrajectoryController;
import frc.robot.Navigation.TargetProgressWatchdog;
import frc.robot.Data.Constants;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.WorldState;
import frc.robot.Intelligence.WorldStateBuilder;
import frc.robot.Navigation.FieldMap;
import frc.robot.Subsystems.Intake.IntakeState;
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
    // Assigned in the constructor: the jitter stream name needs botId/isAlly,
    // which are not set when field initializers run.
    private final ContactWatchdog contactWatchdog;
    private final TargetProgressWatchdog targetProgressWatchdog = new TargetProgressWatchdog();
    // Per-bot objective commitment. Owned here (not on the shared engine) so
    // this bot can never read another bot's decision.
    private final frc.robot.Intelligence.ObjectiveCommitment objectiveCommitment =
            new frc.robot.Intelligence.ObjectiveCommitment();

    // Per-bot fuel-target latch. Same ownership rule as objectiveCommitment and
    // for the same reason: state on the shared engine singleton leaked one bot's
    // decision into another's. Without this the cluster-weighted selector
    // oscillates between comparable pieces as the robot's heading changes, which
    // resets TargetProgressWatchdog's no-progress window every few ticks and
    // produces STALLED_CHURN -- a bot stalled for many seconds on a target that is
    // always seconds old, so the give-up timer never fires.
    private final frc.robot.Intelligence.FuelTargetMemory fuelTargetMemory =
            new frc.robot.Intelligence.FuelTargetMemory();

    // Stall watchdog
    private Pose2d lastActualPose = new Pose2d();
    private double stallDuration = 0.0;
    private boolean lastStallResult = false;
    private double lastStallEvalTimestamp = -1.0;

    // Stationary-arm harvester watchdog (prevents contested-target deadlock inside ARRIVED_M)
    public static final double HARVEST_ARRIVAL_ABANDON_SEC = 1.8;
    private double harvestArrivalHoldSec = 0.0;
    private int lastHarvestFuelCount = 0;
    private Translation2d lastHarvestTargetPos = null;

    // Score-rig instrumentation (read-only; no behaviour depends on it).
    private final BotMatchMetrics matchMetrics = new BotMatchMetrics();

    public AIRobotInstance(int botId, Pose2d queuingPose, Archetype defaultArchetype) {
        this(botId, queuingPose, defaultArchetype, false);
    }

    public AIRobotInstance(int botId, Pose2d queuingPose, Archetype defaultArchetype, boolean isAlly) {
        this.botId = botId;
        this.queuingPose = queuingPose;
        this.archetype = defaultArchetype != null ? defaultArchetype : Archetype.AUTONOMOUS_CYCLER;
        this.isAlly = isAlly;
        // Per-robot jitter stream so one bot's deadlock recovery draws cannot
        // shift another's.
        this.contactWatchdog = new ContactWatchdog(
                "watchdog:" + (isAlly ? "Ally" + (botId - 100) : "Bot" + botId));

        String prefix = isAlly ? SimDashboardKeys.allyPrefix(botId - 100)
                : SimDashboardKeys.botPrefix(botId);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultString(
                prefix + SimDashboardKeys.SUFFIX_ARCHETYPE, this.archetype.name());

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
        reset(queuingPose, AIRobotSim.INITIAL_HELD_BALLS);
    }

    /** Applies a scenario robot's archetype, start pose, and preload. */
    public void reset(TrainingMatchScenario.RobotConfig robotConfig) {
        if (robotConfig == null) {
            throw new IllegalArgumentException("robotConfig must not be null");
        }
        setArchetype(robotConfig.archetype());
        reset(robotConfig.startingPose(), robotConfig.preloadFuel());
    }

    public void setFuelCount(int fuelCount) {
        if (fuelCount < 0 || fuelCount > Constants.IntakeConstants.MAX_HELD_BALLS) {
            throw new IllegalArgumentException("fuelCount is outside the robot hopper capacity");
        }
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(fuelCount);
        }
    }

    /** Resets the bot to scenario-provided initial conditions. */
    public void reset(Pose2d startingPose, int preloadFuel) {
        if (startingPose == null) {
            throw new IllegalArgumentException("startingPose must not be null");
        }
        if (preloadFuel < 0 || preloadFuel > Constants.IntakeConstants.MAX_HELD_BALLS) {
            throw new IllegalArgumentException("preloadFuel is outside the robot hopper capacity");
        }
        setRobotPose(startingPose);
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(preloadFuel);
            intakeSimulation.stopIntake();
        }
        scoreCount = 0;
        stallDuration = 0.0;
        lastShotTimestamp = 0.0;
        contactWatchdog.reset();
        targetProgressWatchdog.reset();
        objectiveCommitment.reset();
        fuelTargetMemory.reset();
        trajectoryController.reset();
        matchMetrics.reset();
        harvestArrivalHoldSec = 0.0;
        lastHarvestFuelCount = preloadFuel;
        lastHarvestTargetPos = null;
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

        // 0. Update Archetype from chooser or dashboard if modified.
        //
        // A training scenario is authoritative: it already assigned every bot an
        // archetype via configureTrainingScenario -> reset(RobotConfig), and
        // re-deriving from the dashboard here silently discarded that for 4 of the
        // 6 headless 3v3 robots. With no operator present the choosers return null
        // and the SmartDashboard string defaults won instead, so bots 1 and 2
        // always played DEFENSE_BULLY / ADAPTIVE_COMPETITOR and allies 1 and 2
        // always played AUTONOMOUS_CYCLER / ADAPTIVE_COMPETITOR -- the scenario's
        // TACTICAL_DEFENDER never appeared in a single headless match. The live
        // archetype is what the score rig's role-aware guardrails key on, so a
        // drifting label would also make those gates unstable.
        AIRobotSim aiSim = AIRobotSim.getInstance();
        boolean scenarioActive = aiSim != null && aiSim.isTrainingScenarioActive();
        if (!scenarioActive) {
            if (isAlly) {
                int allyIndex = botId - 100;
                if (allyIndex == 1 && aiSim != null) {
                    this.archetype = aiSim.getAlly1Archetype();
                } else if (allyIndex == 2 && aiSim != null) {
                    this.archetype = aiSim.getAlly2Archetype();
                } else {
                    String archStr = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString(
                            SimDashboardKeys.allyPrefix(allyIndex) + SimDashboardKeys.SUFFIX_ARCHETYPE,
                            archetype.name());
                    Archetype selectedArch = Archetype.fromString(archStr);
                    if (selectedArch != null) {
                        this.archetype = selectedArch;
                    }
                }
            } else {
                if (botId == 1 && aiSim != null) {
                    this.archetype = aiSim.getBot1Archetype();
                } else if (botId == 2 && aiSim != null) {
                    this.archetype = aiSim.getBot2Archetype();
                } else {
                    String archStr = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString(
                            SimDashboardKeys.botPrefix(botId) + SimDashboardKeys.SUFFIX_ARCHETYPE,
                            archetype.name());
                    Archetype selectedArch = Archetype.fromString(archStr);
                    if (selectedArch != null) {
                        this.archetype = selectedArch;
                    }
                }
            }
        }

        // 1. Build immutable WorldState snapshot. Defensive bots track their
        // assigned mark as the opponent; everyone else defaults to the player.
        WorldState worldState;
        Pose2d opponentPose;
        ChassisSpeeds opponentVel;
        if (markPose != null) {
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

        // 2. Evaluate unified Jev policy (stateless System 2 + System 1).
        // Fuel the watchdog has abandoned this match is excluded so a selector
        // cannot re-pick an unreachable piece every cycle.
        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(
                worldState, knowledge, archetype,
                "Sim/" + (isAlly ? "Alliance/Ally" + (botId - 100) : "Opponents/Bot" + botId),
                targetProgressWatchdog.blockedPoints(), objectiveCommitment, fuelTargetMemory);
        String intentPrefix = isAlly ? "Alliance/Ally" + (botId - 100) : "Opponents/Bot" + botId;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
                intentPrefix + "/NextIntent", intent.plan().nextObjective().name());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                intentPrefix + "/TimeToTransitionSec", intent.plan().timeToTransitionSec());

        // 3. Compute drive trajectory speeds
        currentTargetPose = intent.navigationTarget();
        currentAIStateDetail = intent.objective().name() + " (" + intent.rationale() + ")";
        boolean stalled = isStalled();
        currentTargetSpeeds = trajectoryController.calculate(
                currentPose, currentTargetSpeeds, currentTargetPose, maxSpeed, stalled, true);

        // Track pinning against the assigned mark (or the player by default) -
        // only for opponent bots. RefereeSim scores the actual foul; this drives
        // the backoff maneuver. Unified through ContactWatchdog.
        Pose2d pinReference = (markPose != null && archetype.isDefensive())
                ? markPose
                : ((peerRobotPoses != null && !peerRobotPoses.isEmpty()) ? peerRobotPoses.get(0) : null);

        if (isAlly) {
            // Allies get the geometry escape only. The full update also arms a
            // forced 3-foot G418 backoff from stall + proximity, which would be a
            // rule violation for a bot that is not pinning an opponent.
            contactWatchdog.updateUnstickOnly(currentVel, currentTargetSpeeds, 0.02);
        } else {
            double nearestForPin = pinReference != null
                    ? currentPose.getTranslation().getDistance(pinReference.getTranslation())
                    : Double.MAX_VALUE;
            contactWatchdog.update(
                    currentPose,
                    currentVel,
                    currentTargetSpeeds,
                    0.0,
                    0.0,
                    stalled ? 30.0 : 0.0,
                    nearestForPin,
                    pinReference,
                    0.02);

            if (contactWatchdog.isForcedBackoffActive() && archetype.isDefensive() && pinReference != null) {
                Pose2d backoff = contactWatchdog.getBackOffTarget(currentPose, pinReference);
                currentTargetPose = backoff;
                currentTargetSpeeds = trajectoryController.calculate(
                        currentPose, currentTargetSpeeds, currentTargetPose, maxSpeed, false, false);
                currentAIStateDetail = String.format("PIN_RULE_BACKOFF (%.1fs)", contactWatchdog.getBackoffRemainingSec());
            }
        }

        // Single-owner drive arbitration (Phase 1): the trajectory speeds get
        // exactly one peer correction per tick — deadlock recovery, trench
        // yield, or soft separation — never a sum. Previously the separation
        // nudge was added first and then scaled again by the deadlock recovery,
        // so two corrections fought over one command. See resolvePreProgressCommand.
        boolean inTrenchCorridor = FieldMap.Trenches.isLowClearance(currentPose.getTranslation());
        double nearestPeerDist = Double.MAX_VALUE;
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double d = currentPose.getTranslation().getDistance(peerPose.getTranslation());
                if (d > 0.05 && d < nearestPeerDist) nearestPeerDist = d;
            }
        }
        ContactWatchdog.Resolution deadlock = contactWatchdog.updateDeadlockOnly(
                stalled, nearestPeerDist, 0.02, inTrenchCorridor);

        boolean trenchCoolingYield = inTrenchCorridor
                && contactWatchdog.isDeadlockCooling()
                && nearestPeerDist < ContactWatchdog.PROXIMITY_M;
        ResolvedDrive resolved = resolvePreProgressCommand(
                currentTargetSpeeds, currentPose, peerRobotPoses,
                deadlock, trenchCoolingYield);
        currentTargetSpeeds = resolved.speeds();
        if (resolved.detail() != null) {
            currentAIStateDetail = resolved.detail();
        }

        // Peer-independent geometry escape, applied after the peer corrections so
        // it is a true single-owner override rather than a second correction
        // summed onto this tick. Deadlock cannot cover this case: it only
        // accumulates when a peer is within PROXIMITY_M, so a bot wedged alone
        // against a hub core, ramp, trench wall, or tower post has no peer path
        // to be rescued by. Allies previously skipped the whole watchdog update,
        // so they never even armed this.
        //
        // Priority order, highest first: rule-mandated G418 backoff (inside
        // contactWatchdog.update) > this geometry escape > peer deadlock/yield/
        // separation > raw trajectory. The target-unreachable escape below is
        // peer-independent too and keeps its existing later position in the
        // pipeline; it needs the nav target rather than just a stall signal.
        if (contactWatchdog.isPirouetteActive()) {
            currentTargetSpeeds = contactWatchdog.applyUnstickOnly(currentTargetSpeeds);
            currentAIStateDetail = "STATIC_UNSTICK";
        }

        // Unreachable-target recovery (peer-independent). ContactWatchdog only
        // fires near a peer, so a bot that drives alone into an unreachable
        // fuel target used to hold position for the rest of the match.
        String progressPrefix = (isAlly ? "AI_Telemetry/Ally" + (botId - 100) : "AI_Telemetry/Bot" + botId) + "/";
        // currentVel is the measured chassis velocity, which is what lets the
        // watchdog recognise a physically pinned robot even while the Jev
        // selector churns between fuel pieces. Passing only the commanded speeds
        // made "no progress" depend entirely on the target holding still.
        TargetProgressWatchdog.Result progress = targetProgressWatchdog.update(
                currentPose, currentTargetSpeeds, currentVel, currentTargetPose, 0.02);

        // Stationary-arm harvester watchdog: if arrived inside ARRIVED_M while intaking,
        // but no fuel is being collected (e.g. peer wedging, physical obstacle block),
        // abandon the target after HARVEST_ARRIVAL_ABANDON_SEC so the bot escapes and retargets.
        boolean isHarvesting = (intent.intakeCommand() == IntakeState.INTAKING);
        double distToTarget = currentTargetPose != null
                ? currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                : Double.MAX_VALUE;

        if (isHarvesting && distToTarget < TargetProgressWatchdog.ARRIVED_M && !progress.recovering()) {
            if (lastHarvestTargetPos == null
                    || currentTargetPose.getTranslation().getDistance(lastHarvestTargetPos) > 0.5) {
                lastHarvestTargetPos = currentTargetPose.getTranslation();
                harvestArrivalHoldSec = 0.0;
                lastHarvestFuelCount = heldPieces;
            } else if (heldPieces > lastHarvestFuelCount) {
                harvestArrivalHoldSec = 0.0;
                lastHarvestFuelCount = heldPieces;
            } else {
                harvestArrivalHoldSec += 0.02;
                if (harvestArrivalHoldSec >= HARVEST_ARRIVAL_ABANDON_SEC) {
                    progress = targetProgressWatchdog.abandonTarget(
                            currentTargetPose.getTranslation(), currentPose);
                    harvestArrivalHoldSec = 0.0;
                    lastHarvestFuelCount = heldPieces;
                }
            }
        } else {
            harvestArrivalHoldSec = 0.0;
            lastHarvestFuelCount = heldPieces;
            if (distToTarget >= TargetProgressWatchdog.ARRIVED_M) {
                lastHarvestTargetPos = null;
            }
        }

        if (progress.recovering()) {
            currentTargetSpeeds.vxMetersPerSecond = progress.escapeVector().getX();
            currentTargetSpeeds.vyMetersPerSecond = progress.escapeVector().getY();
            currentTargetSpeeds.omegaRadiansPerSecond = 0.0;
            currentAIStateDetail = String.format("TARGET_UNREACHABLE (%.1fs)", progress.escapeRemainingSec());
            Logger.recordOutput(progressPrefix + "UnreachableRecovering", true);
        } else {
            Logger.recordOutput(progressPrefix + "UnreachableRecovering", false);
        }
        Logger.recordOutput(progressPrefix + "UnreachableNoProgressSec", progress.noProgressSec());

        // Plant-and-fire: aiming + solution ready but still moving means the
        // 80 ms volley would stream shots at transit speed (the dominant sim
        // miss source). Brake translation so the bot settles, then fires;
        // the aim override below keeps turning toward the Hub while planted.
        if (intent.triggerFeedKicker()
                && isShotSolutionReady(currentPose, botAllianceIsRed)
                && !isSettledForShot()) {
            currentTargetSpeeds.vxMetersPerSecond = 0.0;
            currentTargetSpeeds.vyMetersPerSecond = 0.0;
            currentAIStateDetail = "SETTLING_TO_SHOOT";
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
        double commandedSpeed = Math.hypot(
                currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
        Logger.recordOutput(prefix + "CommandedSpeed", commandedSpeed);
        Logger.recordOutput(prefix + "ActualPose", currentPose);
        Logger.recordOutput(prefix + "TargetPose", currentTargetPose);
        Logger.recordOutput(prefix + "Objective", intent.objective().name());
        Logger.recordOutput(prefix + "StateDetail", currentAIStateDetail);
        Logger.recordOutput(prefix + "HeldFuel", heldPieces);
        Logger.recordOutput(prefix + "Score", scoreCount);
        Logger.recordOutput(prefix + "Confidence", intent.confidence());
        Logger.recordOutput(prefix + "Archetype", archetype.name());

        String dashPrefix = (isAlly ? SimDashboardKeys.allyPrefix(botId - 100)
                : SimDashboardKeys.botPrefix(botId)) + "/";
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumberArray(dashPrefix + "Pose", new double[] { currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees() });
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumberArray(dashPrefix + "TargetPose", new double[] { currentTargetPose.getX(), currentTargetPose.getY(), currentTargetPose.getRotation().getDegrees() });
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "Objective", intent.objective().name());
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "StateDetail", currentAIStateDetail);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(dashPrefix + "Fuel", heldPieces);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(dashPrefix + "Score", scoreCount);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(dashPrefix + "Stalled", stalled);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(dashPrefix + "CommandedSpeed", commandedSpeed);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(dashPrefix + "Archetype", archetype.name());

        // 6. Score-rig instrumentation. Sampled last so it sees the settled state.
        // Uses the same `stalled` flag that drives the production watchdogs, so the
        // metric tracks exactly the condition that provokes recoveries. "Active"
        // means on the field (queuing poses sit at y = -5) and DS-enabled, so the
        // auto/teleop disabled gap is not scored as standing still.
        boolean inRecovery = contactWatchdog.isDeadlockRecovering()
                || contactWatchdog.isForcedBackoffActive()
                || contactWatchdog.isPirouetteActive()
                || progress.recovering();
        // Phase 0 stall taxonomy: attribute every stalled sample to the
        // recovery that owns it, so headless replays can separate APF/deadlock
        // stalls from trench-yield holds from unreachable-target freezes.
        // Null while flowing attributes nothing.
        BotMatchMetrics.StallCause stallCause = classifyStallCause(
                new RecoveryState(
                        contactWatchdog.isPirouetteActive(),
                        progress.recovering(),
                        deadlock.recovering(),
                        inTrenchCorridor,
                        trenchCoolingYield,
                        stalled),
                currentTargetPose != null
                        ? currentPose.getTranslation().getDistance(
                                currentTargetPose.getTranslation())
                        : Double.MAX_VALUE,
                targetProgressWatchdog.getTrackedTargetAgeSec(),
                matchMetrics.getCurrentStallSec());
        boolean onField = currentPose.getX() > 0.0 && currentPose.getY() > 0.0;
        matchMetrics.sample(currentPose, stalled, inRecovery,
                onField && edu.wpi.first.wpilibj.DriverStation.isEnabled(),
                edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), stallCause);
        Logger.recordOutput(prefix + "StallCause",
                stallCause == null ? "flowing" : stallCause.name());
        Logger.recordOutput(prefix + "DriveCorrection", resolved.correction().name());
    }

    /**
     * Which pre-progress peer correction owns the command this tick.
     * Exactly one is applied by {@link #resolvePreProgressCommand} — never a sum.
     */
    public enum DriveCorrection {
        NONE,
        SEPARATION,
        DEADLOCK,
        TRENCH_YIELD
    }

    /** Single-owner result of the pre-progress peer arbitration. */
    public record ResolvedDrive(ChassisSpeeds speeds, DriveCorrection correction, String detail) {}

    /** Post-recovery trench yield scale: hold back, do not creep. */
    public static final double TRENCH_YIELD_SCALE = 0.2;

    /**
     * Phase 0b: splits an otherwise-unattributed stall by the watchdog state
     * that explains why no recovery fired. Pure function for unit tests.
     *
     * @param distToTargetM   distance from the measured pose to the nav target
     *                        this tick ({@code Double.MAX_VALUE} when none)
     * @param trackedAgeSec   {@link TargetProgressWatchdog#getTrackedTargetAgeSec()}
     * @param currentStallSec uninterrupted stalled seconds so far
     */
    public static BotMatchMetrics.StallCause classifyUnattributedStall(
            double distToTargetM, double trackedAgeSec, double currentStallSec) {
        if (distToTargetM < TargetProgressWatchdog.ARRIVED_M) {
            return BotMatchMetrics.StallCause.STALLED_ARRIVED;
        }
        if (trackedAgeSec >= 0.0
                && trackedAgeSec < TargetProgressWatchdog.GIVEUP_SEC
                && currentStallSec >= TargetProgressWatchdog.GIVEUP_SEC) {
            return BotMatchMetrics.StallCause.STALLED_CHURN;
        }
        return BotMatchMetrics.StallCause.STALLED_OTHER;
    }

    /**
     * Which recovery state this tick, as read by the stall taxonomy.
     *
     * <p>A record rather than six positional booleans so the call site stays
     * readable and the argument order cannot silently swap.
     */
    public record RecoveryState(
            boolean staticUnstickActive,
            boolean targetUnreachable,
            boolean deadlockRecovering,
            boolean inTrench,
            boolean trenchYieldActive,
            boolean stalled) {
    }

    /**
     * Attributes a tick to the recovery that owns it, or {@code null} while
     * flowing (which attributes nothing).
     *
     * <p>Order mirrors the drive pipeline exactly, highest priority first, because
     * the last correction applied is the one the robot actually executed and
     * therefore the one that explains the stall:
     * <ol>
     *   <li>unreachable-target escape — applied last, so it wins outright,</li>
     *   <li>static-geometry unstick — applied after the peer corrections,</li>
     *   <li>deadlock (trench-aware),</li>
     *   <li>trench yield,</li>
     *   <li>otherwise the unattributed buckets.</li>
     * </ol>
     *
     * <p>Extracted as a pure function for the same reason
     * {@link #classifyUnattributedStall} is one: inlined in {@code update} it
     * could not be unit-tested, and that is how the missing
     * {@link BotMatchMetrics.StallCause#STATIC_UNSTICK} branch went unnoticed in
     * the first place — the unstick recovery was counted as "recovering" but had
     * no cause of its own, so all of its samples landed in
     * {@code STALLED_OTHER} and the newest recovery was the one the report could
     * not distinguish.
     */
    public static BotMatchMetrics.StallCause classifyStallCause(
            RecoveryState s, double distToTargetM, double trackedAgeSec, double currentStallSec) {
        if (s.targetUnreachable()) {
            return BotMatchMetrics.StallCause.TARGET_UNREACHABLE;
        }
        if (s.staticUnstickActive()) {
            return BotMatchMetrics.StallCause.STATIC_UNSTICK;
        }
        if (s.deadlockRecovering()) {
            return s.inTrench()
                    ? BotMatchMetrics.StallCause.DEADLOCK_TRENCH
                    : BotMatchMetrics.StallCause.DEADLOCK_OPEN;
        }
        if (s.trenchYieldActive()) {
            return BotMatchMetrics.StallCause.TRENCH_YIELD;
        }
        if (s.stalled()) {
            return classifyUnattributedStall(distToTargetM, trackedAgeSec, currentStallSec);
        }
        return null;
    }

    /** Peer radius for the soft separation nudge (m). */
    public static final double SEPARATION_RADIUS_M = 1.10;

    /** Peak soft separation nudge speed (m/s at contact). */
    public static final double SEPARATION_NUDGE_MPS = 1.5;

    /**
     * Single-owner peer arbitration over the trajectory speeds.
     *
     * <p>Priority: deadlock recovery &gt; trench yield &gt; soft separation.
     * The winner is applied alone: a trench reverse-out at −0.8x would flip an
     * away-from-peer nudge into a toward-peer push if the two were summed, and
     * an open-field jink plus a radial nudge double-count the same peer.
     * Pure function of its inputs (no watchdog or dashboard reads) so it is
     * unit-testable without a drive simulation.
     *
     * @param trajectorySpeeds speeds from {@link TrajectoryController} (not mutated)
     * @param currentPose      measured robot pose (for the jink frame + nudge geometry)
     * @param peerRobotPoses   peer poses (null/empty = no separation possible)
     * @param deadlock         latest deadlock resolution for this bot
     * @param trenchCoolingYield true while the post-recovery trench hold applies
     * @return resolved speeds plus which correction won (detail null when none did)
     */
    public static ResolvedDrive resolvePreProgressCommand(
            ChassisSpeeds trajectorySpeeds,
            Pose2d currentPose,
            List<Pose2d> peerRobotPoses,
            ContactWatchdog.Resolution deadlock,
            boolean trenchCoolingYield) {
        ChassisSpeeds base = (trajectorySpeeds != null) ? trajectorySpeeds : new ChassisSpeeds();
        Pose2d pose = (currentPose != null) ? currentPose : new Pose2d();

        if (deadlock != null && deadlock.recovering()) {
            Translation2d jinkField = new Translation2d(0, deadlock.lateralJink())
                    .rotateBy(pose.getRotation());
            ChassisSpeeds out = new ChassisSpeeds(
                    base.vxMetersPerSecond * deadlock.forwardScale() + jinkField.getX(),
                    base.vyMetersPerSecond * deadlock.forwardScale() + jinkField.getY(),
                    base.omegaRadiansPerSecond);
            return new ResolvedDrive(out, DriveCorrection.DEADLOCK, "DEADLOCK_RECOVERY");
        }

        if (trenchCoolingYield) {
            // Hold back while the peer is still close so a head-on pair doesn't
            // re-enter in lockstep the moment recovery ends. Jittered cooldowns
            // desynchronize the pair; the shorter one proceeds first.
            ChassisSpeeds out = new ChassisSpeeds(
                    base.vxMetersPerSecond * TRENCH_YIELD_SCALE,
                    base.vyMetersPerSecond * TRENCH_YIELD_SCALE,
                    base.omegaRadiansPerSecond);
            return new ResolvedDrive(out, DriveCorrection.TRENCH_YIELD, "TRENCH_YIELD");
        }

        // Soft peer separation (avoids jamming and scrums between multi-bots).
        // Only reached when no recovery owns the tick.
        double nudgeX = 0.0;
        double nudgeY = 0.0;
        boolean nudged = false;
        if (peerRobotPoses != null) {
            for (Pose2d peerPose : peerRobotPoses) {
                if (peerPose == null) continue;
                double dist = pose.getTranslation().getDistance(peerPose.getTranslation());
                if (dist > 0.05 && dist < SEPARATION_RADIUS_M) {
                    Translation2d diff = pose.getTranslation().minus(peerPose.getTranslation());
                    double scale = (SEPARATION_RADIUS_M - dist) / SEPARATION_RADIUS_M;
                    Translation2d nudge = diff.div(dist).times(scale * SEPARATION_NUDGE_MPS);
                    nudgeX += nudge.getX();
                    nudgeY += nudge.getY();
                    nudged = true;
                }
            }
        }
        if (nudged) {
            ChassisSpeeds out = new ChassisSpeeds(
                    base.vxMetersPerSecond + nudgeX,
                    base.vyMetersPerSecond + nudgeY,
                    base.omegaRadiansPerSecond);
            return new ResolvedDrive(out, DriveCorrection.SEPARATION, null);
        }
        return new ResolvedDrive(
                new ChassisSpeeds(
                        base.vxMetersPerSecond, base.vyMetersPerSecond, base.omegaRadiansPerSecond),
                DriveCorrection.NONE, null);
    }

    /** Score-rig instrumentation for this bot. Never null. */
    public BotMatchMetrics getMatchMetrics() {
        return matchMetrics;
    }

    public boolean canShootNow(Pose2d currentPose, boolean botAllianceIsRed) {
        if (!isShotSolutionReady(currentPose, botAllianceIsRed)) return false;
        if (!isSettledForShot()) {
            String prefix = (isAlly ? "AI_Telemetry/Ally" + (botId - 100) : "AI_Telemetry/Bot" + botId) + "/";
            Logger.recordOutput(prefix + "ShootGate", "not_settled");
            return false;
        }

        double now = Timer.getFPGATimestamp();
        return (now - lastShotTimestamp >= 0.08);
    }

    /**
     * Shot solution ignoring motion and refire timing: hub active, fuel aboard,
     * legal location, clear lane, aimed within 8°. Used both by
     * {@link #canShootNow} and by the plant-and-fire brake (which holds
     * position while the settle gate is the only blocker).
     */
    public boolean isShotSolutionReady(Pose2d currentPose, boolean botAllianceIsRed) {
        if (AIRobotSim.getInstance() == null) { logShootGate("no_sim", currentPose, 0.0); return false; }
        if (!AIRobotSim.getInstance().isHubActiveForAlliance(botAllianceIsRed)) {
            logShootGate("hub_inactive", currentPose, 0.0);
            return false;
        }
        if (intakeSimulation == null || intakeSimulation.getGamePiecesAmount() <= 0) {
            logShootGate("no_fuel", currentPose, 0.0);
            return false;
        }
        if (!AIRobotSim.getInstance().isValidShootingLocation(currentPose, botAllianceIsRed)) {
            logShootGate("bad_location", currentPose, 0.0);
            return false;
        }

        Translation2d hub = FieldMap.Hubs.getHubLocation2d(botAllianceIsRed);
        if (AIRobotSim.getInstance().isShootingLaneBlocked(currentPose, hub)) {
            logShootGate("lane_blocked", currentPose, 0.0);
            return false;
        }

        Rotation2d aimAngle = hub.minus(currentPose.getTranslation()).getAngle();
        double headingErr = Math.abs(currentPose.getRotation().minus(aimAngle).getRadians());
        if (headingErr > Math.toRadians(8.0)) {
            logShootGate("heading", currentPose, headingErr);
            return false;
        }
        logShootGate("ready", currentPose, headingErr);
        return true;
    }

    /**
     * Publishes why {@link #isShotSolutionReady} refused, so a headless replay can name
     * the blocking gate instead of leaving "full hopper, no score" unexplained. Also
     * records the settle gate, which is the other half of {@link #canShootNow}.
     */
    private void logShootGate(String reason, Pose2d pose, double headingErr) {
        String prefix = (isAlly ? "AI_Telemetry/Ally" + (botId - 100) : "AI_Telemetry/Bot" + botId) + "/";
        Logger.recordOutput(prefix + "ShootGate", reason);
        Logger.recordOutput(prefix + "ShootGateHeadingErrDeg",
                Math.toDegrees(headingErr));
    }

    // Fire only when planted: releasing at transit speed was the dominant sim
    // miss source (velocity compensation is approximate plus exit noise).
    public static final double SETTLE_SPEED_MPS = 0.80;
    public static final double SETTLE_OMEGA_RPS = 1.00;

    /** Pure speed check, unit-testable without a drive simulation. */
    public static boolean isSettledSpeeds(ChassisSpeeds fieldRelativeVel) {
        if (fieldRelativeVel == null) return true;
        double trans = Math.hypot(fieldRelativeVel.vxMetersPerSecond, fieldRelativeVel.vyMetersPerSecond);
        return trans <= SETTLE_SPEED_MPS && Math.abs(fieldRelativeVel.omegaRadiansPerSecond) <= SETTLE_OMEGA_RPS;
    }

    /** Measured-speed gate for firing. Null sim (tests) counts as settled. */
    public boolean isSettledForShot() {
        if (driveSimulation == null || driveSimulation.getDriveTrainSimulation() == null) return true;
        return isSettledSpeeds(
                driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative());
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

        // Spread is intentional (it models a real shooter) but must draw from
        // the scenario seed, not Math.random(), or the seed means nothing.
        java.util.Random rng = MatchDeterminism.random("shot:" + botId);
        double randomExitVelocity = exitVel * (1.0 + (rng.nextDouble() - 0.5) * 0.04);
        Rotation2d aimAngle = compensatedTarget.minus(botPos).getAngle();
        Rotation2d randomYaw = aimAngle.plus(Rotation2d.fromDegrees((rng.nextDouble() - 0.5) * 1.6));
        double randomPitch = theta + (rng.nextDouble() - 0.5) * 0.025;

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
            // Sorted, not the arena's HashSet: with overlapping pieces the
            // iteration order decided WHICH fuel filled the hopper, and
            // HashSet order is identity-hash based, so it varied per JVM run.
            var pieces = MatchDeterminism.fuelOnFieldSorted();
            if (pieces.isEmpty()) return;

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

        // Must agree with ContactWatchdog.STALL_CMD_SPEED_MIN: a mismatch left the
        // bot-local stall flag and the watchdog disagreeing, so pirouette/pin were
        // blind to stuck robots while deadlock still fired. 0.80 was too high because
        // TrajectoryController commands as little as 0.25 m/s.
        if (commandedSpeed > ContactWatchdog.STALL_CMD_SPEED_MIN
                && actualSpeed < ContactWatchdog.STALL_ACTUAL_SPEED_MAX) {
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

    public TargetProgressWatchdog getTargetProgressWatchdog() {
        return targetProgressWatchdog;
    }

    public double getHarvestArrivalHoldSec() {
        return harvestArrivalHoldSec;
    }
}
