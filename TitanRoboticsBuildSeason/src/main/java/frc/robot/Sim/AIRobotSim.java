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
import frc.robot.Navigation.ContactWatchdog;
import frc.robot.Navigation.DynamicRouter;
import frc.robot.Navigation.StaticPathfinder;
import frc.robot.Navigation.TrajectoryController;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.AutonConstants;
import frc.robot.Navigation.FieldMap;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Telemetry.Dashboard;
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
        ADAPTIVE_COMPETITOR("Adaptive Match Competitor"),
        CHOREO_PATH("Choreo Path Following"),
        MANUAL_2_PLAYER("Manual 2-Player (Port 2)");

        public final String displayName;

        AIMode(String displayName) {
            this.displayName = displayName;
        }

        public static AIMode fromString(String name) {
            if (name == null)
                return TACTICAL_DEFENSE;
            for (AIMode m : values()) {
                if (m.name().equalsIgnoreCase(name) || m.displayName.equalsIgnoreCase(name)) {
                    return m;
                }
            }
            return TACTICAL_DEFENSE;
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

    public static final Pose2d[] ALLY_QUEUING_POSITIONS = new Pose2d[] {
            new Pose2d(2.5, -5, new Rotation2d()),
            new Pose2d(3.0, -5, new Rotation2d())
    };

    /** Fuel each sim robot (player, opponents, allies) carries at match/reset start. */
    public static final int INITIAL_HELD_BALLS = 8;

    // Phase 5: Bot 0 is opponents.get(0), an AIRobotInstance like every other bot.
    // The legacy driveSimulation/intakeSimulation fields below alias its sim objects
    // so RefereeSim, WorldStateBuilder, and existing tests keep working unchanged.
    private final AIRobotInstance bot0Instance;
    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queuingPose;
    private IntakeSimulation intakeSimulation;

    private Optional<Trajectory<SwerveSample>> trajectory = Optional.empty();
    private final Timer pathTimer = new Timer();
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final TrajectoryController aiTrajectoryController;

    // Legacy Bot-0 contact watchdogs (superseded by the shared pipeline inside
    // AIRobotInstance; kept only so reset() stays symmetric until Phase 7 test moves).
    @Deprecated
    private final ContactWatchdog bot0ContactWatchdog = new ContactWatchdog();

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
    private final SendableChooser<Archetype> bot1ArchetypeChooser = new SendableChooser<>();
    private final SendableChooser<Archetype> bot2ArchetypeChooser = new SendableChooser<>();
    private final SendableChooser<Archetype> ally1ArchetypeChooser = new SendableChooser<>();
    private final SendableChooser<Archetype> ally2ArchetypeChooser = new SendableChooser<>();

    private CyclerPhase cyclerPhase = CyclerPhase.HUNT_FUEL;
    private final Timer cyclerTimer = new Timer();

    private Pose2d lastActualPose = new Pose2d();
    private double lastPoseTimestamp = -1.0;
    private double stallDuration = 0.0;
    private double lastCommandedSpeed = 0.0;
    private boolean lastStallResult = false;
    private double lastStallEvalTimestamp = -1.0;

    // Legacy Bot-0 deadlock helper (superseded by AIRobotInstance pipeline; see above).
    @Deprecated
    private final ContactWatchdog bot0DeadlockContact = new ContactWatchdog();

    // Immutable latched target during transit/staging
    private Pose2d latchedShootTarget = null;

    // Periodic diagnostic console printer
    private double lastConsoleDumpTime = 0.0;

    // Multi-robot sparring pool
    private final List<AIRobotInstance> additionalBots = new ArrayList<>();
    private final List<AIRobotInstance> allyBots = new ArrayList<>();
    private AIRobotInstance trainingBluePrimaryBot;
    private volatile TrainingMatchScenario trainingScenario;

    public static synchronized AIRobotSim getInstance() {
        if (instance == null) {
            instance = new AIRobotSim();
        }
        return instance;
    }

    private AIRobotSim() {
        this.queuingPose = ROBOT_QUEUING_POSITIONS[0];

        // Bot 0 owns its sim objects like every other AIRobotInstance.
        this.bot0Instance = new AIRobotInstance(0, queuingPose, Archetype.AUTONOMOUS_CYCLER);
        this.driveSimulation = bot0Instance.getDriveSimulation();
        this.intakeSimulation = bot0Instance.getIntakeSimulation();

        this.xController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI,
                AutonConstants.AUTO_DRIVE_KD);
        this.yController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI,
                AutonConstants.AUTO_DRIVE_KD);

        var config = SwerveBase.getInstance().getSwerveController().config;
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.aiTrajectoryController = new TrajectoryController(
                new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d));

        this.defenseController = new XboxController(2);

        setupArchetypeChoosers();

        SubsystemManager.registerSubsystem(this);
    }

    public void setTrajectory(String pathName) {
        this.trajectory = Choreo.loadTrajectory(pathName);
        pathTimer.restart();
    }

    public void setRobotPose(Pose2d pose) {
        bot0Instance.setRobotPose(pose);
    }

    /** Enables or clears scenario-controlled team rosters and resets bot state. */
    public synchronized void configureTrainingScenario(TrainingMatchScenario scenario) {
        trainingScenario = scenario;
        if (scenario != null && trainingBluePrimaryBot == null) {
            trainingBluePrimaryBot = new AIRobotInstance(100, ROBOT_QUEUING_POSITIONS[0],
                    scenario.bluePlayerRobot().archetype(), true);
        }
        reset();
        if (scenario == null) return;

        trainingBluePrimaryBot.reset(scenario.bluePlayerRobot());

        for (int i = 1; i < scenario.redOpponentRobots().size(); i++) {
            if (additionalBots.size() < i) {
                additionalBots.add(new AIRobotInstance(i, ROBOT_QUEUING_POSITIONS[i],
                        scenario.redOpponentRobots().get(i).archetype()));
            }
        }
        for (int i = 1; i < scenario.blueRobots().size(); i++) {
            int allyIndex = i - 1;
            if (allyBots.size() <= allyIndex) {
                allyBots.add(new AIRobotInstance(100 + i, ALLY_QUEUING_POSITIONS[allyIndex],
                        scenario.blueRobots().get(i).archetype(), true));
            }
        }

        bot0Instance.reset(scenario.redOpponentRobots().get(0));
        for (int i = 1; i < scenario.redOpponentRobots().size(); i++) {
            additionalBots.get(i - 1).reset(scenario.redOpponentRobots().get(i));
        }
        for (int i = 1; i < scenario.blueRobots().size(); i++) {
            allyBots.get(i - 1).reset(scenario.blueRobots().get(i));
        }
        lastSpawnedPlayerIsRed = AllianceFlipUtil.isRedAlliance();
        wasOpponentEnabled = true;
    }

    public void reset() {
        wasOpponentEnabled = false;
        lastSpawnedPlayerIsRed = false;
        pathTimer.restart();
        cyclerTimer.restart();
        cyclerPhase = CyclerPhase.HUNT_FUEL;
        aiScoreCount = 0;
        aiTrajectoryController.reset();
        bot0DeadlockContact.reset();
        lastPoseTimestamp = -1.0;
        stallDuration = 0.0;
        lastCommandedSpeed = 0.0;
        currentTargetSpeeds = new ChassisSpeeds();
        lastRobotRelativeSpeeds = new ChassisSpeeds();
        lastStallResult = false;
        lastStallEvalTimestamp = -1.0;
        lastShotTimestamp = 0.0;
        bot0ContactWatchdog.reset();
        bot0Instance.reset();
        latchedShootTarget = null;
        try {
            var field = SwerveBase.getInstance().getField();
            field.getObject("OpponentBot0").setPoses(new ArrayList<>());
            field.getObject("OpponentTarget0").setPoses(new ArrayList<>());
            field.getObject("OpponentBot1").setPoses(new ArrayList<>());
            field.getObject("OpponentTarget1").setPoses(new ArrayList<>());
            field.getObject("OpponentBot2").setPoses(new ArrayList<>());
            field.getObject("OpponentTarget2").setPoses(new ArrayList<>());
            field.getObject("AllyBot1").setPoses(new ArrayList<>());
            field.getObject("AllyTarget1").setPoses(new ArrayList<>());
            field.getObject("AllyBot2").setPoses(new ArrayList<>());
            field.getObject("AllyTarget2").setPoses(new ArrayList<>());
        } catch (Exception ignored) {
        }
        for (var bot : additionalBots) {
            bot.reset();
        }
        for (var ally : allyBots) {
            ally.reset();
        }
        if (trainingBluePrimaryBot != null) {
            trainingBluePrimaryBot.reset();
        }
    }

    @Override
    public void simulationUpdate() {
        TrainingMatchScenario scenario = trainingScenario;
        boolean trainingMode = scenario != null;
        boolean opponentEnabled = trainingMode || Dashboard.isOpponentRobotEnabled();
        boolean manualDefenseMode = !trainingMode && Dashboard.is2PlayerDefenseEnabled();
        int allyCount = trainingMode
                ? scenario.blueAllyRobots().size()
                : (int) Math.max(0,
                        Math.min(2, SmartDashboard.getNumber("Simulation/AllyCount", Dashboard.getAllyCount())));
        boolean anyBotActive = opponentEnabled || allyCount > 0;

        if (!anyBotActive) {
            setRobotPose(queuingPose);
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, true);
            wasOpponentEnabled = false;
            currentAIStateDetail = "DISABLED";
            if (intakeSimulation != null && intakeSimulation.isRunning()) {
                intakeSimulation.stopIntake();
            }
            try {
                var field = SwerveBase.getInstance().getField();
                field.getObject("OpponentBot0").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget0").setPoses(new ArrayList<>());
                field.getObject("OpponentBot1").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget1").setPoses(new ArrayList<>());
                field.getObject("OpponentBot2").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget2").setPoses(new ArrayList<>());
                field.getObject("AllyBot1").setPoses(new ArrayList<>());
                field.getObject("AllyTarget1").setPoses(new ArrayList<>());
                field.getObject("AllyBot2").setPoses(new ArrayList<>());
                field.getObject("AllyTarget2").setPoses(new ArrayList<>());
            } catch (Exception ignored) {
            }
            for (var bot : additionalBots) {
                bot.reset();
            }
            for (var ally : allyBots) {
                ally.reset();
            }
            return;
        }

        // Lazy initialization of additional opponent bots
        int opponentCount = trainingMode
                ? scenario.redOpponentRobots().size()
                : (int) Math.max(1,
                        Math.min(3, SmartDashboard.getNumber("Simulation/OpponentCount", Dashboard.getOpponentCount())));
        try {
            if (opponentCount >= 2 && additionalBots.isEmpty()) {
                TrainingMatchScenario.RobotConfig config = trainingMode ? scenario.redOpponentRobots().get(1) : null;
                additionalBots.add(new AIRobotInstance(1,
                        config == null ? ROBOT_QUEUING_POSITIONS[1] : config.startingPose(),
                        config == null ? Archetype.DEFENSE_BULLY : config.archetype()));
            }
            if (opponentCount >= 3 && additionalBots.size() < 2) {
                TrainingMatchScenario.RobotConfig config = trainingMode ? scenario.redOpponentRobots().get(2) : null;
                additionalBots.add(new AIRobotInstance(2,
                        config == null ? ROBOT_QUEUING_POSITIONS[2] : config.startingPose(),
                        config == null ? Archetype.ADAPTIVE_COMPETITOR : config.archetype()));
            }
        } catch (Exception | NoClassDefFoundError e) {
            System.err.println("[AIRobotSim] Failed to spawn additional opponent bot: " + e.getMessage());
            e.printStackTrace();
        }

        // Lazy initialization of ally bots
        try {
            if (allyCount >= 1 && allyBots.isEmpty()) {
                TrainingMatchScenario.RobotConfig config = trainingMode ? scenario.blueAllyRobots().get(0) : null;
                allyBots.add(new AIRobotInstance(101,
                        config == null ? ALLY_QUEUING_POSITIONS[0] : config.startingPose(),
                        config == null ? Archetype.AUTONOMOUS_CYCLER : config.archetype(), true));
            }
            if (allyCount >= 2 && allyBots.size() < 2) {
                TrainingMatchScenario.RobotConfig config = trainingMode ? scenario.blueAllyRobots().get(1) : null;
                allyBots.add(new AIRobotInstance(102,
                        config == null ? ALLY_QUEUING_POSITIONS[1] : config.startingPose(),
                        config == null ? Archetype.ADAPTIVE_COMPETITOR : config.archetype(), true));
            }
        } catch (Exception | NoClassDefFoundError e) {
            System.err.println("[AIRobotSim] Failed to spawn ally bot: " + e.getMessage());
            e.printStackTrace();
        }

        boolean playerIsRed = AllianceFlipUtil.isRedAlliance();
        boolean opponentIsRed = !playerIsRed;

        // ── 1. Map Dashboard Mode to Jev Archetype for Bot 0 ──────────────────
        AIMode activeMode;
        if (manualDefenseMode) {
            activeMode = AIMode.MANUAL_2_PLAYER;
        } else if (aiModeChooser != null && aiModeChooser.getSelected() != null) {
            activeMode = aiModeChooser.getSelected();
        } else {
            String modeStr = SmartDashboard.getString("Simulation/Bot0/Archetype",
                    SmartDashboard.getString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name()));
            activeMode = AIMode.fromString(modeStr);
        }

        // Initial spawn / alliance flip positioning
        if (!wasOpponentEnabled || playerIsRed != lastSpawnedPlayerIsRed) {
            if (trainingMode) {
                List<TrainingMatchScenario.RobotConfig> redRoster = scenario.redOpponentRobots();
                for (int i = 0; i < redRoster.size(); i++) {
                    getOpponents().get(i).reset(redRoster.get(i));
                }
                List<TrainingMatchScenario.RobotConfig> blueAllies = scenario.blueAllyRobots();
                for (int i = 0; i < blueAllies.size(); i++) {
                    allyBots.get(i).reset(blueAllies.get(i));
                }
            } else if (opponentEnabled) {
                Pose2d initialPose = getOpponentSpawnPose(playerIsRed, activeMode);
                setRobotPose(initialPose);
                if (additionalBots.size() >= 1) {
                    additionalBots.get(0).setRobotPose(getOpponentSpawnPose(1, playerIsRed));
                }
                if (additionalBots.size() >= 2) {
                    additionalBots.get(1).setRobotPose(getOpponentSpawnPose(2, playerIsRed));
                }
            }
            if (allyBots.size() >= 1) {
                allyBots.get(0).setRobotPose(getAllySpawnPose(1, playerIsRed));
            }
            if (allyBots.size() >= 2) {
                allyBots.get(1).setRobotPose(getAllySpawnPose(2, playerIsRed));
            }
            pathTimer.restart();
            cyclerTimer.restart();
            wasOpponentEnabled = true;
            lastSpawnedPlayerIsRed = playerIsRed;
        }

        double opponentSpeedPercent = Dashboard.getOpponentSpeedPercent();
        double opponentSpeedScale = Math.max(0.20, Math.min(1.0, opponentSpeedPercent / 100.0));
        double maxSpeed = Constants.MAX_SPEED * opponentSpeedScale;
        double allySpeedScale = Math.max(0.20,
                Math.min(1.0, Dashboard.getAllySpeedPercent() / 100.0));
        double allyMaxSpeed = Constants.MAX_SPEED * allySpeedScale;

        Pose2d playerPose = trainingMode
                ? trainingBluePrimaryBot.getActualPose()
                : SwerveBase.getInstance().getPose();
        ChassisSpeeds playerSpeeds = trainingMode
                ? trainingBluePrimaryBot.getFieldVelocity()
                : SwerveBase.getInstance().getFieldVelocity();
        Pose2d currentPose = driveSimulation.getActualPoseInSimulationWorld();

        // ── 2. Handle Opponent Bot 0 Lifecycle ───────────────────────────────
        if (!opponentEnabled) {
            setRobotPose(queuingPose);
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, true);
            currentAIStateDetail = "DISABLED";
            if (intakeSimulation != null && intakeSimulation.isRunning()) {
                intakeSimulation.stopIntake();
            }
            try {
                var field = SwerveBase.getInstance().getField();
                field.getObject("OpponentBot0").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget0").setPoses(new ArrayList<>());
                field.getObject("OpponentBot1").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget1").setPoses(new ArrayList<>());
                field.getObject("OpponentBot2").setPoses(new ArrayList<>());
                field.getObject("OpponentTarget2").setPoses(new ArrayList<>());
            } catch (Exception ignored) {
            }
            for (var bot : additionalBots) {
                bot.reset();
            }
        } else {
            // Phase 5: Bot 0 ticks through the same AIRobotInstance pipeline as
            // every other sparring bot (Jev policy + trajectory + contact
            // arbitration + intake/shooter + obstacle registration).
            Archetype bot0Archetype;
            if (trainingMode) {
                bot0Archetype = scenario.redOpponentRobots().get(0).archetype();
            } else switch (activeMode) {
                case TACTICAL_DEFENSE:
                    bot0Archetype = Archetype.TACTICAL_DEFENDER;
                    break;
                case LEAD_PURSUIT_INTERCEPT:
                    bot0Archetype = Archetype.LEAD_PURSUIT_INTERCEPTOR;
                    break;
                case PINNING_BULLY:
                    bot0Archetype = Archetype.DEFENSE_BULLY;
                    break;
                case ADAPTIVE_COMPETITOR:
                    bot0Archetype = Archetype.ADAPTIVE_COMPETITOR;
                    break;
                case AUTONOMOUS_CYCLER:
                default:
                    bot0Archetype = Archetype.AUTONOMOUS_CYCLER;
                    break;
            }
            bot0Instance.setArchetype(bot0Archetype);
            SmartDashboard.putString("Simulation/Bot0/Archetype", bot0Archetype.displayName);

            if (activeMode == AIMode.MANUAL_2_PLAYER) {
                // Manual 2-player defense drives Bot 0 chassis directly.
                double x = 0.0, y = 0.0, rot = 0.0;
                if (DriverStation.isJoystickConnected(2)) {
                    x = -defenseController.getLeftY();
                    y = -defenseController.getLeftX();
                    rot = -defenseController.getRightX();
                    x = Math.abs(x) < 0.1 ? 0 : x;
                    y = Math.abs(y) < 0.1 ? 0 : y;
                    rot = Math.abs(rot) < 0.1 ? 0 : rot;
                }
                Pose2d bot0Pose = bot0Instance.getActualPose();
                ChassisSpeeds manualSpeeds = new ChassisSpeeds(x * maxSpeed, y * maxSpeed, rot * 5.0);
                bot0Instance.getDriveSimulation().runChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(manualSpeeds, bot0Pose.getRotation()),
                        new Translation2d(), false, true);
            } else {
                List<Pose2d> bot0Peers = new ArrayList<>();
                for (var b : additionalBots) {
                    bot0Peers.add(b.getActualPose());
                }
                for (var a : allyBots) {
                    bot0Peers.add(a.getActualPose());
                }
                bot0Peers.add(0, playerPose);
                if (trainingMode || bot0Archetype.isDefensive()) {
                    MarkCandidate bot0Mark = resolveDefensiveMark(false, bot0Instance.getActualPose());
                    SmartDashboard.putString("Simulation/Bot0/Mark", bot0Mark.label());
                    bot0Instance.update(bot0Peers, playerIsRed, maxSpeed, bot0Mark.pose(), bot0Mark.velocity());
                } else {
                    bot0Instance.update(bot0Peers, playerIsRed, maxSpeed);
                }
            }
            // Mirror shared-pipeline state into the legacy Bot-0 telemetry fields.
            currentPose = bot0Instance.getActualPose();
            currentTargetPose = bot0Instance.getCurrentTargetPose();
            currentTargetSpeeds = bot0Instance.getCurrentTargetSpeeds();
            currentAIStateDetail = bot0Instance.getCurrentAIStateDetail();
        }

        // ── 3. Fleet update: Bot 0 is opponents.get(0); every bot ticks the
        // shared AIRobotInstance pipeline over one field picture ─────────────
        List<AIRobotInstance> opponents = getOpponents();
        List<Pose2d> fieldPicture = collectActiveRobotPoses(playerPose, currentPose,
                opponentEnabled, opponentCount, allyCount, trainingMode);

        // Route Dashboard choosers straight to their instances.
        if (additionalBots.size() >= 1) {
            additionalBots.get(0).setArchetype(trainingMode
                    ? scenario.redOpponentRobots().get(1).archetype() : getBot1Archetype());
        }
        if (additionalBots.size() >= 2) {
            additionalBots.get(1).setArchetype(trainingMode
                    ? scenario.redOpponentRobots().get(2).archetype() : getBot2Archetype());
        }
        if (allyBots.size() >= 1) {
            allyBots.get(0).setArchetype(trainingMode
                    ? scenario.blueAllyRobots().get(0).archetype() : getAlly1Archetype());
        }
        if (allyBots.size() >= 2) {
            allyBots.get(1).setArchetype(trainingMode
                    ? scenario.blueAllyRobots().get(1).archetype() : getAlly2Archetype());
        }

        int oppCount = opponentEnabled ? Math.min(opponentCount, opponents.size()) : 0;
        for (int i = 1; i < oppCount; i++) {
            AIRobotInstance bot = opponents.get(i);
            List<Pose2d> botPeers = new ArrayList<>(fieldPicture);
            botPeers.remove(bot.getActualPose());
            if (trainingMode || bot.getArchetype().isDefensive()) {
                MarkCandidate mark = resolveDefensiveMark(false, bot.getActualPose());
                SmartDashboard.putString("Simulation/Bot" + bot.getBotId() + "/Mark", mark.label());
                bot.update(botPeers, playerIsRed, maxSpeed, mark.pose(), mark.velocity());
            } else {
                bot.update(botPeers, playerIsRed, maxSpeed);
            }
        }

        for (int i = opponentCount - 1; i < additionalBots.size(); i++) {
            additionalBots.get(i).reset();
        }

        SmartDashboard.putString("Simulation/Bot1/Archetype", getBot1Archetype().displayName);
        SmartDashboard.putString("Simulation/Bot2/Archetype", getBot2Archetype().displayName);

        // Execute Allies
        if (trainingMode) {
            List<Pose2d> primaryPeers = new ArrayList<>(fieldPicture);
            primaryPeers.remove(trainingBluePrimaryBot.getActualPose());
            MarkCandidate mark = resolveDefensiveMark(true, trainingBluePrimaryBot.getActualPose());
            trainingBluePrimaryBot.update(primaryPeers, playerIsRed, allyMaxSpeed,
                    mark.pose(), mark.velocity());
        }
        if (!allyBots.isEmpty()) {
            for (int i = 0; i < allyCount && i < allyBots.size(); i++) {
                AIRobotInstance ally = allyBots.get(i);
                List<Pose2d> allyPeers = new ArrayList<>(fieldPicture);
                allyPeers.remove(ally.getActualPose());
                if (trainingMode || ally.getArchetype().isDefensive()) {
                    MarkCandidate mark = resolveDefensiveMark(true, ally.getActualPose());
                    SmartDashboard.putString(
                            "Simulation/Ally" + (ally.getBotId() - 100) + "/Mark", mark.label());
                    ally.update(allyPeers, playerIsRed, allyMaxSpeed, mark.pose(), mark.velocity());
                } else {
                    ally.update(allyPeers, playerIsRed, allyMaxSpeed);
                }
            }

            for (int i = allyCount; i < allyBots.size(); i++) {
                allyBots.get(i).reset();
            }

            SmartDashboard.putString("Simulation/Ally1/Archetype", getAlly1Archetype().displayName);
            SmartDashboard.putString("Simulation/Ally2/Archetype", getAlly2Archetype().displayName);
        }
    }

    /**
     * Fleet view: Bot 0 is opponents.get(0), an AIRobotInstance like every
     * other sparring bot. Indices 1..2 are the additional opponent bots.
     */
    public List<AIRobotInstance> getOpponents() {
        List<AIRobotInstance> opponents = new ArrayList<>();
        opponents.add(bot0Instance);
        opponents.addAll(additionalBots);
        return Collections.unmodifiableList(opponents);
    }

    /**
     * Collects one shared field picture (player + active opponents + allies)
     * for the fleet tick.
     */
    private List<Pose2d> collectActiveRobotPoses(Pose2d playerPose, Pose2d bot0Pose,
            boolean opponentEnabled, int opponentCount, int allyCount, boolean trainingMode) {
        List<Pose2d> picture = new ArrayList<>();
        picture.add(trainingMode ? trainingBluePrimaryBot.getActualPose() : playerPose);
        if (opponentEnabled && bot0Pose != null && bot0Pose.getY() > 0.0) {
            picture.add(bot0Pose);
        }
        if (opponentEnabled) {
            for (int i = 0; i < opponentCount - 1 && i < additionalBots.size(); i++) {
                picture.add(additionalBots.get(i).getActualPose());
            }
        }
        for (int i = 0; i < allyCount && i < allyBots.size(); i++) {
            picture.add(allyBots.get(i).getActualPose());
        }
        return picture;
    }

    // ── Defensive mark selection ─────────────────────────────────────────
    // Defense bots used to always mark the player. Now each defensive bot marks
    // the most threatening enemy ball-carrier/scorer every tick, so allies can
    // pick up opponent bots and opponent defenders can switch to a hot ally.

    /** One markable enemy robot for defensive assignment. */
    public record MarkCandidate(String label, Pose2d pose, ChassisSpeeds velocity, int heldFuel, int scoredFuel) {}

    /** Threat weights: held fuel (immediate danger) > proven scoring > travel distance. */
    public static final double MARK_FUEL_WEIGHT = 3.0;
    public static final double MARK_SCORE_WEIGHT = 1.0;
    public static final double MARK_DISTANCE_WEIGHT = 0.25;

    /**
     * Pure threat selection over enemy candidates. Returns the fallback when the
     * list is empty (never null when fallback is non-null).
     */
    public static MarkCandidate selectMark(
            Pose2d defenderPose, List<MarkCandidate> candidates, MarkCandidate fallback) {
        MarkCandidate best = null;
        double bestThreat = Double.NEGATIVE_INFINITY;
        if (candidates != null && defenderPose != null) {
            for (MarkCandidate c : candidates) {
                if (c == null || c.pose() == null) {
                    continue;
                }
                double threat = MARK_FUEL_WEIGHT * c.heldFuel()
                        + MARK_SCORE_WEIGHT * c.scoredFuel()
                        - MARK_DISTANCE_WEIGHT
                                * defenderPose.getTranslation().getDistance(c.pose().getTranslation());
                if (threat > bestThreat) {
                    bestThreat = threat;
                    best = c;
                }
            }
        }
        return (best != null) ? best : fallback;
    }

    /**
     * Resolves which enemy a defensive bot should mark this tick.
     *
     * @param defenderIsAlly True for Blue bots (enemies = opponent bots), false for
     *            opponent bots (enemies = player and Blue bots)
     * @param defenderPose Current pose of the defending bot
     * @return Selected mark (player entry doubles as the fallback default)
     */
    public MarkCandidate resolveDefensiveMark(boolean defenderIsAlly, Pose2d defenderPose) {
        boolean trainingMode = trainingScenario != null;
        AIRobotInstance primaryBlue = trainingMode ? trainingBluePrimaryBot : null;
        Pose2d playerPose = primaryBlue != null
                ? primaryBlue.getActualPose() : SwerveBase.getInstance().getPose();
        ChassisSpeeds playerVelocity = primaryBlue != null
                ? primaryBlue.getFieldVelocity() : SwerveBase.getInstance().getFieldVelocity();
        int playerFuel = primaryBlue != null
                ? primaryBlue.getFuelCount() : GameSim.getInstance().getHeldBalls();
        int playerScore = primaryBlue != null
                ? primaryBlue.getScoreCount() : MatchScoreTracker.getInstance().getPlayerShotsScored();
        MarkCandidate playerEntry = new MarkCandidate(primaryBlue != null ? "Ally0" : "Player",
                playerPose, playerVelocity, playerFuel, playerScore);
        List<MarkCandidate> enemies = new ArrayList<>();
        if (!trainingMode) enemies.add(playerEntry);
        try {
            if (!defenderIsAlly) {
                if (primaryBlue != null) enemies.add(playerEntry);
                for (AIRobotInstance ally : allyBots) {
                    if (ally == null || ally.getActualPose() == null) {
                        continue;
                    }
                    enemies.add(new MarkCandidate("Ally" + (ally.getBotId() - 100),
                            ally.getActualPose(), ally.getFieldVelocity(),
                            ally.getFuelCount(), ally.getScoreCount()));
                }
            } else {
                enemies.add(new MarkCandidate("Bot0",
                        bot0Instance.getActualPose(),
                        bot0Instance.getFieldVelocity(),
                        bot0Instance.getFuelCount(),
                        bot0Instance.getScoreCount()));
            }
                for (AIRobotInstance bot : additionalBots) {
                    if (bot == null || bot.getActualPose() == null) {
                        continue;
                    }
                    enemies.add(new MarkCandidate("Bot" + bot.getBotId(),
                            bot.getActualPose(), bot.getFieldVelocity(),
                            bot.getFuelCount(), bot.getScoreCount()));
                }
        } catch (Exception ignored) {}
        return selectMark(defenderPose, enemies, playerEntry);
    }

    public ChassisSpeeds computeDriveToPoseSpeeds(Pose2d currentPose, Pose2d targetPose, double maxSpeed) {
        boolean isStalled = isStalled(currentPose);
        ChassisSpeeds speeds = aiTrajectoryController.calculate(
                currentPose,
                currentTargetSpeeds,
                targetPose,
                maxSpeed,
                isStalled,
                true);
        currentTargetSpeeds = speeds;
        return speeds;
    }

    public boolean isStalled() {
        return isStalled(driveSimulation.getActualPoseInSimulationWorld());
    }

    public boolean isStalled(Pose2d currentWorldPose) {
        double now = Timer.getFPGATimestamp();
        if (now - lastStallEvalTimestamp < 0.015 && lastStallEvalTimestamp > 0.0) {
            return lastStallResult;
        }

        double dt = lastStallEvalTimestamp > 0.0 ? (now - lastStallEvalTimestamp) : 0.02;
        lastStallEvalTimestamp = now;

        if (currentWorldPose == null) {
            currentWorldPose = driveSimulation.getActualPoseInSimulationWorld();
        }
        double actualMoveDist = currentWorldPose.getTranslation().getDistance(lastActualPose.getTranslation());
        double actualSpeed = actualMoveDist / dt;
        double commandedSpeed = Math.hypot(currentTargetSpeeds.vxMetersPerSecond,
                currentTargetSpeeds.vyMetersPerSecond);

        if (commandedSpeed > 0.80 && actualSpeed < 0.15) {
            stallDuration += dt;
        } else {
            stallDuration = Math.max(0.0, stallDuration - dt * 2.0);
        }

        lastActualPose = currentWorldPose;
        lastStallResult = (stallDuration > 0.25);
        return lastStallResult;
    }

    public boolean isHubActiveForAlliance(boolean isRedAlliance) {
        // Official 6.4 schedule (seeded by the AUTO result). The library
        // arena clock is frozen with both hubs physically capturable
        // (see GameSim.resetGame), so this schedule alone decides allowance.
        HubSchedule.refreshFromMatchState();
        return HubSchedule.isHubActiveNow(isRedAlliance);
    }

    public boolean isOpponentHubActive(boolean opponentIsRed) {
        return isHubActiveForAlliance(opponentIsRed);
    }

    public boolean isPoseInLowClearanceZone(Pose2d pose) {
        return FieldMap.Trenches.isLowClearance(pose);
    }

    public boolean isShootingLaneBlocked(Pose2d shooterPose, Translation2d targetHub) {
        Pose2d playerPose = trainingScenario != null && trainingBluePrimaryBot != null
                ? trainingBluePrimaryBot.getActualPose() : SwerveBase.getInstance().getPose();
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
        if (pose == null)
            return false;
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(opponentIsRed);
        double dist = pose.getTranslation().getDistance(hub);
        if (dist < 1.40 || dist > 4.00)
            return false;
        if (isPoseInLowClearanceZone(pose))
            return false;

        double x = pose.getX();
        double y = pose.getY();
        if (x < 0.6 || x > 15.9 || y < 1.60 || y > 6.45)
            return false;
        return FieldMap.AllianceZones.isInAllianceZone(pose, opponentIsRed);
    }

    public boolean canShootNow(Pose2d currentPose, boolean opponentIsRed) {
        if (!isOpponentHubActive(opponentIsRed))
            return false;
        if (getFuelCount() <= 0)
            return false;
        if (!isValidShootingLocation(currentPose, opponentIsRed))
            return false;

        Translation2d hub = opponentIsRed ? Constants.RED_HUB_LOCATION.toTranslation2d()
                : Constants.BLUE_HUB_LOCATION.toTranslation2d();
        if (isShootingLaneBlocked(currentPose, hub))
            return false;

        Rotation2d aimAngle = hub.minus(currentPose.getTranslation()).getAngle();
        double headingErr = Math.abs(currentPose.getRotation().minus(aimAngle).getRadians());
        if (headingErr > Math.toRadians(8.0))
            return false;

        double now = Timer.getFPGATimestamp();
        return (now - lastShotTimestamp >= 0.08); // Changed from 0.15s to 0.08s for continuous streaming
    }

    /**
     * Calculates an optimal standoff pose using a polar sector projection around
     * the Hub.
     * Guarantees target is strictly outside the Hub ramps and inside the Alliance
     * Zone.
     */
    public Pose2d calculatePolarStandoffPose(Pose2d robotPose, boolean opponentIsRed) {
        return JevDecisionEngine.getInstance().calculatePolarStandoffPose(robotPose, opponentIsRed);
    }

    /**
     * Canonical alias for {@link #calculatePolarStandoffPose(Pose2d, boolean)}
     * supporting unit tests and legacy callers.
     */
    public Pose2d getOptimalShootingPose(Pose2d currentPose, boolean opponentIsRed) {
        if (isValidShootingLocation(currentPose, opponentIsRed)) {
            Translation2d hub = FieldMap.Hubs.getHubLocation2d(opponentIsRed);
            if (!isShootingLaneBlocked(currentPose, hub)) {
                return currentPose;
            }
        }
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
                        if (piece == null || !"Fuel".equals(piece.getType()))
                            continue;
                        Translation2d pos = piece.getPoseOnField().getTranslation();

                        // Must be on the field carpet (wall-band balls stay eligible:
                        // the wall standoff below reaches them)
                        if (pos.getX() < 0.05 || pos.getX() > 16.48 || pos.getY() < 0.05 || pos.getY() > 8.00)
                            continue;
                        if (StaticPathfinder.isPointInHardObstacle(pos)
                                || StaticPathfinder.isPointNearDynamicObstacle(pos))
                            continue;

                        // Focus on opponent's half + center zone
                        if (opponentIsRed && pos.getX() < 4.0)
                            continue;
                        if (!opponentIsRed && pos.getX() > 12.5)
                            continue;

                        double dist = currentPose.getTranslation().getDistance(pos);
                        if (dist < minDistance) {
                            minDistance = dist;
                            bestFuel = pos;
                        }
                    }
                }
            } catch (Exception ignored) {
            }
        }

        if (bestFuel != null) {
            // If the ball is tight against a perimeter wall, position the robot center
            // 0.48m off the ball so the front intake rests right on top of it without
            // ramming (shared geometry: StaticPathfinder.wallStandoffApproach).
            return StaticPathfinder.wallStandoffApproach(bestFuel, currentPose.getTranslation());
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
        if (intakeSimulation == null || !intakeSimulation.isRunning())
            return;
        if (intakeSimulation.getGamePiecesAmount() >= Constants.IntakeConstants.MAX_HELD_BALLS)
            return;

        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null)
            return;

        try {
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
            if (pieces == null)
                return;

            Translation2d botPos = robotPose.getTranslation();
            Rotation2d botHeading = robotPose.getRotation();

            int collectedThisTick = 0;
            for (var piece : pieces) {
                if (piece == null || !"Fuel".equals(piece.getType()))
                    continue;
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
                            || collectedThisTick >= 10) { // Changed from 4 to 10
                        break;
                    }
                }
            }
        } catch (Exception ignored) {
        }
    }

    public boolean isNearAnyFuel(Translation2d botPos, double radius) {
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null)
            return false;
        try {
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
            if (pieces != null) {
                for (var piece : pieces) {
                    if (piece != null && "Fuel".equals(piece.getType())) {
                        if (piece.getPoseOnField().getTranslation().getDistance(botPos) <= radius)
                            return true;
                    }
                }
            }
        } catch (Exception ignored) {
        }
        return false;
    }

    public void launchOpponentShot(Pose2d robotPose, Translation2d opponentHub, boolean opponentIsRed) {        RefereeSim.checkShotLegality(robotPose, opponentIsRed, "Bot 0");
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
                    Meters.of(0.53), MetersPerSecond.of(randomExitVelocity), Radians.of(randomPitch));
            // Scoring is resolved by ShotTracker (see class docs): the hub
            // captures balls before the analytic hit-time, so the hit callback
            // alone would silently drop most scores.
            ShotTracker.track(fuelOnFly, funnelTarget, opponentIsRed,
                    () -> recordBot0ScoredHit(opponentIsRed), null);
            fuelOnFly.withTargetPosition(() -> funnelTarget)
                    .withTargetTolerance(new Translation3d(0.38, 0.38, 0.20));
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

        // Phase 5: Bot-0 telemetry mirrors opponents.get(0).
        Pose2d pose = bot0Instance.getActualPose();
        Pose2d bot0Target = bot0Instance.getCurrentTargetPose();
        String bot0Detail = bot0Instance.getCurrentAIStateDetail();
        int bot0Score = bot0Instance.getScoreCount();
        int bot0Fuel = bot0Instance.getFuelCount();
        boolean bot0Stalled = bot0Instance.isStalled();
        TrainingMatchScenario activeScenario = trainingScenario;
        boolean trainingMode = activeScenario != null;
        boolean active = trainingMode || Dashboard.isOpponentRobotEnabled();

        Logger.recordOutput("AI_Telemetry/ActualPose", pose);
        Logger.recordOutput("AI_Telemetry/TargetPose", bot0Target);
        Logger.recordOutput("AI_Telemetry/StateDetail", bot0Detail);
        Logger.recordOutput("AI_Telemetry/CyclerPhase", cyclerPhase.name());
        Logger.recordOutput("AI_Telemetry/ScoreCount", bot0Score);
        Logger.recordOutput("AI_Telemetry/HeldFuel", bot0Fuel);
        Logger.recordOutput("AI_Telemetry/IsStalled", bot0Stalled);
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

        Pose2d playerPose = trainingMode
                ? trainingBluePrimaryBot.getActualPose() : SwerveBase.getInstance().getPose();
        double distToPlayer = pose.getTranslation().getDistance(playerPose.getTranslation());
        Logger.recordOutput("AI_Telemetry/DistanceToPlayer", distToPlayer);

        SmartDashboard.putBoolean("Simulation/OpponentActive", active);
        SmartDashboard.putNumberArray("Simulation/OpponentPose",
                new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
        SmartDashboard.putNumberArray("Simulation/OpponentTargetPose", new double[] { bot0Target.getX(),
                bot0Target.getY(), bot0Target.getRotation().getDegrees() });
        SmartDashboard.putString("Simulation/OpponentAIState", bot0Detail);
        SmartDashboard.putNumber("Simulation/OpponentScoreCount", bot0Score);
        SmartDashboard.putNumber("Simulation/OpponentFuelCount", bot0Fuel);
        SmartDashboard.putBoolean("Simulation/OpponentStalled", bot0Stalled);

        if (active) {
            try {
                var field = SwerveBase.getInstance().getField();
                field.getObject("OpponentBot0").setPose(pose);
                field.getObject("OpponentTarget0").setPose(bot0Target);
            } catch (Exception ignored) {
            }
        }

        // Standardized Bot 0 telemetry
        Logger.recordOutput("AI_Telemetry/Bot0/ActualPose", pose);
        Logger.recordOutput("AI_Telemetry/Bot0/TargetPose", bot0Target);
        Logger.recordOutput("AI_Telemetry/Bot0/StateDetail", bot0Detail);
        Logger.recordOutput("AI_Telemetry/Bot0/Score", bot0Score);
        Logger.recordOutput("AI_Telemetry/Bot0/HeldFuel", bot0Fuel);
        Logger.recordOutput("AI_Telemetry/Bot0/Archetype", getAIMode().name());

        SmartDashboard.putNumberArray("Simulation/Bot0/Pose",
                new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
        SmartDashboard.putNumberArray("Simulation/Bot0/TargetPose", new double[] { bot0Target.getX(),
                bot0Target.getY(), bot0Target.getRotation().getDegrees() });
        SmartDashboard.putString("Simulation/Bot0/StateDetail", bot0Detail);
        SmartDashboard.putString("Simulation/Bot0/Objective", getAIMode().name());
        SmartDashboard.putNumber("Simulation/Bot0/Score", bot0Score);
        SmartDashboard.putNumber("Simulation/Bot0/Fuel", bot0Fuel);
        SmartDashboard.putBoolean("Simulation/Bot0/Stalled", bot0Stalled);
        SmartDashboard.putString("Simulation/Bot0/Archetype", getAIMode().name());

        // Multi-bot aggregate telemetry
        int totalScore = bot0Score;
        int totalFuel = bot0Fuel;
        int opponentCount = trainingMode
                ? activeScenario.redOpponentRobots().size() : Dashboard.getOpponentCount();
        for (int i = 0; i < opponentCount - 1 && i < additionalBots.size(); i++) {
            totalScore += additionalBots.get(i).getScoreCount();
            totalFuel += additionalBots.get(i).getFuelCount();
        }
        SmartDashboard.putNumber("Simulation/TotalOpponentScore", totalScore);
        SmartDashboard.putNumber("Simulation/TotalOpponentFuel", totalFuel);
        SmartDashboard.putNumber("Simulation/MultiBotActiveCount", opponentCount);

        int totalAllyScore = trainingMode ? trainingBluePrimaryBot.getScoreCount() : 0;
        int totalAllyFuel = trainingMode ? trainingBluePrimaryBot.getFuelCount() : 0;
        int allyCount = trainingMode
                ? activeScenario.blueAllyRobots().size() : Dashboard.getAllyCount();
        for (int i = 0; i < allyCount && i < allyBots.size(); i++) {
            totalAllyScore += allyBots.get(i).getScoreCount();
            totalAllyFuel += allyBots.get(i).getFuelCount();
        }
        SmartDashboard.putNumber("Simulation/TotalAllyScore", totalAllyScore);
        SmartDashboard.putNumber("Simulation/TotalAllyFuel", totalAllyFuel);
        SmartDashboard.putNumber("Simulation/AllyActiveCount", allyCount + (trainingMode ? 1 : 0));

        double now = Timer.getFPGATimestamp();
        ChassisSpeeds bot0CmdSpeeds = bot0Instance.getCurrentTargetSpeeds();
        double commandedMag = Math.hypot(bot0CmdSpeeds.vxMetersPerSecond, bot0CmdSpeeds.vyMetersPerSecond);
        double actualMag = Math.hypot(actualPhysicsSpeeds.vxMetersPerSecond, actualPhysicsSpeeds.vyMetersPerSecond);

        boolean isStuck = active && commandedMag > 0.5 && actualMag < 0.15;
        SmartDashboard.putBoolean("AI_Telemetry/IsStuck", isStuck);
        org.littletonrobotics.junction.Logger.recordOutput("AI_Telemetry/IsStuck", isStuck);

        // Only dump multi-line diagnostics to stdout if explicitly requested via
        // SmartDashboard
        boolean debugAI = SmartDashboard.getBoolean("Simulation/DebugAI", false);
        if (isStuck && debugAI && (now - lastConsoleDumpTime > 2.0)) {
            lastConsoleDumpTime = now;
            System.out.printf(
                    "[AI DIAGNOSTIC] STUCK: Mode: %s | Phase: %s | Pose: (%.2f, %.2f) | Target: (%.2f, %.2f) | Stall: %.2fs%n",
                    getAIMode().name(), cyclerPhase.name(),
                    pose.getX(), pose.getY(),
                    bot0Target.getX(), bot0Target.getY(),
                    bot0Instance.getStallDuration());
        }

        if (active && pose.getY() > 0.0 && pose.getX() > 0.0) {
            ChassisSpeeds bot0Cmd = bot0Instance.getCurrentTargetSpeeds();
            Translation2d vel = new Translation2d(bot0Cmd.vxMetersPerSecond,
                    bot0Cmd.vyMetersPerSecond);
            boolean isContacting = distToPlayer < 0.85;
            DynamicRouter.registerObstacle(pose.getTranslation(), vel, 0.55, 0.35, isContacting);
        }
    }

    public void setupArchetypeChoosers() {
        for (AIMode mode : AIMode.values()) {
            aiModeChooser.addOption(mode.displayName, mode);
        }
        aiModeChooser.setDefaultOption(AIMode.AUTONOMOUS_CYCLER.displayName, AIMode.AUTONOMOUS_CYCLER);
        SmartDashboard.putData("Simulation/AIModeChooser", aiModeChooser);
        SmartDashboard.putData("Simulation/Bot0/ArchetypeChooser", aiModeChooser);

        for (Archetype a : Archetype.values()) {
            if (a != Archetype.CO_PILOT) {
                bot1ArchetypeChooser.addOption(a.displayName, a);
                bot2ArchetypeChooser.addOption(a.displayName, a);
                ally1ArchetypeChooser.addOption(a.displayName, a);
                ally2ArchetypeChooser.addOption(a.displayName, a);
            }
        }
        bot1ArchetypeChooser.setDefaultOption(Archetype.DEFENSE_BULLY.displayName, Archetype.DEFENSE_BULLY);
        bot2ArchetypeChooser.setDefaultOption(Archetype.ADAPTIVE_COMPETITOR.displayName, Archetype.ADAPTIVE_COMPETITOR);
        ally1ArchetypeChooser.setDefaultOption(Archetype.AUTONOMOUS_CYCLER.displayName, Archetype.AUTONOMOUS_CYCLER);
        ally2ArchetypeChooser.setDefaultOption(Archetype.ADAPTIVE_COMPETITOR.displayName,
                Archetype.ADAPTIVE_COMPETITOR);

        SmartDashboard.putData("Simulation/Bot1/ArchetypeChooser", bot1ArchetypeChooser);
        SmartDashboard.putData("Simulation/Bot2/ArchetypeChooser", bot2ArchetypeChooser);
        SmartDashboard.putData("Simulation/Ally1/ArchetypeChooser", ally1ArchetypeChooser);
        SmartDashboard.putData("Simulation/Ally2/ArchetypeChooser", ally2ArchetypeChooser);
    }

    public Archetype getBot1Archetype() {
        if (bot1ArchetypeChooser != null && bot1ArchetypeChooser.getSelected() != null) {
            return bot1ArchetypeChooser.getSelected();
        }
        String str = SmartDashboard.getString("Simulation/Bot1/Archetype", Archetype.DEFENSE_BULLY.name());
        return Archetype.fromString(str);
    }

    public Archetype getBot2Archetype() {
        if (bot2ArchetypeChooser != null && bot2ArchetypeChooser.getSelected() != null) {
            return bot2ArchetypeChooser.getSelected();
        }
        String str = SmartDashboard.getString("Simulation/Bot2/Archetype", Archetype.ADAPTIVE_COMPETITOR.name());
        return Archetype.fromString(str);
    }

    public Archetype getAlly1Archetype() {
        if (ally1ArchetypeChooser != null && ally1ArchetypeChooser.getSelected() != null) {
            return ally1ArchetypeChooser.getSelected();
        }
        String str = SmartDashboard.getString("Simulation/Ally1/Archetype", Archetype.AUTONOMOUS_CYCLER.name());
        return Archetype.fromString(str);
    }

    public Archetype getAlly2Archetype() {
        if (ally2ArchetypeChooser != null && ally2ArchetypeChooser.getSelected() != null) {
            return ally2ArchetypeChooser.getSelected();
        }
        String str = SmartDashboard.getString("Simulation/Ally2/Archetype", Archetype.ADAPTIVE_COMPETITOR.name());
        return Archetype.fromString(str);
    }

    public SendableChooser<Archetype> getBot1ArchetypeChooser() {
        return bot1ArchetypeChooser;
    }

    public SendableChooser<Archetype> getBot2ArchetypeChooser() {
        return bot2ArchetypeChooser;
    }

    public SendableChooser<Archetype> getAlly1ArchetypeChooser() {
        return ally1ArchetypeChooser;
    }

    public SendableChooser<Archetype> getAlly2ArchetypeChooser() {
        return ally2ArchetypeChooser;
    }

    public List<AIRobotInstance> getAllyBots() {
        return Collections.unmodifiableList(allyBots);
    }

    /** Returns the independent Blue slot 0 instance while a training scenario is active. */
    public AIRobotInstance getTrainingBluePrimaryBot() {
        return trainingBluePrimaryBot;
    }

    public Archetype getTrainingBluePrimaryArchetype() {
        return trainingScenario == null ? null : trainingScenario.bluePlayerRobot().archetype();
    }

    public boolean isTrainingScenarioActive() {
        return trainingScenario != null;
    }

    @Override
    public void initialize() {
        setTrajectory("OpponentPath");
        SmartDashboard.setDefaultString("Simulation/AIMode", AIMode.AUTONOMOUS_CYCLER.name());
        SmartDashboard.setDefaultNumber("Simulation/OpponentCount", 1.0);
        SmartDashboard.setDefaultNumber("Simulation/OpponentSpeedPercent", 75.0);
        SmartDashboard.setDefaultNumber("Simulation/AllySpeedPercent", 75.0);
        SmartDashboard.setDefaultNumber("Simulation/AllyCount", 0.0);
        SmartDashboard.setDefaultString("Simulation/Bot0/Archetype", AIMode.AUTONOMOUS_CYCLER.name());
        SmartDashboard.setDefaultString("Simulation/Bot1/Archetype", Archetype.DEFENSE_BULLY.name());
        SmartDashboard.setDefaultString("Simulation/Bot2/Archetype", Archetype.ADAPTIVE_COMPETITOR.name());
        SmartDashboard.setDefaultString("Simulation/Ally1/Archetype", Archetype.AUTONOMOUS_CYCLER.name());
        SmartDashboard.setDefaultString("Simulation/Ally2/Archetype", Archetype.ADAPTIVE_COMPETITOR.name());
    }

    public static Pose2d getAllySpawnPose(int allyIndex, boolean playerIsRed) {
        double y;
        switch (allyIndex) {
            case 1:
                y = 5.80;
                break;
            case 2:
                y = 2.25;
                break;
            default:
                y = 4.035;
                break;
        }
        if (playerIsRed) {
            return new Pose2d(AllianceFlipUtil.FIELD_LENGTH - 2.00, y, Rotation2d.fromDegrees(180));
        } else {
            return new Pose2d(2.00, y, Rotation2d.fromDegrees(0));
        }
    }

    public static Pose2d getAllySpawnPose(boolean playerIsRed) {
        return getAllySpawnPose(1, playerIsRed);
    }

    public static Pose2d getOpponentSpawnPose(int botIndex, boolean playerIsRed) {
        double y;
        switch (botIndex) {
            case 1:
                y = 5.80;
                break;
            case 2:
                y = 2.25;
                break;
            case 0:
            default:
                y = 4.035;
                break;
        }
        if (playerIsRed) {
            return new Pose2d(2.00, y, Rotation2d.fromDegrees(0));
        } else {
            return new Pose2d(AllianceFlipUtil.FIELD_LENGTH - 2.00, y, Rotation2d.fromDegrees(180));
        }
    }

    public static Pose2d getOpponentSpawnPose(boolean playerIsRed) {
        return getOpponentSpawnPose(0, playerIsRed);
    }

    public Pose2d getOpponentSpawnPose(int botIndex, boolean playerIsRed, AIMode activeMode) {
        if (botIndex == 0 && activeMode == AIMode.CHOREO_PATH && trajectory.isPresent()) {
            Optional<SwerveSample> initialSample = trajectory.get().sampleAt(0, false);
            if (initialSample.isPresent()) {
                SwerveSample sample = initialSample.get();
                Pose2d startPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
                return mirrorPoseForOpponent(startPose, playerIsRed);
            }
        }
        return getOpponentSpawnPose(botIndex, playerIsRed);
    }

    public Pose2d getOpponentSpawnPose(boolean playerIsRed, AIMode activeMode) {
        return getOpponentSpawnPose(0, playerIsRed, activeMode);
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
        return bot0Instance.getScoreCount();
    }

    public int getFuelCount() {
        return (intakeSimulation != null) ? intakeSimulation.getGamePiecesAmount() : 0;
    }

    public void setFuelCount(int count) {
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(count);
        }
    }

    /**
     * Records one scored ball for Bot 0 (legacy ShotTracker hook; live Bot-0
     * scoring now flows through opponents.get(0)).
     */
    public void recordBot0ScoredHit(boolean opponentIsRed) {
        bot0Instance.noteScoredHit();
        MatchScoreTracker.getInstance().recordBotScore(0, opponentIsRed);
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

    public List<AIRobotInstance> getAdditionalBots() {
        return Collections.unmodifiableList(additionalBots);
    }
}
