package frc.robot.Sim;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Auto.StaticPathfinder;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.AutonConstants;
import frc.robot.Data.GlideConstants;
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

/**
 * AIRobotSim Subsystem
 * High-fidelity opponent robot simulation powered by IronMaple and Jev AI:
 * Supports multiple selectable tactical behaviors:
 * 1. TACTICAL_DEFENSE: System One Jev AI (shooting lane denial, depot contesting, midline shadowing).
 * 2. LEAD_PURSUIT_INTERCEPT: Quadratic trajectory intersection solver cutting off player movement.
 * 3. PINNING_BULLY: Aggressive bumper charging to test driver Swerve Pirouette Slip & Legal Pinning Watchdog.
 * 4. AUTONOMOUS_CYCLER: Plays real 2026 offense: harvests neutral midfield fuel, travels to opponent Hub, and scores.
 * 5. CHOREO_PATH: Follows pre-baked Choreo trajectory.
 * 6. MANUAL_2_PLAYER: Human operator sparring via gamepad on Port 2.
 */
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
            if (name == null) return TACTICAL_DEFENSE;
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

    // Queuing positions in safe corners when opponent robot is disabled
    public static final Pose2d[] ROBOT_QUEUING_POSITIONS = new Pose2d[] {
            new Pose2d(1.0, -5, new Rotation2d()),
            new Pose2d(1.5, -5, new Rotation2d()),
            new Pose2d(2.0, -5, new Rotation2d())
    };

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queuingPose;
    private IntakeSimulation intakeSimulation;

    // Path Following
    private Optional<Trajectory<SwerveSample>> trajectory = Optional.empty();
    private final Timer pathTimer = new Timer();
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    // 2 Player Control
    private final XboxController defenseController;

    // State Tracking
    private boolean wasOpponentEnabled = false;
    private ChassisSpeeds currentTargetSpeeds = new ChassisSpeeds();
    private Pose2d currentTargetPose = new Pose2d();
    private String currentAIStateDetail = "IDLE";
    private int aiScoreCount = 0;

    // Cycler State Machine
    private CyclerPhase cyclerPhase = CyclerPhase.HUNT_FUEL;
    private final Timer cyclerTimer = new Timer();

    // Navigation & Waypoint Tracking State
    private final List<Pose2d> currentPath = new ArrayList<>();
    private int currentPathIndex = 0;
    private Pose2d lastPathTarget = new Pose2d(-999, -999, new Rotation2d());

    // Stuck / Stall Watchdog State
    private Pose2d lastActualPose = new Pose2d();
    private double lastPoseTimestamp = -1.0;
    private double stallDuration = 0.0;
    private double unstickEndTime = -1.0;
    private Translation2d unstickVector = new Translation2d();
    private double lastCommandedSpeed = 0.0;

    public static AIRobotSim getInstance() {
        if (instance == null) {
            instance = new AIRobotSim();
        }
        return instance;
    }

    private AIRobotSim() {
        this.queuingPose = ROBOT_QUEUING_POSITIONS[0];

        // Initialize swerve drive simulation with a default configuration
        this.driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default(),
                        queuingPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

        // Attach IronMaple IntakeSimulation to opponent robot for physical fuel harvesting
        try {
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                    "Fuel",
                    driveSimulation.getDriveTrainSimulation(),
                    Meters.of(0.65), // 65cm width across bumper
                    Meters.of(0.25), // 25cm extension
                    IntakeSimulation.IntakeSide.FRONT,
                    20 // 20 fuel capacity
            );
            intakeSimulation.register(SimulatedArena.getInstance());
        } catch (Exception e) {
            System.err.println("[AIRobotSim] Could not attach IntakeSimulation: " + e.getMessage());
        }

        // Path / Waypoint Controllers
        this.xController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI,
                AutonConstants.AUTO_DRIVE_KD);
        this.yController = new PIDController(AutonConstants.AUTO_DRIVE_KP, AutonConstants.AUTO_DRIVE_KI,
                AutonConstants.AUTO_DRIVE_KD);

        // Heading PID
        var config = SwerveBase.getInstance().getSwerveController().config;
        this.headingController = new PIDController(config.headingPIDF.p, config.headingPIDF.i, config.headingPIDF.d);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.defenseController = new XboxController(2);

        SubsystemManager.registerSubsystem(this);
    }

    public void setTrajectory(String pathName) {
        this.trajectory = Choreo.loadTrajectory(pathName);
        pathTimer.restart();
    }

    /**
     * Resets the opponent robot state.
     */
    public void reset() {
        wasOpponentEnabled = false;
        pathTimer.restart();
        cyclerTimer.restart();
        cyclerPhase = CyclerPhase.HUNT_FUEL;
        aiScoreCount = 0;
        currentPath.clear();
        currentPathIndex = 0;
        lastPathTarget = new Pose2d(-999, -999, new Rotation2d());
        lastPoseTimestamp = -1.0;
        stallDuration = 0.0;
        unstickEndTime = -1.0;
        unstickVector = new Translation2d();
        lastCommandedSpeed = 0.0;
        if (intakeSimulation != null) {
            intakeSimulation.setGamePiecesCount(0);
            intakeSimulation.stopIntake();
        }
    }

    @Override
    public void simulationUpdate() {
        boolean opponentEnabled = Dashboard.isOpponentRobotEnabled();
        boolean manualDefenseMode = Dashboard.is2PlayerDefenseEnabled();

        if (!opponentEnabled) {
            // "Hide" the robot off field
            driveSimulation.setSimulationWorldPose(queuingPose);
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), true, true);
            wasOpponentEnabled = false;
            currentAIStateDetail = "DISABLED";
            if (intakeSimulation != null && intakeSimulation.isRunning()) {
                intakeSimulation.stopIntake();
            }
            return;
        }

        // When opponent robot is first enabled, spawn on opponent alliance side of the field
        if (!wasOpponentEnabled) {
            boolean isRedAlliance = AllianceFlipUtil.isRedAlliance();
            double spawnX = isRedAlliance ? 3.0 : (AllianceFlipUtil.FIELD_LENGTH - 3.0);
            double spawnY = 4.04;
            Rotation2d spawnHeading = Rotation2d.fromDegrees(isRedAlliance ? 0 : 180);
            Pose2d initialPose = new Pose2d(spawnX, spawnY, spawnHeading);

            if (trajectory.isPresent()) {
                Optional<SwerveSample> initialSample = trajectory.get().sampleAt(0, false);
                if (initialSample.isPresent()) {
                    SwerveSample sample = initialSample.get();
                    Pose2d startPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
                    initialPose = mirrorPoseForOpponent(startPose, isRedAlliance);
                }
            }

            driveSimulation.setSimulationWorldPose(initialPose);
            pathTimer.restart();
            cyclerTimer.restart();
            wasOpponentEnabled = true;
        }

        // Determine active AI Mode
        AIMode activeMode;
        if (manualDefenseMode) {
            activeMode = AIMode.MANUAL_2_PLAYER;
        } else {
            String modeStr = SmartDashboard.getString("Simulation/AIMode", AIMode.TACTICAL_DEFENSE.name());
            activeMode = AIMode.fromString(modeStr);
        }

        // Speed scaling (percentage from dashboard [20% to 100%])
        double speedPercent = SmartDashboard.getNumber("Simulation/OpponentSpeedPercent", 75.0);
        double speedScale = Math.max(0.20, Math.min(1.0, speedPercent / 100.0));
        double maxSpeed = Constants.MAX_SPEED * speedScale;

        Pose2d playerPose = SwerveBase.getInstance().getPose();
        Pose2d currentPose = driveSimulation.getActualPoseInSimulationWorld();
        boolean isRedAlliance = AllianceFlipUtil.isRedAlliance();
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

                targetSpeeds = computeNavigatedDriveSpeeds(currentPose, targetPose, maxSpeed, true);
                currentAIStateDetail = String.format("INTERCEPTING @ (%.1f, %.1f)", targetPose.getX(), targetPose.getY());
                break;

            case PINNING_BULLY:
                // Drive directly into player's bumper center to test driver pirouette slip & pin watchdog
                Rotation2d angleToPlayer = playerPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
                targetPose = new Pose2d(playerPose.getTranslation(), angleToPlayer);

                targetSpeeds = computeNavigatedDriveSpeeds(currentPose, targetPose, maxSpeed, false);
                currentAIStateDetail = "PINNING_BULLY (CHARGING_PLAYER)";
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
                        currentAIStateDetail = String.format("CHOREO_PATH (t=%.1fs)", time);
                    } else {
                        targetSpeeds = new ChassisSpeeds();
                        currentAIStateDetail = "CHOREO_PATH_EMPTY";
                    }
                } else {
                    targetSpeeds = new ChassisSpeeds();
                    currentAIStateDetail = "NO_CHOREO_TRAJECTORY";
                }
                break;

            case TACTICAL_DEFENSE:
            default:
                JevDecisionEngine.DecisionResult decision = JevDecisionEngine.getInstance().evaluate(
                        playerPose, currentPose, matchTime, isHubActive, isRedAlliance);

                targetPose = decision.targetPose;
                targetSpeeds = computeNavigatedDriveSpeeds(currentPose, targetPose, maxSpeed, true);
                currentAIStateDetail = decision.action.name() + " (" + String.format("%.0f%% conf", decision.confidence * 100) + ")";
                break;
        }

        currentTargetPose = targetPose;
        currentTargetSpeeds = targetSpeeds;
        driveSimulation.runChassisSpeeds(targetSpeeds, new Translation2d(), true, true);
    }

    public ChassisSpeeds computeDriveToPoseSpeeds(Pose2d currentPose, Pose2d targetPose, double maxSpeed) {
        return computeNavigatedDriveSpeeds(currentPose, targetPose, maxSpeed, true);
    }

    private ChassisSpeeds computeNavigatedDriveSpeeds(Pose2d currentPose, Pose2d targetPose, double maxSpeed, boolean avoidPlayer) {
        double now = Timer.getFPGATimestamp();

        // 1. Stuck / Stall Watchdog & Evasive Backoff
        if (lastPoseTimestamp > 0.0) {
            double dt = Math.max(0.001, now - lastPoseTimestamp);
            double actualMoveDist = currentPose.getTranslation().getDistance(lastActualPose.getTranslation());
            double actualSpeed = actualMoveDist / dt;
            double commandedSpeed = Math.max(lastCommandedSpeed, Math.hypot(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond));

            if (commandedSpeed > 0.8 && actualSpeed < 0.15) {
                stallDuration += dt;
            } else {
                stallDuration = Math.max(0.0, stallDuration - dt * 2.0);
            }

            if (stallDuration > 0.35) {
                // Pin / Obstacle wedge detected: initiate backoff impulse
                unstickEndTime = now + 0.40;
                Translation2d cmdDir = new Translation2d(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
                if (cmdDir.getNorm() < 0.1) {
                    cmdDir = targetPose.getTranslation().minus(currentPose.getTranslation());
                }
                if (cmdDir.getNorm() > 0.1) {
                    unstickVector = cmdDir.div(cmdDir.getNorm()).times(-1.8);
                } else {
                    unstickVector = new Translation2d(-1.5, 0.0);
                }
                stallDuration = 0.0;
                currentPath.clear(); // Recalculate path after unstick
            }
        }
        lastActualPose = currentPose;
        lastPoseTimestamp = now;

        if (now < unstickEndTime) {
            return new ChassisSpeeds(unstickVector.getX(), unstickVector.getY(), 4.0);
        }

        // 2. Collision-Free Path Planning & Waypoint Tracking
        if (currentPath.isEmpty() || targetPose.getTranslation().getDistance(lastPathTarget.getTranslation()) > 0.45) {
            currentPath.clear();
            currentPath.addAll(StaticPathfinder.findPath(currentPose, targetPose));
            currentPathIndex = 0;
            lastPathTarget = targetPose;
        }

        while (currentPathIndex < currentPath.size() - 1) {
            Pose2d wp = currentPath.get(currentPathIndex);
            if (currentPose.getTranslation().getDistance(wp.getTranslation()) < 0.50) {
                currentPathIndex++;
            } else {
                break;
            }
        }

        Pose2d targetWaypoint = currentPath.isEmpty() ? targetPose : currentPath.get(currentPathIndex);

        // 3. Speed Calculation to Target Waypoint
        double vx = xController.calculate(currentPose.getX(), targetWaypoint.getX());
        double vy = yController.calculate(currentPose.getY(), targetWaypoint.getY());

        Rotation2d desiredHeading = targetPose.getRotation();
        if (currentPathIndex < currentPath.size() - 1) {
            Translation2d moveVec = targetWaypoint.getTranslation().minus(currentPose.getTranslation());
            if (moveVec.getNorm() > 0.3) {
                desiredHeading = moveVec.getAngle();
            }
        }

        double omega = headingController.calculate(
                currentPose.getRotation().getRadians(),
                desiredHeading.getRadians());

        double distToGoal = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double effectiveMaxSpeed = (distToGoal < 0.8) ? Math.max(0.40, maxSpeed * (distToGoal / 0.8)) : maxSpeed;

        double currentSpeed = Math.hypot(vx, vy);
        if (currentSpeed > effectiveMaxSpeed) {
            double scale = effectiveMaxSpeed / currentSpeed;
            vx *= scale;
            vy *= scale;
        }
        omega = Math.max(-4.5, Math.min(4.5, omega));

        // 4. Dynamic Avoidance of Player Robot
        if (avoidPlayer) {
            Pose2d playerPose = SwerveBase.getInstance().getPose();
            double distToPlayer = currentPose.getTranslation().getDistance(playerPose.getTranslation());
            if (distToPlayer < 1.6 && distToPlayer > 0.05) {
                Translation2d away = currentPose.getTranslation().minus(playerPose.getTranslation());
                double norm = away.getNorm();
                Translation2d unitAway = (norm > 1e-4) ? away.div(norm) : new Translation2d(1, 0);
                double repForce = 2.5 * (1.0 / distToPlayer - 1.0 / 1.6);
                vx += unitAway.getX() * repForce;
                vy += unitAway.getY() * repForce;

                double repSpeed = Math.hypot(vx, vy);
                if (repSpeed > maxSpeed) {
                    vx = (vx / repSpeed) * maxSpeed;
                    vy = (vy / repSpeed) * maxSpeed;
                }
            }
        }

        lastCommandedSpeed = Math.hypot(vx, vy);
        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Executes the Autonomous Cycler state machine simulating an opposing offense bot.
     */
    private ChassisSpeeds updateAutonomousCycler(Pose2d currentPose, boolean playerIsRed, double maxSpeed) {
        // Opponent alliance hub & neutral ball targets
        // Opponent alliance is opposite of player alliance
        boolean opponentIsRed = !playerIsRed;
        Translation2d opponentHub = opponentIsRed ?
                new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY()) :
                new Translation2d(Constants.BLUE_HUB_LOCATION.getX(), Constants.BLUE_HUB_LOCATION.getY());

        Translation2d hubScoringPos = opponentIsRed ?
                opponentHub.plus(new Translation2d(2.2, 0.0)) :
                opponentHub.minus(new Translation2d(2.2, 0.0));
        Rotation2d faceHubAngle = opponentHub.minus(hubScoringPos).getAngle();
        Pose2d hubTargetPose = new Pose2d(hubScoringPos, faceHubAngle);

        // Neutral Midfield target
        double midX = JevDecisionEngine.CENTERLINE_X;
        double midY = (currentPose.getY() > 4.0) ? 6.10 : 2.10;
        Pose2d huntTargetPose = new Pose2d(midX, midY, Rotation2d.fromDegrees(opponentIsRed ? 0 : 180));

        int heldPieces = (intakeSimulation != null) ? intakeSimulation.getGamePiecesAmount() : 0;

        switch (cyclerPhase) {
            case HUNT_FUEL:
                currentTargetPose = huntTargetPose;
                if (intakeSimulation != null && !intakeSimulation.isRunning()) {
                    intakeSimulation.startIntake();
                }

                double distToHunt = currentPose.getTranslation().getDistance(huntTargetPose.getTranslation());
                if (heldPieces >= 3 || (distToHunt < 0.70 && cyclerTimer.get() > 3.0) || cyclerTimer.get() > 8.0) {
                    cyclerPhase = CyclerPhase.SCORE_HUB;
                    cyclerTimer.restart();
                    if (intakeSimulation != null) intakeSimulation.stopIntake();
                }
                currentAIStateDetail = String.format("CYCLER_HUNTING (Fuel: %d)", heldPieces);
                return computeNavigatedDriveSpeeds(currentPose, huntTargetPose, maxSpeed, true);

            case SCORE_HUB:
                currentTargetPose = hubTargetPose;
                if (intakeSimulation != null && intakeSimulation.isRunning()) {
                    intakeSimulation.stopIntake();
                }

                double distToHub = currentPose.getTranslation().getDistance(hubTargetPose.getTranslation());
                if (distToHub < 0.65 || cyclerTimer.get() > 9.0) {
                    cyclerPhase = CyclerPhase.SHOOTING;
                    cyclerTimer.restart();
                }
                currentAIStateDetail = String.format("CYCLER_TRANSIT_TO_HUB (Fuel: %d)", heldPieces);
                return computeNavigatedDriveSpeeds(currentPose, hubTargetPose, maxSpeed, true);

            case SHOOTING:
                currentTargetPose = hubTargetPose;
                if (cyclerTimer.get() > 1.2) {
                    // Empty fuel hopper & award simulated scores
                    if (intakeSimulation != null) {
                        aiScoreCount += Math.max(1, heldPieces);
                        intakeSimulation.setGamePiecesCount(0);
                    } else {
                        aiScoreCount += 3;
                    }
                    cyclerPhase = CyclerPhase.HUNT_FUEL;
                    cyclerTimer.restart();
                }
                currentAIStateDetail = "CYCLER_SHOOTING_AT_HUB";
                return new ChassisSpeeds(0, 0, 0);

            default:
                cyclerPhase = CyclerPhase.HUNT_FUEL;
                return new ChassisSpeeds();
        }
    }

    @Override
    public void update() {
        if (edu.wpi.first.wpilibj.RobotBase.isReal()) {
            return;
        }

        Pose2d pose = driveSimulation.getActualPoseInSimulationWorld();
        boolean active = Dashboard.isOpponentRobotEnabled();

        // AdvantageKit & SmartDashboard Telemetry
        SmartDashboard.putBoolean("Simulation/OpponentActive", active);
        SmartDashboard.putNumberArray("Simulation/OpponentPose", new double[] {
                pose.getX(), pose.getY(), pose.getRotation().getDegrees()
        });
        SmartDashboard.putNumberArray("Simulation/OpponentTargetPose", new double[] {
                currentTargetPose.getX(), currentTargetPose.getY(), currentTargetPose.getRotation().getDegrees()
        });
        SmartDashboard.putString("Simulation/OpponentAIState", currentAIStateDetail);
        SmartDashboard.putNumber("Simulation/OpponentScoreCount", aiScoreCount);
        SmartDashboard.putNumber("Simulation/OpponentFuelCount", intakeSimulation != null ? intakeSimulation.getGamePiecesAmount() : 0);

        Logger.recordOutput("Simulation/OpponentPose", pose);
        Logger.recordOutput("Simulation/OpponentTargetPose", currentTargetPose);
        Logger.recordOutput("Simulation/OpponentAIState", currentAIStateDetail);

        SmartDashboard.putBoolean("Simulation/OpponentStalled", Timer.getFPGATimestamp() < unstickEndTime);
        Logger.recordOutput("Simulation/OpponentPath", currentPath.toArray(new Pose2d[0]));

        // Register opponent as dynamic obstacle in DynamicRouter
        if (active && pose.getY() > 0.0 && pose.getX() > 0.0) {
            Translation2d vel = new Translation2d(currentTargetSpeeds.vxMetersPerSecond, currentTargetSpeeds.vyMetersPerSecond);
            Pose2d playerPose = SwerveBase.getInstance().getPose();
            boolean isContacting = pose.getTranslation().getDistance(playerPose.getTranslation()) < 0.85;

            // Flag proprioceptive stall if opponent is actively making contact
            DynamicRouter.registerObstacle(pose.getTranslation(), vel, 0.55, 0.35, isContacting);
        }
    }

    @Override
    public void initialize() {
        setTrajectory("OpponentPath");
        SmartDashboard.setDefaultString("Simulation/AIMode", AIMode.TACTICAL_DEFENSE.name());
        SmartDashboard.setDefaultNumber("Simulation/OpponentSpeedPercent", 75.0);
    }

    /**
     * Mirrors a pose to the opposite alliance side of the field.
     */
    private Pose2d mirrorPoseForOpponent(Pose2d pose, boolean playerIsRed) {
        if (playerIsRed) {
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

    // Getters for testing and diagnostic inspection
    public AIMode getAIMode() {
        String modeStr = SmartDashboard.getString("Simulation/AIMode", AIMode.TACTICAL_DEFENSE.name());
        return AIMode.fromString(modeStr);
    }

    public CyclerPhase getCyclerPhase() {
        return cyclerPhase;
    }

    public int getAiScoreCount() {
        return aiScoreCount;
    }

    public SelfControlledSwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public List<Pose2d> getCurrentPath() {
        return Collections.unmodifiableList(currentPath);
    }

    public boolean isStalled() {
        return (Timer.getFPGATimestamp() < unstickEndTime) || (stallDuration > 0.30);
    }
}
