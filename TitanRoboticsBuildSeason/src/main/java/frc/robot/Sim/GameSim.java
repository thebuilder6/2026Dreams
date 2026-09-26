package frc.robot.Sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Navigation.FieldMap;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;

/**
 * GameSim manages the simulation state for the 2026 robotics game.
 * Handles ball pickup, scoring, respawn logic, and telemetry.
 * Uses singleton pattern with thread-safe initialization.
 */
public class GameSim implements Subsystem {

    // Configuration constants
    private static final class Config {
        static final double MATCH_DURATION_SEC = 150.0;
        static final double PICKUP_RADIUS_M = 0.45;
        static final double PICKUP_ANGLE_RAD = Math.PI / 2; // 90 degrees
        static final int MAX_HELD_BALLS = Constants.IntakeConstants.MAX_HELD_BALLS;
        static final int PICKUP_PER_CHECK_LIMIT = 10;
        static final double MIN_RESPAWN_INTERVAL = 0.2;
        static final double PUBLISH_INTERVAL_SEC = 0.1; // 10Hz
        static final int PICKUP_CHECK_INTERVAL = 3; // Every 3 loops
        static final double SIMULATION_PERIOD = 0.02; // 50Hz

        // Field boundaries for center half spawning
        static final double CENTER_HALF_X_MIN = 6.0;
        static final double CENTER_HALF_X_MAX = 10.0;
        static final double CENTER_HALF_Y_MIN = 2.0;
        static final double CENTER_HALF_Y_MAX = 6.0;

        // Field boundaries for validation (Consolidated via FieldMap)
        static final double FIELD_X_MIN = 0.0;
        static final double FIELD_X_MAX = FieldMap.FIELD_LENGTH;
        static final double FIELD_Y_MIN = 0.0;
        static final double FIELD_Y_MAX = FieldMap.FIELD_WIDTH;

        // Initial game state
        static final int INITIAL_HELD_BALLS = 8;
        static final int LIGHTWEIGHT_BALL_COUNT = 54; // Strategic balanced physics mode (54 balls: 12 Blue, 12 Red, 30
                                                      // Center)
        static final String DEFAULT_GAME_MESSAGE = "R";

        // Official 2026 Rebuilt Depot coordinates (meters, from FieldMap.Depots)
        // Blue Depot (Top-Left inside Blue driver station wall X ~ 0m, Y ~ 5.53m -
        // 6.44m)
        static final double BLUE_DEPOT_X = FieldMap.Depots.BLUE_DEPOT_LOAD_POINT.getX();
        static final double BLUE_DEPOT_Y = FieldMap.Depots.BLUE_DEPOT_LOAD_POINT.getY(); // Centered 3-row start
        static final double BLUE_DEPOT_FULL_Y = 5.58; // Full 6-row start

        // Red Depot (Bottom-Right inside Red driver station wall X ~ 16.54m, Y ~ 1.65m
        // - 2.56m)
        static final double RED_DEPOT_X = FieldMap.Depots.RED_DEPOT_LOAD_POINT.getX();
        static final double RED_DEPOT_Y = FieldMap.Depots.RED_DEPOT_LOAD_POINT.getY(); // Centered 3-row start
        static final double RED_DEPOT_FULL_Y = 1.72; // Full 6-row start

        // Grid spacing for Fuel balls
        static final double BALL_SPACING_X = 0.152;
        static final double BALL_SPACING_Y = 0.151;
    }

    private static final AtomicReference<GameSim> instance = new AtomicReference<>();

    // Game state
    private volatile int heldBalls = 0;
    private volatile int score = 0;
    private volatile double simTimeRemainingSec = Config.MATCH_DURATION_SEC;
    private volatile boolean simRunning = false;

    // Telemetry
    private final StructArrayPublisher<Pose3d> gamePiecePublisher;

    // Shot tracking
    private volatile long lastSimScoreCount = 0;
    private volatile long shotsConsumedWithBall = 0;
    private volatile boolean lastShotScored = false;

    // Respawn system
    private final Random rng = new Random();
    private volatile int pendingRespawns = 0;
    private volatile double lastRespawnTime = 0;

    // Performance optimization
    private volatile int simLoopCounter = 0;
    private volatile double lastPublishTime = 0;

    // Cached references for performance
    private SimulatedArena cachedArena = null;
    private double lastArenaCacheTime = 0;
    private static final double ARENA_CACHE_DURATION = 1.0;

    public static GameSim getInstance() {
        GameSim current = instance.get();
        if (current == null) {
            current = new GameSim();
            instance.compareAndSet(null, current);
        }
        return instance.get();
    }

    private GameSim() {
        this.gamePiecePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Simulation/GamePieces", Pose3d.struct)
                .publish();
        // Skip calling resetGame() here, move setup to initialize()
        // to ensure it runs after SimulatedArena is stable.
        SubsystemManager.registerSubsystem(this);
    }

    public double getSimTimeRemainingSec() {
        return simTimeRemainingSec;
    }

    public void setSimTimeRemainingSec(double timeSec) {
        this.simTimeRemainingSec = timeSec;
    }

    /**
     * Consumes held balls for shooting with validation and error handling.
     * 
     * @param maxToConsume Maximum number of balls to consume
     * @return Actual number of balls consumed
     */
    public synchronized int consumeHeldBallsForShot(int maxToConsume) {
        if (!RobotBase.isSimulation()) {
            return maxToConsume;
        }

        if (maxToConsume < 0) {
            System.err.println("GameSim: Invalid maxToConsume value: " + maxToConsume);
            return 0;
        }

        if (heldBalls <= 0) {
            return 0;
        }

        int toConsume = Math.min(heldBalls, Math.min(maxToConsume, Config.MAX_HELD_BALLS));
        heldBalls -= toConsume;
        shotsConsumedWithBall += toConsume;

        // Synchronize with MapleSim intake simulation buffer
        var mapleIntake = Intake.getInstance().getMapleIntakeSim();
        if (mapleIntake != null) {
            for (int i = 0; i < toConsume; i++) {
                mapleIntake.obtainGamePieceFromIntake();
            }
        }

        return toConsume;
    }

    public int getHeldBalls() {
        return heldBalls;
    }

    @Override
    public void update() {
    }

    /**
     * Main simulation update loop with improved error handling and performance.
     */
    @Override
    public void simulationUpdate() {
        if (!RobotBase.isSimulation())
            return;

        try {
            simLoopCounter++;
            handleDashboardCommands();
            updateSimulationTime();
            if (simLoopCounter % Config.PICKUP_CHECK_INTERVAL == 0) {
                handlePickup();
            }
            handleShotsAndScoring();

            double now = Timer.getFPGATimestamp();
            if (now - lastPublishTime >= Config.PUBLISH_INTERVAL_SEC) {
                publish();
                lastPublishTime = now;
            }
        } catch (Exception e) {
            logRateLimitedError("simulationUpdate", e);
        }
    }

    /**
     * Handles dashboard command inputs with validation.
     */
    private void handleDashboardCommands() {
        try {
            boolean reset = SmartDashboard.getBoolean("Simulation/Reset", false);
            if (reset) {
                SmartDashboard.putBoolean("Simulation/Reset", false);
                resetGame();
            }

            boolean respawn = SmartDashboard.getBoolean("Simulation/RespawnBalls", false);
            if (respawn) {
                SmartDashboard.putBoolean("Simulation/RespawnBalls", false);
                spawnPickupBalls();
            }
        } catch (Exception e) {
            logRateLimitedError("handleDashboardCommands", e);
        }
    }

    /**
     * Updates simulation time based on DriverStation or internal timer.
     */
    private void updateSimulationTime() {
        try {
            double dsTimeRemainingSec = DriverStation.getMatchTime();
            boolean isDsTimeValid = dsTimeRemainingSec >= 0.0;
            SmartDashboard.putBoolean("Simulation/TimeRemainingValid", isDsTimeValid);

            if (isDsTimeValid) {
                simTimeRemainingSec = dsTimeRemainingSec;
                simRunning = DriverStation.isEnabled() && simTimeRemainingSec > 0.0;
            } else {
                if (!DriverStation.isEnabled()) {
                    simRunning = false;
                } else if (simTimeRemainingSec > 0.0) {
                    simRunning = true;
                }

                if (simRunning) {
                    simTimeRemainingSec = Math.max(0.0, simTimeRemainingSec - Config.SIMULATION_PERIOD);
                    if (simTimeRemainingSec <= 0.0) {
                        simRunning = false;
                    }
                }

                // Keep the official hub schedule in step with match time.
                HubSchedule.update(simTimeRemainingSec, DriverStation.isAutonomous());
            }
        } catch (Exception e) {
            logRateLimitedError("updateSimulationTime", e);
        }
    }

    /**
     * Publishes telemetry data with error handling and arena caching.
     */
    private void publish() {
        try {
            SmartDashboard.putNumber("Simulation/TimeRemainingSec", simTimeRemainingSec);
            SmartDashboard.putBoolean("Simulation/Running", simRunning);
            SmartDashboard.putNumber("Simulation/Score", score);
            SmartDashboard.putNumber("Simulation/HeldBalls", heldBalls);
            SmartDashboard.putBoolean("Simulation/LastShotScored", lastShotScored);

            // --- Rebuilt 2026 Specific Telemetry (official 6.4 schedule) ---
            SimulatedArena arena = getCachedArena();
            if (arena instanceof Arena2026Rebuilt) {
                SmartDashboard.putBoolean("Simulation/HubActive/Blue", HubSchedule.isHubActiveNow(false));
                SmartDashboard.putBoolean("Simulation/HubActive/Red", HubSchedule.isHubActiveNow(true));
            }

            // --- AdvantageScope Consolidation ---
            Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
            gamePiecePublisher.set(fuelPoses);
            org.littletonrobotics.junction.Logger.recordOutput("FieldSimulation/Fuel", fuelPoses);
        } catch (Exception e) {
            logRateLimitedError("publish", e);
        }
    }

    /**
     * Gets cached SimulatedArena instance for performance.
     * 
     * @return SimulatedArena instance
     */
    private SimulatedArena getCachedArena() {
        double now = Timer.getFPGATimestamp();
        if (cachedArena == null || now - lastArenaCacheTime > ARENA_CACHE_DURATION) {
            cachedArena = SimulatedArena.getInstance();
            lastArenaCacheTime = now;
        }
        return cachedArena;
    }

    /**
     * Normalizes angle to [-PI, PI] range efficiently.
     * 
     * @param angle Input angle in radians
     * @return Normalized angle in [-PI, PI]
     */
    private static double normalizeAngle(double angle) {
        // Use Math.IEEEremainder for efficient angle normalization
        double normalized = Math.IEEEremainder(angle, 2 * Math.PI);
        return normalized;
    }

    /**
     * Handles ball pickup logic using MapleSim physics intake simulation with
     * geometric fallback.
     */
    private void handlePickup() {
        if (heldBalls >= Config.MAX_HELD_BALLS) {
            return;
        }

        try {
            // Primary: Check MapleSim physics-based IntakeSimulation
            var mapleIntake = Intake.getInstance().getMapleIntakeSim();
            if (mapleIntake != null) {
                // Synchronize heldBalls directly with MapleSim intake piece count
                heldBalls = mapleIntake.getGamePiecesAmount();
                return;
            }

            if (Intake.getInstance().getState() != Intake.IntakeState.INTAKING) {
                return;
            }

            // Fallback: Geometric proximity check when MapleSim intake isn't bound yet
            Pose2d robotPose = SwerveBase.getInstance().getSimulationPose();
            if (robotPose == null) {
                return;
            }

            Translation2d robot = robotPose.getTranslation();
            Rotation2d robotHeading = robotPose.getRotation();

            SimulatedArena arena = getCachedArena();
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();

            int ballsPickedUp = 0;
            for (var piece : pieces) {
                if (ballsPickedUp >= Config.PICKUP_PER_CHECK_LIMIT) {
                    break;
                }

                Translation2d ball = piece.getPoseOnField().getTranslation();
                double distance = ball.getDistance(robot);

                if (distance <= Config.PICKUP_RADIUS_M) {
                    Translation2d robotToBall = ball.minus(robot);
                    double angleToBall = normalizeAngle(
                            robotToBall.getAngle().minus(robotHeading).getRadians());

                    if (Math.abs(angleToBall) <= Config.PICKUP_ANGLE_RAD) {
                        arena.removeGamePiece(piece);
                        heldBalls++;
                        ballsPickedUp++;

                        if (heldBalls >= Config.MAX_HELD_BALLS) {
                            break;
                        }
                    }
                }
            }
        } catch (Exception e) {
            logRateLimitedError("handlePickup", e);
        }
    }

    /**
     * Handles shot scoring and respawn queue management.
     */
    private void handleShotsAndScoring() {
        try {
            long simScoreCount = Shooter.getInstance().getSimScoreCount();
            long newScores = simScoreCount - lastSimScoreCount;
            lastSimScoreCount = simScoreCount;

            long maxAdditionalScoresAllowed = Math.max(0, shotsConsumedWithBall - score);
            long scoresToApply = Math.min(newScores, maxAdditionalScoresAllowed);

            if (scoresToApply > 0) {
                score += scoresToApply;
                lastShotScored = true;
                // Note: Disabled center half respawn on scores because MapleSim's RebuiltHub
                // already physically recycles scored balls back onto the field through its exit
                // chutes.
            } else if (newScores > 0) {
                lastShotScored = false;
            }
        } catch (Exception e) {
            logRateLimitedError("handleShotsAndScoring", e);
        }
    }

    /**
     * Resets the game to initial state with comprehensive cleanup.
     */
    public void resetGame() {
        try {
            heldBalls = Config.INITIAL_HELD_BALLS;
            var mapleIntake = Intake.getInstance().getMapleIntakeSim();
            if (mapleIntake != null) {
                mapleIntake.setGamePiecesCount(Config.INITIAL_HELD_BALLS);
            }
            score = 0;
            MatchScoreTracker.getInstance().reset();
            simTimeRemainingSec = Config.MATCH_DURATION_SEC;
            simRunning = false;
            lastSimScoreCount = Shooter.getInstance().getSimScoreCount();
            shotsConsumedWithBall = 0;
            lastShotScored = false;
            pendingRespawns = 0;
            lastRespawnTime = 0;
            simLoopCounter = 0;

            // Clear arena cache
            cachedArena = null;
            lastArenaCacheTime = 0;

            SimulatedArena arena = SimulatedArena.getInstance();
            if (arena instanceof Arena2026Rebuilt arena2026) {
                // Keep efficiency mode active: reduces active ball count from 360+ to ~120 on
                // the carpet
                arena2026.setEfficiencyMode(true);
                // Freeze the library's own 25 s hub clock with both hubs physically
                // capturable. Hub allowance/scoring follows the official 6.4
                // schedule (HubSchedule) alone, so arbitrary library flips can
                // never desync shot legality from the rulebook.
                arena2026.setShouldRunClock(false);
            }
            arena.clearGamePieces();
            spawnPickupBalls();

            // Default shift order until the AUTO result seeds it at teleopInit.
            HubSchedule.reset();
            frc.robot.Telemetry.Dashboard.getInstance().setGameData(Config.DEFAULT_GAME_MESSAGE);

            // Reset dashboard commands
            SmartDashboard.putBoolean("Simulation/Reset", false);
            SmartDashboard.putBoolean("Simulation/RespawnBalls", false);

            if (RobotBase.isSimulation()) {
                DriverStationSim.setGameSpecificMessage(Config.DEFAULT_GAME_MESSAGE);
            }
        } catch (Exception e) {
            logRateLimitedError("resetGame", e);
        }
    }

    /**
     * Spawns a ball in the center half of the field with validation.
     */
    private void spawnBallInCenterHalf() {
        try {
            double x = Config.CENTER_HALF_X_MIN + rng.nextDouble() *
                    (Config.CENTER_HALF_X_MAX - Config.CENTER_HALF_X_MIN);
            double y = Config.CENTER_HALF_Y_MIN + rng.nextDouble() *
                    (Config.CENTER_HALF_Y_MAX - Config.CENTER_HALF_Y_MIN);

            SimulatedArena arena = getCachedArena();
            arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
        } catch (Exception e) {
            logRateLimitedError("spawnBallInCenterHalf", e);
        }
    }

    /**
     * Spawns pickup balls using lightweight strategic distribution (54 balls) or
     * full density.
     * Accurately places depot balls inside official 2026 Rebuilt human player depot
     * bays.
     */
    private void spawnPickupBalls() {
        try {
            SimulatedArena arena = SimulatedArena.getInstance();
            arena.clearGamePieces();

            boolean fullDensity = SmartDashboard.getBoolean("Simulation/FullMatchBallDensity", false);
            int depotRows = fullDensity ? 6 : 3;

            // 1. Blue Alliance Depot (Top-Left corner against driver station wall X ~ 0m, Y
            // ~ 5.53m - 6.44m)
            for (int i = 0; i < 4; i++) {
                double x = Config.BLUE_DEPOT_X + (i * Config.BALL_SPACING_X);
                for (int j = 0; j < depotRows; j++) {
                    double y = (fullDensity ? Config.BLUE_DEPOT_FULL_Y : Config.BLUE_DEPOT_Y)
                            + (j * Config.BALL_SPACING_Y);
                    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
                }
            }

            // 2. Red Alliance Depot (Bottom-Right corner against driver station wall X ~
            // 16.54m, Y ~ 1.65m - 2.56m)
            for (int i = 0; i < 4; i++) {
                double x = Config.RED_DEPOT_X + (i * Config.BALL_SPACING_X);
                for (int j = 0; j < depotRows; j++) {
                    double y = (fullDensity ? Config.RED_DEPOT_FULL_Y : Config.RED_DEPOT_Y)
                            + (j * Config.BALL_SPACING_Y);
                    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
                }
            }

            // 3. Center Neutral Zone (Full or lightweight 3x10 grid along the centerline)
            if (fullDensity) {
                // Full center grid: 12 columns x 30 rows
                for (int col = 0; col < 12; col++) {
                    double x = 7.36 + (col * Config.BALL_SPACING_X);
                    for (int row = 0; row < 30; row++) {
                        double y = 1.72 + (row * Config.BALL_SPACING_Y);
                        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
                    }
                }
            } else {
                // Strategic Balanced Physics Mode (30 balls: 3x10 grid along the centerline)
                for (int col = 0; col < 3; col++) {
                    double x = 8.02 + (col * 0.25);
                    for (int row = 0; row < 10; row++) {
                        double y = 1.6 + (row * 0.53);
                        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
                    }
                }
            }
        } catch (Exception e) {
            logRateLimitedError("spawnPickupBalls", e);
        }
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation()) {
            resetGame();
        }
    }

    @Override
    public void log() {
        // Logging is handled in publish() method
    }

    @Override
    public boolean isEnabled() {
        return RobotBase.isSimulation();
    }

    @Override
    public String getName() {
        return "GameSim";
    }

    private static double lastErrorLogTimestamp = 0.0;

    private static void logRateLimitedError(String context, Throwable t) {
        double now = Timer.getFPGATimestamp();
        if (now - lastErrorLogTimestamp > 2.0) {
            lastErrorLogTimestamp = now;
            DriverStation.reportError("GameSim [" + context + "]: " + t.getMessage(), false);
        }
    }
}
