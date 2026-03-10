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
        static final int MAX_HELD_BALLS = 50;
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
        
        // Field boundaries for validation
        static final double FIELD_X_MIN = 0.0;
        static final double FIELD_X_MAX = 16.54;
        static final double FIELD_Y_MIN = 0.0;
        static final double FIELD_Y_MAX = 8.02;
        
        // Initial game state
        static final int INITIAL_HELD_BALLS = 8;
        static final String DEFAULT_GAME_MESSAGE = "R";
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

    /**
     * Consumes held balls for shooting with validation and error handling.
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
        return toConsume;
    }

    @Override
    public void update() {
    }

    /**
     * Main simulation update loop with improved error handling and performance.
     */
    @Override
    public void simulationUpdate() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        try {
            simLoopCounter++;

            handleDashboardCommands();
            updateSimulationTime();
            
            // Optimize: Check pickup every 3 loops (~60ms)
            if (simLoopCounter % Config.PICKUP_CHECK_INTERVAL == 0) {
                handlePickup();
            }

            handleShotsAndScoring();

            // Throttle publishing to 10Hz
            double now = Timer.getFPGATimestamp();
            if (now - lastPublishTime >= Config.PUBLISH_INTERVAL_SEC) {
                publish();
                lastPublishTime = now;
            }
        } catch (Exception e) {
            System.err.println("GameSim: Error in simulationUpdate: " + e.getMessage());
            e.printStackTrace();
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
            System.err.println("GameSim: Error handling dashboard commands: " + e.getMessage());
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
            }
        } catch (Exception e) {
            System.err.println("GameSim: Error updating simulation time: " + e.getMessage());
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

            // --- Rebuilt 2026 Specific Telemetry ---
            SimulatedArena arena = getCachedArena();
            if (arena instanceof Arena2026Rebuilt) {
                Arena2026Rebuilt arena2026 = (Arena2026Rebuilt) arena;
                SmartDashboard.putBoolean("Simulation/HubActive/Blue", arena2026.isActive(true));
                SmartDashboard.putBoolean("Simulation/HubActive/Red", arena2026.isActive(false));
            }

            // --- AdvantageScope Consolidation ---
            Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
            gamePiecePublisher.set(fuelPoses);
        } catch (Exception e) {
            System.err.println("GameSim: Error in publish: " + e.getMessage());
        }
    }
    
    /**
     * Gets cached SimulatedArena instance for performance.
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
     * @param angle Input angle in radians
     * @return Normalized angle in [-PI, PI]
     */
    private static double normalizeAngle(double angle) {
        // Use Math.IEEEremainder for efficient angle normalization
        double normalized = Math.IEEEremainder(angle, 2 * Math.PI);
        return normalized;
    }

    /**
     * Handles ball pickup logic with improved performance and validation.
     */
    private void handlePickup() {
        if (heldBalls >= Config.MAX_HELD_BALLS) {
            return;
        }

        try {
            if (Intake.getInstance().getState() != Intake.IntakeState.INTAKING) {
                return;
            }

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
                        robotToBall.getAngle().minus(robotHeading).getRadians()
                    );

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
            System.err.println("GameSim: Error in handlePickup: " + e.getMessage());
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
                pendingRespawns += scoresToApply;
            } else if (newScores > 0) {
                lastShotScored = false;
            }

            // Handle respawn queue with throttling
            double now = Timer.getFPGATimestamp();
            if (pendingRespawns > 0 && now - lastRespawnTime >= Config.MIN_RESPAWN_INTERVAL) {
                spawnBallInCenterHalf();
                pendingRespawns--;
                lastRespawnTime = now;
            }
        } catch (Exception e) {
            System.err.println("GameSim: Error in handleShotsAndScoring: " + e.getMessage());
        }
    }

    /**
     * Resets the game to initial state with comprehensive cleanup.
     */
    public void resetGame() {
        try {
            heldBalls = Config.INITIAL_HELD_BALLS;
            score = 0;
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
            arena.clearGamePieces();
            spawnPickupBalls();

            // Reset dashboard commands
            SmartDashboard.putBoolean("Simulation/Reset", false);
            SmartDashboard.putBoolean("Simulation/RespawnBalls", false);

            if (RobotBase.isSimulation()) {
                DriverStationSim.setGameSpecificMessage(Config.DEFAULT_GAME_MESSAGE);
            }
        } catch (Exception e) {
            System.err.println("GameSim: Error in resetGame: " + e.getMessage());
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
            System.err.println("GameSim: Error in spawnBallInCenterHalf: " + e.getMessage());
        }
    }

    /**
     * Spawns pickup balls using official 2026 layout with field boundary validation.
     */
    private void spawnPickupBalls() {
        try {
            SimulatedArena arena = SimulatedArena.getInstance();
            arena.clearGamePieces();
            arena.placeGamePiecesOnField();

            // Pruning: Remove "Outpost" balls in the corners (human player stations)
            // while keeping the ground balls (center) and staging balls (depots).
            Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
            List<GamePieceOnFieldSimulation> toRemove = new ArrayList<>();

            for (var piece : pieces) {
                Translation2d pos = piece.getPoseOnField().getTranslation();
                double x = pos.getX();
                double y = pos.getY();

                // Remove anything literally outside the field boundaries (safety)
                if (x < Config.FIELD_X_MIN || x > Config.FIELD_X_MAX || 
                    y < Config.FIELD_Y_MIN || y > Config.FIELD_Y_MAX) {
                    toRemove.add(piece);
                }
            }

            for (var piece : toRemove) {
                arena.removeGamePiece(piece);
            }
        } catch (Exception e) {
            System.err.println("GameSim: Error in spawnPickupBalls: " + e.getMessage());
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
}
