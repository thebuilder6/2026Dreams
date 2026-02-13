package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

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

//jdt://contents/YAGSL-java-2026.1.14.jar/swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026/Arena2026Rebuilt.class?=TitanRoboticsBuildSeason/C:\/Users\/jumpi\/.gradle\/caches\/modules-2\/files-2.1\/swervelib\/YAGSL-java\/2026.1.14\/2d5926d32cee7003bb639b2000ad1afc3ccb0db9\/YAGSL-java-2026.1.14.jar=/gradle_used_by_scope=/main,test=/<swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026(Arena2026Rebuilt.class

public class GameSim implements Subsystem {

    private static GameSim instance;

    private int heldBalls = 0;
    private int score = 0;

    private double simTimeRemainingSec = 150.0;
    private boolean simRunning = false;
    private final StructArrayPublisher<Pose3d> gamePiecePublisher;

    private long lastSimScoreCount = 0;
    private long shotsConsumedWithBall = 0;
    private boolean lastShotScored = false;
    private final Random rng = new Random();
    private int pendingRespawns = 0;
    private double lastRespawnTime = 0;
    private static final double MIN_RESPAWN_INTERVAL = 0.2;

    private int simLoopCounter = 0;
    private double lastPublishTime = 0;
    private static final double PUBLISH_INTERVAL_SEC = 0.1; // 10Hz publishing

    public static GameSim getInstance() {
        if (instance == null) {
            instance = new GameSim();
        }
        return instance;
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

    public synchronized int consumeHeldBallsForShot(int maxToConsume) {
        if (!RobotBase.isSimulation()) {
            return maxToConsume;
        }
        int toConsume = Math.min(heldBalls, maxToConsume);
        heldBalls -= toConsume;
        shotsConsumedWithBall += toConsume;
        return toConsume;
    }

    @Override
    public void update() {
    }

    @Override
    public void simulationUpdate() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        simLoopCounter++;

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
                simTimeRemainingSec = Math.max(0.0, simTimeRemainingSec - 0.02);
                if (simTimeRemainingSec <= 0.0) {
                    simRunning = false;
                }
            }
        }

        // Optimize: Check pickup every 3 loops (~60ms)
        if (simLoopCounter % 3 == 0) {
            handlePickup();
        }

        handleShotsAndScoring();

        // Throttle publishing to 10Hz
        double now = Timer.getFPGATimestamp();
        if (now - lastPublishTime >= PUBLISH_INTERVAL_SEC) {
            publish();
            lastPublishTime = now;
        }
    }

    private void publish() {
        SmartDashboard.putNumber("Simulation/TimeRemainingSec", simTimeRemainingSec);
        SmartDashboard.putBoolean("Simulation/Running", simRunning);
        SmartDashboard.putNumber("Simulation/Score", score);
        SmartDashboard.putNumber("Simulation/HeldBalls", heldBalls);
        SmartDashboard.putBoolean("Simulation/LastShotScored", lastShotScored);

        // --- Rebuilt 2026 Specific Telemetry ---
        if (SimulatedArena.getInstance() instanceof Arena2026Rebuilt) {
            Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
            SmartDashboard.putBoolean("Simulation/HubActive/Blue", arena.isActive(true));
            SmartDashboard.putBoolean("Simulation/HubActive/Red", arena.isActive(false));

            // The arena manages its own clock in simulationSubTick
            // We can surface it here if it's not already on NT (it is, but let's
            // centralize)
        }

        // --- AdvantageScope Consolidation ---
        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");

        gamePiecePublisher.set(fuelPoses);
    }

    private void handlePickup() {
        if (heldBalls >= 50) {
            return;
        }

        if (Intake.getInstance().getState() != Intake.IntakeState.INTAKING) {
            return;
        }

        Pose2d robotPose = SwerveBase.getInstance().getSimulationPose();
        Translation2d robot = robotPose.getTranslation();
        Rotation2d robotHeading = robotPose.getRotation();

        double pickupRadiusM = 0.45;
        double maxPickupAngleRad = Math.PI / 2; // 90 degrees in front

        Set<GamePieceOnFieldSimulation> pieces = SimulatedArena.getInstance().gamePiecesOnField();

        for (var piece : pieces) {
            Translation2d ball = piece.getPoseOnField().getTranslation();
            double distance = ball.getDistance(robot);

            if (distance <= pickupRadiusM) {
                Translation2d robotToBall = ball.minus(robot);
                double angleToBall = robotToBall.getAngle().minus(robotHeading).getRadians();

                while (angleToBall > Math.PI)
                    angleToBall -= 2 * Math.PI;
                while (angleToBall < -Math.PI)
                    angleToBall += 2 * Math.PI;

                if (Math.abs(angleToBall) <= maxPickupAngleRad) {
                    SimulatedArena.getInstance().removeGamePiece(piece);
                    heldBalls++;
                    if (heldBalls >= 10)
                        break; // Limit pickup per check
                }
            }
        }
    }

    private void handleShotsAndScoring() {
        long simScoreCount = Shooter.getInstance().getSimScoreCount();
        long newScores = simScoreCount - lastSimScoreCount;
        lastSimScoreCount = simScoreCount;

        long maxAdditionalScoresAllowed = Math.max(0, shotsConsumedWithBall - score);
        long scoresToApply = Math.min(newScores, maxAdditionalScoresAllowed);

        if (scoresToApply > 0) {
            score += scoresToApply;
            lastShotScored = true;
            pendingRespawns += scoresToApply; // Increment queue for all scored balls
        } else if (newScores > 0) {
            lastShotScored = false;
        }

        // Handle respawn queue with a throttle
        double now = Timer.getFPGATimestamp();
        if (pendingRespawns > 0 && now - lastRespawnTime >= MIN_RESPAWN_INTERVAL) {
            spawnBallInCenterHalf();
            pendingRespawns--;
            lastRespawnTime = now;
        }
    }

    public void resetGame() {
        heldBalls = 8;
        score = 0;
        simTimeRemainingSec = 150.0;
        simRunning = false;
        lastSimScoreCount = Shooter.getInstance().getSimScoreCount();
        shotsConsumedWithBall = 0;
        lastShotScored = false;
        pendingRespawns = 0;
        lastRespawnTime = 0;
        simLoopCounter = 0;

        SimulatedArena.getInstance().clearGamePieces();
        spawnPickupBalls();

        SmartDashboard.putBoolean("Simulation/Reset", false);
        SmartDashboard.putBoolean("Simulation/RespawnBalls", false);

        if (RobotBase.isSimulation()) {
            // Set a default game data for 2026 hub shifts: 'R' (Red starts inactive)
            DriverStationSim.setGameSpecificMessage("R");
        }
    }

    private void spawnBallInCenterHalf() {
        // Center half of the field: X in [6.0, 10.0], Y in [2.0, 6.0]
        double x = 6.0 + rng.nextDouble() * 4.0;
        double y = 2.0 + rng.nextDouble() * 4.0;
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
    }

    private void spawnPickupBalls() {
        // Use library's official 2026 layout to maintain "official" ground balls
        SimulatedArena.getInstance().clearGamePieces();
        SimulatedArena.getInstance().placeGamePiecesOnField();

        // Pruning: Remove "Outpost" balls in the corners (human player stations)
        // while keeping the ground balls (center) and staging balls (depots).
        Set<GamePieceOnFieldSimulation> pieces = SimulatedArena.getInstance().gamePiecesOnField();
        List<GamePieceOnFieldSimulation> toRemove = new ArrayList<>();

        for (var piece : pieces) {
            Translation2d pos = piece.getPoseOnField().getTranslation();
            double x = pos.getX();
            double y = pos.getY();

            // 1. Remove anything literally outside the field boundaries (safety)
            if (x < 0 || x > 16.54 || y < 0 || y > 8.02) {
                toRemove.add(piece);
            }
        }

        for (var piece : toRemove) {
            SimulatedArena.getInstance().removeGamePiece(piece);
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
