package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

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
    private double nextRespawnTimeSec = -1.0;
    private static final double RESPAWN_DELAY_SEC = 0.3;

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
        resetGame();
        SubsystemManager.registerSubsystem(this);
    }

    public double getSimTimeRemainingSec() {
        return simTimeRemainingSec;
    }

    public synchronized boolean consumeHeldBallForShot() {
        if (!RobotBase.isSimulation()) {
            return true;
        }
        if (heldBalls <= 0) {
            return false;
        }
        heldBalls--;
        shotsConsumedWithBall++;
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void simulationUpdate() {
        if (!RobotBase.isSimulation()) {
            return;
        }

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

        handlePickup();
        handleShotsAndScoring();
        publish();
    }

    private void publish() {
        SmartDashboard.putNumber("Simulation/TimeRemainingSec", simTimeRemainingSec);
        SmartDashboard.putBoolean("Simulation/Running", simRunning);
        SmartDashboard.putNumber("Simulation/Score", score);
        SmartDashboard.putNumber("Simulation/HeldBalls", heldBalls);
        SmartDashboard.putBoolean("Simulation/LastShotScored", lastShotScored);

        // --- AdvantageScope Consolidation ---
        // Get all game pieces on field
        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");

        // Publish as a binary struct array (NT4 protocol)
        // This creates ONE single entry in NT instead of 400+ separate numeric topics.
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

        // Query SimulatedArena for game pieces
        // We iterate through "Fuel" pieces and check distance
        // Since we can't easily get the object list to remove directly without
        // iterating or using a query,
        // we'll use a simpler approach if possible, but for now assuming we can get
        // poses.
        // Actually, SimulatedArena likely doesn't expose a "remove nearest" easily
        // without the object reference.
        // Let's check documentation or assume we can iterate.
        // Docs said: .getGamePiecesByType("Fuel") returns a List of GamePieceOnField

        // Note: Since I don't have the full javadoc for `getGamePiecesByType` return
        // type in the chunk,
        // I will assume it returns a list of objects that have a pose.
        // However, `getGamePiecesArrayByType` returns Pose3d[].

        // The best way to interact is probably to check distance to poses, giving us a
        // hint,
        // but removing them requires the object instance.
        // Docs chunk 6 mentioned:
        // `SimulatedArena.getInstance().getGamePiecesByType("Fuel")`

        // I will use `SimulatedArena.getInstance().removeGamePiece(gamePiece)` if I can
        // find it.
        // I'll try to iterate over the objects.

        Set<GamePieceOnFieldSimulation> pieces = SimulatedArena.getInstance().gamePiecesOnField();

        for (var piece : pieces) {
            Translation2d ball = piece.getPoseOnField().getTranslation();
            double distance = ball.getDistance(robot);

            if (distance <= pickupRadiusM) {
                // Check if ball is in front of robot
                Translation2d robotToBall = ball.minus(robot);
                double angleToBall = robotToBall.getAngle().minus(robotHeading).getRadians();

                // Normalize angle to [-pi, pi]
                while (angleToBall > Math.PI)
                    angleToBall -= 2 * Math.PI;
                while (angleToBall < -Math.PI)
                    angleToBall += 2 * Math.PI;

                // Check if ball is within 90 degrees in front
                if (Math.abs(angleToBall) <= maxPickupAngleRad) {
                    SimulatedArena.getInstance().removeGamePiece(piece);
                    heldBalls++;
                    if (heldBalls >= 8)
                        break; // Limit pickup per loop
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
            // Schedule respawn of scored balls
            nextRespawnTimeSec = Timer.getFPGATimestamp() + RESPAWN_DELAY_SEC;
        } else if (newScores > 0) {
            lastShotScored = false;
        }

        // Handle respawn timer
        double now = Timer.getFPGATimestamp();
        if (nextRespawnTimeSec > 0.0 && now >= nextRespawnTimeSec) {
            spawnBallInCenterHalf();
            nextRespawnTimeSec = -1.0;
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
        nextRespawnTimeSec = -1.0;

        SimulatedArena.getInstance().clearGamePieces();
        spawnPickupBalls();

        SmartDashboard.putBoolean("Simulation/Reset", false);
        SmartDashboard.putBoolean("Simulation/RespawnBalls", false);
    }

    private void spawnBallInCenterHalf() {
        // Center half of the field: X in [6.0, 10.0], Y in [2.0, 6.0]
        double x = 6.0 + rng.nextDouble() * 4.0;
        double y = 2.0 + rng.nextDouble() * 4.0;
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
    }

    private void spawnPickupBalls() {
        SimulatedArena.getInstance().clearGamePieces();

        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 2.0)));
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 4.0)));
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 6.0)));
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(8.0, 2.0)));
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(8.0, 4.0)));
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(8.0, 6.0)));

        // Add a few more in the center half for variety
        for (int i = 0; i < 50; i++) {
            spawnBallInCenterHalf();
        }
    }

    @Override
    public void initialize() {
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
