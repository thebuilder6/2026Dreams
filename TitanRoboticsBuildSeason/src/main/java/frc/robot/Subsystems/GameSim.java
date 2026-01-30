package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;

public class GameSim implements Subsystem {

    private static GameSim instance;

    private final List<Translation2d> pickupBalls = new ArrayList<>();

    private int heldBalls = 0;
    private int score = 0;

    private double simTimeRemainingSec = 150.0;
    private boolean simRunning = false;
    private double lastTimestampSec = -1.0;

    private long lastSimScoreCount = 0;
    private long shotsConsumedWithBall = 0;
    private boolean lastShotScored = false;
    private final Random rng = new Random();
    private double nextRespawnTimeSec = -1.0;
    private static final double RESPAWN_DELAY_SEC = 2.0;

    public static GameSim getInstance() {
        if (instance == null) {
            instance = new GameSim();
        }
        return instance;
    }

    private GameSim() {
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

        double nowSec = Timer.getFPGATimestamp();
        double dt = (lastTimestampSec < 0.0) ? 0.02 : (nowSec - lastTimestampSec);
        lastTimestampSec = nowSec;

        boolean reset = SmartDashboard.getBoolean("Game/Reset", false);
        if (reset) {
            SmartDashboard.putBoolean("Game/Reset", false);
            resetGame();
        }

        boolean respawn = SmartDashboard.getBoolean("Game/RespawnBalls", false);
        if (respawn) {
            SmartDashboard.putBoolean("Game/RespawnBalls", false);
            spawnPickupBalls();
        }

        double dsTimeRemainingSec = DriverStation.getMatchTime();
        boolean isDsTimeValid = dsTimeRemainingSec >= 0.0;
        SmartDashboard.putBoolean("Game/TimeRemainingValid", isDsTimeValid);

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
                simTimeRemainingSec = Math.max(0.0, simTimeRemainingSec - dt);
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
        SmartDashboard.putNumber("Game/TimeRemainingSec", simTimeRemainingSec);
        SmartDashboard.putBoolean("Game/Running", simRunning);
        SmartDashboard.putNumber("Game/Score", score);
        SmartDashboard.putNumber("Game/HeldBalls", heldBalls);
        SmartDashboard.putNumber("Game/BallsRemaining", pickupBalls.size());
        SmartDashboard.putBoolean("Game/LastShotScored", lastShotScored);

        List<Pose2d> poses = new ArrayList<>(pickupBalls.size());
        List<Pose3d> poses3d = new ArrayList<>(pickupBalls.size());
        for (Translation2d t : pickupBalls) {
            poses.add(new Pose2d(t, new Rotation2d()));
            // Add 3D poses with balls on the ground (z=0)
            poses3d.add(new Pose3d(new Translation3d(t.getX(), t.getY(), 0.0), new Rotation3d()));
        }
        SwerveBase.getInstance().getField().getObject("PickupBalls").setPoses(poses);
        
        // Publish 3D poses for better visualization
        double[] flatPoses = new double[poses3d.size() * 7];
        for (int i = 0; i < poses3d.size(); i++) {
            Pose3d p = poses3d.get(i);
            int idx = i * 7;
            flatPoses[idx] = p.getX();
            flatPoses[idx + 1] = p.getY();
            flatPoses[idx + 2] = p.getZ();
            flatPoses[idx + 3] = p.getRotation().getQuaternion().getW();
            flatPoses[idx + 4] = p.getRotation().getQuaternion().getX();
            flatPoses[idx + 5] = p.getRotation().getQuaternion().getY();
            flatPoses[idx + 6] = p.getRotation().getQuaternion().getZ();
        }
        SmartDashboard.putNumberArray("Game/PickupBallPoses3d", flatPoses);
    }

    private void handlePickup() {
        if (heldBalls >= 5) {
            return;
        }

        if (Intake.getInstance().getState() != Intake.IntakeState.INTAKING) {
            return;
        }

        Pose2d robotPose = SwerveBase.getInstance().getPose();
        Translation2d robot = robotPose.getTranslation();
        Rotation2d robotHeading = robotPose.getRotation();

        double pickupRadiusM = 0.45;
        double maxPickupAngleRad = Math.PI / 2; // 90 degrees in front

        for (int i = 0; i < pickupBalls.size(); i++) {
            Translation2d ball = pickupBalls.get(i);
            double distance = ball.getDistance(robot);
            
            if (distance <= pickupRadiusM) {
                // Check if ball is in front of robot
                Translation2d robotToBall = ball.minus(robot);
                double angleToBall = robotToBall.getAngle().minus(robotHeading).getRadians();
                
                // Normalize angle to [-pi, pi]
                while (angleToBall > Math.PI) angleToBall -= 2 * Math.PI;
                while (angleToBall < -Math.PI) angleToBall += 2 * Math.PI;
                
                // Check if ball is within 90 degrees in front
                if (Math.abs(angleToBall) <= maxPickupAngleRad) {
                    pickupBalls.remove(i);
                    heldBalls++;
                    break;
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

    private void resetGame() {
        heldBalls = 8;
        score = 0;
        simTimeRemainingSec = 150.0;
        simRunning = false;
        lastTimestampSec = -1.0;
        lastSimScoreCount = Shooter.getInstance().getSimScoreCount();
        shotsConsumedWithBall = 0;
        lastShotScored = false;
        nextRespawnTimeSec = -1.0;
        spawnPickupBalls();

        SmartDashboard.putBoolean("Game/Reset", false);
        SmartDashboard.putBoolean("Game/RespawnBalls", false);
    }

    private void spawnBallInCenterHalf() {
        // Center half of the field: X in [4.0, 8.0], Y in [2.0, 6.0]
        double x = 4.0 + rng.nextDouble() * 4.0;
        double y = 2.0 + rng.nextDouble() * 4.0;
        pickupBalls.add(new Translation2d(x, y));
    }

    private void spawnPickupBalls() {
        pickupBalls.clear();
        pickupBalls.add(new Translation2d(7.0, 2.0));
        pickupBalls.add(new Translation2d(7.0, 4.0));
        pickupBalls.add(new Translation2d(7.0, 6.0));
        pickupBalls.add(new Translation2d(9.0, 2.0));
        pickupBalls.add(new Translation2d(9.0, 4.0));
        pickupBalls.add(new Translation2d(9.0, 6.0));
        // Add a few more in the center half for variety
        for (int i = 0; i < 10; i++) {
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
