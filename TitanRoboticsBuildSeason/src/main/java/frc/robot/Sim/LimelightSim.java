package frc.robot.Sim;

import java.util.Set;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Data.Constants;

public class LimelightSim {

        // Simple simulation: Assume a tag at the red and blue speaker interactions
        // In a real match, you'd iterate over the AprilTagLayout
        private static final Pose3d BLUE_TAG = new Pose3d(Constants.FieldConstants.BLUE_GOAL_LOCATION,
                        new Rotation3d(0, 0, 0));
        private static final Pose3d RED_TAG = new Pose3d(Constants.FieldConstants.RED_GOAL_LOCATION,
                        new Rotation3d(0, 0, Math.PI));

        // Cached entries to prevent spamming subscribers
        private static final NetworkTable LL_TABLE = NetworkTableInstance.getDefault().getTable("limelight");
        private static final DoubleArrayEntry MT2_ENTRY = LL_TABLE.getDoubleArrayTopic("botpose_orb_wpiblue")
                        .getEntry(new double[0]);
        private static final DoubleArrayEntry BOTPOSE_ENTRY = LL_TABLE.getDoubleArrayTopic("botpose_wpiblue")
                        .getEntry(new double[0]);
        private static final DoubleEntry TID_ENTRY = LL_TABLE.getDoubleTopic("tid")
                        .getEntry(-1.0);

        public static void update(Pose2d robotPose) {
                // Decide which tag we "see" based on proximity
                Pose3d targetTag = null;
                int tagId = -1;

                double distToBlue = robotPose.getTranslation().getDistance(BLUE_TAG.toPose2d().getTranslation());
                double distToRed = robotPose.getTranslation().getDistance(RED_TAG.toPose2d().getTranslation());

                if (distToBlue < distToRed && distToBlue < 8.0) { // Max range 8m
                        targetTag = BLUE_TAG;
                        tagId = 7; // Approx Blue Speaker ID
                } else if (distToRed < 8.0) {
                        targetTag = RED_TAG;
                        tagId = 4; // Approx Red Speaker ID
                }

                if (targetTag == null) {
                        // No tags visible
                        writeEmptyData();
                        return;
                }

                // Add some noise to the robot pose for "realism"
                double noiseX = (Math.random() - 0.5) * 0.05; // +/- 2.5cm
                double noiseY = (Math.random() - 0.5) * 0.05;
                double noiseRot = (Math.random() - 0.5) * Units.degreesToRadians(0.5);

                Pose2d noisyPose = new Pose2d(
                                robotPose.getTranslation().plus(new Translation2d(noiseX, noiseY)),
                                robotPose.getRotation().plus(new edu.wpi.first.math.geometry.Rotation2d(noiseRot)));

                // Calculate "MegaTag2" result (which is basically just the robot pose +
                // timestamp + tag count)
                // We'll simulate a valid MT2 update
                double timestamp = Timer.getFPGATimestamp();
                double latency = 0.020; // 20ms latency

                // Format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist,
                // avgArea,
                // tagID, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
                double[] mt2Entry = new double[11 + (7 * 1)]; // 11 base + 7 per tag
                mt2Entry[0] = noisyPose.getX();
                mt2Entry[1] = noisyPose.getY();
                mt2Entry[2] = 0;
                mt2Entry[3] = 0;
                mt2Entry[4] = 0;
                mt2Entry[5] = noisyPose.getRotation().getDegrees();
                mt2Entry[6] = latency * 1000; // ms
                mt2Entry[7] = 1; // Tag count
                mt2Entry[8] = 0.5; // Span
                mt2Entry[9] = Math.min(distToBlue, distToRed); // Avg Dist
                mt2Entry[10] = 1.0; // Area

                // Per-tag data (First Tag)
                mt2Entry[11] = tagId;
                mt2Entry[12] = 0; // txnc
                mt2Entry[13] = 0; // tync
                mt2Entry[14] = 1.0; // ta
                mt2Entry[15] = mt2Entry[9]; // distToCamera
                mt2Entry[16] = mt2Entry[9]; // distToRobot
                mt2Entry[17] = 0.05; // ambiguity (low)

                // NT4 timestamp needs to be micro seconds
                long ntTimestamp = (long) (timestamp * 1000000);

                MT2_ENTRY.set(mt2Entry, ntTimestamp);
                BOTPOSE_ENTRY.set(mt2Entry, ntTimestamp);
                TID_ENTRY.set(tagId);

                updateBallTracking(robotPose);
        }

        private static void updateBallTracking(Pose2d robotPose) {
                if (!RobotBase.isSimulation()) {
                        return;
                }

                Set<GamePieceOnFieldSimulation> pieces = SimulatedArena.getInstance().gamePiecesOnField();
                GamePieceOnFieldSimulation nearestPiece = null;
                double minDistance = Double.MAX_VALUE;

                for (var piece : pieces) {
                        double dist = piece.getPoseOnField().getTranslation().getDistance(robotPose.getTranslation());
                        if (dist < minDistance) {
                                minDistance = dist;
                                nearestPiece = piece;
                        }
                }

                // If a ball is within 5m, "see" it
                if (nearestPiece != null && minDistance < 5.0) {
                        Translation2d ballOnField = nearestPiece.getPoseOnField().getTranslation();
                        // Calculate relative translation to robot
                        Translation2d relativeBall = ballOnField.minus(robotPose.getTranslation())
                                        .rotateBy(robotPose.getRotation().unaryMinus());

                        // tx is horizontal angle in degrees
                        double tx = Math.toDegrees(Math.atan2(relativeBall.getY(), relativeBall.getX()));
                        // ty is vertical angle, simplified since we're in 2D simulation for now
                        double ty = 0.0;

                        LL_TABLE.getEntry("tv").setDouble(1.0);
                        LL_TABLE.getEntry("tx").setDouble(tx);
                        LL_TABLE.getEntry("ty").setDouble(ty);
                        LL_TABLE.getEntry("ta").setDouble(1.0 / (1.0 + minDistance)); // Area substitute
                } else {
                        LL_TABLE.getEntry("tv").setDouble(0.0);
                }
        }

        private static void writeEmptyData() {
                MT2_ENTRY.set(new double[0]);
                LL_TABLE.getEntry("tv").setDouble(0.0);
        }
}
