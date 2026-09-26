package frc.robot.Hardware.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Hardware IO implementation for PhotonVision running on a coprocessor (Orange Pi 5).
 * Interfaces with NetworkTables for:
 * 1. Multi-Tag PNP AprilTag pose estimation.
 * 2. YOLOv8 Neural Network object detection ("Ball Hunt" game piece targeting).
 */
public class VisionIOPhotonVision implements VisionIO {

    private final String cameraName;
    private final NetworkTable cameraTable;
    private final NetworkTable neuralTable;

    // Multi-tag pose entries
    private final DoubleArrayEntry robotPoseEntry;
    private final DoubleEntry latencyEntry;
    private final DoubleEntry tagCountEntry;
    private final DoubleEntry avgDistEntry;

    // AprilTag targeting entries
    private final BooleanEntry hasTargetEntry;
    private final DoubleEntry targetYawEntry;
    private final DoubleEntry targetPitchEntry;
    private final DoubleEntry targetAreaEntry;

    // Neural Network (YOLOv8 game piece detector) entries
    private final BooleanEntry neuralHasTargetEntry;
    private final DoubleEntry neuralYawEntry;
    private final DoubleEntry neuralPitchEntry;
    private final DoubleEntry neuralAreaEntry;

    public VisionIOPhotonVision(String cameraName) {
        this.cameraName = cameraName;
        NetworkTable root = NetworkTableInstance.getDefault().getTable("photonvision");
        this.cameraTable = root.getSubTable(cameraName);
        this.neuralTable = root.getSubTable(cameraName + "-neural");

        this.robotPoseEntry = cameraTable.getDoubleArrayTopic("robotPose").getEntry(new double[0]);
        this.latencyEntry = cameraTable.getDoubleTopic("latencyMillis").getEntry(0.0);
        this.tagCountEntry = cameraTable.getDoubleTopic("tagCount").getEntry(0.0);
        this.avgDistEntry = cameraTable.getDoubleTopic("avgDist").getEntry(0.0);

        this.hasTargetEntry = cameraTable.getBooleanTopic("hasTarget").getEntry(false);
        this.targetYawEntry = cameraTable.getDoubleTopic("targetYaw").getEntry(0.0);
        this.targetPitchEntry = cameraTable.getDoubleTopic("targetPitch").getEntry(0.0);
        this.targetAreaEntry = cameraTable.getDoubleTopic("targetArea").getEntry(0.0);

        this.neuralHasTargetEntry = neuralTable.getBooleanTopic("hasTarget").getEntry(false);
        this.neuralYawEntry = neuralTable.getDoubleTopic("targetYaw").getEntry(0.0);
        this.neuralPitchEntry = neuralTable.getDoubleTopic("targetPitch").getEntry(0.0);
        this.neuralAreaEntry = neuralTable.getDoubleTopic("targetArea").getEntry(0.0);
    }

    public VisionIOPhotonVision() {
        this("rubik-pi-coprocessor");
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Read AprilTag telemetry
        boolean hasTarget = hasTargetEntry.get(false);
        inputs.hasTarget = hasTarget;
        inputs.targetTx = targetYawEntry.get(0.0);
        inputs.targetTy = targetPitchEntry.get(0.0);
        inputs.targetTa = targetAreaEntry.get(0.0);
        inputs.latencyMs = latencyEntry.get(0.0);
        inputs.tagCount = (int) tagCountEntry.get(0.0);
        inputs.avgTagDist = avgDistEntry.get(0.0);

        double[] poseData = robotPoseEntry.get(new double[0]);
        if (poseData.length >= 3 && inputs.tagCount > 0) {
            inputs.estimatedPose = new Pose2d(
                    poseData[0],
                    poseData[1],
                    Rotation2d.fromDegrees(poseData.length >= 6 ? poseData[5] : poseData[2]));
            inputs.timestamp = Timer.getFPGATimestamp() - (inputs.latencyMs / 1000.0);
        } else {
            inputs.hasTarget = false;
            inputs.tagCount = 0;
            inputs.estimatedPose = new Pose2d();
        }

        // Read Neural Network Object Detection ("Ball Hunt")
        boolean hasObject = neuralHasTargetEntry.get(false);
        inputs.hasGamePiece = hasObject;
        inputs.gamePieceYaw = neuralYawEntry.get(0.0);
        inputs.gamePiecePitch = neuralPitchEntry.get(0.0);
        inputs.gamePieceArea = neuralAreaEntry.get(0.0);
    }

    public String getCameraName() {
        return cameraName;
    }
}
