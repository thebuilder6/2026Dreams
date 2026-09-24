package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.ThirdParty.LimelightHelpers;

/**
 * Real hardware implementation of VisionIO using Limelight MegaTag2.
 */
public class VisionIOLimelight implements VisionIO {

    private final String cameraName;

    public VisionIOLimelight(String cameraName) {
        this.cameraName = cameraName;
    }

    public VisionIOLimelight() {
        this("limelight-front");
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = LimelightHelpers.getTV(cameraName);
        inputs.targetTx = LimelightHelpers.getTX(cameraName);
        inputs.targetTy = LimelightHelpers.getTY(cameraName);
        inputs.targetTa = LimelightHelpers.getTA(cameraName);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        if (mt2 != null && mt2.tagCount > 0) {
            inputs.tagCount = mt2.tagCount;
            inputs.avgTagDist = mt2.avgTagDist;
            inputs.timestamp = mt2.timestampSeconds;
            inputs.latencyMs = mt2.latency;
            inputs.estimatedPose = mt2.pose;
        } else {
            inputs.tagCount = 0;
            inputs.avgTagDist = 0.0;
        }
    }

    @Override
    public void setRobotOrientation(double yawDeg, double yawRateDegPerSec, double pitchDeg, double rollDeg) {
        LimelightHelpers.SetRobotOrientation(cameraName, yawDeg, yawRateDegPerSec, pitchDeg, 0, rollDeg, 0);
    }
}
