package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.ThirdParty.LimelightHelpers;

/**
 * Real hardware implementation of VisionIO using Limelight (MegaTag2 and MegaTag1).
 * Pushes physical camera mount offsets to Limelight and supports live telemetry & control.
 */
public class VisionIOLimelight implements VisionIO {

    private final String cameraName;
    private final CameraConfig config;
    private double lastHeartbeat = -1.0;
    private double lastHeartbeatChangeTime = 0.0;

    public VisionIOLimelight(CameraConfig config) {
        this.config = config;
        this.cameraName = config.getName();
        applyCameraPose(config.getRobotToCamera());
    }

    public VisionIOLimelight(String cameraName) {
        this(CameraConfig.limelight(cameraName));
    }

    public VisionIOLimelight() {
        this("limelight-front");
    }

    private void applyCameraPose(Transform3d robotToCamera) {
        if (robotToCamera == null) return;
        LimelightHelpers.setCameraPose_RobotSpace(
                cameraName,
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ()));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Heartbeat / connection check
        double hb = LimelightHelpers.getLimelightNTDouble(cameraName, "hb");
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (hb != lastHeartbeat && hb > 0) {
            lastHeartbeat = hb;
            lastHeartbeatChangeTime = now;
            inputs.isConnected = true;
        } else {
            inputs.isConnected = (now - lastHeartbeatChangeTime < 1.0);
        }

        inputs.hasTarget = LimelightHelpers.getTV(cameraName);
        inputs.targetTx = LimelightHelpers.getTX(cameraName);
        inputs.targetTy = LimelightHelpers.getTY(cameraName);
        inputs.targetTa = LimelightHelpers.getTA(cameraName);

        // Fetch pose estimate: prefer MegaTag2 if configured, fall back to MegaTag1
        LimelightHelpers.PoseEstimate estimate = null;
        if (config == null || config.isMegaTag2()) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        }
        if (estimate == null || estimate.tagCount == 0) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        }

        if (estimate != null && estimate.tagCount > 0) {
            inputs.hasTarget = true;
            inputs.tagCount = estimate.tagCount;
            inputs.avgTagDist = estimate.avgTagDist;
            inputs.timestamp = estimate.timestampSeconds;
            inputs.latencyMs = estimate.latency;
            inputs.estimatedPose = estimate.pose;
            if (estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
                inputs.ambiguity = estimate.rawFiducials[0].ambiguity;
            } else {
                inputs.ambiguity = 0.0;
            }
        } else {
            inputs.tagCount = 0;
            inputs.avgTagDist = 0.0;
            inputs.ambiguity = 0.0;
        }
    }

    @Override
    public void setRobotOrientation(double yawDeg, double yawRateDegPerSec, double pitchDeg, double rollDeg) {
        LimelightHelpers.SetRobotOrientation(cameraName, yawDeg, yawRateDegPerSec, pitchDeg, 0.0, rollDeg, 0.0);
    }

    @Override
    public void setCameraPose(Transform3d robotToCamera) {
        applyCameraPose(robotToCamera);
    }

    @Override
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(cameraName, pipeline);
    }

    @Override
    public void setLEDMode(int mode) {
        switch (mode) {
            case 1:
                LimelightHelpers.setLEDMode_ForceOff(cameraName);
                break;
            case 2:
                LimelightHelpers.setLEDMode_ForceBlink(cameraName);
                break;
            case 3:
                LimelightHelpers.setLEDMode_ForceOn(cameraName);
                break;
            default:
                LimelightHelpers.setLEDMode_PipelineControl(cameraName);
                break;
        }
    }

    @Override
    public void setEnabled(boolean enabled) {
        // Limelight stream mode or processing toggle
        LimelightHelpers.setStreamMode_Standard(cameraName);
    }

    @Override
    public String getName() {
        return cameraName;
    }
}
