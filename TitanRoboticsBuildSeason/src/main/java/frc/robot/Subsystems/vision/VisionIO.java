package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Hardware IO abstraction interface for Vision coprocessors (Limelight / PhotonVision).
 * Follows the AdvantageKit pattern to isolate network calls and camera simulation.
 */
public interface VisionIO {

    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public int tagCount = 0;
        public double avgTagDist = 0.0;
        public double timestamp = 0.0;
        public double latencyMs = 0.0;
        public Pose2d estimatedPose = new Pose2d();
        public double targetTx = 0.0;
        public double targetTy = 0.0;
        public double targetTa = 0.0;

        // Neural Network Object Detection ("Ball Hunt" YOLOv8 pipeline)
        public boolean hasGamePiece = false;
        public double gamePieceYaw = 0.0;
        public double gamePiecePitch = 0.0;
        public double gamePieceArea = 0.0;
    }

    /** Updates inputs struct from hardware camera or simulation. */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** Feeds robot gyro orientation to the vision system for MegaTag2 localization. */
    public default void setRobotOrientation(double yawDeg, double yawRateDegPerSec, double pitchDeg, double rollDeg) {}
}
