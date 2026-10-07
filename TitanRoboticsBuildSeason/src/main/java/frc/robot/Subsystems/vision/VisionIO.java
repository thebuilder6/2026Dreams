package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware IO abstraction interface for Vision coprocessors (Limelight / PhotonVision).
 * Follows the AdvantageKit pattern to isolate network calls and camera simulation.
 */
public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public boolean isConnected = false;
        public boolean hasTarget = false;
        public int tagCount = 0;
        public double avgTagDist = 0.0;
        public double timestamp = 0.0;
        public double latencyMs = 0.0;
        public Pose2d estimatedPose = new Pose2d();
        public double ambiguity = 0.0;
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

    /** Sets the robot-to-camera mount transform in robot coordinates. */
    public default void setCameraPose(Transform3d robotToCamera) {}

    /** Sets the camera pipeline index. */
    public default void setPipeline(int pipeline) {}

    /** Sets LED mode (e.g. 0=pipeline, 1=off, 2=blink, 3=on). */
    public default void setLEDMode(int mode) {}

    /** Enables or disables camera streaming/processing. */
    public default void setEnabled(boolean enabled) {}

    /** Returns the identifier/name of this camera. */
    public default String getName() { return ""; }
}
