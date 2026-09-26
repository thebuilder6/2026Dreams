package frc.robot.Hardware.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Sim.LimelightSim;
import frc.robot.Sim.VisionSim;
import frc.robot.Subsystems.SwerveBase;

/**
 * Desktop simulation implementation of VisionIO supporting both Limelight (MegaTag2)
 * and Rubik Pi 3 (PhotonVision + Neural Game Piece tracking via VisionSim).
 */
public class VisionIOSim implements VisionIO {

    public enum CameraType {
        LIMELIGHT,
        RUBIK_PI
    }

    private final CameraType cameraType;
    private Pose2d customSimPose = null;

    // Manual test overrides
    private boolean manualGamePieceOverride = false;
    private boolean simGamePieceDetected = false;
    private double simGamePieceYaw = 0.0;
    private double simGamePiecePitch = 0.0;
    private double simGamePieceArea = 0.0;

    public VisionIOSim(CameraType cameraType) {
        this.cameraType = cameraType;
    }

    public VisionIOSim() {
        this(CameraType.LIMELIGHT);
    }

    public void setSimulatedPose(Pose2d pose) {
        this.customSimPose = pose;
    }

    public void setRobotOrientation(double yaw, double pitch, double roll, double yawRate) {
        // Stub for orientation injection in simulation tests
    }

    public void setGamePieceDetected(boolean detected, double yaw, double pitch, double area) {
        this.manualGamePieceOverride = true;
        this.simGamePieceDetected = detected;
        this.simGamePieceYaw = yaw;
        this.simGamePiecePitch = pitch;
        this.simGamePieceArea = area;
    }

    public void clearManualGamePieceOverride() {
        this.manualGamePieceOverride = false;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d simPose = customSimPose;
        if (simPose == null) {
            try {
                simPose = SwerveBase.getInstance().getSimulationPose();
            } catch (Throwable t) {
                simPose = new Pose2d(4.0, 4.0, new Rotation2d());
            }
        }

        if (cameraType == CameraType.LIMELIGHT) {
            updateLimelightInputs(simPose, inputs);
        } else {
            updateRubikPiInputs(simPose, inputs);
        }
    }

    private void updateLimelightInputs(Pose2d simPose, VisionIOInputs inputs) {
        LimelightSim.update(simPose);

        // Read simulated result
        double[] botPose = NetworkTableInstance.getDefault()
                .getTable("limelight").getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

        if (botPose.length >= 10 && botPose[7] > 0) {
            inputs.hasTarget = true;
            inputs.tagCount = (int) botPose[7];
            inputs.avgTagDist = botPose[9];
            inputs.latencyMs = botPose[6];
            inputs.timestamp = Timer.getFPGATimestamp() - (botPose[6] / 1000.0);
            inputs.estimatedPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
        } else {
            inputs.hasTarget = false;
            inputs.tagCount = 0;
            inputs.avgTagDist = 0.0;
        }

        inputs.targetTx = NetworkTableInstance.getDefault()
                .getTable("limelight").getEntry("tx").getDouble(0.0);
        inputs.targetTy = NetworkTableInstance.getDefault()
                .getTable("limelight").getEntry("ty").getDouble(0.0);
        inputs.targetTa = NetworkTableInstance.getDefault()
                .getTable("limelight").getEntry("ta").getDouble(0.0);

        // Simulated game piece
        if (manualGamePieceOverride) {
            inputs.hasGamePiece = simGamePieceDetected;
            inputs.gamePieceYaw = simGamePieceYaw;
            inputs.gamePiecePitch = simGamePiecePitch;
            inputs.gamePieceArea = simGamePieceArea;
        } else {
            inputs.hasGamePiece = false;
            inputs.gamePieceYaw = 0.0;
            inputs.gamePiecePitch = 0.0;
            inputs.gamePieceArea = 0.0;
        }
    }

    private void updateRubikPiInputs(Pose2d simPose, VisionIOInputs inputs) {
        VisionSim visionSim = VisionSim.getInstance();
        visionSim.update(simPose);

        NetworkTable photonTable = NetworkTableInstance.getDefault()
                .getTable("photonvision").getSubTable("rubik-pi-coprocessor");

        // AprilTag detection from PhotonCameraSim
        boolean hasTag = photonTable.getEntry("hasTarget").getBoolean(false);
        inputs.hasTarget = hasTag;
        inputs.targetTx = photonTable.getEntry("targetYaw").getDouble(0.0);
        inputs.targetTy = photonTable.getEntry("targetPitch").getDouble(0.0);
        inputs.targetTa = photonTable.getEntry("targetArea").getDouble(0.0);
        inputs.latencyMs = photonTable.getEntry("latencyMillis").getDouble(8.0);
        inputs.tagCount = (int) photonTable.getEntry("tagCount").getDouble(hasTag ? 1.0 : 0.0);
        inputs.avgTagDist = photonTable.getEntry("avgDist").getDouble(0.0);

        double[] poseData = photonTable.getEntry("robotPose").getDoubleArray(new double[0]);
        if (poseData.length >= 3 && inputs.tagCount > 0) {
            inputs.estimatedPose = new Pose2d(
                    poseData[0],
                    poseData[1],
                    Rotation2d.fromDegrees(poseData.length >= 6 ? poseData[5] : poseData[2]));
            inputs.timestamp = Timer.getFPGATimestamp() - (inputs.latencyMs / 1000.0);
        }

        // Neural Game Piece Detection (MapleSim SimulatedArena fuel tracking)
        if (manualGamePieceOverride) {
            inputs.hasGamePiece = simGamePieceDetected;
            inputs.gamePieceYaw = simGamePieceYaw;
            inputs.gamePiecePitch = simGamePiecePitch;
            inputs.gamePieceArea = simGamePieceArea;
        } else if (visionSim.hasSimulatedNeuralTarget()) {
            inputs.hasGamePiece = true;
            inputs.gamePieceYaw = visionSim.getSimulatedNeuralYaw();
            inputs.gamePiecePitch = visionSim.getSimulatedNeuralPitch();
            inputs.gamePieceArea = visionSim.getSimulatedNeuralArea();
        } else {
            inputs.hasGamePiece = false;
            inputs.gamePieceYaw = 0.0;
            inputs.gamePiecePitch = 0.0;
            inputs.gamePieceArea = 0.0;
        }
    }

    public CameraType getCameraType() {
        return cameraType;
    }
}
