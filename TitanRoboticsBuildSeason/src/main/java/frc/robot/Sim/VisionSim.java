package frc.robot.Sim;

import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Data.Constants.DrivebaseConstants;

/**
 * PhotonVision simulation engine combining:
 * 1. PhotonLib VisionSystemSim + PhotonCameraSim for AprilTag localization.
 * 2. MapleSim SimulatedArena game piece tracking for neural "Ball Hunt".
 */
public class VisionSim {

    private static VisionSim instance;

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;

    // Neural Network simulated table entries for Rubik Pi 3
    private final NetworkTable neuralTable;
    private final BooleanEntry neuralHasTarget;
    private final DoubleEntry neuralYaw;
    private final DoubleEntry neuralPitch;
    private final DoubleEntry neuralArea;
    private final DoubleEntry neuralDistance;

    // Simulation target tracking state
    private boolean lastTargetFound = false;
    private double lastTargetYaw = 0.0;
    private double lastTargetPitch = 0.0;
    private double lastTargetArea = 0.0;
    private double lastTargetDist = 0.0;

    public static synchronized VisionSim getInstance() {
        if (instance == null) {
            instance = new VisionSim("rubik-pi-coprocessor");
        }
        return instance;
    }

    public VisionSim(String cameraName) {
        this.visionSim = new VisionSystemSim("main");

        // Load 2026 Rebuilt field tags with safe fallback
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Throwable t) {
            try {
                layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            } catch (Throwable ignored) {
            }
        }
        if (layout != null) {
            visionSim.addAprilTags(layout);
        }

        // Calibrate simulated camera for Rubik Pi 3 (Qualcomm QCS6490 NPU)
        // 960x720, 70 deg FOV, 90 FPS, 8ms latency
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(70.0));
        cameraProp.setCalibError(0.20, 0.05);
        cameraProp.setFPS(90.0);
        cameraProp.setAvgLatencyMs(8.0);
        cameraProp.setLatencyStdDevMs(2.0);

        this.camera = new PhotonCamera(cameraName);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Robot-to-camera mount transform
        Transform3d robotToCamera = new Transform3d(
                new Translation3d(
                        DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS,
                        0.0,
                        DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS),
                new Rotation3d(0.0, Units.degreesToRadians(DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG), 0.0));

        visionSim.addCamera(cameraSim, robotToCamera);

        // Setup neural NetworkTables topics
        NetworkTable root = NetworkTableInstance.getDefault().getTable("photonvision");
        this.neuralTable = root.getSubTable(cameraName + "-neural");
        this.neuralHasTarget = neuralTable.getBooleanTopic("hasTarget").getEntry(false);
        this.neuralYaw = neuralTable.getDoubleTopic("targetYaw").getEntry(0.0);
        this.neuralPitch = neuralTable.getDoubleTopic("targetPitch").getEntry(0.0);
        this.neuralArea = neuralTable.getDoubleTopic("targetArea").getEntry(0.0);
        this.neuralDistance = neuralTable.getDoubleTopic("targetDistance").getEntry(0.0);
    }

    /**
     * Updates simulation of AprilTags and MapleSim Fuel neural detections.
     *
     * @param robotPose Ground truth robot pose on the field
     */
    public void update(Pose2d robotPose) {
        if (!RobotBase.isSimulation() || robotPose == null) {
            return;
        }

        // 1. Update PhotonVision VisionSystemSim for AprilTags
        visionSim.update(robotPose);

        // 2. Update Neural Fuel Game Piece Tracking from MapleSim
        updateNeuralFuelTracking(robotPose);
    }

    private void updateNeuralFuelTracking(Pose2d robotPose) {
        SimulatedArena arena;
        try {
            arena = SimulatedArena.getInstance();
            if (arena == null) {
                clearNeuralTarget();
                return;
            }
        } catch (Throwable t) {
            clearNeuralTarget();
            return;
        }

        Set<GamePieceOnFieldSimulation> pieces = arena.gamePiecesOnField();
        if (pieces == null || pieces.isEmpty()) {
            clearNeuralTarget();
            return;
        }

        // Camera geometry
        double camX = robotPose.getX() + DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS * robotPose.getRotation().getCos();
        double camY = robotPose.getY() + DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS * robotPose.getRotation().getSin();
        double camHeight = DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS;
        double camPitchDeg = DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG; // -15 deg
        double targetHeight = DrivebaseConstants.FUEL_TARGET_HEIGHT_METERS; // 0.075m

        GamePieceOnFieldSimulation bestPiece = null;
        double closestDist = Double.MAX_VALUE;
        double bestYaw = 0.0;
        double bestPitch = 0.0;

        for (GamePieceOnFieldSimulation piece : pieces) {
            Translation2d piecePos = piece.getPoseOnField().getTranslation();
            double dxField = piecePos.getX() - camX;
            double dyField = piecePos.getY() - camY;
            double dist2d = Math.hypot(dxField, dyField);

            // Maximum range 6 meters, minimum range 0.2m
            if (dist2d > 6.0 || dist2d < 0.2) {
                continue;
            }

            // Transform into robot-relative heading
            double angleToTarget = Math.atan2(dyField, dxField);
            double yawRad = angleToTarget - robotPose.getRotation().getRadians();
            // Wrap to [-PI, PI]
            yawRad = Math.IEEEremainder(yawRad, 2.0 * Math.PI);
            double yawDeg = Math.toDegrees(yawRad);

            // Horizontal camera FOV check (+/- 35 degrees)
            if (Math.abs(yawDeg) > 35.0) {
                continue;
            }

            // Vertical pitch calculation relative to camera optical axis
            // Angle of line-of-sight relative to horizontal:
            double alphaDeg = Math.toDegrees(Math.atan2(targetHeight - camHeight, dist2d));
            // Target pitch relative to camera center axis:
            double pitchDeg = alphaDeg - camPitchDeg;

            // Vertical FOV check (+/- 25 degrees)
            if (Math.abs(pitchDeg) > 25.0) {
                continue;
            }

            if (dist2d < closestDist) {
                closestDist = dist2d;
                bestPiece = piece;
                bestYaw = yawDeg;
                bestPitch = pitchDeg;
            }
        }

        if (bestPiece != null) {
            lastTargetFound = true;
            lastTargetYaw = bestYaw;
            lastTargetPitch = bestPitch;
            lastTargetDist = closestDist;
            lastTargetArea = Math.min(1.0, 1.0 / (1.0 + closestDist));

            neuralHasTarget.set(true);
            neuralYaw.set(lastTargetYaw);
            neuralPitch.set(lastTargetPitch);
            neuralArea.set(lastTargetArea);
            neuralDistance.set(lastTargetDist);
        } else {
            clearNeuralTarget();
        }
    }

    private void clearNeuralTarget() {
        lastTargetFound = false;
        lastTargetYaw = 0.0;
        lastTargetPitch = 0.0;
        lastTargetArea = 0.0;
        lastTargetDist = 0.0;

        neuralHasTarget.set(false);
        neuralYaw.set(0.0);
        neuralPitch.set(0.0);
        neuralArea.set(0.0);
        neuralDistance.set(0.0);
    }

    public boolean hasSimulatedNeuralTarget() {
        return lastTargetFound;
    }

    public double getSimulatedNeuralYaw() {
        return lastTargetYaw;
    }

    public double getSimulatedNeuralPitch() {
        return lastTargetPitch;
    }

    public double getSimulatedNeuralArea() {
        return lastTargetArea;
    }

    public double getSimulatedNeuralDistance() {
        return lastTargetDist;
    }

    public VisionSystemSim getVisionSystemSim() {
        return visionSim;
    }

    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }
}
