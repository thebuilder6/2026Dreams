package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.DrivebaseConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Navigation.FieldMap;
import frc.robot.Subsystems.vision.CameraConfig;
import frc.robot.Subsystems.vision.CameraConfig.CameraRole;
import frc.robot.Subsystems.vision.CameraConfig.CameraType;
import frc.robot.Subsystems.vision.VisionConfig;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.Subsystems.vision.VisionIOPhotonVision;
import frc.robot.Subsystems.vision.VisionIOSim;
import frc.robot.Telemetry.Alert;
import frc.robot.Telemetry.Alert.AlertType;

/**
 * Overhauled Vision Subsystem supporting arbitrary N-camera configurations,
 * declarative camera definitions, live dashboard tuning, multi-tag pose fusion,
 * connection watchdogs, and neural object tracking.
 */
public class Vision implements Subsystem {

    public enum RejectionReason {
        ACCEPTED("Accepted into Pose Estimator"),
        NO_TARGET("No AprilTag Visible"),
        HIGH_DISTANCE("Distance Exceeds Threshold"),
        HIGH_YAW_RATE("Angular Velocity Exceeds Threshold"),
        HIGH_LATENCY("Frame Latency Exceeds Threshold"),
        HIGH_AMBIGUITY("Single-Tag Ambiguity Exceeds Threshold"),
        OUTSIDE_FIELD("Pose Estimate Outside Field Boundaries"),
        CAMERA_OFFLINE("Camera Disconnected or Stale"),
        VISION_DISABLED("Vision Estimation Disabled");

        public final String description;

        RejectionReason(String description) {
            this.description = description;
        }
    }

    /**
     * Managed camera tracker containing state, filters, and telemetry for one camera.
     */
    public static class ManagedCamera {
        private final CameraConfig config;
        private final VisionIO io;
        private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
        private final MedianFilter tagDistFilter = new MedianFilter(5);

        private double filteredTagDist = 0.0;
        private boolean isAccepted = false;
        private double stdDev = 0.0;
        private RejectionReason rejectionReason = RejectionReason.NO_TARGET;
        private double lastFrameTimestamp = 0.0;
        private boolean isConnected = RobotBase.isSimulation();
        private final Alert disconnectAlert;
        private final BooleanEntry enableEntry;

        public ManagedCamera(CameraConfig config, VisionIO io) {
            this.config = config;
            this.io = io;
            this.disconnectAlert = new Alert("Vision", "Camera [" + config.getName() + "] is offline", AlertType.WARNING);
            var nt = NetworkTableInstance.getDefault();
            this.enableEntry = nt.getTable("SmartDashboard")
                    .getSubTable("Vision")
                    .getSubTable(config.getName())
                    .getBooleanTopic("Enabled")
                    .getEntry(config.isEnabled());
            this.enableEntry.setDefault(config.isEnabled());
        }

        public void setEnabled(boolean enabled) {
            config.setEnabled(enabled);
            enableEntry.set(enabled);
        }

        public boolean isEnabled() {
            return config.isEnabled();
        }

        public void updateEnabledFromDashboard() {
            boolean dashboardVal = enableEntry.get(config.isEnabled());
            if (dashboardVal != config.isEnabled()) {
                config.setEnabled(dashboardVal);
            }
        }

        public CameraConfig getConfig() {
            return config;
        }

        public VisionIO getIO() {
            return io;
        }

        public VisionIOInputsAutoLogged getInputs() {
            return inputs;
        }

        public String getName() {
            return config.getName();
        }

        public boolean isAccepted() {
            return isAccepted;
        }

        public double getStdDev() {
            return stdDev;
        }

        public double getFilteredTagDist() {
            return filteredTagDist;
        }

        public RejectionReason getRejectionReason() {
            return rejectionReason;
        }

        public boolean isConnected() {
            return isConnected;
        }

        private void updateConnectionWatchdog(double now) {
            if (inputs.isConnected) {
                lastFrameTimestamp = now;
                isConnected = true;
            } else if (inputs.hasTarget || inputs.tagCount > 0) {
                lastFrameTimestamp = now;
                isConnected = true;
            } else {
                isConnected = (now - lastFrameTimestamp < 1.5) || RobotBase.isSimulation();
            }
            disconnectAlert.set(!isConnected && config.isEnabled());
        }
    }

    private static Vision instance;

    private final List<ManagedCamera> managedCameras = new ArrayList<>();
    private final MedianFilter gamePieceDistFilter = new MedianFilter(5);

    public static synchronized Vision getInstance() {
        if (instance == null) {
            List<CameraConfig> configs = VisionConfig.getCameras();
            List<VisionIO> ios = new ArrayList<>();
            for (CameraConfig cfg : configs) {
                if (RobotBase.isSimulation()) {
                    ios.add(new VisionIOSim(cfg));
                } else if (cfg.getType() == CameraType.LIMELIGHT) {
                    ios.add(new VisionIOLimelight(cfg));
                } else {
                    ios.add(new VisionIOPhotonVision(cfg));
                }
            }
            instance = new Vision(configs, ios);
        }
        return instance;
    }

    /**
     * Primary constructor accepting configured camera definitions and paired IO implementations.
     */
    public Vision(List<CameraConfig> configs, List<VisionIO> ios) {
        for (int i = 0; i < configs.size(); i++) {
            CameraConfig cfg = configs.get(i);
            VisionIO io = (i < ios.size()) ? ios.get(i) : new VisionIOSim(cfg);
            managedCameras.add(new ManagedCamera(cfg, io));
        }
        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Legacy convenience constructor for single-camera setup.
     */
    public Vision(VisionIO primaryIO) {
        this(
                primaryIO,
                RobotBase.isSimulation()
                        ? new VisionIOSim(CameraConfig.photonVision("rubik-pi-coprocessor"))
                        : new VisionIOPhotonVision("rubik-pi-coprocessor"));
    }

    /**
     * Legacy convenience constructor for dual-camera setup.
     */
    public Vision(VisionIO primaryIO, VisionIO secondaryIO) {
        CameraConfig primaryCfg = CameraConfig.limelight("limelight-front")
                .withRole(CameraRole.HYBRID)
                .withStdDevMultiplier(1.0)
                .withMegaTag2(true);
        CameraConfig secondaryCfg = CameraConfig.photonVision("rubik-pi-coprocessor")
                .withRole(CameraRole.HYBRID)
                .withStdDevMultiplier(1.2);

        managedCameras.add(new ManagedCamera(primaryCfg, primaryIO));
        if (secondaryIO != null) {
            managedCameras.add(new ManagedCamera(secondaryCfg, secondaryIO));
        }
        SubsystemManager.registerSubsystem(this);
    }

    @Override
    public void update() {
        VisionConfig.updateTunables();
        boolean masterEnabled = VisionConfig.isMasterEnabled();

        SwerveBase swerve = SwerveBase.getInstance();
        double yawRateDegPerSec = swerve.getGyroYawVelocityDegPerSec();
        double yawRateAbs = Math.abs(yawRateDegPerSec);
        double headingDeg = swerve.getHeading().getDegrees();
        double pitchDeg = swerve.getPitch().getDegrees();
        double now = Timer.getFPGATimestamp();

        for (ManagedCamera camera : managedCameras) {
            camera.updateEnabledFromDashboard();

            // Feed gyro orientation to MegaTag2 Limelights
            if (camera.config.getType() == CameraType.LIMELIGHT && camera.config.isMegaTag2()) {
                camera.io.setRobotOrientation(headingDeg, yawRateDegPerSec, pitchDeg, 0.0);
            }

            // Update IO and record to AdvantageKit
            camera.io.updateInputs(camera.inputs);
            org.littletonrobotics.junction.Logger.processInputs("Vision/" + camera.getName(), camera.inputs);
            camera.updateConnectionWatchdog(now);

            // Gating checks
            if (!masterEnabled || !camera.config.isEnabled()) {
                camera.rejectionReason = RejectionReason.VISION_DISABLED;
                camera.isAccepted = false;
                continue;
            }

            if (!camera.isConnected) {
                camera.rejectionReason = RejectionReason.CAMERA_OFFLINE;
                camera.isAccepted = false;
                continue;
            }

            if (!camera.inputs.hasTarget || camera.inputs.tagCount <= 0) {
                camera.tagDistFilter.reset();
                camera.filteredTagDist = 0.0;
                camera.rejectionReason = RejectionReason.NO_TARGET;
                camera.isAccepted = false;
                continue;
            }

            camera.filteredTagDist = camera.tagDistFilter.calculate(camera.inputs.avgTagDist);

            // Rejection Gates Matrix
            if (yawRateAbs > VisionConfig.MAX_YAW_RATE.get()) {
                camera.rejectionReason = RejectionReason.HIGH_YAW_RATE;
                camera.isAccepted = false;
            } else if (camera.inputs.latencyMs > VisionConfig.MAX_LATENCY_MS.get()) {
                camera.rejectionReason = RejectionReason.HIGH_LATENCY;
                camera.isAccepted = false;
            } else if (camera.filteredTagDist > VisionConfig.MAX_TAG_DIST.get()
                    || (camera.inputs.tagCount == 1 && camera.filteredTagDist > VisionConfig.SINGLE_TAG_MAX_DIST.get())) {
                camera.rejectionReason = RejectionReason.HIGH_DISTANCE;
                camera.isAccepted = false;
            } else if (camera.inputs.tagCount == 1 && camera.inputs.ambiguity > VisionConfig.MAX_AMBIGUITY.get() && camera.inputs.ambiguity > 0.0) {
                camera.rejectionReason = RejectionReason.HIGH_AMBIGUITY;
                camera.isAccepted = false;
            } else {
                // Field Boundary Sanity Gate
                double poseX = camera.inputs.estimatedPose.getX();
                double poseY = camera.inputs.estimatedPose.getY();
                if (poseX < -0.5 || poseX > FieldMap.FIELD_LENGTH + 0.5 || poseY < -0.5 || poseY > FieldMap.FIELD_WIDTH + 0.5) {
                    camera.rejectionReason = RejectionReason.OUTSIDE_FIELD;
                    camera.isAccepted = false;
                } else {
                    camera.rejectionReason = RejectionReason.ACCEPTED;
                    camera.isAccepted = true;
                }
            }

            // Dynamic Standard Deviation Weighting
            if (camera.isAccepted) {
                double stdDev = VisionConfig.BASE_STD_DEV.get();
                if (camera.inputs.tagCount == 1) {
                    stdDev += VisionConfig.SINGLE_TAG_PENALTY.get();
                }
                stdDev += (camera.filteredTagDist * camera.filteredTagDist) / VisionConfig.DIST_PENALTY_DIVISOR.get();
                stdDev *= camera.config.getStdDevMultiplier();
                camera.stdDev = stdDev;

                Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(900));
                swerve.addVisionMeasurement(camera.inputs.estimatedPose, camera.inputs.timestamp, visionStdDevs);
            }
        }
    }

    public List<ManagedCamera> getCameras() {
        return Collections.unmodifiableList(managedCameras);
    }

    public ManagedCamera getCamera(String name) {
        for (ManagedCamera c : managedCameras) {
            if (c.getName().equalsIgnoreCase(name)) {
                return c;
            }
        }
        return null;
    }

    public ManagedCamera getPrimaryCamera() {
        return managedCameras.isEmpty() ? null : managedCameras.get(0);
    }

    public ManagedCamera getSecondaryCamera() {
        return managedCameras.size() > 1 ? managedCameras.get(1) : null;
    }

    public boolean isCameraConnected(String name) {
        ManagedCamera cam = getCamera(name);
        return cam != null && cam.isConnected();
    }

    public boolean isAllCamerasConnected() {
        if (managedCameras.isEmpty()) return false;
        for (ManagedCamera c : managedCameras) {
            if (c.getConfig().isEnabled() && !c.isConnected()) {
                return false;
            }
        }
        return true;
    }

    public void setCameraEnabled(String name, boolean enabled) {
        ManagedCamera cam = getCamera(name);
        if (cam != null) {
            cam.setEnabled(enabled);
        }
    }

    public boolean isCameraEnabled(String name) {
        ManagedCamera cam = getCamera(name);
        return cam != null && cam.isEnabled();
    }

    public RejectionReason getRejectionReason(String name) {
        ManagedCamera cam = getCamera(name);
        return cam != null ? cam.getRejectionReason() : RejectionReason.CAMERA_OFFLINE;
    }

    // ── Target & Pose Queries ────────────────────────────────────────────────

    public boolean hasTarget() {
        for (ManagedCamera c : managedCameras) {
            if (c.inputs.hasTarget && c.inputs.tagCount > 0) return true;
        }
        return false;
    }

    public double getTX() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.inputs.targetTx : 0.0;
    }

    public double getTY() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.inputs.targetTy : 0.0;
    }

    public Pose2d getEstimatedPose() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.inputs.estimatedPose : new Pose2d();
    }

    public boolean isAccepted() {
        for (ManagedCamera c : managedCameras) {
            if (c.isAccepted) return true;
        }
        return false;
    }

    public double getFilteredPrimaryTagDist() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.getFilteredTagDist() : 0.0;
    }

    public double getFilteredSecondaryTagDist() {
        ManagedCamera secondary = getSecondaryCamera();
        return secondary != null ? secondary.getFilteredTagDist() : 0.0;
    }

    public VisionIO getIO() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.getIO() : null;
    }

    public VisionIO getSecondaryIO() {
        ManagedCamera secondary = getSecondaryCamera();
        return secondary != null ? secondary.getIO() : null;
    }

    public VisionIOInputsAutoLogged getInputs() {
        ManagedCamera primary = getPrimaryCamera();
        return primary != null ? primary.getInputs() : new VisionIOInputsAutoLogged();
    }

    public VisionIOInputsAutoLogged getSecondaryInputs() {
        ManagedCamera secondary = getSecondaryCamera();
        return secondary != null ? secondary.getInputs() : new VisionIOInputsAutoLogged();
    }

    // ── Target Estimates & Calibration Stubs ─────────────────────────────────

    public static class VisionTargetEstimate {
        public final Pose3d pose;
        public final double avgTagDist;
        public final double timestampSeconds;

        public VisionTargetEstimate(Pose2d pose2d, double avgTagDist, double timestampSeconds) {
            this.pose = new Pose3d(pose2d);
            this.avgTagDist = avgTagDist;
            this.timestampSeconds = timestampSeconds;
        }
    }

    public VisionTargetEstimate getLimelightTarget() {
        for (ManagedCamera c : managedCameras) {
            if (c.config.getType() == CameraType.LIMELIGHT && c.inputs.hasTarget && c.inputs.tagCount > 0) {
                return new VisionTargetEstimate(c.inputs.estimatedPose, c.inputs.avgTagDist, c.inputs.timestamp);
            }
        }
        return null;
    }

    public VisionTargetEstimate getPhotonTarget() {
        for (ManagedCamera c : managedCameras) {
            if (c.config.getType() == CameraType.PHOTONVISION && c.inputs.hasTarget && c.inputs.tagCount > 0) {
                return new VisionTargetEstimate(c.inputs.estimatedPose, c.inputs.avgTagDist, c.inputs.timestamp);
            }
        }
        return null;
    }

    public VisionTargetEstimate getBestTarget() {
        VisionTargetEstimate ll = getLimelightTarget();
        if (ll != null) return ll;
        return getPhotonTarget();
    }

    public Pose2d getVisionPose() {
        return getEstimatedPose();
    }

    public void setLimelightEnabled(boolean enabled) {
        for (ManagedCamera c : managedCameras) {
            if (c.config.getType() == CameraType.LIMELIGHT) {
                c.io.setEnabled(enabled);
            }
        }
    }

    public void setLimelightLED(String mode) {
        int code = 0;
        if ("force_off".equalsIgnoreCase(mode)) code = 1;
        else if ("blink".equalsIgnoreCase(mode)) code = 2;
        else if ("force_on".equalsIgnoreCase(mode)) code = 3;

        for (ManagedCamera c : managedCameras) {
            if (c.config.getType() == CameraType.LIMELIGHT) {
                c.io.setLEDMode(code);
            }
        }
    }

    public void setPhotonPipeline(int pipeline) {
        for (ManagedCamera c : managedCameras) {
            if (c.config.getType() == CameraType.PHOTONVISION) {
                c.io.setPipeline(pipeline);
            }
        }
    }

    // ── Neural Network Game Piece Object Detection ("Ball Hunt") ─────────────

    private ManagedCamera getGamePieceCamera() {
        // Find first camera detecting a game piece, prioritizing object detection / hybrid roles
        for (ManagedCamera c : managedCameras) {
            if (c.config.getRole() != CameraRole.APRILTAG && c.inputs.hasGamePiece) {
                return c;
            }
        }
        // Fallback: check any camera
        for (ManagedCamera c : managedCameras) {
            if (c.inputs.hasGamePiece) return c;
        }
        return null;
    }

    public boolean hasGamePiece() {
        return getGamePieceCamera() != null;
    }

    public double getGamePieceYaw() {
        ManagedCamera cam = getGamePieceCamera();
        return cam != null ? cam.inputs.gamePieceYaw : 0.0;
    }

    public double getGamePiecePitch() {
        ManagedCamera cam = getGamePieceCamera();
        return cam != null ? cam.inputs.gamePiecePitch : 0.0;
    }

    public double getGamePieceArea() {
        ManagedCamera cam = getGamePieceCamera();
        return cam != null ? cam.inputs.gamePieceArea : 0.0;
    }

    public double getGamePieceDistanceMeters() {
        ManagedCamera cam = getGamePieceCamera();
        if (cam == null) {
            gamePieceDistFilter.reset();
            return 0.0;
        }

        double cameraHeight = cam.config.getCameraHeightMeters();
        double targetHeight = DrivebaseConstants.FUEL_TARGET_HEIGHT_METERS;
        double cameraPitchRads = Math.toRadians(cam.config.getCameraPitchDegrees());
        double targetPitchRads = Units.degreesToRadians(cam.inputs.gamePiecePitch);

        // Fallback to constants if camera mount was not configured
        if (cameraHeight <= 0.01) {
            cameraHeight = DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS;
            cameraPitchRads = Units.degreesToRadians(DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG);
        }

        double totalAngleRads = cameraPitchRads + targetPitchRads;
        if (Math.abs(Math.tan(totalAngleRads)) < 0.01 || totalAngleRads >= 0) {
            return 0.0;
        }

        double rawDist = Math.abs((cameraHeight - targetHeight) / Math.tan(totalAngleRads));
        return gamePieceDistFilter.calculate(rawDist);
    }

    public Translation2d getGamePieceRobotRelativeTranslation() {
        double distance = getGamePieceDistanceMeters();
        if (distance <= 0.01) {
            return new Translation2d();
        }

        ManagedCamera cam = getGamePieceCamera();
        double forwardOffset = cam != null && cam.config.getCameraForwardOffsetMeters() > 0.01
                ? cam.config.getCameraForwardOffsetMeters()
                : DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS;

        double yawRads = -Units.degreesToRadians(getGamePieceYaw()); // CCW positive
        double forward = distance * Math.cos(yawRads) + forwardOffset;
        double left = distance * Math.sin(yawRads);
        return new Translation2d(forward, left);
    }

    public Pose2d getGamePieceFieldPose() {
        if (!hasGamePiece()) {
            return null;
        }
        Pose2d robotPose = SwerveBase.getInstance().getPose();
        Translation2d robotRel = getGamePieceRobotRelativeTranslation();
        Translation2d fieldPos = robotPose.getTranslation().plus(robotRel.rotateBy(robotPose.getRotation()));
        return new Pose2d(fieldPos, robotPose.getRotation());
    }

    public Translation2d registerDetectedBumperObstacle(double targetYaw, double targetPitch, double radius) {
        ManagedCamera cam = getGamePieceCamera();
        double cameraHeight = cam != null && cam.config.getCameraHeightMeters() > 0.01
                ? cam.config.getCameraHeightMeters()
                : DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS;
        double cameraPitchDeg = cam != null
                ? cam.config.getCameraPitchDegrees()
                : DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG;
        double forwardOffset = cam != null && cam.config.getCameraForwardOffsetMeters() > 0.01
                ? cam.config.getCameraForwardOffsetMeters()
                : DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS;

        double bumperHeight = 0.12;
        double cameraPitchRads = Units.degreesToRadians(cameraPitchDeg);
        double targetPitchRads = Units.degreesToRadians(targetPitch);

        double totalAngleRads = cameraPitchRads + targetPitchRads;
        if (totalAngleRads >= 0 || Math.abs(Math.tan(totalAngleRads)) < 0.01) {
            return null;
        }

        double groundDist = Math.abs((cameraHeight - bumperHeight) / Math.tan(totalAngleRads));
        if (groundDist > 7.0 || groundDist < 0.3) {
            return null;
        }

        double yawRads = Units.degreesToRadians(targetYaw);
        double relX = groundDist * Math.cos(yawRads) + forwardOffset;
        double relY = groundDist * Math.sin(yawRads);

        Pose2d robotPose = SwerveBase.getInstance().getPose();
        Translation2d robotRel = new Translation2d(relX, relY);
        Translation2d fieldPos = robotPose.getTranslation().plus(robotRel.rotateBy(robotPose.getRotation()));

        frc.robot.Navigation.DynamicRouter.registerObstacle(fieldPos, new Translation2d(), radius, 0.40);
        return fieldPos;
    }

    // ── Subsystem Interface ──────────────────────────────────────────────────

    @Override
    public void initialize() {}

    @Override
    public void log() {
        // Individual camera status & diagnostics
        for (ManagedCamera c : managedCameras) {
            String pfx = "Vision/" + c.getName() + "/";
            SmartDashboard.putBoolean(pfx + "Connected", c.isConnected);
            SmartDashboard.putBoolean(pfx + "IsAccepted", c.isAccepted);
            SmartDashboard.putString(pfx + "RejectionReason", c.rejectionReason.name());
            SmartDashboard.putNumber(pfx + "TagCount", c.inputs.tagCount);
            SmartDashboard.putNumber(pfx + "AvgDistance", c.inputs.avgTagDist);
            SmartDashboard.putNumber(pfx + "FilteredAvgDistance", c.filteredTagDist);
            SmartDashboard.putNumber(pfx + "StdDev", c.stdDev);
            SmartDashboard.putNumber(pfx + "LatencyMs", c.inputs.latencyMs);

            if (c.inputs.hasTarget) {
                org.littletonrobotics.junction.Logger.recordOutput("Vision/" + c.getName() + "/Pose", c.inputs.estimatedPose);
            }
        }

        // Legacy / Primary / Secondary mappings for Elastic Dashboard compatibility
        ManagedCamera primary = getPrimaryCamera();
        if (primary != null) {
            SmartDashboard.putNumber("Vision/Primary/TagCount", primary.inputs.tagCount);
            SmartDashboard.putNumber("Vision/Primary/AvgDistance", primary.inputs.avgTagDist);
            SmartDashboard.putNumber("Vision/Primary/FilteredAvgDistance", primary.filteredTagDist);
            SmartDashboard.putNumber("Vision/Primary/StdDev", primary.stdDev);
            SmartDashboard.putBoolean("Vision/Primary/IsAccepted", primary.isAccepted);
            SmartDashboard.putBoolean("Vision/Primary/HasTarget", primary.inputs.hasTarget);
            if (primary.inputs.hasTarget) {
                org.littletonrobotics.junction.Logger.recordOutput("Vision/PrimaryPose", primary.inputs.estimatedPose);
            }
        }

        ManagedCamera secondary = getSecondaryCamera();
        if (secondary != null) {
            SmartDashboard.putNumber("Vision/Secondary/TagCount", secondary.inputs.tagCount);
            SmartDashboard.putNumber("Vision/Secondary/AvgDistance", secondary.inputs.avgTagDist);
            SmartDashboard.putNumber("Vision/Secondary/FilteredAvgDistance", secondary.filteredTagDist);
            SmartDashboard.putBoolean("Vision/Secondary/HasTarget", secondary.inputs.hasTarget);
            SmartDashboard.putNumber("Vision/Secondary/LatencyMs", secondary.inputs.latencyMs);
            if (secondary.inputs.hasTarget) {
                org.littletonrobotics.junction.Logger.recordOutput("Vision/SecondaryPose", secondary.inputs.estimatedPose);
            }
        }

        // Neural Network Object Detection Telemetry ("Ball Hunt")
        boolean hasBall = hasGamePiece();
        SmartDashboard.putBoolean("Vision/BallHunt/HasTarget", hasBall);
        SmartDashboard.putNumber("Vision/BallHunt/TargetYaw", getGamePieceYaw());
        SmartDashboard.putNumber("Vision/BallHunt/TargetPitch", getGamePiecePitch());
        SmartDashboard.putNumber("Vision/BallHunt/TargetArea", getGamePieceArea());
        SmartDashboard.putNumber("Vision/BallHunt/DistanceMeters", getGamePieceDistanceMeters());
        org.littletonrobotics.junction.Logger.recordOutput("Vision/BallHunt/HasTarget", hasBall);
        org.littletonrobotics.junction.Logger.recordOutput("Vision/BallHunt/DistanceMeters", getGamePieceDistanceMeters());

        Pose2d ballFieldPose = getGamePieceFieldPose();
        if (ballFieldPose != null) {
            org.littletonrobotics.junction.Logger.recordOutput("Vision/BallHunt/FieldPose", ballFieldPose);
        }
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Vision";
    }
}
