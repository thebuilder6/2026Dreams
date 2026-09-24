package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.DrivebaseConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.Subsystems.vision.VisionIOPhotonVision;
import frc.robot.Subsystems.vision.VisionIOSim;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Vision Subsystem following the AdvantageKit IO abstraction pattern.
 * Manages camera inputs, MegaTag2 pose gating, and std-dev dynamic weighting.
 */
public class Vision implements Subsystem {

    private static Vision instance;

    private final VisionIO primaryIO;
    private final VisionIO secondaryIO;
    private final VisionIOInputs primaryInputs = new VisionIOInputs();
    private final VisionIOInputs secondaryInputs = new VisionIOInputs();

    // Telemetry state
    private double stdDev = 0;
    private boolean isAccepted = false;

    public static Vision getInstance() {
        if (instance == null) {
            VisionIO primary = RobotBase.isSimulation()
                    ? new VisionIOSim(VisionIOSim.CameraType.LIMELIGHT)
                    : new VisionIOLimelight("limelight-front");
            VisionIO secondary = RobotBase.isSimulation()
                    ? new VisionIOSim(VisionIOSim.CameraType.RUBIK_PI)
                    : new VisionIOPhotonVision("rubik-pi-coprocessor");
            instance = new Vision(primary, secondary);
        }
        return instance;
    }

    public Vision(VisionIO primaryIO) {
        this(primaryIO, RobotBase.isSimulation()
                ? new VisionIOSim(VisionIOSim.CameraType.RUBIK_PI)
                : new VisionIOPhotonVision("rubik-pi-coprocessor"));
    }

    public Vision(VisionIO primaryIO, VisionIO secondaryIO) {
        this.primaryIO = primaryIO;
        this.secondaryIO = secondaryIO;
        SubsystemManager.registerSubsystem(this);
    }

    @Override
    public void update() {
        SwerveBase swerve = SwerveBase.getInstance();
        double yawRateDegPerSec = swerve.getGyroYawVelocityDegPerSec();
        double yawRateAbs = Math.abs(yawRateDegPerSec);

        // ── 1. Primary Camera (Limelight MegaTag2) ───────────────────────────
        primaryIO.setRobotOrientation(
                swerve.getHeading().getDegrees(),
                yawRateDegPerSec,
                swerve.getPitch().getDegrees(),
                0.0);

        primaryIO.updateInputs(primaryInputs);

        if (primaryInputs.hasTarget && primaryInputs.tagCount > 0) {
            boolean doReject = false;
            if (yawRateAbs > DrivebaseConstants.VISION_MAX_YAW_RATE) doReject = true;
            if (primaryInputs.avgTagDist > DrivebaseConstants.VISION_MAX_TAG_DIST) doReject = true;
            if (primaryInputs.latencyMs > 150.0) doReject = true;

            stdDev = DrivebaseConstants.VISION_BASE_STD_DEV;
            if (primaryInputs.tagCount == 1) stdDev += DrivebaseConstants.VISION_SINGLE_TAG_PENALTY;
            stdDev += (primaryInputs.avgTagDist * primaryInputs.avgTagDist) / DrivebaseConstants.VISION_DIST_PENALTY_DIVISOR;

            isAccepted = !doReject;
            if (isAccepted) {
                Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDev, stdDev, Units.degreesToRadians(900));
                swerve.addVisionMeasurement(primaryInputs.estimatedPose, primaryInputs.timestamp, visionStdDevs);
            }
        } else {
            isAccepted = false;
        }

        // ── 2. Secondary Coprocessor (Orange Pi 5 PhotonVision) ─────────────
        if (secondaryIO != null) {
            secondaryIO.updateInputs(secondaryInputs);

            if (secondaryInputs.hasTarget && secondaryInputs.tagCount > 0 && secondaryInputs.latencyMs < 150.0) {
                if (secondaryInputs.avgTagDist < DrivebaseConstants.VISION_MAX_TAG_DIST && yawRateAbs <= DrivebaseConstants.VISION_MAX_YAW_RATE) {
                    double secStdDev = DrivebaseConstants.VISION_BASE_STD_DEV + 0.15;
                    if (secondaryInputs.tagCount == 1) secStdDev += DrivebaseConstants.VISION_SINGLE_TAG_PENALTY;
                    secStdDev += (secondaryInputs.avgTagDist * secondaryInputs.avgTagDist) / DrivebaseConstants.VISION_DIST_PENALTY_DIVISOR;

                    Matrix<N3, N1> secStdDevs = VecBuilder.fill(secStdDev, secStdDev, Units.degreesToRadians(900));
                    swerve.addVisionMeasurement(secondaryInputs.estimatedPose, secondaryInputs.timestamp, secStdDevs);
                }
            }
        }
    }

    public boolean hasTarget() {
        return primaryInputs.hasTarget || (secondaryInputs != null && secondaryInputs.hasTarget);
    }

    public boolean hasGamePiece() {
        return (secondaryInputs != null && secondaryInputs.hasGamePiece) || primaryInputs.hasGamePiece;
    }

    public double getGamePieceYaw() {
        if (secondaryInputs != null && secondaryInputs.hasGamePiece) {
            return secondaryInputs.gamePieceYaw;
        }
        return primaryInputs.gamePieceYaw;
    }

    public double getGamePiecePitch() {
        if (secondaryInputs != null && secondaryInputs.hasGamePiece) {
            return secondaryInputs.gamePiecePitch;
        }
        return primaryInputs.gamePiecePitch;
    }

    public double getGamePieceArea() {
        if (secondaryInputs != null && secondaryInputs.hasGamePiece) {
            return secondaryInputs.gamePieceArea;
        }
        return primaryInputs.gamePieceArea;
    }

    /**
     * Calculates distance from camera to the game piece on the carpet using trigonometry.
     * d = (h_camera - h_target) / tan(pitch_camera + pitch_target)
     * @return Distance in meters, or 0.0 if no game piece is detected
     */
    public double getGamePieceDistanceMeters() {
        if (!hasGamePiece()) {
            return 0.0;
        }
        double cameraHeight = DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS;
        double targetHeight = DrivebaseConstants.FUEL_TARGET_HEIGHT_METERS;
        double cameraPitchRads = Units.degreesToRadians(DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG);
        double targetPitchRads = Units.degreesToRadians(getGamePiecePitch());

        double totalAngleRads = cameraPitchRads + targetPitchRads;
        // Avoid division by zero or negative distances if looking parallel/up
        if (Math.abs(Math.tan(totalAngleRads)) < 0.01 || totalAngleRads >= 0) {
            return 0.0;
        }

        // Camera is higher than target, total angle is negative (tilted down)
        return Math.abs((cameraHeight - targetHeight) / Math.tan(totalAngleRads));
    }

    /**
     * Calculates the robot-relative (X, Y) Translation2d to the detected game piece.
     * @return Translation2d in robot frame (+X forward, +Y left)
     */
    public edu.wpi.first.math.geometry.Translation2d getGamePieceRobotRelativeTranslation() {
        double distance = getGamePieceDistanceMeters();
        if (distance <= 0.01) {
            return new edu.wpi.first.math.geometry.Translation2d();
        }
        double yawRads = -Units.degreesToRadians(getGamePieceYaw()); // CCW positive
        double forward = distance * Math.cos(yawRads) + DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS;
        double left = distance * Math.sin(yawRads);
        return new edu.wpi.first.math.geometry.Translation2d(forward, left);
    }

    /**
     * Projects the detected game piece to global field coordinates using current odometry.
     * @return Pose2d of game piece on field, or null if no target detected
     */
    public Pose2d getGamePieceFieldPose() {
        if (!hasGamePiece()) {
            return null;
        }
        Pose2d robotPose = SwerveBase.getInstance().getPose();
        edu.wpi.first.math.geometry.Translation2d robotRel = getGamePieceRobotRelativeTranslation();
        edu.wpi.first.math.geometry.Translation2d fieldPos = robotPose.getTranslation().plus(
                robotRel.rotateBy(robotPose.getRotation()));
        return new Pose2d(fieldPos, robotPose.getRotation());
    }

    /**
     * Projects a detected robot bumper bounding box onto the field ground plane (Z = 0)
     * and registers it with DynamicRouter.
     *
     * @param targetYaw Camera-relative horizontal angle (degrees, +left)
     * @param targetPitch Camera-relative vertical angle (degrees, +up)
     * @param radius Bumper bounding radius (meters)
     * @return Global field Translation2d of the obstacle center, or null if invalid
     */
    public edu.wpi.first.math.geometry.Translation2d registerDetectedBumperObstacle(
            double targetYaw, double targetPitch, double radius) {

        double cameraHeight = DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS; // 0.45m
        double bumperHeight = 0.12; // Bumper center approx 12cm off carpet
        double cameraPitchRads = Units.degreesToRadians(DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG); // -15 deg
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
        double relX = groundDist * Math.cos(yawRads) + DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS;
        double relY = groundDist * Math.sin(yawRads);

        Pose2d robotPose = SwerveBase.getInstance().getPose();
        edu.wpi.first.math.geometry.Translation2d robotRel = new edu.wpi.first.math.geometry.Translation2d(relX, relY);
        edu.wpi.first.math.geometry.Translation2d fieldPos = robotPose.getTranslation().plus(robotRel.rotateBy(robotPose.getRotation()));

        frc.robot.Auto.DynamicRouter.registerObstacle(fieldPos, new edu.wpi.first.math.geometry.Translation2d(), radius, 0.40);
        return fieldPos;
    }

    public double getTX() {
        return primaryInputs.targetTx;
    }

    public double getTY() {
        return primaryInputs.targetTy;
    }

    public Pose2d getEstimatedPose() {
        return primaryInputs.estimatedPose;
    }

    public boolean isAccepted() {
        return isAccepted;
    }

    public VisionIO getIO() {
        return primaryIO;
    }

    public VisionIO getSecondaryIO() {
        return secondaryIO;
    }

    public VisionIOInputs getInputs() {
        return primaryInputs;
    }

    public VisionIOInputs getSecondaryInputs() {
        return secondaryInputs;
    }

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

    /**
     * Get the latest Limelight target pose estimate
     */
    public VisionTargetEstimate getLimelightTarget() {
        if (!primaryInputs.hasTarget || primaryInputs.tagCount == 0) {
            return null;
        }
        return new VisionTargetEstimate(primaryInputs.estimatedPose, primaryInputs.avgTagDist, primaryInputs.timestamp);
    }

    /**
     * Get the latest PhotonVision target
     */
    public VisionTargetEstimate getPhotonTarget() {
        if (secondaryInputs == null || !secondaryInputs.hasTarget || secondaryInputs.tagCount == 0) {
            return null;
        }
        return new VisionTargetEstimate(secondaryInputs.estimatedPose, secondaryInputs.avgTagDist, secondaryInputs.timestamp);
    }

    /**
     * Get the best available target from any vision source
     */
    public VisionTargetEstimate getBestTarget() {
        VisionTargetEstimate primary = getLimelightTarget();
        if (primary != null) return primary;
        return getPhotonTarget();
    }

    /**
     * Get the latest vision pose estimate
     */
    public Pose2d getVisionPose() {
        return getEstimatedPose();
    }

    /**
     * Enable or disable Limelight
     */
    public void setLimelightEnabled(boolean enabled) {
        System.out.println("[Vision] Limelight enabled: " + enabled);
    }

    /**
     * Set Limelight LED mode
     */
    public void setLimelightLED(String mode) {
        System.out.println("[Vision] Limelight LED mode: " + mode);
    }

    /**
     * Set PhotonVision pipeline
     */
    public void setPhotonPipeline(int pipeline) {
        System.out.println("[Vision] PhotonVision pipeline: " + pipeline);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Vision/Primary/TagCount", primaryInputs.tagCount);
        SmartDashboard.putNumber("Vision/Primary/AvgDistance", primaryInputs.avgTagDist);
        SmartDashboard.putNumber("Vision/Primary/StdDev", stdDev);
        SmartDashboard.putBoolean("Vision/Primary/IsAccepted", isAccepted);
        SmartDashboard.putBoolean("Vision/Primary/HasTarget", primaryInputs.hasTarget);

        if (primaryInputs.hasTarget) {
            org.littletonrobotics.junction.Logger.recordOutput("Vision/PrimaryPose", primaryInputs.estimatedPose);
        }

        if (secondaryIO != null) {
            SmartDashboard.putNumber("Vision/Secondary/TagCount", secondaryInputs.tagCount);
            SmartDashboard.putNumber("Vision/Secondary/AvgDistance", secondaryInputs.avgTagDist);
            SmartDashboard.putBoolean("Vision/Secondary/HasTarget", secondaryInputs.hasTarget);
            SmartDashboard.putNumber("Vision/Secondary/LatencyMs", secondaryInputs.latencyMs);
            if (secondaryInputs.hasTarget) {
                org.littletonrobotics.junction.Logger.recordOutput("Vision/SecondaryPose", secondaryInputs.estimatedPose);
            }
        }

        // Neural Network Object Detection Telemetry ("Ball Hunt" YOLOv8 / Rubik Pi 3)
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
