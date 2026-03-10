package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.DrivebaseConstants;
import frc.robot.Interfaces.Subsystem;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

/**
 * Vision Subsystem
 * Logic:
 * 1. Processes Limelight/PhotonVision data.
 * 2. Implements rejection logic based on angular velocity, distance, and
 * ambiguity.
 * 3. Updates SwerveBase pose estimator using MegaTag2.
 */
public class Vision implements Subsystem {

    private static Vision instance;
    private final Limelight camera;
    private final LimelightPoseEstimator poseEstimator;

    // Logging data
    private int tagCount = 0;
    private double avgDist = 0;
    private double stdDev = 0;
    private boolean isAccepted = false;
    private boolean hasTarget = false;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        camera = new Limelight("limelight-front");
        poseEstimator = camera.createPoseEstimator(EstimationMode.MEGATAG2);
        SubsystemManager.registerSubsystem(this);
    }

    @Override
    public void update() {
        updateVisionOdometry();
    }

    private void updateVisionOdometry() {
        SwerveBase swerve = SwerveBase.getInstance();
        AngularVelocity yawRate = swerve.getSwerveDrive().getGyro().getYawAngularVelocity();

        // Update robot orientation in LL for MT2
        camera.getSettings()
                .withRobotOrientation(new Orientation3d(
                        new Rotation3d(0, 0, swerve.getPose().getRotation().getRadians()),
                        yawRate,
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0)))
                .save();

        PoseEstimate mt2 = poseEstimator.getPoseEstimate().orElse(null);

        if (mt2 == null || mt2.tagCount == 0) {
            hasTarget = false;
            isAccepted = false;
            tagCount = 0;
            avgDist = 0;
            return;
        }

        hasTarget = true;
        tagCount = mt2.tagCount;
        avgDist = mt2.avgTagDist;

        boolean doRejectUpdate = false;

        // 1. Angular Velocity Rejection
        if (Math.abs(yawRate.in(DegreesPerSecond)) > DrivebaseConstants.VISION_MAX_YAW_RATE) {
            doRejectUpdate = true;
        }

        // 2. Tag Distance Rejection
        if (mt2.avgTagDist > DrivebaseConstants.VISION_MAX_TAG_DIST) {
            doRejectUpdate = true;
        }

        // 3. Single Tag Rejection at distance or high ambiguity
        if (mt2.tagCount == 1) {
            if (mt2.avgTagDist > DrivebaseConstants.VISION_SINGLE_TAG_MAX_DIST) {
                doRejectUpdate = true;
            }

            for (RawFiducial tag : mt2.rawFiducials) {
                if (tag != null && tag.ambiguity > DrivebaseConstants.VISION_MAX_AMBIGUITY) {
                    doRejectUpdate = true;
                }
            }
        }

        // Calculate dynamic trust (Standard Deviation)
        stdDev = DrivebaseConstants.VISION_BASE_STD_DEV;
        if (mt2.tagCount == 1) {
            stdDev += DrivebaseConstants.VISION_SINGLE_TAG_PENALTY;
        }
        stdDev += (mt2.avgTagDist * mt2.avgTagDist) / DrivebaseConstants.VISION_DIST_PENALTY_DIVISOR;

        isAccepted = !doRejectUpdate;

        if (isAccepted) {
            Matrix<N3, N1> visionStdDevs = edu.wpi.first.math.VecBuilder.fill(stdDev, stdDev,
                    Units.degreesToRadians(900));
            swerve.addVisionMeasurement(mt2.pose.toPose2d(), mt2.timestampSeconds, visionStdDevs);
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void simulationUpdate() {
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Vision/TagCount", tagCount);
        SmartDashboard.putNumber("Vision/AvgDistance", avgDist);
        SmartDashboard.putNumber("Vision/StdDev", stdDev);
        SmartDashboard.putBoolean("Vision/IsAccepted", isAccepted);
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    }

    public boolean hasTarget() {
        return camera.getData().targetData.getTargetStatus();
    }

    public double getTX() {
        return camera.getData().targetData.getHorizontalOffset();
    }

    /**
     * Get the latest Limelight target
     */
    public PoseEstimate getLimelightTarget() {
        return poseEstimator.getPoseEstimate().orElse(null);
    }

    /**
     * Get the latest PhotonVision target (placeholder implementation)
     */
    public PoseEstimate getPhotonTarget() {
        // This would integrate with PhotonVision if available
        // For now, return the same as Limelight
        return getLimelightTarget();
    }

    /**
     * Get the best available target from any vision source
     */
    public PoseEstimate getBestTarget() {
        return getLimelightTarget();
    }

    /**
     * Get the latest vision pose estimate
     */
    public Pose2d getVisionPose() {
        PoseEstimate estimate = getLimelightTarget();
        return estimate != null ? estimate.pose.toPose2d() : null;
    }

    /**
     * Enable or disable Limelight
     */
    public void setLimelightEnabled(boolean enabled) {
        // Simplified LED control - just log for now
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
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Vision";
    }
}
