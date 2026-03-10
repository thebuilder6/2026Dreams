package frc.robot.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.TunableNumber;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Vision;

/**
 * Vision system testing and calibration.
 * 
 * Features:
 * - Camera target detection testing
 * - Limelight functionality validation
 * - PhotonVision integration testing
 * - Vision-based odometry testing
 * - Target tracking accuracy
 */
public class VisionTesting {
    
    private enum TestMode {
        LIMELIGHT_TESTING,
        PHOTONVISION_TESTING,
        TARGET_TRACKING,
        VISION_ODOMETRY,
        CAMERA_CALIBRATION
    }
    
    private TestMode currentTestMode = TestMode.LIMELIGHT_TESTING;
    private boolean testRunning = false;
    private double testStartTime = 0;
    
    // Vision subsystem reference
    private final Vision vision;
    private final SwerveBase swerve;
    
    // Test parameters
    private final TunableNumber targetDistance = new TunableNumber("Test/Vision/TargetDistance", 3.0);
    private final TunableNumber targetAngle = new TunableNumber("Test/Vision/TargetAngle", 0.0);
    private final TunableNumber confidenceThreshold = new TunableNumber("Test/Vision/ConfidenceThreshold", 0.9);
    
    // Performance tracking
    private boolean hasValidTarget = false;
    private double lastTargetDistance = 0;
    private double lastTargetAngle = 0;
    private double lastLatency = 0;
    private int targetDetections = 0;
    private int totalFrames = 0;
    
    public VisionTesting() {
        vision = Vision.getInstance();
        swerve = SwerveBase.getInstance();
        setupDashboard();
    }
    
    /**
     * Update vision testing based on controller input
     */
    public void update(Controller driverController, Controller operatorController) {
        handleModeSwitching(driverController);
        
        switch (currentTestMode) {
            case LIMELIGHT_TESTING:
                handleLimelightTesting(driverController, operatorController);
                break;
            case PHOTONVISION_TESTING:
                handlePhotonVisionTesting(driverController, operatorController);
                break;
            case TARGET_TRACKING:
                handleTargetTracking(driverController, operatorController);
                break;
            case VISION_ODOMETRY:
                handleVisionOdometry(driverController, operatorController);
                break;
            case CAMERA_CALIBRATION:
                handleCameraCalibration(driverController, operatorController);
                break;
        }
        
        updatePerformanceMetrics();
    }
    
    /**
     * Handle test mode switching
     */
    private void handleModeSwitching(Controller driverController) {
        // Use A, B, X, Y for mode switching
        if (driverController.getAButtonPressed()) {
            currentTestMode = TestMode.LIMELIGHT_TESTING;
            System.out.println("[VisionTesting] Mode: LIMELIGHT_TESTING");
            resetTest();
        } else if (driverController.getBButtonPressed()) {
            currentTestMode = TestMode.PHOTONVISION_TESTING;
            System.out.println("[VisionTesting] Mode: PHOTONVISION_TESTING");
            resetTest();
        } else if (driverController.getXButtonPressed()) {
            currentTestMode = TestMode.TARGET_TRACKING;
            System.out.println("[VisionTesting] Mode: TARGET_TRACKING");
            resetTest();
        } else if (driverController.getYButtonPressed()) {
            currentTestMode = TestMode.VISION_ODOMETRY;
            System.out.println("[VisionTesting] Mode: VISION_ODOMETRY");
            resetTest();
        }
        
        // Camera calibration with both bumpers
        if (driverController.getLeftBumperButton() && driverController.getRightBumperButton()) {
            currentTestMode = TestMode.CAMERA_CALIBRATION;
            System.out.println("[VisionTesting] Mode: CAMERA_CALIBRATION");
            resetTest();
        }
    }
    
    /**
     * Limelight testing mode
     */
    private void handleLimelightTesting(Controller driverController, Controller operatorController) {
        // Test Limelight functionality
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Enable Limelight and get target data
            vision.setLimelightEnabled(true);
            
            // Get target information
            var target = vision.getLimelightTarget();
            if (target != null) {
                hasValidTarget = true;
                lastTargetDistance = target.avgTagDist;
                lastTargetAngle = Math.toDegrees(target.pose.getRotation().getZ());
                lastLatency = target.timestampSeconds * 1000; // Convert to milliseconds
                targetDetections++;
            } else {
                hasValidTarget = false;
            }
            
            totalFrames++;
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            vision.setLimelightEnabled(false);
            testRunning = false;
        }
        
        // Switch LED modes with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Force on
                vision.setLimelightLED("force_on");
                break;
            case 90: // Right - Force off
                vision.setLimelightLED("force_off");
                break;
            case 180: // Down - Blink
                vision.setLimelightLED("blink");
                break;
            case 270: // Left - Pipeline default
                vision.setLimelightLED("pipeline");
                break;
        }
    }
    
    /**
     * PhotonVision testing mode
     */
    private void handlePhotonVisionTesting(Controller driverController, Controller operatorController) {
        // Test PhotonVision functionality
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Enable PhotonVision and get target data
            var target = vision.getPhotonTarget();
            
            if (target != null) {
                hasValidTarget = true;
                lastTargetDistance = target.avgTagDist;
                lastTargetAngle = Math.toDegrees(target.pose.getRotation().getZ());
                lastLatency = target.timestampSeconds * 1000; // Convert to milliseconds
                targetDetections++;
            } else {
                hasValidTarget = false;
            }
            
            totalFrames++;
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            testRunning = false;
        }
        
        // Switch pipeline with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Pipeline 0
                vision.setPhotonPipeline(0);
                break;
            case 90: // Right - Pipeline 1
                vision.setPhotonPipeline(1);
                break;
            case 180: // Down - Pipeline 2
                vision.setPhotonPipeline(2);
                break;
            case 270: // Left - Pipeline 3
                vision.setPhotonPipeline(3);
                break;
        }
    }
    
    /**
     * Target tracking test mode
     */
    private void handleTargetTracking(Controller driverController, Controller operatorController) {
        // Test automatic target tracking
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Enable target tracking
            var target = vision.getBestTarget();
            
            if (target != null) {
                hasValidTarget = true;
                
                // Calculate desired robot movement to track target
                double distanceError = targetDistance.get() - lastTargetDistance;
                double angleError = targetAngle.get() - lastTargetAngle;
                
                // Simple tracking logic (would be more sophisticated in real implementation)
                double forwardSpeed = distanceError * 0.1;
                double rotationSpeed = angleError * 0.05;
                
                // Apply movement
                swerve.drive(new edu.wpi.first.math.geometry.Translation2d(forwardSpeed, 0), rotationSpeed, true);
                
                targetDetections++;
            } else {
                hasValidTarget = false;
                swerve.stop();
            }
            
            totalFrames++;
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            swerve.stop();
            testRunning = false;
        }
    }
    
    /**
     * Vision odometry test mode
     */
    private void handleVisionOdometry(Controller driverController, Controller operatorController) {
        // Test vision-based odometry
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Get vision-based pose estimate
            var visionPose = vision.getVisionPose();
            
            if (visionPose != null) {
                hasValidTarget = true;
                
                // Compare with odometry pose
                var odometryPose = swerve.getPose();
                double poseError = visionPose.getTranslation().getDistance(odometryPose.getTranslation());
                
                // Log pose difference
                System.out.println("[VisionTesting] Pose error: " + poseError + " meters");
                
                targetDetections++;
            } else {
                hasValidTarget = false;
            }
            
            totalFrames++;
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            testRunning = false;
        }
    }
    
    /**
     * Camera calibration test mode
     */
    private void handleCameraCalibration(Controller driverController, Controller operatorController) {
        // Test camera calibration accuracy
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Move robot to known positions and verify vision measurements
            double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime;
            
            if (elapsed < 3.0) {
                // Position 1: 2 meters from target
                swerve.setPose(new Pose2d(2.0, 0, new Rotation2d()));
            } else if (elapsed < 6.0) {
                // Position 2: 3 meters from target, 30 degrees
                swerve.setPose(new Pose2d(3.0, 1.73, new Rotation2d(Math.toRadians(30))));
            } else if (elapsed < 9.0) {
                // Position 3: 4 meters from target, -30 degrees
                swerve.setPose(new Pose2d(4.0, -2.31, new Rotation2d(Math.toRadians(-30))));
            } else {
                swerve.stop();
                testRunning = false;
            }
            
            // Check vision measurements
            var target = vision.getBestTarget();
            if (target != null) {
                hasValidTarget = true;
                lastTargetDistance = target.avgTagDist;
                lastTargetAngle = Math.toDegrees(target.pose.getRotation().getZ());
                targetDetections++;
            } else {
                hasValidTarget = false;
            }
            
            totalFrames++;
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            swerve.stop();
            testRunning = false;
        }
    }
    
    /**
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        // Calculate detection rate
        if (totalFrames > 0) {
            double detectionRate = (double) targetDetections / totalFrames * 100.0;
            SmartDashboard.putNumber("Test/Vision/DetectionRate", detectionRate);
        }
    }
    
    /**
     * Reset test state
     */
    private void resetTest() {
        testRunning = false;
        testStartTime = 0;
        hasValidTarget = false;
        lastTargetDistance = 0;
        lastTargetAngle = 0;
        lastLatency = 0;
        targetDetections = 0;
        totalFrames = 0;
        swerve.stop();
    }
    
    /**
     * Setup dashboard controls
     */
    public void setupDashboard() {
        SmartDashboard.putString("Test/Vision/Mode", currentTestMode.name());
        SmartDashboard.putNumber("Test/Vision/TargetDistance", targetDistance.get());
        SmartDashboard.putNumber("Test/Vision/TargetAngle", targetAngle.get());
        SmartDashboard.putNumber("Test/Vision/ConfidenceThreshold", confidenceThreshold.get());
    }
    
    /**
     * Update dashboard values
     */
    public void updateDashboard() {
        SmartDashboard.putString("Test/Vision/Mode", currentTestMode.name());
        SmartDashboard.putBoolean("Test/Vision/Running", testRunning);
        SmartDashboard.putBoolean("Test/Vision/HasValidTarget", hasValidTarget);
        SmartDashboard.putNumber("Test/Vision/LastDistance", lastTargetDistance);
        SmartDashboard.putNumber("Test/Vision/LastLatency", lastLatency);
        SmartDashboard.putNumber("Test/Vision/DetectionRate", calculateDetectionRate());
        SmartDashboard.putNumber("Test/Vision/TargetDetections", targetDetections);
        SmartDashboard.putNumber("Test/Vision/TotalFrames", totalFrames);
        
        // Update tunable numbers
        targetDistance.update();
        targetAngle.update();
        confidenceThreshold.update();
    }
    
    /**
     * Check if there's a valid target
     */
    public boolean hasValidTarget() {
        return hasValidTarget;
    }
    
    /**
     * Calculate detection rate as percentage
     */
    private double calculateDetectionRate() {
        if (totalFrames == 0) return 0.0;
        return (double) targetDetections / totalFrames * 100.0;
    }
    
    /**
     * Check if test is currently running
     */
    public boolean isTestRunning() {
        return testRunning;
    }
    
    /**
     * Cleanup method
     */
    public void cleanup() {
        resetTest();
        vision.setLimelightEnabled(false);
    }
}
