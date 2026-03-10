package frc.robot.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.TunableNumber;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

/**
 * Comprehensive shooter tuning and testing system.
 * 
 * Features:
 * - Manual velocity control
 * - Auto-aim testing at different distances
 * - PID gain tuning via NetworkTables
 * - Shooting solution validation
 * - Performance metrics
 */
public class ShooterTuning {
    
    private enum TestMode {
        MANUAL_VELOCITY,
        AUTO_AIM_TEST,
        PID_TUNING,
        CHARACTERIZATION
    }
    
    private TestMode currentTestMode = TestMode.MANUAL_VELOCITY;
    private boolean testRunning = false;
    private double testStartTime = 0;
    
    // Tuning parameters
    private final TunableNumber testVelocity = new TunableNumber("Test/Shooter/VelocityRPM", 3000);
    private final TunableNumber testDistance = new TunableNumber("Test/Shooter/DistanceMeters", 3.0);
    private final TunableNumber testAngle = new TunableNumber("Test/Shooter/AngleDegrees", 45.0);
    
    // Performance tracking
    private double lastVelocityError = 0;
    private double timeToTargetVelocity = 0;
    private int shotsTaken = 0;
    private int successfulShots = 0;
    
    public ShooterTuning() {
        setupDashboard();
    }
    
    /**
     * Update shooter tuning based on controller input
     */
    public void update(Controller driverController, Controller operatorController) {
        handleModeSwitching(driverController);
        
        switch (currentTestMode) {
            case MANUAL_VELOCITY:
                handleManualVelocity(driverController, operatorController);
                break;
            case AUTO_AIM_TEST:
                handleAutoAimTest(driverController, operatorController);
                break;
            case PID_TUNING:
                handlePIDTuning(driverController, operatorController);
                break;
            case CHARACTERIZATION:
                handleCharacterization(driverController, operatorController);
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
            currentTestMode = TestMode.MANUAL_VELOCITY;
            System.out.println("[ShooterTuning] Mode: MANUAL_VELOCITY");
            resetTest();
        } else if (driverController.getBButtonPressed()) {
            currentTestMode = TestMode.AUTO_AIM_TEST;
            System.out.println("[ShooterTuning] Mode: AUTO_AIM_TEST");
            resetTest();
        } else if (driverController.getXButtonPressed()) {
            currentTestMode = TestMode.PID_TUNING;
            System.out.println("[ShooterTuning] Mode: PID_TUNING");
            resetTest();
        } else if (driverController.getYButtonPressed()) {
            currentTestMode = TestMode.CHARACTERIZATION;
            System.out.println("[ShooterTuning] Mode: CHARACTERIZATION");
            resetTest();
        }
    }
    
    /**
     * Manual velocity control mode
     */
    private void handleManualVelocity(Controller driverController, Controller operatorController) {
        Shooter shooter = Shooter.getInstance();
        
        // Use operator right trigger for velocity control
        double trigger = operatorController.getRightTriggerAxis();
        double targetVelocity = testVelocity.get() * trigger;
        
        if (trigger > 0.1) {
            shooter.setFlywheelVelocity(targetVelocity);
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            shooter.stop();
            testRunning = false;
        }
        
        // Manual kicker control with operator right bumper
        if (operatorController.getRightBumperButton()) {
            shooter.setKickerSpeed(Constants.ShooterConstants.FEED_SPEED);
        } else {
            shooter.setKickerSpeed(0);
        }
        
        // Quick velocity presets with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - High shot
                testVelocity.setDefault(5000);
                break;
            case 90: // Right - Medium shot
                testVelocity.setDefault(3500);
                break;
            case 180: // Down - Low shot
                testVelocity.setDefault(2000);
                break;
            case 270: // Left - Custom
                // Keep current value
                break;
        }
    }
    
    /**
     * Auto-aim testing mode
     */
    private void handleAutoAimTest(Controller driverController, Controller operatorController) {
        Shooter shooter = Shooter.getInstance();
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Create test pose at specified distance and angle
        double distance = testDistance.get();
        double angleDeg = testAngle.get();
        double angleRad = Math.toRadians(angleDeg);
        
        Pose2d testPose = new Pose2d(
            distance * Math.cos(angleRad),
            distance * Math.sin(angleRad),
            new Rotation2d()
        );
        
        // Test shooting solution
        if (driverController.getRightTriggerAxis() > 0.5) {
            var solution = shooter.calculateShootingSolution(testPose, swerve.getFieldVelocity());
            
            if (solution.possible()) {
                shooter.setFlywheelVelocity(solution.flywheelRPM());
                
                // Check if ready to fire
                if (shooter.isReadyToFire(solution.turretAngle())) {
                    shooter.setKickerSpeed(Constants.ShooterConstants.FEED_SPEED);
                    shotsTaken++;
                    
                    if (Math.abs(shooter.getFlywheelLeftVelocityRPM() - solution.flywheelRPM()) < Constants.ShooterConstants.RPM_TOLERANCE) {
                        successfulShots++;
                    }
                } else {
                    shooter.setKickerSpeed(0);
                }
                
                if (!testRunning) {
                    testRunning = true;
                    testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                }
            } else {
                System.out.println("[ShooterTuning] No shooting solution available for test pose");
                shooter.stop();
            }
        } else {
            shooter.stop();
            testRunning = false;
        }
    }
    
    /**
     * PID tuning mode
     */
    private void handlePIDTuning(Controller driverController, Controller operatorController) {
        Shooter shooter = Shooter.getInstance();
        
        // Use tunable numbers for PID gains
        TunableNumber kP = new TunableNumber("Test/Shooter/kP", Constants.ShooterConstants.kFlywheelP.get());
        TunableNumber kI = new TunableNumber("Test/Shooter/kI", 0.0);
        TunableNumber kD = new TunableNumber("Test/Shooter/kD", 0.0);
        TunableNumber kS = new TunableNumber("Test/Shooter/kS", Constants.ShooterConstants.kFlywheelS.get());
        TunableNumber kV = new TunableNumber("Test/Shooter/kV", Constants.ShooterConstants.kFlywheelV.get());
        TunableNumber kA = new TunableNumber("Test/Shooter/kA", Constants.ShooterConstants.kFlywheelA.get());
        
        // Apply PID gains (this would need to be implemented in Shooter subsystem)
        // shooter.updatePIDGains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
        
        // Test step response
        if (driverController.getRightTriggerAxis() > 0.5) {
            shooter.setFlywheelVelocity(testVelocity.get());
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            shooter.stop();
            testRunning = false;
        }
    }
    
    /**
     * Characterization mode for system identification
     */
    private void handleCharacterization(Controller driverController, Controller operatorController) {
        Shooter shooter = Shooter.getInstance();
        
        // Voltage ramp test
        if (driverController.getRightTriggerAxis() > 0.5) {
            double voltage = operatorController.getLeftY() * 6.0; // -6V to +6V
            shooter.setFlywheelVoltages(voltage, voltage);
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            shooter.stop();
            testRunning = false;
        }
    }
    
    /**
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        Shooter shooter = Shooter.getInstance();
        
        // Calculate velocity error
        double targetVelocity = testVelocity.get();
        double actualVelocity = shooter.getFlywheelLeftVelocityRPM();
        lastVelocityError = Math.abs(targetVelocity - actualVelocity);
        
        // Calculate time to reach target velocity
        if (testRunning && lastVelocityError < Constants.ShooterConstants.RPM_TOLERANCE) {
            timeToTargetVelocity = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime;
        }
    }
    
    /**
     * Reset test state
     */
    private void resetTest() {
        testRunning = false;
        testStartTime = 0;
        shotsTaken = 0;
        successfulShots = 0;
        timeToTargetVelocity = 0;
        Shooter.getInstance().stop();
    }
    
    /**
     * Setup dashboard controls
     */
    public void setupDashboard() {
        SmartDashboard.putString("Test/Shooter/Mode", currentTestMode.name());
        SmartDashboard.putNumber("Test/Shooter/VelocityRPM", testVelocity.get());
        SmartDashboard.putNumber("Test/Shooter/DistanceMeters", testDistance.get());
        SmartDashboard.putNumber("Test/Shooter/AngleDegrees", testAngle.get());
    }
    
    /**
     * Update dashboard values
     */
    public void updateDashboard() {
        SmartDashboard.putString("Test/Shooter/Mode", currentTestMode.name());
        SmartDashboard.putBoolean("Test/Shooter/Running", testRunning);
        SmartDashboard.putNumber("Test/Shooter/VelocityError", lastVelocityError);
        SmartDashboard.putNumber("Test/Shooter/TimeToTarget", timeToTargetVelocity);
        SmartDashboard.putNumber("Test/Shooter/ShotsTaken", shotsTaken);
        SmartDashboard.putNumber("Test/Shooter/SuccessfulShots", successfulShots);
        
        if (shotsTaken > 0) {
            double successRate = (double) successfulShots / shotsTaken * 100.0;
            SmartDashboard.putNumber("Test/Shooter/SuccessRate", successRate);
        }
        
        // Update tunable numbers (TunableNumber doesn't have update method, values are read directly)
        // testVelocity.update();
        // testDistance.update();
        // testAngle.update();
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
    }
}
