package frc.robot.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.TunableNumber;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Intake;

/**
 * Intake system testing and calibration.
 * 
 * Features:
 * - Arm position control and testing
 * - Roller speed testing
 * - Hopper functionality testing
 * - Jam detection testing
 * - Position accuracy validation
 */
public class IntakeTesting {
    
    private enum TestMode {
        ARM_CONTROL,
        ROLLER_TESTING,
        HOPPER_TESTING,
        JAM_DETECTION,
        POSITION_CALIBRATION
    }
    
    private TestMode currentTestMode = TestMode.ARM_CONTROL;
    private boolean testRunning = false;
    private double testStartTime = 0;
    
    // Tuning parameters
    private final TunableNumber targetArmPosition = new TunableNumber("Test/Intake/ArmPositionDeg", 45.0);
    private final TunableNumber rollerSpeed = new TunableNumber("Test/Intake/RollerSpeed", 0.5);
    private final TunableNumber hopperSpeed = new TunableNumber("Test/Intake/HopperSpeed", 0.5);
    private final TunableNumber armTestVoltage = new TunableNumber("Test/Intake/ArmTestVoltage", 2.0);
    
    // Performance tracking
    private double lastPositionError = 0;
    private double timeToTargetPosition = 0;
    private int jamEvents = 0;
    private double maxCurrentDraw = 0;
    
    public IntakeTesting() {
        setupDashboard();
    }
    
    /**
     * Update intake testing based on controller input
     */
    public void update(Controller driverController, Controller operatorController) {
        handleModeSwitching(driverController);
        
        switch (currentTestMode) {
            case ARM_CONTROL:
                handleArmControl(driverController, operatorController);
                break;
            case ROLLER_TESTING:
                handleRollerTesting(driverController, operatorController);
                break;
            case HOPPER_TESTING:
                handleHopperTesting(driverController, operatorController);
                break;
            case JAM_DETECTION:
                handleJamDetection(driverController, operatorController);
                break;
            case POSITION_CALIBRATION:
                handlePositionCalibration(driverController, operatorController);
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
            currentTestMode = TestMode.ARM_CONTROL;
            System.out.println("[IntakeTesting] Mode: ARM_CONTROL");
            resetTest();
        } else if (driverController.getBButtonPressed()) {
            currentTestMode = TestMode.ROLLER_TESTING;
            System.out.println("[IntakeTesting] Mode: ROLLER_TESTING");
            resetTest();
        } else if (driverController.getXButtonPressed()) {
            currentTestMode = TestMode.HOPPER_TESTING;
            System.out.println("[IntakeTesting] Mode: HOPPER_TESTING");
            resetTest();
        } else if (driverController.getYButtonPressed()) {
            currentTestMode = TestMode.JAM_DETECTION;
            System.out.println("[IntakeTesting] Mode: JAM_DETECTION");
            resetTest();
        }
        
        // Position calibration with both bumpers
        if (driverController.getLeftBumperButton() && driverController.getRightBumperButton()) {
            currentTestMode = TestMode.POSITION_CALIBRATION;
            System.out.println("[IntakeTesting] Mode: POSITION_CALIBRATION");
            resetTest();
        }
    }
    
    /**
     * Arm position control mode
     */
    private void handleArmControl(Controller driverController, Controller operatorController) {
        Intake intake = Intake.getInstance();
        
        // Use operator left joystick for manual arm control
        double manualArmY = -operatorController.getLeftY();
        if (Math.abs(manualArmY) > 0.1) {
            intake.setArmVoltage(manualArmY * armTestVoltage.get());
            testRunning = true;
        } else {
            // Automatic position control with right trigger
            if (operatorController.getRightTriggerAxis() > 0.5) {
                double targetDeg = targetArmPosition.get();
                intake.setArmPosition(Math.toRadians(targetDeg));
                
                if (!testRunning) {
                    testRunning = true;
                    testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                }
            } else {
                intake.setArmVoltage(0);
                testRunning = false;
            }
        }
        
        // Quick position presets with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Intake position
                targetArmPosition.setDefault(Constants.IntakeConstants.ARM_INTAKE_POS);
                break;
            case 180: // Down - Idle position
                targetArmPosition.setDefault(Constants.IntakeConstants.ARM_IDLE_POS);
                break;
            case 90: // Right - Mid position
                targetArmPosition.setDefault(45.0);
                break;
            case 270: // Left - Custom
                // Keep current value
                break;
        }
    }
    
    /**
     * Roller testing mode
     */
    private void handleRollerTesting(Controller driverController, Controller operatorController) {
        Intake intake = Intake.getInstance();
        
        // Use operator right trigger for roller speed control
        double trigger = operatorController.getRightTriggerAxis();
        
        if (trigger > 0.1) {
            intake.setRollerVoltage(trigger * 6.0); // Convert speed to voltage
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            intake.setRollerVoltage(0);
            testRunning = false;
        }
        
        // Reverse with left bumper
        if (operatorController.getLeftBumperButton()) {
            intake.setRollerVoltage(-rollerSpeed.get() * 6.0);
            testRunning = true;
        }
        
        // Speed presets with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Full speed
                rollerSpeed.setDefault(1.0);
                break;
            case 90: // Right - Medium speed
                rollerSpeed.setDefault(0.7);
                break;
            case 180: // Down - Slow speed
                rollerSpeed.setDefault(0.3);
                break;
            case 270: // Left - Custom
                // Keep current value
                break;
        }
    }
    
    /**
     * Hopper testing mode
     */
    private void handleHopperTesting(Controller driverController, Controller operatorController) {
        Intake intake = Intake.getInstance();
        
        // Use operator right trigger for hopper speed control
        double trigger = operatorController.getRightTriggerAxis();
        
        if (trigger > 0.1) {
            intake.setHopperVoltage(trigger * 6.0); // Convert speed to voltage
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            intake.setHopperVoltage(0);
            testRunning = false;
        }
        
        // Reverse with left bumper
        if (operatorController.getLeftBumperButton()) {
            intake.setHopperVoltage(-hopperSpeed.get() * 6.0);
            testRunning = true;
        }
        
        // Speed presets with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Full speed
                hopperSpeed.setDefault(1.0);
                break;
            case 90: // Right - Medium speed
                hopperSpeed.setDefault(0.7);
                break;
            case 180: // Down - Slow speed
                hopperSpeed.setDefault(0.3);
                break;
            case 270: // Left - Custom
                // Keep current value
                break;
        }
    }
    
    /**
     * Jam detection testing mode
     */
    private void handleJamDetection(Controller driverController, Controller operatorController) {
        Intake intake = Intake.getInstance();
        
        // Run rollers and monitor for jams
        if (driverController.getRightTriggerAxis() > 0.5) {
            intake.setRollerVoltage(rollerSpeed.get() * 6.0);
            
            // Simulate jam detection (this would normally come from current monitoring)
            double simulatedCurrent = Math.random() * 40; // 0-40 amps
            maxCurrentDraw = Math.max(maxCurrentDraw, simulatedCurrent);
            
            if (simulatedCurrent > Constants.IntakeConstants.STALL_CURRENT_LIMIT) {
                jamEvents++;
                System.out.println("[IntakeTesting] Jam detected! Current: " + simulatedCurrent + "A");
                
                // Simulate jam response
                intake.setRollerVoltage(-rollerSpeed.get() * 6.0);
                edu.wpi.first.wpilibj.Timer.delay(Constants.IntakeConstants.EJECT_TIME);
                intake.setRollerVoltage(rollerSpeed.get() * 6.0);
            }
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }
        } else {
            intake.setRollerVoltage(0);
            testRunning = false;
        }
    }
    
    /**
     * Position calibration mode
     */
    private void handlePositionCalibration(Controller driverController, Controller operatorController) {
        Intake intake = Intake.getInstance();
        
        // Manual arm control for calibration
        double manualArmY = -operatorController.getLeftY();
        if (Math.abs(manualArmY) > 0.1) {
            intake.setArmVoltage(manualArmY * 2.0); // Low voltage for precise control
            testRunning = true;
        } else {
            intake.setArmVoltage(0);
            testRunning = false;
        }
        
        // Save current position as preset with right bumper
        if (operatorController.getRightBumperButton()) {
            double currentPos = Math.toDegrees(intake.getArmPosition());
            targetArmPosition.setDefault(currentPos);
            System.out.println("[IntakeTesting] Saved current position: " + currentPos + "°");
        }
    }
    
    /**
     * Update performance metrics
     */
    private void updatePerformanceMetrics() {
        Intake intake = Intake.getInstance();
        
        // Calculate position error
        double targetPos = Math.toRadians(targetArmPosition.get());
        double actualPos = intake.getArmPosition();
        lastPositionError = Math.abs(targetPos - actualPos);
        
        // Calculate time to reach target position
        if (testRunning && lastPositionError < Math.toRadians(2.0)) { // 2 degree tolerance
            timeToTargetPosition = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime;
        }
    }
    
    /**
     * Reset test state
     */
    private void resetTest() {
        testRunning = false;
        testStartTime = 0;
        jamEvents = 0;
        maxCurrentDraw = 0;
        timeToTargetPosition = 0;
        Intake.getInstance().stop();
    }
    
    /**
     * Setup dashboard controls
     */
    public void setupDashboard() {
        SmartDashboard.putString("Test/Intake/Mode", currentTestMode.name());
        SmartDashboard.putNumber("Test/Intake/ArmPositionDeg", targetArmPosition.get());
        SmartDashboard.putNumber("Test/Intake/RollerSpeed", rollerSpeed.get());
        SmartDashboard.putNumber("Test/Intake/HopperSpeed", hopperSpeed.get());
        SmartDashboard.putNumber("Test/Intake/ArmTestVoltage", armTestVoltage.get());
    }
    
    /**
     * Update dashboard values
     */
    public void updateDashboard() {
        SmartDashboard.putString("Test/Intake/Mode", currentTestMode.name());
        SmartDashboard.putBoolean("Test/Intake/Running", testRunning);
        SmartDashboard.putNumber("Test/Intake/PositionError", Math.toDegrees(lastPositionError));
        SmartDashboard.putNumber("Test/Intake/TimeToTarget", timeToTargetPosition);
        SmartDashboard.putNumber("Test/Intake/JamEvents", jamEvents);
        SmartDashboard.putNumber("Test/Intake/MaxCurrent", maxCurrentDraw);
        
        // Update tunable numbers (TunableNumber doesn't have update method)
        // targetArmPosition.update();
        // rollerSpeed.update();
        // hopperSpeed.update();
        // armTestVoltage.update();
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
