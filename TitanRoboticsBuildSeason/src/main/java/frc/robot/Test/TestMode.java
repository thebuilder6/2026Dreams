package frc.robot.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;

/**
 * Comprehensive test mode for robot subsystem tuning and diagnostics.
 * 
 * Features:
 * - Shooter tuning with velocity control and auto-aim testing
 * - Intake testing and calibration
 * - Swerve drive characterization
 * - System diagnostics
 * - LED feedback for test status
 */
public class TestMode {
    
    private static TestMode instance = null;
    
    public enum TestCategory {
        SHOOTER_TUNING,
        INTAKE_TESTING,
        DRIVE_CHARACTERIZATION,
        SYSTEM_DIAGNOSTICS,
        VISION_TESTING
    }
    
    private TestCategory activeCategory = TestCategory.SHOOTER_TUNING;
    private boolean testModeEnabled = false;
    
    // Controllers
    private Controller testController;
    private Controller operatorController;
    
    // Test components
    private ShooterTuning shooterTuning;
    private IntakeTesting intakeTesting;
    private DriveCharacterization driveCharacterization;
    private VisionTesting visionTesting;
    
    // Integration with existing systems
    private SysID sysID;
    private Diagnostics diagnostics;
    
    // LED feedback
    private LEDs leds;
    
    private TestMode() {
        testController = new Controller(PortMap.DRIVER_CONTROLLER);
        operatorController = new Controller(PortMap.OPERATOR_CONTROLLER);
        
        shooterTuning = new ShooterTuning();
        intakeTesting = new IntakeTesting();
        driveCharacterization = new DriveCharacterization();
        visionTesting = new VisionTesting();
        
        // Initialize existing systems
        if (Constants.TUNING_MODE) {
            sysID = new SysID(Shooter.getInstance(), Intake.getInstance(), SwerveBase.getInstance());
        }
        diagnostics = Diagnostics.getInstance();
        
        // Register diagnostics with subsystem manager
        SubsystemManager.registerSubsystem(diagnostics);
        
        leds = LEDs.getInstance();
        
        // Register dashboard controls
        setupDashboard();
    }
    
    public static TestMode getInstance() {
        if (instance == null) {
            instance = new TestMode();
        }
        return instance;
    }
    
    /**
     * Main update loop for test mode
     */
    public void update() {
        if (!testModeEnabled) {
            return;
        }
        
        // Handle category switching
        handleCategorySwitching();
        
        // Update active test category
        switch (activeCategory) {
            case SHOOTER_TUNING:
                shooterTuning.update(testController, operatorController);
                break;
            case INTAKE_TESTING:
                intakeTesting.update(testController, operatorController);
                break;
            case DRIVE_CHARACTERIZATION:
                driveCharacterization.update(testController, operatorController);
                break;
            case SYSTEM_DIAGNOSTICS:
                handleSystemDiagnostics(testController, operatorController);
                break;
            case VISION_TESTING:
                visionTesting.update(testController, operatorController);
                break;
        }
        
        // Update LED feedback
        updateLEDFeedback();
        
        // Update dashboard
        updateDashboard();
    }
    
    /**
     * Enable/disable test mode
     */
    public void setEnabled(boolean enabled) {
        if (enabled && !testModeEnabled) {
            // Entering test mode
            testModeEnabled = true;
            System.out.println("[TestMode] ENABLED - Category: " + activeCategory.name());
            
            // Stop all normal robot operations
            Shooter.getInstance().stop();
            Intake.getInstance().stop();
            SwerveBase.getInstance().stop();
            
        } else if (!enabled && testModeEnabled) {
            // Exiting test mode
            testModeEnabled = false;
            System.out.println("[TestMode] DISABLED");
            
            // Clean up all test systems
            shooterTuning.cleanup();
            intakeTesting.cleanup();
            driveCharacterization.cleanup();
            visionTesting.cleanup();
            
            // Diagnostics runs continuously, no cleanup needed
        }
    }
    
    /**
     * Handle system diagnostics using existing Diagnostics subsystem
     */
    private void handleSystemDiagnostics(Controller driverController, Controller operatorController) {
        // Use existing diagnostics dashboard controls
        // The diagnostics subsystem handles its own dashboard interaction
        // We just need to ensure it's running and provide feedback
        
        // SysID integration if available
        if (sysID != null && Constants.TUNING_MODE) {
            sysID.runTest(testController);
        }
    }
    
    /**
     * Check if diagnostics has errors (simplified)
     */
    private boolean hasDiagnosticErrors() {
        // This would need to be implemented based on diagnostics subsystem
        // For now, return false as a placeholder
        return false;
    }

    /**
     * Handle category switching via D-pad
     */
    private void handleCategorySwitching() {
        int pov = testController.getPOV();
        
        if (pov == 0) { // Up
            activeCategory = TestCategory.SHOOTER_TUNING;
            System.out.println("[TestMode] Category: SHOOTER_TUNING");
        } else if (pov == 90) { // Right
            activeCategory = TestCategory.INTAKE_TESTING;
            System.out.println("[TestMode] Category: INTAKE_TESTING");
        } else if (pov == 180) { // Down
            activeCategory = TestCategory.DRIVE_CHARACTERIZATION;
            System.out.println("[TestMode] Category: DRIVE_CHARACTERIZATION");
        } else if (pov == 270) { // Left
            activeCategory = TestCategory.SYSTEM_DIAGNOSTICS;
            System.out.println("[TestMode] Category: SYSTEM_DIAGNOSTICS");
        }
        
        // Vision testing with both bumpers
        if (testController.getLeftBumperButton() && testController.getRightBumperButton()) {
            activeCategory = TestCategory.VISION_TESTING;
            System.out.println("[TestMode] Category: VISION_TESTING");
        }
    }
    
    /**
     * Update LED feedback based on test status
     */
    private void updateLEDFeedback() {
        double ledPattern = Constants.LEDConstants.SOLID_BLUE;
        
        switch (activeCategory) {
            case SHOOTER_TUNING:
                ledPattern = shooterTuning.isTestRunning() ? 
                    Constants.LEDConstants.STROBE_RED : Constants.LEDConstants.SOLID_RED;
                break;
            case INTAKE_TESTING:
                ledPattern = intakeTesting.isTestRunning() ? 
                    Constants.LEDConstants.STROBE_BLUE : Constants.LEDConstants.SOLID_BLUE;
                break;
            case DRIVE_CHARACTERIZATION:
                ledPattern = driveCharacterization.isTestRunning() ? 
                    Constants.LEDConstants.STROBE_BLUE : Constants.LEDConstants.SOLID_BLUE;
                break;
            case SYSTEM_DIAGNOSTICS:
                ledPattern = hasDiagnosticErrors() ? 
                    Constants.LEDConstants.HEARTBEAT_RED : Constants.LEDConstants.BREATH_BLUE;
                break;
            case VISION_TESTING:
                ledPattern = visionTesting.hasValidTarget() ? 
                    Constants.LEDConstants.STROBE_GOLD : Constants.LEDConstants.SOLID_YELLOW;
                break;
        }
        
        leds.setPattern(ledPattern);
    }
    
    /**
     * Setup dashboard controls
     */
    private void setupDashboard() {
        SmartDashboard.putBoolean("TestMode/Enabled", false);
        SmartDashboard.putString("TestMode/Category", activeCategory.name());
        
        // Category-specific setup
        shooterTuning.setupDashboard();
        intakeTesting.setupDashboard();
        driveCharacterization.setupDashboard();
        diagnostics.setupDashboard();
        visionTesting.setupDashboard();
    }
    
    /**
     * Update dashboard values
     */
    private void updateDashboard() {
        SmartDashboard.putBoolean("TestMode/Enabled", testModeEnabled);
        SmartDashboard.putString("TestMode/Category", activeCategory.name());
        
        // Category-specific updates
        shooterTuning.updateDashboard();
        intakeTesting.updateDashboard();
        driveCharacterization.updateDashboard();
        visionTesting.updateDashboard();
        
        // Diagnostics updates (existing dashboard controls)
        if (diagnostics != null) {
            diagnostics.log();
        }
    }
    
    /**
     * Get current test category
     */
    public TestCategory getActiveCategory() {
        return activeCategory;
    }
    
    /**
     * Check if test mode is enabled
     */
    public boolean isEnabled() {
        return testModeEnabled;
    }
    
    /**
     * Cleanup method for when robot is disabled
     */
    public void cleanup() {
        setEnabled(false);
    }
}
