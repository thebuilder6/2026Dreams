package frc.robot.Test;

import frc.robot.Telemetry.Dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.PortMap;
import frc.robot.HMI.Controller;
import frc.robot.Subsystems.Intake;
import frc.robot.HMI.LEDs;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;

/**
 * Comprehensive test mode for robot subsystem tuning, SysID, and diagnostics.
 */
public class TestMode {
    
    private static TestMode instance = null;
    
    public enum TestCategory {
        SYSID_CHARACTERIZATION,
        SYSTEM_DIAGNOSTICS,
        SHOOTER_TUNING,
        INTAKE_TESTING,
        DRIVE_CHARACTERIZATION,
        VISION_TESTING
    }
    
    private TestCategory activeCategory = TestCategory.SYSID_CHARACTERIZATION;
    private boolean testModeEnabled = false;
    
    // Controllers
    private Controller testController;
    private Controller operatorController;
    
    // Test components
    private SysIdManager sysIdManager;
    private Diagnostics diagnostics;
    private ShooterTuning shooterTuning;
    private IntakeTesting intakeTesting;
    private DriveCharacterization driveCharacterization;
    private VisionTesting visionTesting;
    
    // LED feedback
    private LEDs leds;
    
    private TestMode() {
        testController = new Controller(PortMap.DRIVER_CONTROLLER);
        operatorController = new Controller(PortMap.OPERATOR_CONTROLLER);
        
        sysIdManager = SysIdManager.getInstance();
        diagnostics = Diagnostics.getInstance();
        shooterTuning = new ShooterTuning();
        intakeTesting = new IntakeTesting();
        driveCharacterization = new DriveCharacterization();
        visionTesting = new VisionTesting();
        
        leds = LEDs.getInstance();
        
        setupDashboard();
    }
    
    public static TestMode getInstance() {
        if (instance == null) {
            instance = new TestMode();
        }
        return instance;
    }
    
    /**
     * Main update loop for test mode.
     */
    public void update() {
        // Sync with dashboard enable switch
        boolean dashEnabled = SmartDashboard.getBoolean("TestMode/Enabled", testModeEnabled);
        if (dashEnabled != testModeEnabled) {
            setEnabled(dashEnabled);
        }

        if (!testModeEnabled) {
            return;
        }
        
        // Handle category switching
        handleCategorySwitching();
        
        // Update active test category
        switch (activeCategory) {
            case SYSID_CHARACTERIZATION:
                sysIdManager.updateController(testController);
                break;
            case SYSTEM_DIAGNOSTICS:
                handleSystemDiagnostics(testController, operatorController);
                break;
            case SHOOTER_TUNING:
                shooterTuning.update(testController, operatorController);
                break;
            case INTAKE_TESTING:
                intakeTesting.update(testController, operatorController);
                break;
            case DRIVE_CHARACTERIZATION:
                driveCharacterization.update(testController, operatorController);
                break;
            case VISION_TESTING:
                visionTesting.update(testController, operatorController);
                break;
        }
        
        updateLEDFeedback();
        updateDashboard();
    }
    
    /**
     * Enable/disable test mode.
     */
    public void setEnabled(boolean enabled) {
        if (enabled && !testModeEnabled) {
            testModeEnabled = true;
            System.out.println("[TestMode] ENABLED - Category: " + activeCategory.name());
            
            // Stop all normal robot operations
            Shooter.getInstance().stop();
            Intake.getInstance().stop();
            SwerveBase.getInstance().stop();
            
        } else if (!enabled && testModeEnabled) {
            testModeEnabled = false;
            System.out.println("[TestMode] DISABLED");
            
            // Clean up all test systems
            sysIdManager.abort();
            diagnostics.cancelPreFlightCheck();
            shooterTuning.cleanup();
            intakeTesting.cleanup();
            driveCharacterization.cleanup();
            visionTesting.cleanup();
        }
    }
    
    private void handleSystemDiagnostics(Controller driverController, Controller operatorController) {
        if (driverController.getAButtonPressed()) {
            diagnostics.startPreFlightCheck();
        } else if (driverController.getBButtonPressed()) {
            diagnostics.cancelPreFlightCheck();
        }
    }
    
    private boolean hasDiagnosticErrors() {
        return diagnostics != null && diagnostics.hasErrors();
    }

    private void handleCategorySwitching() {
        // Select category from dashboard if available
        String catFromDash = SmartDashboard.getString("TestMode/SelectCategory", "");
        if (!catFromDash.isEmpty() && !catFromDash.equals(activeCategory.name())) {
            try {
                activeCategory = TestCategory.valueOf(catFromDash);
                SmartDashboard.putString("TestMode/SelectCategory", "");
                System.out.println("[TestMode] Category switched via Dashboard: " + activeCategory.name());
            } catch (IllegalArgumentException ignored) {}
        }

        // Controller bumper + D-Pad switching
        if (testController.getLeftBumperButton()) {
            int pov = testController.getPOV();
            if (pov == 0) {
                activeCategory = TestCategory.SYSID_CHARACTERIZATION;
            } else if (pov == 90) {
                activeCategory = TestCategory.SYSTEM_DIAGNOSTICS;
            } else if (pov == 180) {
                activeCategory = TestCategory.SHOOTER_TUNING;
            } else if (pov == 270) {
                activeCategory = TestCategory.INTAKE_TESTING;
            }
        }
    }
    
    private void updateLEDFeedback() {
        double ledPattern = Constants.LEDConstants.SOLID_BLUE;
        
        switch (activeCategory) {
            case SYSID_CHARACTERIZATION:
                ledPattern = sysIdManager.isRunning() ? 
                    Constants.LEDConstants.STROBE_BLUE : Constants.LEDConstants.SOLID_BLUE;
                break;
            case SYSTEM_DIAGNOSTICS:
                ledPattern = hasDiagnosticErrors() ? 
                    Constants.LEDConstants.HEARTBEAT_RED : (diagnostics.isPreFlightRunning() ? Constants.LEDConstants.STROBE_GOLD : Constants.LEDConstants.BREATH_BLUE);
                break;
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
            case VISION_TESTING:
                ledPattern = visionTesting.hasValidTarget() ? 
                    Constants.LEDConstants.STROBE_GOLD : Constants.LEDConstants.SOLID_YELLOW;
                break;
        }
        
        leds.setPattern(ledPattern);
    }
    
    public void setupDashboard() {
        SmartDashboard.putBoolean("TestMode/Enabled", false);
        SmartDashboard.putString("TestMode/Category", activeCategory.name());
        SmartDashboard.putString("TestMode/SelectCategory", "");
        
        sysIdManager.setupDashboard();
        diagnostics.setupDashboard();
        shooterTuning.setupDashboard();
        intakeTesting.setupDashboard();
        driveCharacterization.setupDashboard();
        visionTesting.setupDashboard();
    }
    
    public void updateDashboard() {
        SmartDashboard.putBoolean("TestMode/Enabled", testModeEnabled);
        SmartDashboard.putString("TestMode/Category", activeCategory.name());
        
        sysIdManager.log();
        if (diagnostics != null) diagnostics.log();
        shooterTuning.updateDashboard();
        intakeTesting.updateDashboard();
        driveCharacterization.updateDashboard();
        visionTesting.updateDashboard();
    }
    
    public TestCategory getActiveCategory() {
        return activeCategory;
    }
    
    public boolean isEnabled() {
        return testModeEnabled;
    }
    
    public void cleanup() {
        setEnabled(false);
    }
}
