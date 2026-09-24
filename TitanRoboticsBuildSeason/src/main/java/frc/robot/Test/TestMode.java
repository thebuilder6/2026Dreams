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
 * - SysId automated characterization manager
 * - Pre-flight automated diagnostics and single component bench tests
 * - Shooter tuning with velocity control and auto-aim testing
 * - Intake testing and calibration
 * - Swerve drive characterization
 * - LED feedback for test status
 */
public class TestMode {

    private static TestMode instance = null;

    public enum TestCategory {
        SYSID_TESTING,
        PREFLIGHT_DIAGNOSTICS,
        SHOOTER_TUNING,
        INTAKE_TESTING,
        DRIVE_CHARACTERIZATION,
        VISION_TESTING
    }

    private TestCategory activeCategory = TestCategory.SYSID_TESTING;
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

    public static synchronized TestMode getInstance() {
        if (instance == null) {
            instance = new TestMode();
        }
        return instance;
    }

    /**
     * Main update loop for test mode
     */
    public void update() {
        // Allow dashboard toggle of test mode
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
            case SYSID_TESTING:
                sysIdManager.update(testController);
                break;
            case PREFLIGHT_DIAGNOSTICS:
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
            sysIdManager.cancelTest();
            diagnostics.cancelPreFlightCheck();
            shooterTuning.cleanup();
            intakeTesting.cleanup();
            driveCharacterization.cleanup();
            visionTesting.cleanup();
        }
    }

    /**
     * Handle system diagnostics using existing Diagnostics subsystem
     */
    private void handleSystemDiagnostics(Controller driverController, Controller operatorController) {
        if (driverController.getAButtonPressed()) {
            diagnostics.startPreFlightCheck();
        } else if (driverController.getBButtonPressed()) {
            diagnostics.cancelPreFlightCheck();
        }
    }

    /**
     * Check if diagnostics has errors
     */
    private boolean hasDiagnosticErrors() {
        return diagnostics != null && diagnostics.hasErrors();
    }

    /**
     * Handle category switching via D-pad / bumpers
     */
    private void handleCategorySwitching() {
        // Category switching from dashboard topic
        String dashCat = SmartDashboard.getString("TestMode/SelectCategory", "");
        if (!dashCat.isEmpty()) {
            for (TestCategory cat : TestCategory.values()) {
                if (cat.name().equalsIgnoreCase(dashCat)) {
                    activeCategory = cat;
                    SmartDashboard.putString("TestMode/SelectCategory", "");
                    break;
                }
            }
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
            case SYSID_TESTING:
                ledPattern = sysIdManager.isRunning() ? Constants.LEDConstants.STROBE_GOLD
                        : Constants.LEDConstants.SOLID_BLUE;
                break;
            case SHOOTER_TUNING:
                ledPattern = shooterTuning.isTestRunning() ? Constants.LEDConstants.STROBE_RED
                        : Constants.LEDConstants.SOLID_RED;
                break;
            case INTAKE_TESTING:
                ledPattern = intakeTesting.isTestRunning() ? Constants.LEDConstants.STROBE_BLUE
                        : Constants.LEDConstants.SOLID_BLUE;
                break;
            case DRIVE_CHARACTERIZATION:
                ledPattern = driveCharacterization.isTestRunning() ? Constants.LEDConstants.STROBE_BLUE
                        : Constants.LEDConstants.SOLID_BLUE;
                break;
            case PREFLIGHT_DIAGNOSTICS:
                ledPattern = hasDiagnosticErrors() ? Constants.LEDConstants.HEARTBEAT_RED
                        : (diagnostics.isPreFlightRunning() ? Constants.LEDConstants.BREATH_BLUE
                                : Constants.LEDConstants.SOLID_GREEN);
                break;
            case VISION_TESTING:
                ledPattern = visionTesting.hasValidTarget() ? Constants.LEDConstants.STROBE_GOLD
                        : Constants.LEDConstants.SOLID_YELLOW;
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
        SmartDashboard.putString("TestMode/SelectCategory", "");

        // Category-specific setup
        sysIdManager.setupDashboard();
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
        sysIdManager.updateDashboard();
        shooterTuning.updateDashboard();
        intakeTesting.updateDashboard();
        driveCharacterization.updateDashboard();
        visionTesting.updateDashboard();

        if (diagnostics != null) {
            diagnostics.log();
        }
    }

    public TestCategory getActiveCategory() {
        return activeCategory;
    }

    public void setActiveCategory(TestCategory category) {
        this.activeCategory = category;
    }

    public boolean isEnabled() {
        return testModeEnabled;
    }

    public void cleanup() {
        setEnabled(false);
    }
}
