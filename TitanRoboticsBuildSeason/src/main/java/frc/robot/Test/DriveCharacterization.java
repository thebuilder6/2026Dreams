package frc.robot.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import frc.robot.Devices.Controller;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Data.TunableNumber;

/**
 * Swerve drive characterization and testing system.
 * 
 * Features:
 * - SysID integration for drive system identification
 * - Module individual testing
 * - Kinematics validation
 * - Odometry accuracy testing
 * - Performance metrics
 */
public class DriveCharacterization {
    
    private enum TestMode {
        SYSID_QUASISTATIC,
        SYSID_DYNAMIC,
        MODULE_INDIVIDUAL,
        KINEMATICS_TEST,
        ODOMETRY_TEST
    }
    
    private TestMode currentTestMode = TestMode.SYSID_QUASISTATIC;
    private boolean testRunning = false;
    private double testStartTime = 0;
    
    // SysID routines
    private final SysIdRoutine driveRoutine;
    private final SysIdRoutine steerRoutine;
    
    // Tuning parameters
    private final TunableNumber testVoltage = new TunableNumber("Test/Drive/TestVoltage", 2.0);
    private final TunableNumber testDistance = new TunableNumber("Test/Drive/TestDistance", 2.0);
    private final TunableNumber testAngle = new TunableNumber("Test/Drive/TestAngle", 90.0);
    
    // Performance tracking
    private double maxVelocity = 0;
    private double maxAcceleration = 0;
    private double totalDistance = 0;
    private double startHeading = 0;
    private double totalRotation = 0;
    
    // Module testing
    private int activeModule = 0; // 0=FL, 1=FR, 2=BL, 3=BR
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    
    public DriveCharacterization() {
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Drive SysID routine
        driveRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setDriveVoltage(volts.in(Volts)),
                (log) -> {
                    var vels = swerve.getDriveMotorVelocities();
                    var positions = swerve.getDriveMotorPositions();
                    var voltages = swerve.getDriveMotorVoltages();

                    // Log average of all 4 drive motors
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPos = positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgVel = vels.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                    log.motor("drive-linear-avg")
                        .voltage(Volts.of(avgVolts))
                        .linearPosition(Meters.of(avgPos))
                        .linearVelocity(MetersPerSecond.of(avgVel));
                },
                null
            )
        );
        
        // Steer SysID routine
        steerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> swerve.setSteerVoltage(volts.in(Volts)),
                (log) -> {
                    var positions = swerve.getSteerMotorPositions();
                    var voltages = swerve.getSteerMotorVoltages();

                    // Log average of all 4 steer motors
                    double avgVolts = voltages.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double avgPos = positions.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                    log.motor("steer-angle-avg")
                        .voltage(Volts.of(avgVolts))
                        .angularPosition(edu.wpi.first.units.Units.Radians.of(avgPos));
                },
                null
            )
        );
        
        setupDashboard();
    }
    
    /**
     * Update drive characterization based on controller input
     */
    public void update(Controller driverController, Controller operatorController) {
        handleModeSwitching(driverController);
        
        switch (currentTestMode) {
            case SYSID_QUASISTATIC:
                handleSysIdQuasistatic(driverController, operatorController);
                break;
            case SYSID_DYNAMIC:
                handleSysIdDynamic(driverController, operatorController);
                break;
            case MODULE_INDIVIDUAL:
                handleModuleIndividual(driverController, operatorController);
                break;
            case KINEMATICS_TEST:
                handleKinematicsTest(driverController, operatorController);
                break;
            case ODOMETRY_TEST:
                handleOdometryTest(driverController, operatorController);
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
            currentTestMode = TestMode.SYSID_QUASISTATIC;
            System.out.println("[DriveCharacterization] Mode: SYSID_QUASISTATIC");
            resetTest();
        } else if (driverController.getBButtonPressed()) {
            currentTestMode = TestMode.SYSID_DYNAMIC;
            System.out.println("[DriveCharacterization] Mode: SYSID_DYNAMIC");
            resetTest();
        } else if (driverController.getXButtonPressed()) {
            currentTestMode = TestMode.MODULE_INDIVIDUAL;
            System.out.println("[DriveCharacterization] Mode: MODULE_INDIVIDUAL");
            resetTest();
        } else if (driverController.getYButtonPressed()) {
            currentTestMode = TestMode.KINEMATICS_TEST;
            System.out.println("[DriveCharacterization] Mode: KINEMATICS_TEST");
            resetTest();
        }
        
        // Odometry test with both bumpers
        if (driverController.getLeftBumperButton() && driverController.getRightBumperButton()) {
            currentTestMode = TestMode.ODOMETRY_TEST;
            System.out.println("[DriveCharacterization] Mode: ODOMETRY_TEST");
            resetTest();
        }
    }
    
    /**
     * SysID quasistatic test mode
     */
    private void handleSysIdQuasistatic(Controller driverController, Controller operatorController) {
        // Use face buttons for SysID commands
        if (driverController.getAButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting quasistatic forward");
            driveRoutine.quasistatic(Direction.kForward).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else if (driverController.getBButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting quasistatic reverse");
            driveRoutine.quasistatic(Direction.kReverse).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        
        // Steer testing with X/Y
        if (driverController.getXButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting steer quasistatic forward");
            steerRoutine.quasistatic(Direction.kForward).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else if (driverController.getYButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting steer quasistatic reverse");
            steerRoutine.quasistatic(Direction.kReverse).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
    }
    
    /**
     * SysID dynamic test mode
     */
    private void handleSysIdDynamic(Controller driverController, Controller operatorController) {
        // Use face buttons for SysID commands
        if (driverController.getAButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting dynamic forward");
            driveRoutine.dynamic(Direction.kForward).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else if (driverController.getBButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting dynamic reverse");
            driveRoutine.dynamic(Direction.kReverse).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        
        // Steer testing with X/Y
        if (driverController.getXButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting steer dynamic forward");
            steerRoutine.dynamic(Direction.kForward).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else if (driverController.getYButtonPressed()) {
            System.out.println("[DriveCharacterization] Starting steer dynamic reverse");
            steerRoutine.dynamic(Direction.kReverse).schedule();
            testRunning = true;
            testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
    }
    
    /**
     * Individual module testing mode
     */
    private void handleModuleIndividual(Controller driverController, Controller operatorController) {
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Module switching with D-pad
        int pov = operatorController.getPOV();
        switch (pov) {
            case 0: // Up - Next module
                activeModule = (activeModule + 1) % 4;
                System.out.println("[DriveCharacterization] Active module: " + MODULE_NAMES[activeModule]);
                break;
            case 180: // Down - Previous module
                activeModule = (activeModule - 1 + 4) % 4;
                System.out.println("[DriveCharacterization] Active module: " + MODULE_NAMES[activeModule]);
                break;
        }
        
        // Manual control with right joystick
        double rightY = -operatorController.getRightY();
        if (Math.abs(rightY) > 0.1) {
            // Drive motor test
            swerve.setModuleDriveVoltage(activeModule, rightY * testVoltage.get());
            testRunning = true;
        } else {
            swerve.setModuleDriveVoltage(activeModule, 0);
        }
        
        double rightX = operatorController.getRightX();
        if (Math.abs(rightX) > 0.1) {
            // Steer motor test
            swerve.setModuleAngleVoltage(activeModule, rightX * testVoltage.get());
            testRunning = true;
        } else {
            swerve.setModuleAngleVoltage(activeModule, 0);
        }
        
        if (Math.abs(rightY) < 0.1 && Math.abs(rightX) < 0.1) {
            testRunning = false;
        }
    }
    
    /**
     * Kinematics test mode
     */
    private void handleKinematicsTest(Controller driverController, Controller operatorController) {
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Test different drive patterns
        if (driverController.getRightTriggerAxis() > 0.5) {
            // Forward drive
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(1.0, 0), 0, false);
            testRunning = true;
        } else if (driverController.getLeftTriggerAxis() > 0.5) {
            // Strafe drive
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 1.0), 0, false);
            testRunning = true;
        } else if (Math.abs(driverController.getRightX()) > 0.5) {
            // Rotation
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), driverController.getRightX(), false);
            testRunning = true;
        } else {
            swerve.stop();
            testRunning = false;
        }
    }
    
    /**
     * Odometry test mode
     */
    private void handleOdometryTest(Controller driverController, Controller operatorController) {
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Drive in square pattern
        if (driverController.getRightTriggerAxis() > 0.5) {
            double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime;
            
            if (elapsed < 2.0) {
                // Forward for 2 seconds
                swerve.drive(new edu.wpi.first.math.geometry.Translation2d(1.0, 0), 0, false);
            } else if (elapsed < 4.0) {
                // Right for 2 seconds
                swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, -1.0), 0, false);
            } else if (elapsed < 6.0) {
                // Back for 2 seconds
                swerve.drive(new edu.wpi.first.math.geometry.Translation2d(-1.0, 0), 0, false);
            } else if (elapsed < 8.0) {
                // Left for 2 seconds
                swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 1.0), 0, false);
            } else {
                swerve.stop();
                testRunning = false;
            }
            
            if (!testRunning) {
                testRunning = true;
                testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                startHeading = swerve.getPose().getRotation().getRadians();
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
        SwerveBase swerve = SwerveBase.getInstance();
        
        // Track max velocity
        double currentVelocity = swerve.getChassisSpeeds().vxMetersPerSecond;
        maxVelocity = Math.max(maxVelocity, Math.abs(currentVelocity));
        
        // Track max acceleration (simplified)
        if (testRunning) {
            double acceleration = Math.abs(currentVelocity) / (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime);
            maxAcceleration = Math.max(maxAcceleration, acceleration);
        }
        
        // Track total distance
        totalDistance += Math.abs(currentVelocity * 0.02); // Assuming 20ms loop
        
        // Track total rotation
        double currentHeading = swerve.getPose().getRotation().getRadians();
        totalRotation = Math.abs(currentHeading - startHeading);
    }
    
    /**
     * Reset test state
     */
    private void resetTest() {
        testRunning = false;
        testStartTime = 0;
        maxVelocity = 0;
        maxAcceleration = 0;
        totalDistance = 0;
        totalRotation = 0;
        startHeading = 0;
        SwerveBase.getInstance().stop();
    }
    
    /**
     * Setup dashboard controls
     */
    public void setupDashboard() {
        SmartDashboard.putString("Test/Drive/Mode", currentTestMode.name());
        SmartDashboard.putNumber("Test/Drive/TestVoltage", testVoltage.get());
        SmartDashboard.putNumber("Test/Drive/TestDistance", testDistance.get());
        SmartDashboard.putNumber("Test/Drive/TestAngle", testAngle.get());
        SmartDashboard.putString("Test/Drive/ActiveModule", MODULE_NAMES[activeModule]);
    }
    
    /**
     * Update dashboard values
     */
    public void updateDashboard() {
        SmartDashboard.putString("Test/Drive/Mode", currentTestMode.name());
        SmartDashboard.putBoolean("Test/Drive/Running", testRunning);
        SmartDashboard.putNumber("Test/Drive/MaxVelocity", maxVelocity);
        SmartDashboard.putNumber("Test/Drive/MaxAcceleration", maxAcceleration);
        SmartDashboard.putNumber("Test/Drive/TotalDistance", totalDistance);
        SmartDashboard.putNumber("Test/Drive/TotalRotation", Math.toDegrees(totalRotation));
        SmartDashboard.putString("Test/Drive/ActiveModule", MODULE_NAMES[activeModule]);
        
        // Update tunable numbers
        testVoltage.update();
        testDistance.update();
        testAngle.update();
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
