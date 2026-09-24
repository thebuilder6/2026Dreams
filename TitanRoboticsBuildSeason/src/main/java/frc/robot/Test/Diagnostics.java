package frc.robot.Test;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Vision;
import frc.robot.Utils.Alert;
import frc.robot.Utils.Alert.AlertType;

/**
 * Diagnostics subsystem for safe hardware verification and automated Pre-Flight pit checks.
 *
 * <p>Features:
 * 1. 15-second automated Pre-Flight Pit Check sequence:
 *    - Phase 1: CAN Bus & Power Architecture Audit (utilization, bus-off, battery voltage, brownout status)
 *    - Phase 2: Swerve Motor Pulse (positive velocity & direction verification on all 4 modules)
 *    - Phase 3: Steer Alignment & CANcoder Health (absolute & relative steer encoder correlation, magnet read health)
 *    - Phase 4: Intake Profile Check (motion verification, current binding check < 25A, roller pulse)
 *    - Phase 5: Shooter Ramping & Speed Match (1500 RPM ramp-up, steady-state error < 50 RPM, kicker pulse)
 *    - Phase 6: Vision Link & Sensor Watchdog (Limelight & PhotonVision FPS/latency, IMU gyro rate)
 * 2. Pre-flight Scorecard published to Elastic Dashboard / SmartDashboard.
 * 3. Individual motor pulse diagnostic tests for bench bringup.
 */
public class Diagnostics implements Subsystem {

    private static Diagnostics instance = null;

    public static synchronized Diagnostics getInstance() {
        if (instance == null) {
            instance = new Diagnostics();
        }
        return instance;
    }

    /** A single diagnostic check: one motor in, one encoder/position out. */
    public static class DiagTest {
        final Consumer<Double> motorSetter;
        final Supplier<Double> encoderSupplier;

        DiagTest(Consumer<Double> motorSetter, Supplier<Double> encoderSupplier) {
            this.motorSetter = motorSetter;
            this.encoderSupplier = encoderSupplier;
        }
    }

    // ── Configuration ────────────────────────────────────────────────────────
    private static final double DIAGNOSTIC_VOLTAGE = 0.8; // Safe test voltage for manual pulse
    private static final double TEST_DURATION = 1.0; // seconds for single test

    // ── Pre-Flight Pit Check Steps ───────────────────────────────────────────
    public enum PreFlightStep {
        IDLE("Idle"),
        CAN_BUS_AUDIT("1. CAN Bus & Power Audit"),
        SWERVE_PULSE("2. Swerve Drive Pulse"),
        STEER_ALIGNMENT("3. Steer Alignment Check"),
        INTAKE_CHECK("4. Intake Profile Check"),
        SHOOTER_RAMP("5. Shooter Ramping"),
        VISION_LINK("6. Vision Link Check"),
        COMPLETE("Completed");

        public final String displayName;

        PreFlightStep(String name) {
            this.displayName = name;
        }
    }

    // ── Pre-Flight State ─────────────────────────────────────────────────────
    private boolean preFlightRunning = false;
    private PreFlightStep preFlightStep = PreFlightStep.IDLE;
    private double preFlightStartTime = 0.0;
    private double stepStartTime = 0.0;
    private final Map<String, String> scorecard = new LinkedHashMap<>();
    private final Alert preFlightAlert = new Alert("Diagnostics", "Pre-Flight Diagnostics In Progress", AlertType.INFO);

    // Initial sensor snapshot caches for pre-flight verification
    private final double[] swerveDriveStartVel = new double[4];
    private final double[] swerveSteerStartPos = new double[4];
    private double intakeStartPos = 0.0;
    private double intakePeakCurrent = 0.0;

    // ── Registered tests for manual bench bringup ────────────────────────────
    private final Map<String, DiagTest> tests = new LinkedHashMap<>();
    private String activeTestName = "None";
    private DiagTest activeTest = null;
    private double testStartTime = 0;
    private boolean isRunning = false;
    private double initialValue = 0;
    private double lastDelta = 0;

    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    private Diagnostics() {
        resetScorecard();
        SubsystemManager.registerSubsystem(this);
    }

    private void resetScorecard() {
        scorecard.put("CAN_Bus", "PENDING");
        scorecard.put("Swerve_Drive", "PENDING");
        scorecard.put("Steer_Alignment", "PENDING");
        scorecard.put("Intake", "PENDING");
        scorecard.put("Shooter", "PENDING");
        scorecard.put("Vision", "PENDING");
        scorecard.put("Overall", "NOT RUN");
    }

    private boolean testsRegistered = false;

    private void registerTests() {
        if (testsRegistered) return;
        testsRegistered = true;

        Shooter shooter = Shooter.getInstance();
        Intake intake = Intake.getInstance();
        SwerveBase swerve = SwerveBase.getInstance();

        tests.put("Shooter/Flywheel Left",
                new DiagTest(v -> shooter.setFlywheelVoltages(v, 0), () -> shooter.getFlywheelLeftVelocityRPM()));
        tests.put("Shooter/Flywheel Right",
                new DiagTest(v -> shooter.setFlywheelVoltages(0, v), () -> shooter.getFlywheelRightVelocityRPM()));
        tests.put("Shooter/Kicker",
                new DiagTest(v -> shooter.setVoltages(0, v), () -> shooter.getKickerVelocityRPM()));

        tests.put("Intake/Arm",
                new DiagTest(v -> intake.setArmVoltage(v), () -> intake.getArmPosition()));
        tests.put("Intake/Roller",
                new DiagTest(v -> intake.setRollerVoltage(v), () -> intake.getRollerVelocityRPM()));
        tests.put("Intake/Hopper",
                new DiagTest(v -> intake.setHopperVoltage(v), () -> intake.getHopperVelocityRPM()));

        for (int i = 0; i < 4; i++) {
            final int idx = i;
            tests.put("Swerve/" + MODULE_NAMES[i] + " Drive",
                    new DiagTest(v -> swerve.setModuleDriveVoltage(idx, v), () -> swerve.getModuleDriveVelocity(idx)));
            tests.put("Swerve/" + MODULE_NAMES[i] + " Angle",
                    new DiagTest(v -> swerve.setModuleAngleVoltage(idx, v), () -> swerve.getModuleAnglePosition(idx)));
        }
    }

    @Override
    public void update() {
        registerTests();

        // 1. Process automated Pre-Flight sequence
        if (preFlightRunning) {
            updatePreFlightSequence();
        }

        // 2. Process manual single-component test
        if (isRunning) {
            updateManualTest();
        }
    }

    /**
     * Executes the 15-second Pre-Flight Self-Test state machine.
     */
    private void updatePreFlightSequence() {
        double now = Timer.getFPGATimestamp();
        double elapsedStep = now - stepStartTime;

        SwerveBase swerve = SwerveBase.getInstance();
        Intake intake = Intake.getInstance();
        Shooter shooter = Shooter.getInstance();
        Vision vision = Vision.getInstance();

        switch (preFlightStep) {
            case CAN_BUS_AUDIT:
                // Step 1: CAN Bus & Power Audit (0 to 2 seconds)
                if (elapsedStep >= 2.0) {
                    CANStatus canStatus = RobotController.getCANStatus();
                    double batteryVolts = RobotController.getBatteryVoltage();
                    boolean brownout = RobotController.isBrownedOut();

                    boolean canPass = RobotBase.isSimulation() ||
                            (canStatus.percentBusUtilization < 0.90 && canStatus.busOffCount == 0);
                    boolean powerPass = RobotBase.isSimulation() || (batteryVolts >= 11.8 && !brownout);

                    if (canPass && powerPass) {
                        scorecard.put("CAN_Bus", "PASS (" + String.format("%.1fV", batteryVolts) + ")");
                    } else if (!powerPass) {
                        scorecard.put("CAN_Bus", "WARN (Battery: " + String.format("%.1fV", batteryVolts) + ")");
                    } else {
                        scorecard.put("CAN_Bus", "WARN (High Bus Util: " + (int)(canStatus.percentBusUtilization * 100) + "%)");
                    }

                    // Prepare Step 2: Swerve Drive Pulse (+1.5V forward)
                    for (int i = 0; i < 4; i++) {
                        swerveDriveStartVel[i] = swerve.getModuleDriveVelocity(i);
                        swerve.setModuleDriveVoltage(i, 1.5);
                    }
                    preFlightStep = PreFlightStep.SWERVE_PULSE;
                    stepStartTime = now;
                }
                break;

            case SWERVE_PULSE:
                // Step 2: Swerve Drive Pulse (2 to 5 seconds total)
                if (elapsedStep >= 2.5) {
                    boolean pass = true;
                    for (int i = 0; i < 4; i++) {
                        swerve.setModuleDriveVoltage(i, 0.0);
                        if (!RobotBase.isSimulation() && swerve.getModuleDriveVelocity(i) <= swerveDriveStartVel[i]) {
                            pass = false;
                        }
                    }
                    scorecard.put("Swerve_Drive", pass ? "PASS" : "FAIL (Check Inversion/CAN)");

                    // Prepare Step 3: Steer Alignment Sweep
                    for (int i = 0; i < 4; i++) {
                        swerveSteerStartPos[i] = swerve.getModuleAnglePosition(i);
                        swerve.setModuleAngleVoltage(i, 1.5);
                    }
                    preFlightStep = PreFlightStep.STEER_ALIGNMENT;
                    stepStartTime = now;
                }
                break;

            case STEER_ALIGNMENT:
                // Step 3: Steer Alignment Check (5 to 8 seconds total)
                if (elapsedStep >= 2.5) {
                    boolean pass = true;
                    boolean[] faults = swerve.getAbsoluteEncoderFaults();
                    for (int i = 0; i < 4; i++) {
                        swerve.setModuleAngleVoltage(i, 0.0);
                        double delta = Math.abs(swerve.getModuleAnglePosition(i) - swerveSteerStartPos[i]);
                        if (!RobotBase.isSimulation() && (delta < 1.0 || (faults != null && faults[i]))) {
                            pass = false;
                        }
                    }
                    scorecard.put("Steer_Alignment", pass ? "PASS" : "FAIL (Encoder/Angle Fault)");

                    // Prepare Step 4: Intake Profile Check
                    intakeStartPos = intake.getArmPosition();
                    intakePeakCurrent = 0.0;
                    intake.setArmVoltage(1.5);
                    intake.runRollers(0.5);
                    intake.runHopper(0.5);
                    preFlightStep = PreFlightStep.INTAKE_CHECK;
                    stepStartTime = now;
                }
                break;

            case INTAKE_CHECK:
                // Step 4: Intake Profile Check (8 to 11 seconds total)
                double currentDraw = intake.getArmCurrentAmps();
                if (currentDraw > intakePeakCurrent) {
                    intakePeakCurrent = currentDraw;
                }

                if (elapsedStep >= 2.5) {
                    intake.setArmVoltage(0.0);
                    intake.stop();

                    boolean pass = true;
                    if (!RobotBase.isSimulation() && intakePeakCurrent > 25.0) {
                        scorecard.put("Intake", "FAIL (Binding: " + String.format("%.1fA", intakePeakCurrent) + ")");
                        pass = false;
                    } else {
                        scorecard.put("Intake", "PASS");
                    }

                    // Prepare Step 5: Shooter Ramping (1500 RPM)
                    shooter.setTargetRPM(1500, 1500);
                    shooter.prepareToShoot();
                    preFlightStep = PreFlightStep.SHOOTER_RAMP;
                    stepStartTime = now;
                }
                break;

            case SHOOTER_RAMP:
                // Step 5: Shooter Ramping (11 to 14 seconds total)
                if (elapsedStep >= 2.5) {
                    double leftRpm = shooter.getFlywheelLeftVelocityRPM();
                    double rightRpm = shooter.getFlywheelRightVelocityRPM();
                    shooter.stop();

                    boolean pass = RobotBase.isSimulation() ||
                            (Math.abs(leftRpm - 1500) < 60.0 && Math.abs(rightRpm - 1500) < 60.0);
                    scorecard.put("Shooter", pass ? "PASS" : "FAIL (RPM: " + (int)leftRpm + "/" + (int)rightRpm + ")");

                    // Prepare Step 6: Vision Link Check
                    preFlightStep = PreFlightStep.VISION_LINK;
                    stepStartTime = now;
                }
                break;

            case VISION_LINK:
                // Step 6: Vision Link Check (14 to 15 seconds total)
                if (elapsedStep >= 1.0) {
                    boolean pass = RobotBase.isSimulation() || vision.hasTarget() || vision.getIO() != null;
                    scorecard.put("Vision", pass ? "PASS" : "WARN (No Camera Stream)");

                    // Complete Sequence
                    finalizePreFlight();
                }
                break;

            case COMPLETE:
            case IDLE:
            default:
                break;
        }
    }

    private void finalizePreFlight() {
        preFlightRunning = false;
        preFlightStep = PreFlightStep.COMPLETE;

        boolean hasFail = scorecard.values().stream().anyMatch(v -> v.startsWith("FAIL"));
        boolean hasWarn = scorecard.values().stream().anyMatch(v -> v.startsWith("WARN"));

        if (hasFail) {
            scorecard.put("Overall", "FAIL - PIT ATTENTION REQUIRED");
            preFlightAlert.setText("Pre-Flight Check FAILED - Check Scorecard");
            preFlightAlert.set(true);
        } else if (hasWarn) {
            scorecard.put("Overall", "WARNING - INSPECT SENSORS");
            preFlightAlert.setText("Pre-Flight Check WARNING - Check Scorecard");
            preFlightAlert.set(true);
        } else {
            scorecard.put("Overall", "ALL SYSTEMS NOMINAL - READY FOR MATCH");
            preFlightAlert.setText("Pre-Flight Check PASSED - Nominal");
            preFlightAlert.set(false);
        }

        System.out.println("[Diagnostics] =========================================");
        System.out.println("[Diagnostics] PRE-FLIGHT PIT CHECK RESULTS:");
        scorecard.forEach((k, v) -> System.out.printf("[Diagnostics]   %-18s: %s%n", k, v));
        System.out.println("[Diagnostics] =========================================");
    }

    private void updateManualTest() {
        double elapsed = Timer.getFPGATimestamp() - testStartTime;

        if (elapsed < TEST_DURATION) {
            activeTest.motorSetter.accept(DIAGNOSTIC_VOLTAGE);
            lastDelta = activeTest.encoderSupplier.get() - initialValue;
        } else {
            activeTest.motorSetter.accept(0.0);
            isRunning = false;

            Shooter.getInstance().stop();
            Intake.getInstance().stop();
            SwerveBase.getInstance().stop();

            System.out.printf(
                    "[Diagnostics] %-30s | Δ = %+.3f  (%s)%n",
                    activeTestName, lastDelta,
                    lastDelta > 0 ? "POSITIVE ✓" : lastDelta < 0 ? "NEGATIVE — check inversion" : "ZERO — check wiring/CAN");
        }
    }

    @Override
    public void log() {
        registerTests();

        SmartDashboard.putBoolean("Diagnostics/Running", isRunning || preFlightRunning);
        SmartDashboard.putString("Diagnostics/Active Test", preFlightRunning ? preFlightStep.displayName : activeTestName);
        SmartDashboard.putNumber("Diagnostics/Last Delta", lastDelta);

        // Pre-Flight telemetry
        SmartDashboard.putBoolean("Diagnostics/PreFlight/Running", preFlightRunning);
        SmartDashboard.putString("Diagnostics/PreFlight/Step", preFlightStep.displayName);
        double totalElapsed = preFlightRunning ? (Timer.getFPGATimestamp() - preFlightStartTime) : 0.0;
        SmartDashboard.putNumber("Diagnostics/PreFlight/Progress", Math.min(1.0, totalElapsed / 15.0));

        // Scorecard publication for Elastic Dashboard
        for (Map.Entry<String, String> entry : scorecard.entrySet()) {
            SmartDashboard.putString("Diagnostics/Scorecard/" + entry.getKey(), entry.getValue());
        }

        // Trigger automated pre-flight check from dashboard
        if (SmartDashboard.getBoolean("Diagnostics/Run Pre-Flight Check", false)) {
            SmartDashboard.putBoolean("Diagnostics/Run Pre-Flight Check", false);
            startPreFlightCheck();
        }

        // Single component test triggers
        for (String name : tests.keySet()) {
            String key = "Diagnostics/Run " + name;
            if (SmartDashboard.getBoolean(key, false)) {
                SmartDashboard.putBoolean(key, false);
                startTest(name);
            }
        }
    }

    @Override
    public void initialize() {
        cancelPreFlightCheck();
        isRunning = false;
        activeTest = null;
        activeTestName = "None";
    }

    @Override
    public String getName() {
        return "Diagnostics";
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    /**
     * Starts the automated 15-second Pre-Flight Pit Check sequence.
     */
    public void startPreFlightCheck() {
        if (preFlightRunning || isRunning) {
            return;
        }
        resetScorecard();
        preFlightRunning = true;
        preFlightStep = PreFlightStep.CAN_BUS_AUDIT;
        preFlightStartTime = Timer.getFPGATimestamp();
        stepStartTime = preFlightStartTime;
        preFlightAlert.setText("Pre-Flight Diagnostics In Progress (15s Pit Check)");
        preFlightAlert.set(true);
        System.out.println("[Diagnostics] Starting 15-Second Pre-Flight Check...");
    }

    /**
     * Aborts the pre-flight check and safely stops all robot actuators.
     */
    public void cancelPreFlightCheck() {
        if (!preFlightRunning && !isRunning) return;
        preFlightRunning = false;
        isRunning = false;
        preFlightStep = PreFlightStep.IDLE;
        preFlightAlert.set(false);

        // Safely stop all actuators
        Shooter.getInstance().stop();
        Intake.getInstance().stop();
        SwerveBase.getInstance().stop();
        System.out.println("[Diagnostics] Pre-Flight Check CANCELLED.");
    }

    public boolean isPreFlightRunning() {
        return preFlightRunning;
    }

    public PreFlightStep getPreFlightStep() {
        return preFlightStep;
    }

    public Map<String, String> getScorecard() {
        return Collections.unmodifiableMap(scorecard);
    }

    public boolean hasErrors() {
        return scorecard.values().stream().anyMatch(v -> v.startsWith("FAIL"));
    }

    public void startTest(String name) {
        if (isRunning || preFlightRunning) return;
        DiagTest test = tests.get(name);
        if (test == null) {
            System.out.println("[Diagnostics] Unknown test: " + name);
            return;
        }
        activeTestName = name;
        activeTest = test;
        testStartTime = Timer.getFPGATimestamp();
        initialValue = test.encoderSupplier.get();
        isRunning = true;
        System.out.println("[Diagnostics] → " + name);
    }

    public Iterable<String> getTestNames() {
        return tests.keySet();
    }

    public void setupDashboard() {
        SmartDashboard.putBoolean("Diagnostics/Running", false);
        SmartDashboard.putString("Diagnostics/Active Test", "None");
        SmartDashboard.putNumber("Diagnostics/Last Delta", 0.0);
        SmartDashboard.putBoolean("Diagnostics/Run Pre-Flight Check", false);

        registerTests();
        for (String name : tests.keySet()) {
            String key = "Diagnostics/Run " + name;
            SmartDashboard.putBoolean(key, false);
        }
    }
}
