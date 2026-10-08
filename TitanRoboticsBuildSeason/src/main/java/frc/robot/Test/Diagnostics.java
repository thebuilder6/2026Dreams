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
import frc.robot.Telemetry.Alert;
import frc.robot.Telemetry.Alert.AlertType;
import frc.robot.Telemetry.TelemetryKeys;

/**
 * Diagnostics subsystem for safe hardware verification and automated Pre-Flight pit checks.
 *
 * <p>Features:
 * 1. 15-second automated Pre-Flight Pit Check sequence:
 *    - Step 1: CAN Bus & System Power Audit (utilization, bus-off, battery voltage, power rails)
 *    - Step 2: Swerve Motor Pulse (positive velocity verification on all 4 modules)
 *    - Step 3: Steer Alignment & CANcoder Audit (absolute & relative steer encoder correlation and health)
 *    - Step 4: Intake Profile Check (motion verification & current binding check > 25A)
 *    - Step 5: Shooter Ramping (1500 RPM ramp-up, steady-state error & left/right symmetry check)
 *    - Step 6: Vision Link Check (FPS, latency & camera link check)
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

        public DiagTest(Consumer<Double> motorSetter, Supplier<Double> encoderSupplier) {
            this.motorSetter = motorSetter;
            this.encoderSupplier = encoderSupplier;
        }
    }

    // ── Configuration ────────────────────────────────────────────────────────
    private static final double DIAGNOSTIC_VOLTAGE = 1.5; // Volts for manual single-pulse
    private static final double TEST_DURATION = 1.0; // seconds for single test

    // Pre-flight pass/fail thresholds (single-owned here, not duplicated per step)
    static final double INTAKE_MIN_MOVEMENT_DEG = 2.0;
    static final double CAN_UTIL_WARN = 0.70;
    static final double CAN_UTIL_FAIL = 0.90;
    static final double VISION_FRESHNESS_SEC = 1.0;
    static final double VISION_MAX_LATENCY_MS = 500.0;

    // ── Pre-Flight Pit Check Steps ───────────────────────────────────────────
    public enum PreFlightStep {
        IDLE("Idle"),
        CAN_BUS_AUDIT("1. CAN & Power Audit"),
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
    private double[] swerveDriveStartVel = new double[4];
    private double[] swerveSteerStartPos = new double[4];
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

    public void resetScorecard() {
        scorecard.put("CAN_Bus", "PENDING");
        scorecard.put("Power_PDH", "PENDING");
        scorecard.put("Storage_Logs", "PENDING");
        scorecard.put("Memory_Heap", "PENDING");
        scorecard.put("Swerve_Drive", "PENDING");
        scorecard.put("Steer_Alignment", "PENDING");
        scorecard.put("Intake", "PENDING");
        scorecard.put("Shooter", "PENDING");
        scorecard.put("Vision", "PENDING");
        scorecard.put("Overall", "NOT RUN");
    }

    private final Map<String, String> testKeys = new LinkedHashMap<>();
    private boolean scorecardDirty = true;

    private boolean testsRegistered = false;

    public void registerTests() {
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

        for (String name : tests.keySet()) {
            testKeys.put(name, "Diagnostics/Run " + name);
        }
    }

    @Override
    public void update() {
        registerTests();

        // Safety: pre-flight + manual jogs are disabled-only on real hardware.
        // If the robot is enabled mid-sequence (e.g. DS enable during pit check),
        // abort and stop actuators rather than fighting the enabled loop.
        if (!RobotBase.isSimulation() && !DriverStation.isDisabled()) {
            if (preFlightRunning) {
                cancelPreFlightCheck();
            }
            if (isRunning && activeTest != null) {
                activeTest.motorSetter.accept(0.0);
                isRunning = false;
            }
            return;
        }

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
    public void updatePreFlightSequence() {
        double now = Timer.getFPGATimestamp();
        double elapsedStep = now - stepStartTime;

        SwerveBase swerve = SwerveBase.getInstance();
        Intake intake = Intake.getInstance();
        Shooter shooter = Shooter.getInstance();
        Vision vision = Vision.getInstance();

        switch (preFlightStep) {
            case CAN_BUS_AUDIT:
                // Step 1: CAN Bus & System Power Audit (0 to 2 seconds)
                if (elapsedStep >= 2.0) {
                    CANStatus canStatus = RobotController.getCANStatus();
                    double batteryVolts = RobotController.getBatteryVoltage();
                    boolean brownout = RobotController.isBrownedOut();

                    if (RobotBase.isSimulation()) {
                        scorecard.put("CAN_Bus", "PASS");
                    } else if (canStatus.busOffCount > 0) {
                        scorecard.put("CAN_Bus", "FAIL (Bus-Off: " + canStatus.busOffCount + ")");
                    } else if (canStatus.txFullCount > 0) {
                        scorecard.put("CAN_Bus", "FAIL (TX Full: " + canStatus.txFullCount + " overflow)");
                    } else if (brownout) {
                        scorecard.put("CAN_Bus", "FAIL (Brownout)");
                    } else if (batteryVolts < 12.0) {
                        scorecard.put("CAN_Bus", "WARN (Low Battery: " + String.format("%.1fV", batteryVolts) + ")");
                    } else if (canStatus.percentBusUtilization >= CAN_UTIL_FAIL) {
                        scorecard.put("CAN_Bus", "FAIL (Bus Util: " + (int)(canStatus.percentBusUtilization * 100) + "%)");
                    } else if (canStatus.percentBusUtilization >= CAN_UTIL_WARN) {
                        scorecard.put("CAN_Bus", "WARN (Bus Util: " + (int)(canStatus.percentBusUtilization * 100) + "%)");
                    } else if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
                        scorecard.put("CAN_Bus", "WARN (RX Err: " + canStatus.receiveErrorCount
                                + " TX Err: " + canStatus.transmitErrorCount + ")");
                    } else {
                        scorecard.put("CAN_Bus", "PASS");
                    }

                    // Prepare Step 2: Swerve Drive Pulse
                    for (int i = 0; i < 4; i++) {
                        swerveDriveStartVel[i] = swerve.getModuleDriveVelocity(i);
                        swerve.setModuleDriveVoltage(i, 1.5); // +1.5V forward pulse
                    }
                    auditSystemHealth();
                    preFlightStep = PreFlightStep.SWERVE_PULSE;
                    stepStartTime = now;
                }
                break;

            case SWERVE_PULSE:
                // Step 2: Swerve Drive Pulse (2 to 5 seconds total)
                if (elapsedStep >= 2.5) {
                    boolean pass = true;
                    for (int i = 0; i < 4; i++) {
                        swerve.setModuleDriveVoltage(i, 0.0); // Stop drive motor
                        if (!RobotBase.isSimulation() && swerve.getModuleDriveVelocity(i) <= swerveDriveStartVel[i]) {
                            pass = false;
                        }
                    }
                    scorecard.put("Swerve_Drive", pass ? "PASS" : "FAIL (Check Inversion/CAN)");

                    // Prepare Step 3: Steer Alignment Sweep
                    for (int i = 0; i < 4; i++) {
                        swerveSteerStartPos[i] = swerve.getModuleAnglePosition(i);
                        swerve.setModuleAngleVoltage(i, 1.5); // +1.5V steering sweep
                    }
                    preFlightStep = PreFlightStep.STEER_ALIGNMENT;
                    stepStartTime = now;
                }
                break;

            case STEER_ALIGNMENT:
                // Step 3: Steer Alignment Check (5 to 8 seconds total)
                if (elapsedStep >= 2.5) {
                    boolean pass = true;
                    boolean encoderIssue = swerve.hasAbsoluteEncoderIssues();

                    for (int i = 0; i < 4; i++) {
                        swerve.setModuleAngleVoltage(i, 0.0); // Stop angle motor
                        double delta = Math.abs(swerve.getModuleAnglePosition(i) - swerveSteerStartPos[i]);
                        if (!RobotBase.isSimulation() && delta < 1.0) {
                            pass = false;
                        }
                    }
                    if (encoderIssue) {
                        scorecard.put("Steer_Alignment", "FAIL (CANcoder Read Issue)");
                    } else {
                        scorecard.put("Steer_Alignment", pass ? "PASS" : "FAIL (No Movement)");
                    }

                    // Prepare Step 4: Intake Profile Check
                    intakeStartPos = intake.getArmPosition();
                    intakePeakCurrent = 0.0;
                    intake.setArmVoltage(1.5);
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

                    double intakeEndPos = intake.getArmPosition();
                    double rawDelta = Math.abs(intakeEndPos - intakeStartPos);
                    double intakeDelta = Math.min(rawDelta, 360.0 - rawDelta);
                    boolean encoderOk = RobotBase.isSimulation() || intake.getInputs().encoderConnected;

                    if (!RobotBase.isSimulation() && !encoderOk) {
                        scorecard.put("Intake", "FAIL (Encoder Disconnected)");
                    } else if (!RobotBase.isSimulation() && intakeDelta < INTAKE_MIN_MOVEMENT_DEG) {
                        scorecard.put("Intake", "FAIL (No Movement: " + String.format("%.1f deg", intakeDelta) + ")");
                    } else if (!RobotBase.isSimulation() && intakePeakCurrent > 25.0) {
                        scorecard.put("Intake", "FAIL (Binding: " + String.format("%.1fA", intakePeakCurrent) + ")");
                    } else {
                        scorecard.put("Intake", "PASS");
                    }

                    // Prepare Step 5: Shooter Ramping
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
                            (Math.abs(leftRpm - 1500) < 50.0 && Math.abs(rightRpm - 1500) < 50.0);
                    scorecard.put("Shooter", pass ? "PASS" : "FAIL (RPM: " + (int)leftRpm + "/" + (int)rightRpm + ")");

                    // Prepare Step 6: Vision Link Check
                    preFlightStep = PreFlightStep.VISION_LINK;
                    stepStartTime = now;
                }
                break;

            case VISION_LINK:
                // Step 6: Vision Link Check (14 to 15 seconds total)
                // A live IO object is not a live stream: require a fresh timestamp
                // (or a current target) instead of getIO() != null.
                if (elapsedStep >= 1.0) {
                    // Link-first gate (merged 2026-10-07): the pre-flight Vision step
                    // verifies the camera *link*, not tag visibility — a tag-less pit
                    // is normal. PASS requires every enabled camera to report
                    // connected via its heartbeat watchdog
                    // (Vision.isAllCamerasConnected). Freshness (hasTarget or a
                    // timestamp <1.0 s old with latency <500 ms) is reported as
                    // detail so a live tag sighting is visible, but a healthy
                    // link with nothing in view still passes.
                    boolean pass;
                    String status;
                    if (RobotBase.isSimulation()) {
                        pass = true;
                        status = "PASS";
                    } else {
                        int totalCount = vision.getCameras().size();
                        int onlineCount = 0;
                        for (var c : vision.getCameras()) {
                            if (c.isConnected()) onlineCount++;
                        }
                        boolean connected = !vision.getCameras().isEmpty()
                                && vision.isAllCamerasConnected();
                        boolean liveTarget = vision.hasTarget()
                                || isVisionInputFresh(vision.getInputs(), now)
                                || (vision.getSecondaryInputs() != null
                                        && isVisionInputFresh(vision.getSecondaryInputs(), now));
                        if (!connected) {
                            pass = false;
                            status = "WARN (" + onlineCount + "/" + totalCount + " Online)";
                        } else if (liveTarget) {
                            pass = true;
                            status = "PASS (" + onlineCount + "/" + totalCount + " Online, Target/Fresh)";
                        } else {
                            pass = true;
                            status = "PASS (" + onlineCount + "/" + totalCount + " Online, No Target In View)";
                        }
                    }
                    scorecard.put("Vision", status);

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

    /**
     * System-health audit covering the Rio log faults of 2026-10-07: PDH
     * {@code getTotalCurrent()} throwing {@code CAN: Message not found}
     * (SwerveBase update/log), missing {@code /U} USB log target plus
     * {@code /home/lvuser/logs} below 50 MB free, and JVM heap exhaustion
     * ({@code std::bad_alloc} / {@code commit_memory failed}). Runs inside the
     * CAN &amp; Power step so it needs no mechanism motion. All probes are
     * sim-safe and never throw.
     */
    public void auditSystemHealth() {
        // 1. Power distribution current read (live CAN probe).
        if (RobotBase.isSimulation()) {
            scorecard.put("Power_PDH", "SKIP (Sim)");
        } else {
            try {
                edu.wpi.first.wpilibj.PowerDistribution pd = SwerveBase.getInstance().getPowerDistribution();
                if (pd == null) {
                    scorecard.put("Power_PDH", "FAIL (PD uninitialized)");
                } else {
                    double amps = pd.getTotalCurrent();
                    scorecard.put("Power_PDH", "PASS (" + String.format("%.1fA", amps) + ")");
                }
            } catch (Throwable t) {
                scorecard.put("Power_PDH", "FAIL (PD CAN: Message not found)");
            }
        }

        // 2. Log storage: /U USB stick plus internal fallback dir.
        try {
            java.io.File usb = new java.io.File("/U");
            java.io.File internal = new java.io.File("/home/lvuser/logs");
            long usbFree = usb.exists() ? usb.getUsableSpace() : -1L;
            long internalFree = internal.exists() ? internal.getUsableSpace()
                    : new java.io.File("/tmp").getUsableSpace();
            final long lowBytes = 50L * 1024L * 1024L;
            if (!usb.exists()) {
                scorecard.put("Storage_Logs", "WARN (No /U USB; AdvantageKit falls back to internal)");
            } else if (usbFree >= 0 && usbFree < lowBytes) {
                scorecard.put("Storage_Logs",
                        "WARN (/U low: " + (usbFree / 1024 / 1024) + " MB free)");
            } else if (internal.exists() && internalFree < lowBytes) {
                scorecard.put("Storage_Logs",
                        "WARN (internal logs low: " + (internalFree / 1024 / 1024) + " MB free)");
            } else {
                scorecard.put("Storage_Logs", "PASS");
            }
        } catch (Throwable t) {
            scorecard.put("Storage_Logs", "WARN (storage check unavailable)");
        }

        // 3. JVM heap headroom (OOM precursor, not a guarantee).
        try {
            Runtime rt = Runtime.getRuntime();
            long headroomMb = (rt.maxMemory() - (rt.totalMemory() - rt.freeMemory())) / 1024 / 1024;
            if (headroomMb < 50) {
                scorecard.put("Memory_Heap", "WARN (heap headroom " + headroomMb + " MB)");
            } else {
                scorecard.put("Memory_Heap", "PASS (" + headroomMb + " MB headroom)");
            }
        } catch (Throwable t) {
            scorecard.put("Memory_Heap", "WARN (heap check unavailable)");
        }
        scorecardDirty = true;
    }

    static boolean isVisionInputFresh(frc.robot.Subsystems.vision.VisionIO.VisionIOInputs inputs, double now) {
        if (inputs == null || inputs.timestamp <= 0) {
            return false;
        }
        double ageSec = now - inputs.timestamp;
        return ageSec >= 0 && ageSec < VISION_FRESHNESS_SEC
                && inputs.latencyMs >= 0 && inputs.latencyMs < VISION_MAX_LATENCY_MS;
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
        SmartDashboard.putString(TelemetryKeys.Diagnostics.PREFLIGHT_STEP, preFlightStep.displayName);
        double totalElapsed = preFlightRunning ? (Timer.getFPGATimestamp() - preFlightStartTime) : 0.0;
        SmartDashboard.putNumber(TelemetryKeys.Diagnostics.PREFLIGHT_PROGRESS, Math.min(1.0, totalElapsed / 15.0));

        // Scorecard publication for Elastic Dashboard (only publish when changed or during pre-flight)
        if (preFlightRunning || scorecardDirty) {
            for (Map.Entry<String, String> entry : scorecard.entrySet()) {
                SmartDashboard.putString("Diagnostics/Scorecard/" + entry.getKey(), entry.getValue());
            }
            scorecardDirty = false;
        }

        // Trigger automated pre-flight check from dashboard
        if (SmartDashboard.getBoolean(TelemetryKeys.Diagnostics.RUN_PREFLIGHT, false)) {
            SmartDashboard.putBoolean(TelemetryKeys.Diagnostics.RUN_PREFLIGHT, false);
            startPreFlightCheck();
        }

        // Single component test triggers (only check if in test mode or diagnostics running)
        if (edu.wpi.first.wpilibj.DriverStation.isTest() || isRunning) {
            for (Map.Entry<String, String> entry : testKeys.entrySet()) {
                if (SmartDashboard.getBoolean(entry.getValue(), false)) {
                    SmartDashboard.putBoolean(entry.getValue(), false);
                    startTest(entry.getKey());
                }
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
        if (!RobotBase.isSimulation() && !DriverStation.isDisabled()) {
            System.out.println("[Diagnostics] Pre-Flight REFUSED - robot must be disabled.");
            preFlightAlert.setText("Pre-Flight REFUSED - robot must be disabled");
            preFlightAlert.set(true);
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
        if (!preFlightRunning) return;
        preFlightRunning = false;
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
        if (!RobotBase.isSimulation() && !DriverStation.isDisabled()) {
            System.out.println("[Diagnostics] Manual test REFUSED - robot must be disabled: " + name);
            return;
        }
        registerTests();
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
        System.out.println("[Diagnostics] -> " + name);
    }

    public Iterable<String> getTestNames() {
        registerTests();
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
