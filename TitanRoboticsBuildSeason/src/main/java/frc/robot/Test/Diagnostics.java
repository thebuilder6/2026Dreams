package frc.robot.Test;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;

/**
 * Diagnostics subsystem for safe hardware verification.
 *
 * <p>Runs each motor at a capped voltage for a short period and reports the
 * resulting encoder change. A positive delta means the encoder increases with
 * positive voltage — confirming correct wiring and direction before SysID.</p>
 *
 * <p>Usage: toggle a boolean widget in Elastic/ShuffleBoard under
 * "Diagnostics/Run [TestName]". Results appear on the console and under
 * "Diagnostics/Last Delta".</p>
 */
public class Diagnostics implements Subsystem {

    private static Diagnostics instance = null;

    public static Diagnostics getInstance() {
        if (instance == null) {
            instance = new Diagnostics();
        }
        return instance;
    }

    /** A single diagnostic check: one motor in, one encoder/position out. */
    private static class DiagTest {
        final Consumer<Double> motorSetter;
        final Supplier<Double> encoderSupplier;

        DiagTest(Consumer<Double> motorSetter, Supplier<Double> encoderSupplier) {
            this.motorSetter = motorSetter;
            this.encoderSupplier = encoderSupplier;
        }
    }

    // ── Configuration ────────────────────────────────────────────────────────
    /** Voltage applied during each test. Safe for any NEO on the bench. */
    private static final double DIAGNOSTIC_VOLTAGE = 0.5; // Volts
    /** How long each pulse runs before stopping and reporting. */
    private static final double TEST_DURATION = 1.0; // seconds

    // ── Registered tests (name → test), ordered for predictable dashboard order
    private final Map<String, DiagTest> tests = new LinkedHashMap<>();

    // ── Runtime state ─────────────────────────────────────────────────────────
    private String activeTestName  = "None";
    private DiagTest activeTest    = null;
    private double testStartTime   = 0;
    private boolean isRunning      = false;
    private double initialValue    = 0;
    private double lastDelta       = 0;

    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    // ─────────────────────────────────────────────────────────────────────────

    private Diagnostics() {
        SubsystemManager.registerSubsystem(this);
    }

    /**
     * Registers all tests once references are available.
     * Called lazily on first update so subsystems are guaranteed to be initialised.
     */
    private boolean testsRegistered = false;

    private void registerTests() {
        if (testsRegistered) return;
        testsRegistered = true;

        Shooter shooter = Shooter.getInstance();
        Intake intake   = Intake.getInstance();
        SwerveBase swerve = SwerveBase.getInstance();

        tests.put("Shooter/Flywheel Left",
            new DiagTest(
                v -> shooter.setFlywheelVoltages(v, 0),
                () -> shooter.getFlywheelLeftVelocityRPM()));

        // Right flywheel: CAN 11
        tests.put("Shooter/Flywheel Right",
            new DiagTest(
                v -> shooter.setFlywheelVoltages(0, v),
                () -> shooter.getFlywheelRightVelocityRPM()));

        // Kicker: CAN 13  —  positive should push ball toward flywheels
        tests.put("Shooter/Kicker",
            new DiagTest(
                v -> shooter.setVoltages(0, v),
                () -> shooter.getKickerVelocityRPM()));

        // ── Intake ───────────────────────────────────────────────────────────
        // Arm: CAN 10  —  positive voltage should move arm DOWN (toward intake position)
        tests.put("Intake/Arm",
            new DiagTest(
                v -> intake.setArmVoltage(v),
                () -> intake.getArmPosition()));

        // Roller: CAN 14  —  positive voltage should pull game piece IN
        tests.put("Intake/Roller",
            new DiagTest(
                v -> intake.setRollerVoltage(v),
                () -> intake.getRollerVelocityRPM()));

        // Hopper: CAN 9  —  positive voltage should feed game piece toward shooter
        tests.put("Intake/Hopper",
            new DiagTest(
                v -> intake.setHopperVoltage(v),
                () -> intake.getHopperVelocityRPM()));

        // ── Swerve (per module) ───────────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            final int idx = i;
            // Drive: positive voltage should move robot FORWARD for FL/FR, BACKWARD for BL/BR
            tests.put("Swerve/" + MODULE_NAMES[i] + " Drive",
                new DiagTest(
                    v -> swerve.setModuleDriveVoltage(idx, v),
                    () -> swerve.getModuleDriveVelocity(idx)));

            // Angle: positive voltage should rotate the module CCW
            tests.put("Swerve/" + MODULE_NAMES[i] + " Angle",
                new DiagTest(
                    v -> swerve.setModuleAngleVoltage(idx, v),
                    () -> swerve.getModuleAnglePosition(idx)));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystem API
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void update() {
        registerTests(); // no-op after first call

        if (!isRunning) return;

        double elapsed = Timer.getFPGATimestamp() - testStartTime;

        if (elapsed < TEST_DURATION) {
            activeTest.motorSetter.accept(DIAGNOSTIC_VOLTAGE);
            lastDelta = activeTest.encoderSupplier.get() - initialValue;
        } else {
            activeTest.motorSetter.accept(0.0); // stop motor
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

        SmartDashboard.putBoolean("Diagnostics/Running", isRunning);
        SmartDashboard.putString("Diagnostics/Active Test", activeTestName);
        SmartDashboard.putNumber("Diagnostics/Last Delta", lastDelta);

        // One boolean trigger per test, grouped by path (Elastic collapses by prefix)
        for (String name : tests.keySet()) {
            String key = "Diagnostics/Run " + name;
            if (SmartDashboard.getBoolean(key, false)) {
                SmartDashboard.putBoolean(key, false); // auto-reset
                startTest(name);
            }
        }
    }

    @Override
    public void initialize() {
        isRunning = false;
        activeTest = null;
        activeTestName = "None";
    }

    @Override
    public String getName() { return "Diagnostics"; }

    @Override
    public boolean isEnabled() { return true; }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — also callable from code/Teleop if needed
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Starts a named test by key (e.g. "Shooter/Flywheel Left").
     * No-op if a test is already running.
     */
    public void startTest(String name) {
        if (isRunning) return;
        DiagTest test = tests.get(name);
        if (test == null) {
            System.out.println("[Diagnostics] Unknown test: " + name);
            return;
        }
        activeTestName  = name;
        activeTest      = test;
        testStartTime   = Timer.getFPGATimestamp();
        initialValue    = test.encoderSupplier.get();
        isRunning       = true;
        System.out.println("[Diagnostics] → " + name);
    }

    /**
     * Runs all registered tests sequentially (use for automated bringup scripts).
     * Not safe for use on a live robot — intended for simulation only.
     */
    public Iterable<String> getTestNames() {
        return tests.keySet();
    }

    /**
     * Setup dashboard controls for diagnostics
     */
    public void setupDashboard() {
        SmartDashboard.putBoolean("Diagnostics/Running", false);
        SmartDashboard.putString("Diagnostics/Active Test", "None");
        SmartDashboard.putNumber("Diagnostics/Last Delta", 0.0);
        
        // Initialize test triggers to false
        registerTests();
        for (String name : tests.keySet()) {
            String key = "Diagnostics/Run " + name;
            SmartDashboard.putBoolean(key, false);
        }
    }
}
