package frc.robot.Hardware;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Telemetry.Alert;
import frc.robot.Telemetry.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * PowerBudgetManager: Dynamic energy and current budgeting manager.
 *
 * <p>Inspired by Team 6328's "Finance Department", this manager protects the robot
 * against two critical failure modes during high-load match conditions:
 * <ul>
 *   <li><b>Battery Brownouts (V_batt &lt; 7.5V):</b> When heavy concurrent accelerations
 *       or stall currents depress system voltage, the manager applies an asymptotic
 *       throttle to drive translation, keeping logic circuits and CAN buses alive.</li>
 *   <li><b>Main Breaker Thermal Trips (I^2*t Overload):</b> Tracks cumulative thermal
 *       damage via {@link BreakerModel} and aggressively limits drive current before
 *       the bimetallic strip reaches trip temperature.</li>
 * </ul>
 */
public class PowerBudgetManager {

    private static PowerBudgetManager instance;

    /** Safe brownout cutoff voltage threshold below which motor output is strictly capped. */
    public static final double MIN_BROWNOUT_VOLTAGE = 7.50;

    /** Battery voltage where voltage-based drive derating begins. */
    public static final double VOLTAGE_DERATING_START = 9.20;

    /** Minimum scaling applied to drive translation under extreme brownout (preserves steer authority). */
    public static final double MIN_DRIVE_SCALE = 0.35;

    /** Breaker damage fraction where warning alert is published. */
    public static final double BREAKER_DAMAGE_WARNING_THRESHOLD = 0.50;

    /** Breaker damage fraction where thermal throttling activates. */
    public static final double BREAKER_DAMAGE_THROTTLE_THRESHOLD = 0.75;

    /** Normal maximum peak bus current budget, in amperes. */
    public static final double MAX_CURRENT_BUDGET_AMPS = 200.0;

    private final BreakerModel breakerModel = new BreakerModel();

    private final Alert breakerThermalAlert =
            new Alert("Electrical", "Main breaker thermal load elevated (>50%)", AlertType.WARNING);
    private final Alert brownoutRiskAlert =
            new Alert("Electrical", "Battery voltage critical: brownout throttle active", AlertType.WARNING);

    private double lastTimestampSeconds = -1.0;
    private double currentBatteryVoltage = 12.5;
    private double currentBusAmps = 0.0;
    private double currentDriveScale = 1.0;

    public static synchronized PowerBudgetManager getInstance() {
        if (instance == null) {
            instance = new PowerBudgetManager();
        }
        return instance;
    }

    public PowerBudgetManager() {
        reset();
    }

    /**
     * Resets internal timers, breaker damage, and alert states.
     */
    public void reset() {
        breakerModel.reset();
        lastTimestampSeconds = -1.0;
        currentBatteryVoltage = 12.5;
        currentBusAmps = 0.0;
        currentDriveScale = 1.0;
        breakerThermalAlert.set(false);
        brownoutRiskAlert.set(false);
    }

    /**
     * Updates power model with explicit parameters (suitable for simulation and unit testing).
     *
     * @param totalCurrentAmps Instantaneous bus current draw
     * @param batteryVoltage   Measured battery voltage
     * @param dtSeconds        Elapsed cycle time
     * @return Resulting drive speed scale in range [MIN_DRIVE_SCALE, 1.0]
     */
    public double update(double totalCurrentAmps, double batteryVoltage, double dtSeconds) {
        this.currentBusAmps = Math.max(0.0, totalCurrentAmps);
        this.currentBatteryVoltage = Math.max(0.0, batteryVoltage);

        double damage = breakerModel.update(this.currentBusAmps, dtSeconds);
        this.currentDriveScale = calculateDriveScale(this.currentBatteryVoltage, damage);

        // Update alerts
        breakerThermalAlert.set(damage >= BREAKER_DAMAGE_WARNING_THRESHOLD);
        brownoutRiskAlert.set(this.currentBatteryVoltage < VOLTAGE_DERATING_START);

        // Log telemetry
        Logger.recordOutput("Power/BreakerDamageFraction", damage);
        Logger.recordOutput("Power/TotalCurrentAmps", this.currentBusAmps);
        Logger.recordOutput("Power/BatteryVoltage", this.currentBatteryVoltage);
        Logger.recordOutput("Power/DriveSpeedScale", this.currentDriveScale);
        Logger.recordOutput("Power/IsBreakerTripped", breakerModel.isTripped());

        return this.currentDriveScale;
    }

    /**
     * Periodic update hook for real hardware and default simulation execution.
     */
    public void periodic() {
        double now = Timer.getTimestamp();
        double dt = (lastTimestampSeconds > 0.0) ? (now - lastTimestampSeconds) : 0.020;
        lastTimestampSeconds = now;

        double voltage = RobotController.getBatteryVoltage();
        // Estimated or measured current (summing subsystem loads if available, or heuristic baseline)
        double amps = estimateTotalCurrentDraw(voltage);

        update(amps, voltage, dt);
    }

    /**
     * Pure calculation evaluating available drive speed scale from voltage and breaker state.
     *
     * @param batteryVoltage Measured or simulated battery voltage
     * @param breakerDamage  Cumulative breaker damage fraction [0.0, 1.0]
     * @return Combined speed scale factor in range [MIN_DRIVE_SCALE, 1.0]
     */
    public static double calculateDriveScale(double batteryVoltage, double breakerDamage) {
        // 1. Voltage-based scale
        double voltScale;
        if (batteryVoltage >= VOLTAGE_DERATING_START) {
            voltScale = 1.0;
        } else if (batteryVoltage <= MIN_BROWNOUT_VOLTAGE) {
            voltScale = MIN_DRIVE_SCALE;
        } else {
            double range = VOLTAGE_DERATING_START - MIN_BROWNOUT_VOLTAGE;
            double progress = (batteryVoltage - MIN_BROWNOUT_VOLTAGE) / range;
            voltScale = MIN_DRIVE_SCALE + (1.0 - MIN_DRIVE_SCALE) * Math.max(0.0, Math.min(1.0, progress));
        }

        // 2. Thermal damage-based scale
        double thermalScale;
        if (breakerDamage <= BREAKER_DAMAGE_THROTTLE_THRESHOLD) {
            thermalScale = 1.0;
        } else if (breakerDamage >= 1.0) {
            thermalScale = MIN_DRIVE_SCALE;
        } else {
            double range = 1.0 - BREAKER_DAMAGE_THROTTLE_THRESHOLD;
            double progress = (1.0 - breakerDamage) / range;
            thermalScale = MIN_DRIVE_SCALE + (1.0 - MIN_DRIVE_SCALE) * Math.max(0.0, Math.min(1.0, progress));
        }

        return Math.min(voltScale, thermalScale);
    }

    private double estimateTotalCurrentDraw(double voltage) {
        // Fallback approximation when PDP/PDH is not queried directly:
        // Quiescent idle draw ~5A, plus voltage sag heuristic (R_internal ~ 0.025 ohms)
        double sag = Math.max(0.0, 12.6 - voltage);
        double estimatedSagCurrent = sag / 0.025; // I = V_sag / R_int
        return Math.min(MAX_CURRENT_BUDGET_AMPS, Math.max(5.0, estimatedSagCurrent));
    }

    public double getDriveScale() {
        return currentDriveScale;
    }

    public double getBreakerDamageFraction() {
        return breakerModel.getDamageFraction();
    }

    public boolean isBreakerTripped() {
        return breakerModel.isTripped();
    }

    public double getBatteryVoltage() {
        return currentBatteryVoltage;
    }

    public double getTotalCurrentAmps() {
        return currentBusAmps;
    }

    public BreakerModel getBreakerModel() {
        return breakerModel;
    }
}
