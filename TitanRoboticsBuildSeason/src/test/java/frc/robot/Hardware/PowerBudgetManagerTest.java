package frc.robot.Hardware;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PowerBudgetManagerTest {

    private PowerBudgetManager manager;

    @BeforeEach
    void setup() {
        manager = PowerBudgetManager.getInstance();
        manager.reset();
    }

    @Test
    void testNominalConditionsPreservesFullDriveScale() {
        double scale = PowerBudgetManager.calculateDriveScale(12.5, 0.0);
        assertEquals(1.0, scale, 1e-6);

        double liveScale = manager.update(40.0, 12.4, 0.020);
        assertEquals(1.0, liveScale, 1e-6);
        assertFalse(manager.isBreakerTripped());
    }

    @Test
    void testVoltageDeratingUnderBrownout() {
        // At or above 9.20V -> full scale (1.0)
        assertEquals(1.0, PowerBudgetManager.calculateDriveScale(9.5, 0.0), 1e-6);
        assertEquals(1.0, PowerBudgetManager.calculateDriveScale(9.2, 0.0), 1e-6);

        // At or below 7.50V -> minimum scale (0.35)
        assertEquals(0.35, PowerBudgetManager.calculateDriveScale(7.5, 0.0), 1e-6);
        assertEquals(0.35, PowerBudgetManager.calculateDriveScale(6.8, 0.0), 1e-6);

        // Midpoint: 8.35V (halfway between 7.50 and 9.20)
        double expectedMid = 0.35 + (1.0 - 0.35) * 0.50;
        assertEquals(expectedMid, PowerBudgetManager.calculateDriveScale(8.35, 0.0), 0.01);
    }

    @Test
    void testThermalDeratingUnderHighBreakerDamage() {
        // At or below 0.75 damage -> full scale
        assertEquals(1.0, PowerBudgetManager.calculateDriveScale(12.5, 0.50), 1e-6);
        assertEquals(1.0, PowerBudgetManager.calculateDriveScale(12.5, 0.75), 1e-6);

        // At 1.0 damage -> minimum scale
        assertEquals(0.35, PowerBudgetManager.calculateDriveScale(12.5, 1.0), 1e-6);

        // Midpoint: 0.875 damage (halfway between 0.75 and 1.0)
        double expectedMid = 0.35 + (1.0 - 0.35) * 0.50;
        assertEquals(expectedMid, PowerBudgetManager.calculateDriveScale(12.5, 0.875), 0.01);
    }

    @Test
    void testWorstOfVoltageAndThermalWins() {
        // Voltage demands 0.80, Thermal demands 0.50 -> result is 0.50
        double voltOnly = PowerBudgetManager.calculateDriveScale(8.86, 0.0);
        double thermalOnly = PowerBudgetManager.calculateDriveScale(12.5, 0.90);

        double combined = PowerBudgetManager.calculateDriveScale(8.86, 0.90);
        assertEquals(Math.min(voltOnly, thermalOnly), combined, 1e-6);
    }

    @Test
    void testManagerAccumulatesDamageAndDerates() {
        // Draw 360A (trip time 15s) for 12s -> damage should reach 12/15 = 0.80 (> 0.75 throttle threshold)
        double scale = manager.update(360.0, 11.5, 12.0);

        assertTrue(manager.getBreakerDamageFraction() >= 0.75);
        assertTrue(scale < 1.0);
        assertTrue(scale >= PowerBudgetManager.MIN_DRIVE_SCALE);
    }

    @Test
    void testResetRestoresInitialHealthyState() {
        manager.update(360.0, 7.0, 15.0);
        assertTrue(manager.isBreakerTripped());
        assertEquals(PowerBudgetManager.MIN_DRIVE_SCALE, manager.getDriveScale(), 1e-6);

        manager.reset();
        assertFalse(manager.isBreakerTripped());
        assertEquals(0.0, manager.getBreakerDamageFraction(), 1e-6);
        assertEquals(1.0, manager.getDriveScale(), 1e-6);
    }
}
