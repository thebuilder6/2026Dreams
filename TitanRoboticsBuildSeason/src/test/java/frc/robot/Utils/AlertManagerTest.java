package frc.robot.Utils;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.Utils.Alert.AlertType;

public class AlertManagerTest {

    @BeforeEach
    public void setup() {
        // Reset or prepare tests
    }

    @Test
    public void testAlertActivationAndSeverity() {
        Alert infoAlert = new Alert("SubsystemA", "System Initialized", AlertType.INFO);
        Alert warnAlert = new Alert("Vision", "Vision Degraded", AlertType.WARNING);
        Alert errorAlert = new Alert("Intake", "Encoder Disconnected", AlertType.ERROR);

        assertFalse(infoAlert.isActive());
        assertFalse(warnAlert.isActive());
        assertFalse(errorAlert.isActive());

        infoAlert.set(true);
        assertTrue(infoAlert.isActive());
        assertEquals(AlertType.INFO, AlertManager.getHighestSeverity());
        assertFalse(AlertManager.hasActiveErrors());
        assertFalse(AlertManager.hasActiveWarnings());

        warnAlert.set(true);
        assertTrue(warnAlert.isActive());
        assertEquals(AlertType.WARNING, AlertManager.getHighestSeverity());
        assertFalse(AlertManager.hasActiveErrors());
        assertTrue(AlertManager.hasActiveWarnings());

        errorAlert.set(true);
        assertTrue(errorAlert.isActive());
        assertEquals(AlertType.ERROR, AlertManager.getHighestSeverity());
        assertTrue(AlertManager.hasActiveErrors());

        // Clearing error drops highest severity back to warning
        errorAlert.set(false);
        assertFalse(errorAlert.isActive());
        assertEquals(AlertType.WARNING, AlertManager.getHighestSeverity());

        // Clearing warning drops highest severity back to info
        warnAlert.set(false);
        assertEquals(AlertType.INFO, AlertManager.getHighestSeverity());

        infoAlert.set(false);
        assertNull(AlertManager.getHighestSeverity());
    }

    @Test
    public void testAlertManagerUpdateDoesNotThrow() {
        Alert testAlert = new Alert("Drive", "Test Alert", AlertType.WARNING);
        testAlert.set(true);
        assertDoesNotThrow(() -> AlertManager.update());
        testAlert.set(false);
    }
}
