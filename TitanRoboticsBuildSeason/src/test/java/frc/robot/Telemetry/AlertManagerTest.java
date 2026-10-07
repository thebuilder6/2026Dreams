package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.Telemetry.Alert;
import frc.robot.Telemetry.AlertManager;
import frc.robot.Telemetry.Alert.AlertType;

public class AlertManagerTest {

    @BeforeEach
    public void setup() {
        AlertManager.resetAll();
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

    @Test
    public void testAlertContentChangeReactivity() {
        Alert err1 = new Alert("SubsystemA", "Error 1", AlertType.ERROR);
        Alert err2 = new Alert("SubsystemB", "Error 2", AlertType.ERROR);

        err1.set(true);
        err2.set(true);
        AlertManager.update();

        String[] publishedErrors = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getStringArray("Alerts/Errors", new String[0]);
        assertEquals(2, publishedErrors.length);
        assertEquals("[SubsystemA] Error 1", publishedErrors[0]);
        assertEquals("[SubsystemB] Error 2", publishedErrors[1]);

        // Change text of err2 while keeping the count identical (2 errors)
        err2.setText("Error 2 Details Changed");
        AlertManager.update();

        String[] updatedErrors = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getStringArray("Alerts/Errors", new String[0]);
        assertEquals(2, updatedErrors.length);
        assertEquals("[SubsystemA] Error 1", updatedErrors[0]);
        assertEquals("[SubsystemB] Error 2 Details Changed", updatedErrors[1]);

        err1.set(false);
        err2.set(false);
    }
}
