package frc.robot.Test;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Map;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.Test.Diagnostics.PreFlightStep;

public class DiagnosticsTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testPreFlightStepsEnum() {
        assertEquals("Idle", PreFlightStep.IDLE.displayName);
        assertEquals("1. CAN & Power Audit", PreFlightStep.CAN_BUS_AUDIT.displayName);
        assertEquals("2. Swerve Drive Pulse", PreFlightStep.SWERVE_PULSE.displayName);
        assertEquals("3. Steer Alignment Check", PreFlightStep.STEER_ALIGNMENT.displayName);
        assertEquals("4. Intake Profile Check", PreFlightStep.INTAKE_CHECK.displayName);
        assertEquals("5. Shooter Ramping", PreFlightStep.SHOOTER_RAMP.displayName);
        assertEquals("6. Vision Link Check", PreFlightStep.VISION_LINK.displayName);
        assertEquals("Completed", PreFlightStep.COMPLETE.displayName);
    }

    @Test
    public void testScorecardStructure() {
        Diagnostics diag = Diagnostics.getInstance();
        Map<String, String> scorecard = diag.getScorecard();

        assertNotNull(scorecard);
        assertTrue(scorecard.containsKey("CAN_Bus"));
        assertTrue(scorecard.containsKey("Swerve_Drive"));
        assertTrue(scorecard.containsKey("Steer_Alignment"));
        assertTrue(scorecard.containsKey("Intake"));
        assertTrue(scorecard.containsKey("Shooter"));
        assertTrue(scorecard.containsKey("Vision"));
        assertTrue(scorecard.containsKey("Overall"));
    }

    @Test
    public void testStartAndCancelPreFlightCheck() {
        Diagnostics diag = Diagnostics.getInstance();

        diag.startPreFlightCheck();
        assertTrue(diag.isPreFlightRunning());
        assertEquals(PreFlightStep.CAN_BUS_AUDIT, diag.getPreFlightStep());

        diag.cancelPreFlightCheck();
        assertFalse(diag.isPreFlightRunning());
        assertEquals(PreFlightStep.IDLE, diag.getPreFlightStep());
    }

    @Test
    public void testManualTestRegistrationAndExecution() {
        Diagnostics diag = Diagnostics.getInstance();
        diag.registerTests();

        var testNames = diag.getTestNames();
        assertNotNull(testNames);
        assertTrue(testNames.iterator().hasNext());

        assertDoesNotThrow(() -> diag.startTest("Shooter/Flywheel Left"));
        assertDoesNotThrow(() -> diag.initialize());
    }
}
