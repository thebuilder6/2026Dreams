package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import frc.robot.Intelligence.utility.Consideration;
import frc.robot.Intelligence.utility.ResponseCurve;
import frc.robot.Intelligence.utility.UtilityAction;

public class ConsiderationTest {

    @Test
    public void testNormalizationAndEvaluation() {
        // Linear curve mapping raw input [10, 30] to [0.0, 1.0]
        Consideration c = new Consideration("FuelCount", 10.0, 30.0, ResponseCurve.linear(1.0, 0.0, 0.0));
        assertEquals(0.0, c.evaluate(10.0), 1e-6);
        assertEquals(0.5, c.evaluate(20.0), 1e-6);
        assertEquals(1.0, c.evaluate(30.0), 1e-6);

        // Clamping outside bounds
        assertEquals(0.0, c.evaluate(5.0), 1e-6);
        assertEquals(1.0, c.evaluate(35.0), 1e-6);
    }

    @Test
    public void testVetoCondition() {
        Consideration c = new Consideration("Distance", 0.0, 10.0, ResponseCurve.reverseRamp(0.0, 1.0));
        assertFalse(c.isVeto(5.0));
        assertTrue(c.isVeto(10.0));
        assertTrue(c.isVeto(12.0));
    }

    @Test
    public void testUtilityActionMultiplicativeVeto() {
        Consideration fuel = new Consideration("Fuel", 0.0, 20.0, ResponseCurve.linear(1.0, 0.0, 0.0));
        Consideration dist = new Consideration("Dist", 1.0, 5.0, ResponseCurve.reverseRamp(0.0, 1.0));

        UtilityAction action = new UtilityAction("ScoreHub", 0.95, java.util.List.of(fuel, dist));

        // When fuel is 0.0 -> veto collapses action to 0.0
        assertEquals(0.0, action.evaluate(0.0, 2.0), 1e-6);

        // When both are non-zero -> multiplicative compromise
        double score = action.evaluate(10.0, 3.0); // fuel norm 0.5, dist norm 0.5 -> score = 0.95 * 0.5 * 0.5 = 0.2375
        assertEquals(0.2375, score, 1e-4);

        // When both are optimal -> full base weight
        assertEquals(0.95, action.evaluate(20.0, 1.0), 1e-4);
    }
}
