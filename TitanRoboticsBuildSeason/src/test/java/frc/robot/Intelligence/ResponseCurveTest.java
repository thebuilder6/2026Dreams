package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import frc.robot.Intelligence.utility.ResponseCurve;

public class ResponseCurveTest {

    @Test
    public void testLinearCurve() {
        ResponseCurve linear = ResponseCurve.linear(1.0, 0.0, 0.0);
        assertEquals(0.0, linear.calculate(0.0), 1e-6);
        assertEquals(0.5, linear.calculate(0.5), 1e-6);
        assertEquals(1.0, linear.calculate(1.0), 1e-6);

        // Clamping check
        assertEquals(0.0, linear.calculate(-0.5), 1e-6);
        assertEquals(1.0, linear.calculate(1.5), 1e-6);
    }

    @Test
    public void testRampAndReverseRamp() {
        ResponseCurve ramp = ResponseCurve.ramp(0.2, 0.8);
        assertEquals(0.0, ramp.calculate(0.1), 1e-6);
        assertEquals(0.0, ramp.calculate(0.2), 1e-6);
        assertEquals(0.5, ramp.calculate(0.5), 1e-6);
        assertEquals(1.0, ramp.calculate(0.8), 1e-6);
        assertEquals(1.0, ramp.calculate(0.9), 1e-6);

        ResponseCurve rev = ResponseCurve.reverseRamp(0.2, 0.8);
        assertEquals(1.0, rev.calculate(0.1), 1e-6);
        assertEquals(1.0, rev.calculate(0.2), 1e-6);
        assertEquals(0.5, rev.calculate(0.5), 1e-6);
        assertEquals(0.0, rev.calculate(0.8), 1e-6);
        assertEquals(0.0, rev.calculate(0.9), 1e-6);
    }

    @Test
    public void testPolynomialCurve() {
        ResponseCurve poly = ResponseCurve.polynomial(1.0, 2.0, 0.0, 0.0);
        assertEquals(0.0, poly.calculate(0.0), 1e-6);
        assertEquals(0.25, poly.calculate(0.5), 1e-6);
        assertEquals(1.0, poly.calculate(1.0), 1e-6);
    }

    @Test
    public void testLogisticCurve() {
        ResponseCurve sigmoid = ResponseCurve.logistic(10.0, 0.5);
        // At midpoint x = 0.5, logistic value should be 0.5
        assertEquals(0.5, sigmoid.calculate(0.5), 1e-4);
        assertTrue(sigmoid.calculate(0.1) < 0.1);
        assertTrue(sigmoid.calculate(0.9) > 0.9);
    }

    @Test
    public void testBellCurve() {
        ResponseCurve bell = ResponseCurve.bell(0.5, 0.15);
        // Peak at midpoint
        assertEquals(1.0, bell.calculate(0.5), 1e-6);
        // Symmetrical falloff
        assertEquals(bell.calculate(0.35), bell.calculate(0.65), 1e-6);
        assertTrue(bell.calculate(0.0) < 0.01);
        assertTrue(bell.calculate(1.0) < 0.01);
    }

    @Test
    public void testNaNHandling() {
        ResponseCurve linear = ResponseCurve.linear(1.0, 0.0, 0.0);
        assertEquals(0.0, linear.calculate(Double.NaN), 1e-6);
    }
}
