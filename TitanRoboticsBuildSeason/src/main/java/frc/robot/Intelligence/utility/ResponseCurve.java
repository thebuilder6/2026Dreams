package frc.robot.Intelligence.utility;

/**
 * Dave Mark's Infinite Axis Utility System (IAUS) parameterized continuous response curve.
 *
 * <p>Supports Linear, Polynomial, Logistic (Sigmoid), Normal (Gaussian Bell), and Ramp curves.
 * Transforms a normalized input in [0.0, 1.0] to a utility score in [0.0, 1.0].
 * All operations are purely mathematical, thread-safe, and allocation-free.
 */
public final class ResponseCurve {

    public enum CurveType {
        LINEAR,
        POLYNOMIAL,
        LOGISTIC,
        NORMAL,
        RAMP
    }

    private final CurveType type;
    private final double slope;      // m
    private final double exponent;   // k
    private final double midpoint;   // c
    private final double base;       // b
    private final double sigma;      // for normal distribution

    private ResponseCurve(CurveType type, double slope, double exponent, double midpoint, double base, double sigma) {
        this.type = type;
        this.slope = slope;
        this.exponent = exponent;
        this.midpoint = midpoint;
        this.base = base;
        this.sigma = sigma;
    }

    /**
     * Creates a Linear response curve: y = slope * (x - midpoint) + base.
     */
    public static ResponseCurve linear(double slope, double midpoint, double base) {
        return new ResponseCurve(CurveType.LINEAR, slope, 1.0, midpoint, base, 0.0);
    }

    /**
     * Creates a standard linear ramp from 0.0 to 1.0 over [minX, maxX].
     */
    public static ResponseCurve ramp(double minX, double maxX) {
        double slope = (maxX > minX) ? 1.0 / (maxX - minX) : 1.0;
        return new ResponseCurve(CurveType.RAMP, slope, 1.0, minX, 0.0, 0.0);
    }

    /**
     * Creates a reverse linear ramp from 1.0 to 0.0 over [minX, maxX].
     */
    public static ResponseCurve reverseRamp(double minX, double maxX) {
        double slope = (maxX > minX) ? -1.0 / (maxX - minX) : -1.0;
        return new ResponseCurve(CurveType.RAMP, slope, 1.0, minX, 1.0, 0.0);
    }

    /**
     * Creates a Polynomial response curve: y = slope * (x - midpoint)^exponent + base.
     */
    public static ResponseCurve polynomial(double slope, double exponent, double midpoint, double base) {
        return new ResponseCurve(CurveType.POLYNOMIAL, slope, exponent, midpoint, base, 0.0);
    }

    /**
     * Creates a Logistic (Sigmoid) response curve:
     * y = 1.0 / (1.0 + exp(-steepness * (x - midpoint)))
     */
    public static ResponseCurve logistic(double steepness, double midpoint) {
        return new ResponseCurve(CurveType.LOGISTIC, steepness, 1.0, midpoint, 0.0, 0.0);
    }

    /**
     * Creates a Gaussian Bell curve centered at {@code midpoint} with standard deviation {@code sigma}:
     * y = exp(-((x - midpoint)^2) / (2 * sigma^2))
     */
    public static ResponseCurve bell(double midpoint, double sigma) {
        return new ResponseCurve(CurveType.NORMAL, 1.0, 1.0, midpoint, 0.0, sigma);
    }

    /**
     * Evaluates the response curve for input x.
     * Both input and output are clamped to [0.0, 1.0].
     *
     * @param x input value in [0.0, 1.0]
     * @return utility value in [0.0, 1.0]
     */
    public double calculate(double x) {
        if (Double.isNaN(x)) {
            return 0.0;
        }
        double clampedX = Math.max(0.0, Math.min(1.0, x));
        double result;

        switch (type) {
            case POLYNOMIAL:
                double delta = clampedX - midpoint;
                if (delta >= 0.0) {
                    result = slope * Math.pow(delta, exponent) + base;
                } else {
                    result = slope * -Math.pow(-delta, exponent) + base;
                }
                break;

            case LOGISTIC:
                result = 1.0 / (1.0 + Math.exp(-slope * (clampedX - midpoint)));
                break;

            case NORMAL:
                double diff = clampedX - midpoint;
                double s = (sigma > 1e-6) ? sigma : 0.2;
                result = Math.exp(-(diff * diff) / (2.0 * s * s));
                break;

            case RAMP:
            case LINEAR:
            default:
                result = slope * (clampedX - midpoint) + base;
                break;
        }

        if (Double.isNaN(result)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, result));
    }

    public CurveType type() { return type; }
    public double slope() { return slope; }
    public double exponent() { return exponent; }
    public double midpoint() { return midpoint; }
    public double base() { return base; }
    public double sigma() { return sigma; }
}
