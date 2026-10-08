package frc.robot.Intelligence.utility;

/**
 * Maps raw sensor / game metrics into [0.0, 1.0] domain and evaluates a {@link ResponseCurve}.
 */
public final class Consideration {
    private final String name;
    private final double minInput;
    private final double maxInput;
    private final ResponseCurve curve;

    public Consideration(String name, double minInput, double maxInput, ResponseCurve curve) {
        this.name = name;
        this.minInput = minInput;
        this.maxInput = maxInput;
        this.curve = curve;
    }

    public double evaluate(double rawInput) {
        if (Double.isNaN(rawInput)) {
            return 0.0;
        }
        double range = maxInput - minInput;
        double normalized = (Math.abs(range) > 1e-9)
                ? (rawInput - minInput) / range
                : (rawInput >= maxInput ? 1.0 : 0.0);
        return curve.calculate(normalized);
    }

    public boolean isVeto(double rawInput) {
        return evaluate(rawInput) <= 1e-6;
    }

    public String name() {
        return name;
    }

    public double minInput() {
        return minInput;
    }

    public double maxInput() {
        return maxInput;
    }

    public ResponseCurve curve() {
        return curve;
    }
}
