package frc.robot.Intelligence.utility;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Dave Mark's IAUS Multiplicative Action Evaluator.
 *
 * <p>Calculates action utility by multiplying consideration scores:
 * <pre>
 *   ActionUtility = Weight * C_1 * C_2 * ... * C_n
 * </pre>
 *
 * <p>Key properties:
 * <ul>
 *   <li><b>Natural Veto</b>: If any consideration drops to 0.0 (e.g. out of range, 0 fuel),
 *       the entire product collapses to 0.0 without needing rigid boolean if-statements.</li>
 *   <li><b>Graceful Compromise</b>: If one factor is sub-optimal (e.g. 7 balls instead of 8),
 *       utility degrades smoothly rather than dropping off a cliff.</li>
 * </ul>
 */
public final class UtilityAction {

    public record BoundConsideration(Consideration consideration, double rawInput) {
        public double evaluate() {
            return consideration.evaluate(rawInput);
        }
    }

    private final String name;
    private final double baseWeight;
    private final List<Consideration> considerations;

    public UtilityAction(String name, double baseWeight, List<Consideration> considerations) {
        this.name = name;
        this.baseWeight = baseWeight;
        this.considerations = Collections.unmodifiableList(new ArrayList<>(considerations));
    }

    /**
     * Evaluates this action given raw inputs corresponding to each consideration in order.
     *
     * @param rawInputs raw input values matching the order of considerations
     * @return computed utility in [0.0, 1.0]
     */
    public double evaluate(double... rawInputs) {
        if (baseWeight <= 1e-6 || considerations.isEmpty()) {
            return 0.0;
        }

        double product = baseWeight;
        int count = Math.min(considerations.size(), rawInputs.length);

        for (int i = 0; i < count; i++) {
            double score = considerations.get(i).evaluate(rawInputs[i]);
            if (score <= 1e-6) {
                return 0.0; // Natural Veto
            }
            product *= score;
        }

        return Math.max(0.0, Math.min(1.0, product));
    }

    /**
     * Evaluates action utility directly from a list of pre-evaluated consideration scores.
     *
     * @param scores scores in [0.0, 1.0]
     * @return final multiplicative utility in [0.0, 1.0]
     */
    public static double evaluateProduct(double baseWeight, double... scores) {
        if (baseWeight <= 1e-6) {
            return 0.0;
        }
        double product = baseWeight;
        for (double s : scores) {
            if (Double.isNaN(s) || s <= 1e-6) {
                return 0.0; // Natural Veto
            }
            product *= Math.max(0.0, Math.min(1.0, s));
        }
        return Math.max(0.0, Math.min(1.0, product));
    }

    public String name() {
        return name;
    }

    public double baseWeight() {
        return baseWeight;
    }

    public List<Consideration> considerations() {
        return considerations;
    }
}
