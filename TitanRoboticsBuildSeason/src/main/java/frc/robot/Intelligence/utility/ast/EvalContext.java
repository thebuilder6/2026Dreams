package frc.robot.Intelligence.utility.ast;

import java.util.Locale;

/**
 * Immutable, tier-aware snapshot of the terminals an {@link ExpressionNode}
 * reads. Values are normalized to {@code [0,1]} at construction; reads are
 * clamped and NaN-safe.
 *
 * <p>Tier honesty is enforced here, not in the genome: when {@link #observed()}
 * is true, every {@link Terminal.Tier#CLAIRVOYANT} terminal reads {@code 0.0},
 * which is the honest value a real robot would measure. A genome therefore
 * cannot tell whether it is running clairvoyant or observed except through the
 * values it actually gets.
 */
public final class EvalContext {

    private final double[] values;
    private final boolean observed;

    private EvalContext(double[] values, boolean observed) {
        this.values = values;
        this.observed = observed;
    }

    /** Value of {@code terminal} in {@code [0,1]}; {@code 0} when it is unavailable. */
    public double value(Terminal terminal) {
        if (observed && terminal.tier() == Terminal.Tier.CLAIRVOYANT) {
            return 0.0;
        }
        double v = values[terminal.ordinal()];
        if (Double.isNaN(v)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, v));
    }

    /** True when this context carries only on-board + FMS knowledge. */
    public boolean observed() {
        return observed;
    }

    public static Builder builder() {
        return new Builder();
    }

    /** Mutable builder; not thread-safe, expected to be used per tick. */
    public static final class Builder {
        private final double[] values = new double[Terminal.values().length];
        private boolean observed;

        public Builder set(Terminal terminal, double value) {
            values[terminal.ordinal()] = value;
            return this;
        }

        /** Sets a boolean terminal (active flags, in-zone, etc.). */
        public Builder setFlag(Terminal terminal, boolean value) {
            return set(terminal, value ? 1.0 : 0.0);
        }

        public Builder observed(boolean observed) {
            this.observed = observed;
            return this;
        }

        public EvalContext build() {
            return new EvalContext(values.clone(), observed);
        }
    }

    /** Debug helper: all values in ordinal order, clairvoyant masked when observed. */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("EvalContext{observed=")
                .append(observed).append(", ");
        for (Terminal t : Terminal.values()) {
            sb.append(t.name().toLowerCase(Locale.ROOT)).append('=')
                    .append(String.format(Locale.ROOT, "%.3f", value(t))).append(' ');
        }
        return sb.append('}').toString();
    }
}
