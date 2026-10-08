package frc.robot.Intelligence.utility.ast;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A mutable-by-construction expression tree that evaluates to a utility in
 * {@code [0,1]} — the evolvable form of a Jev objective's score.
 *
 * <p>This is Approach A from the design (symbolic utility trees over the
 * existing IAUS architecture): instead of a hand-written Java formula per
 * objective, the score is data that can be printed, mutated, and crossed over.
 * Evaluation is pure and allocation-free for a built tree, so it is safe in the
 * 20 ms loop and re-entrant across sparring bots.
 *
 * <p>Node set is deliberately closed (one file, all implementations nested) so
 * an S-expression genome can only name operators that exist. Signals that must
 * stay hand-written — archetype zeroing, the harvest-deadline force, G420
 * suppression, the no-information fallback, and the commitment inertia — are
 * <b>not</b> expressible here; they remain engine scaffolding.
 *
 * <p>{@link #evaluate} clamps at each operator, so a tree is total and bounded
 * for any finite input. {@link Constant} is the one node that may hold a value
 * outside {@code [0,1]} (thresholds, slopes, midpoints).
 */
public sealed interface ExpressionNode {

    /** Evaluates this subtree. Never NaN; result of operators is in {@code [0,1]}. */
    double evaluate(EvalContext ctx);

    /** Child subtrees, in evaluation order. Empty for leaves. */
    List<ExpressionNode> children();

    /**
     * Rebuilds this node with new children (same arity). Leaves reject a
     * non-empty list. Used by the genetic operators so they need no per-type
     * knowledge.
     */
    ExpressionNode withChildren(List<ExpressionNode> children);

    /** Human-readable pseudo-code, e.g. {@code product(my_hub_active, sigmoid(held_ratio, 6, 0.27))}. */
    String toReadableString();

    /** Deep copy. */
    default ExpressionNode copy() {
        List<ExpressionNode> kids = children();
        if (kids.isEmpty()) {
            return this;
        }
        List<ExpressionNode> copied = new ArrayList<>(kids.size());
        for (ExpressionNode kid : kids) {
            copied.add(kid.copy());
        }
        return withChildren(copied);
    }

    /** Depth of the tree; a leaf is 1. */
    default int depth() {
        int max = 0;
        for (ExpressionNode kid : children()) {
            max = Math.max(max, kid.depth());
        }
        return 1 + max;
    }

    /** Node count including this node. */
    default int size() {
        int n = 1;
        for (ExpressionNode kid : children()) {
            n += kid.size();
        }
        return n;
    }

    static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    static String fmt(double v) {
        String s = String.format(Locale.ROOT, "%.4f", v);
        while (s.contains(".") && (s.endsWith("0"))) {
            s = s.substring(0, s.length() - 1);
        }
        return s.endsWith(".") ? s + "0" : s;
    }

    // ------------------------------------------------------------------
    // Leaves
    // ------------------------------------------------------------------

    /** An evolved constant: threshold, midpoint, slope, or weight. */
    record Constant(double value) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            return value;
        }
        @Override public List<ExpressionNode> children() {
            return List.of();
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            if (!children.isEmpty()) {
                throw new IllegalArgumentException("Constant is a leaf");
            }
            return this;
        }
        @Override public String toReadableString() {
            return fmt(value);
        }
    }

    /** Reads one {@link Terminal} from the context. */
    record TerminalNode(Terminal terminal) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            return ctx.value(terminal);
        }
        @Override public List<ExpressionNode> children() {
            return List.of();
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            if (!children.isEmpty()) {
                throw new IllegalArgumentException("TerminalNode is a leaf");
            }
            return this;
        }
        @Override public String toReadableString() {
            return terminal.name().toLowerCase(Locale.ROOT);
        }
    }

    // ------------------------------------------------------------------
    // Aggregation
    // ------------------------------------------------------------------

    /** Multiplicative veto, mirroring {@code UtilityAction.evaluateProduct}. */
    record Product(List<ExpressionNode> factors) implements ExpressionNode {
        public Product {
            factors = List.copyOf(factors);
        }
        @Override public double evaluate(EvalContext ctx) {
            if (factors.isEmpty()) {
                return 0.0;
            }
            double product = 1.0;
            for (ExpressionNode f : factors) {
                double s = clamp01(f.evaluate(ctx));
                if (s <= 1e-6) {
                    return 0.0;
                }
                product *= s;
            }
            return clamp01(product);
        }
        @Override public List<ExpressionNode> children() {
            return factors;
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            return new Product(children);
        }
        @Override public String toReadableString() {
            return "product(" + join(factors) + ")";
        }
    }

    /** Bounded additive combination, clamped to {@code [0,1]}. */
    record Sum(List<ExpressionNode> terms) implements ExpressionNode {
        public Sum {
            terms = List.copyOf(terms);
        }
        @Override public double evaluate(EvalContext ctx) {
            double total = 0.0;
            for (ExpressionNode t : terms) {
                total += clamp01(t.evaluate(ctx));
            }
            return clamp01(total);
        }
        @Override public List<ExpressionNode> children() {
            return terms;
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            return new Sum(children);
        }
        @Override public String toReadableString() {
            return "sum(" + join(terms) + ")";
        }
    }

    /** Worst-case (natural veto) combination. */
    record Min(List<ExpressionNode> terms) implements ExpressionNode {
        public Min {
            terms = List.copyOf(terms);
        }
        @Override public double evaluate(EvalContext ctx) {
            if (terms.isEmpty()) {
                return 0.0;
            }
            double best = 1.0;
            for (ExpressionNode t : terms) {
                best = Math.min(best, clamp01(t.evaluate(ctx)));
            }
            return best;
        }
        @Override public List<ExpressionNode> children() {
            return terms;
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            return new Min(children);
        }
        @Override public String toReadableString() {
            return "min(" + join(terms) + ")";
        }
    }

    /** Best-case combination. */
    record Max(List<ExpressionNode> terms) implements ExpressionNode {
        public Max {
            terms = List.copyOf(terms);
        }
        @Override public double evaluate(EvalContext ctx) {
            double best = 0.0;
            for (ExpressionNode t : terms) {
                best = Math.max(best, clamp01(t.evaluate(ctx)));
            }
            return best;
        }
        @Override public List<ExpressionNode> children() {
            return terms;
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            return new Max(children);
        }
        @Override public String toReadableString() {
            return "max(" + join(terms) + ")";
        }
    }

    // ------------------------------------------------------------------
    // Branching & shaping
    // ------------------------------------------------------------------

    /** Branching rule: {@code cond > 0.5 ? then : else}. */
    record IfThenElse(ExpressionNode condition, ExpressionNode thenExpr, ExpressionNode elseExpr)
            implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            ExpressionNode chosen = condition.evaluate(ctx) > 0.5 ? thenExpr : elseExpr;
            return clamp01(chosen.evaluate(ctx));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(condition, thenExpr, elseExpr);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 3, "IfThenElse");
            return new IfThenElse(children.get(0), children.get(1), children.get(2));
        }
        @Override public String toReadableString() {
            return "if(" + condition.toReadableString() + ", " + thenExpr.toReadableString()
                    + ", " + elseExpr.toReadableString() + ")";
        }
    }

    /** Hard gate: {@code x > threshold ? above : below}. */
    record Threshold(ExpressionNode x, double threshold, double below, double above)
            implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            return clamp01(x.evaluate(ctx) > threshold ? above : below);
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Threshold");
            return new Threshold(children.get(0), threshold, below, above);
        }
        @Override public String toReadableString() {
            return "threshold(" + x.toReadableString() + ", " + fmt(threshold) + ", "
                    + fmt(below) + ", " + fmt(above) + ")";
        }
    }

    /** Logistic response curve: {@code 1 / (1 + exp(-k (x - mid)))}. */
    record Sigmoid(ExpressionNode x, double steepness, double midpoint) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            double v = x.evaluate(ctx);
            return clamp01(1.0 / (1.0 + Math.exp(-steepness * (v - midpoint))));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Sigmoid");
            return new Sigmoid(children.get(0), steepness, midpoint);
        }
        @Override public String toReadableString() {
            return "sigmoid(" + x.toReadableString() + ", " + fmt(steepness) + ", " + fmt(midpoint) + ")";
        }
    }

    /** Gaussian response curve centered on {@code center}. */
    record Gaussian(ExpressionNode x, double center, double sigma) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            double s = sigma > 1e-6 ? sigma : 0.2;
            double d = x.evaluate(ctx) - center;
            return clamp01(Math.exp(-(d * d) / (2.0 * s * s)));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Gaussian");
            return new Gaussian(children.get(0), center, sigma);
        }
        @Override public String toReadableString() {
            return "gaussian(" + x.toReadableString() + ", " + fmt(center) + ", " + fmt(sigma) + ")";
        }
    }

    /** Power shaping, guarded to the non-negative domain. */
    record Power(ExpressionNode x, double exponent) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            return clamp01(Math.pow(Math.max(0.0, clamp01(x.evaluate(ctx))), exponent));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Power");
            return new Power(children.get(0), exponent);
        }
        @Override public String toReadableString() {
            return "pow(" + x.toReadableString() + ", " + fmt(exponent) + ")";
        }
    }

    /** Bounds the input domain before further shaping. */
    record Clamp(ExpressionNode x, double min, double max) implements ExpressionNode {
        public Clamp {
            if (max < min) {
                double t = min;
                min = max;
                max = t;
            }
        }
        @Override public double evaluate(EvalContext ctx) {
            return Math.max(min, Math.min(max, x.evaluate(ctx)));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Clamp");
            return new Clamp(children.get(0), min, max);
        }
        @Override public String toReadableString() {
            return "clamp(" + x.toReadableString() + ", " + fmt(min) + ", " + fmt(max) + ")";
        }
    }

    /** Logical negation on the {@code [0,1]} domain: {@code 1 - x}. */
    record Not(ExpressionNode x) implements ExpressionNode {
        @Override public double evaluate(EvalContext ctx) {
            return 1.0 - clamp01(x.evaluate(ctx));
        }
        @Override public List<ExpressionNode> children() {
            return List.of(x);
        }
        @Override public ExpressionNode withChildren(List<ExpressionNode> children) {
            require(children, 1, "Not");
            return new Not(children.get(0));
        }
        @Override public String toReadableString() {
            return "not(" + x.toReadableString() + ")";
        }
    }

    // shared helpers for the nested records
    private static void require(List<ExpressionNode> children, int arity, String name) {
        if (children.size() != arity) {
            throw new IllegalArgumentException(name + " expects " + arity + " children");
        }
    }

    private static String join(List<ExpressionNode> nodes) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < nodes.size(); i++) {
            if (i > 0) {
                sb.append(", ");
            }
            sb.append(nodes.get(i).toReadableString());
        }
        return sb.toString();
    }
}
