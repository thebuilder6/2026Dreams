package frc.robot.Intelligence.utility.ast;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import frc.robot.Intelligence.utility.ast.ExpressionNode.Constant;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Gaussian;
import frc.robot.Intelligence.utility.ast.ExpressionNode.IfThenElse;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Max;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Min;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Not;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Power;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Product;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Sigmoid;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Sum;
import frc.robot.Intelligence.utility.ast.ExpressionNode.TerminalNode;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Threshold;

/**
 * The genetic operators for expression-tree genomes: initialization, point and
 * structural mutation, subtree crossover, and depth pruning.
 *
 * <p>All operators are pure (they return new trees) and take an explicit
 * {@link Random}, so a whole tournament is reproducible from one seed. The
 * operators work through {@link ExpressionNode#children()} /
 * {@link ExpressionNode#withChildren(List)}, so adding a node type needs no
 * change here.
 */
public final class GeneticOperators {

    /** Numeric operator params (thresholds, slopes, midpoints) are evolved here. */
    public static final double CONST_MIN = -2.0;
    public static final double CONST_MAX = 2.0;
    /** Leaf constants are utility weights, so they live in [0,1]. */
    public static final double WEIGHT_MIN = 0.0;
    public static final double WEIGHT_MAX = 1.0;

    private GeneticOperators() {}

    /** A random leaf: a weight constant in {@code [0,1]}, or a terminal. */
    public static ExpressionNode randomLeaf(Random rng) {
        if (rng.nextDouble() < 0.5) {
            return new Constant(WEIGHT_MIN + rng.nextDouble() * (WEIGHT_MAX - WEIGHT_MIN));
        }
        Terminal[] terminals = Terminal.values();
        return new TerminalNode(terminals[rng.nextInt(terminals.length)]);
    }

    /** A random tree at most {@code maxDepth} deep (a leaf when {@code maxDepth <= 1}). */
    public static ExpressionNode randomTree(Random rng, int maxDepth) {
        if (maxDepth <= 1 || rng.nextDouble() < 0.25) {
            return randomLeaf(rng);
        }
        int d = maxDepth - 1;
        return switch (rng.nextInt(10)) {
            case 0 -> new Product(children(rng, d, 2 + rng.nextInt(2)));
            case 1 -> new Sum(children(rng, d, 2));
            case 2 -> new Min(children(rng, d, 2));
            case 3 -> new Max(children(rng, d, 2));
            case 4 -> new IfThenElse(randomTree(rng, d), randomTree(rng, d), randomTree(rng, d));
            case 5 -> new Threshold(randomTree(rng, d), randomConstant(rng), 0.0, 1.0);
            case 6 -> new Sigmoid(randomTree(rng, d), 1.0 + rng.nextDouble() * 9.0, rng.nextDouble());
            case 7 -> new Gaussian(randomTree(rng, d), rng.nextDouble(), 0.05 + rng.nextDouble() * 0.45);
            case 8 -> new Power(randomTree(rng, d), 0.5 + rng.nextDouble() * 2.5);
            default -> new Not(randomTree(rng, d));
        };
    }

    /**
     * Point + structural mutation: each node may have its constant jittered or
     * be replaced by a fresh small subtree, at probability {@code rate}.
     */
    public static ExpressionNode mutate(ExpressionNode node, Random rng, double rate) {
        if (node instanceof Constant c && rng.nextDouble() < rate) {
            return new Constant(jitter(c.value(), rng));
        }
        if (rng.nextDouble() < rate) {
            return randomTree(rng, 2);
        }
        List<ExpressionNode> kids = node.children();
        if (kids.isEmpty()) {
            return node;
        }
        List<ExpressionNode> mutated = new ArrayList<>(kids.size());
        for (ExpressionNode kid : kids) {
            mutated.add(mutate(kid, rng, rate));
        }
        return node.withChildren(mutated);
    }

    /** Subtree crossover: a random subtree of {@code b} replaces one of {@code a}. */
    public static ExpressionNode crossover(ExpressionNode a, ExpressionNode b, Random rng) {
        List<ExpressionNode> aNodes = collect(a);
        List<ExpressionNode> bNodes = collect(b);
        int ia = rng.nextInt(aNodes.size());
        int ib = rng.nextInt(bNodes.size());
        return replaceAt(a, ia, bNodes.get(ib).copy());
    }

    /** Truncates subtrees deeper than {@code maxDepth} to fresh leaves. */
    public static ExpressionNode prune(ExpressionNode node, int maxDepth, Random rng) {
        if (maxDepth <= 1) {
            return (node instanceof TerminalNode || node instanceof Constant) ? node : randomLeaf(rng);
        }
        List<ExpressionNode> kids = node.children();
        if (kids.isEmpty()) {
            return node;
        }
        List<ExpressionNode> pruned = new ArrayList<>(kids.size());
        for (ExpressionNode kid : kids) {
            pruned.add(prune(kid, maxDepth - 1, rng));
        }
        return node.withChildren(pruned);
    }

    /** Preorder list of every node in the tree (root first). */
    public static List<ExpressionNode> collect(ExpressionNode node) {
        List<ExpressionNode> out = new ArrayList<>();
        collectInto(node, out);
        return out;
    }

    private static void collectInto(ExpressionNode node, List<ExpressionNode> out) {
        out.add(node);
        for (ExpressionNode kid : node.children()) {
            collectInto(kid, out);
        }
    }

    /** Replaces the node at preorder {@code index} in {@code root}. */
    public static ExpressionNode replaceAt(ExpressionNode root, int index, ExpressionNode replacement) {
        if (index < 0 || index >= collect(root).size()) {
            throw new IndexOutOfBoundsException("node index " + index + " out of range");
        }
        return replace(root, index, replacement, new int[] {0});
    }

    private static ExpressionNode replace(ExpressionNode node, int index,
            ExpressionNode replacement, int[] counter) {
        if (counter[0] == index) {
            counter[0]++;
            return replacement;
        }
        counter[0]++;
        List<ExpressionNode> kids = node.children();
        if (kids.isEmpty()) {
            return node;
        }
        List<ExpressionNode> rebuilt = new ArrayList<>(kids.size());
        for (ExpressionNode kid : kids) {
            rebuilt.add(replace(kid, index, replacement, counter));
        }
        return node.withChildren(rebuilt);
    }

    private static List<ExpressionNode> children(Random rng, int depth, int count) {
        List<ExpressionNode> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            out.add(randomTree(rng, depth));
        }
        return out;
    }

    private static double randomConstant(Random rng) {
        return CONST_MIN + rng.nextDouble() * (CONST_MAX - CONST_MIN);
    }

    private static double jitter(double value, Random rng) {
        double v = value + rng.nextGaussian() * 0.3;
        return Math.max(WEIGHT_MIN, Math.min(WEIGHT_MAX, v));
    }
}
