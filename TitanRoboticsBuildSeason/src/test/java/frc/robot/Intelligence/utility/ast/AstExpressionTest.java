package frc.robot.Intelligence.utility.ast;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import frc.robot.Intelligence.utility.ast.ExpressionNode.Constant;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Product;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Sigmoid;
import frc.robot.Intelligence.utility.ast.ExpressionNode.TerminalNode;
import frc.robot.Intelligence.utility.ast.ExpressionNode.Threshold;

/**
 * The evolvable expression tree: evaluation semantics, tier honesty, the
 * string genome codec, and the genetic operators.
 */
public class AstExpressionTest {

    private static EvalContext ctx(boolean observed) {
        return EvalContext.builder()
                .set(Terminal.HELD_RATIO, 0.5)
                .set(Terminal.DIST_TO_HUB, 0.7)
                .set(Terminal.MIDFIELD_FUEL, 0.9)
                .set(Terminal.SCORE_DIFF, 0.4)
                .setFlag(Terminal.MY_HUB_ACTIVE, true)
                .setFlag(Terminal.OPP_IN_TRENCH, true)
                .observed(observed)
                .build();
    }

    @Test
    void leavesAndOperatorsEvaluateAsExpected() {
        EvalContext c = ctx(false);
        assertEquals(1.5, new Constant(1.5).evaluate(c), 1e-12);
        assertEquals(0.5, new TerminalNode(Terminal.HELD_RATIO).evaluate(c), 1e-12);
        // sigmoid(x, k, mid): monotonically increasing in x
        double lo = new Sigmoid(new Constant(0.2), 8.0, 0.5).evaluate(c);
        double hi = new Sigmoid(new Constant(0.8), 8.0, 0.5).evaluate(c);
        assertTrue(hi > lo, "sigmoid must be monotone");
        // threshold
        assertEquals(1.0, new Threshold(new Constant(0.6), 0.5, 0.0, 1.0).evaluate(c), 1e-12);
        assertEquals(0.0, new Threshold(new Constant(0.4), 0.5, 0.0, 1.0).evaluate(c), 1e-12);
    }

    @Test
    void productIsANaturalVeto() {
        EvalContext c = ctx(false);
        Product withZero = new Product(java.util.List.of(
                new Constant(0.9), new Constant(0.0), new Constant(0.9)));
        assertEquals(0.0, withZero.evaluate(c), 1e-12);
        Product noVeto = new Product(java.util.List.of(new Constant(0.5), new Constant(0.8)));
        assertEquals(0.4, noVeto.evaluate(c), 1e-12);
    }

    @Test
    void clairvoyantTerminalsReadZeroUnderObservedTier() {
        assertEquals(0.9, new TerminalNode(Terminal.MIDFIELD_FUEL).evaluate(ctx(false)), 1e-12);
        assertEquals(0.0, new TerminalNode(Terminal.MIDFIELD_FUEL).evaluate(ctx(true)), 1e-12);
        // observed-only terminal is unaffected by the tier
        assertEquals(0.4, new TerminalNode(Terminal.SCORE_DIFF).evaluate(ctx(true)), 1e-12);
    }

    @Test
    void sExpressionRoundTripPreservesEvaluation() {
        ExpressionNode genome = new Product(java.util.List.of(
                new TerminalNode(Terminal.MY_HUB_ACTIVE),
                new Sigmoid(new TerminalNode(Terminal.HELD_RATIO), 6.0, 0.27),
                new Threshold(new TerminalNode(Terminal.DIST_TO_HUB), 0.5, 0.65, 1.0)));
        String text = SExpressions.write(genome);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed), "round trip is stable");
        assertEquals(genome.depth(), parsed.depth());
        for (int i = 0; i < 20; i++) {
            EvalContext c = EvalContext.builder()
                    .set(Terminal.HELD_RATIO, i / 20.0)
                    .set(Terminal.DIST_TO_HUB, 1.0 - i / 20.0)
                    .setFlag(Terminal.MY_HUB_ACTIVE, i % 2 == 0)
                    .build();
            assertEquals(genome.evaluate(c), parsed.evaluate(c), 1e-12,
                    "parsed genome must match the original");
        }
    }

    @Test
    void malformedGenomesAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> SExpressions.read("bogus_terminal"));
        assertThrows(IllegalArgumentException.class, () -> SExpressions.read("sigmoid(held_ratio, 6)"));
        assertThrows(IllegalArgumentException.class, () -> SExpressions.read("product(held_ratio,)"));
        assertThrows(IllegalArgumentException.class, () -> SExpressions.read(""));
        assertThrows(IllegalArgumentException.class, () -> SExpressions.read("(held_ratio)"));
    }

    @Test
    void geneticOperatorsProduceWellFormedTreesDeterministically() {
        Random a = new Random(7);
        Random b = new Random(7);
        ExpressionNode treeA = GeneticOperators.randomTree(a, 4);
        ExpressionNode treeB = GeneticOperators.randomTree(b, 4);
        assertEquals(SExpressions.write(treeA), SExpressions.write(treeB), "same seed, same tree");
        assertTrue(treeA.depth() <= 4);

        // mutate/crossover stay parseable and finite
        ExpressionNode mutated = GeneticOperators.mutate(treeA, a, 0.4);
        ExpressionNode child = GeneticOperators.crossover(treeA, treeB, a);
        ExpressionNode pruned = GeneticOperators.prune(treeA, 2, a);
        for (ExpressionNode g : new ExpressionNode[] {mutated, child, pruned}) {
            ExpressionNode reparsed = SExpressions.read(SExpressions.write(g));
            assertFalse(Double.isNaN(reparsed.evaluate(ctx(false))));
        }
        assertTrue(pruned.depth() <= 2);

        // same operation seed reproduces the same child
        ExpressionNode childAgain = GeneticOperators.crossover(treeA, treeB, new Random(11));
        ExpressionNode childRepro = GeneticOperators.crossover(treeA, treeB, new Random(11));
        assertEquals(SExpressions.write(childAgain), SExpressions.write(childRepro));
    }

    @Test
    void crossoverIntroducesTheDonorSubtree() {
        ExpressionNode donor = new TerminalNode(Terminal.OPP_IN_TRENCH);
        ExpressionNode host = new Product(java.util.List.of(
                new Constant(0.5), new Constant(0.5)));
        // Replace index 1 (a Constant) with the donor terminal.
        ExpressionNode crossed = GeneticOperators.replaceAt(host, 1, donor);
        assertEquals(3, crossed.size());
        assertTrue(SExpressions.write(crossed).contains("opp_in_trench"));
    }

    @Test
    void clairvoyantUsageIsDetectableForDeploySafety() {
        ExpressionNode observedOnly = new Product(java.util.List.of(
                new TerminalNode(Terminal.MY_HUB_ACTIVE), new Constant(0.8)));
        ExpressionNode clairvoyant = new Product(java.util.List.of(
                new TerminalNode(Terminal.MIDFIELD_FUEL), new Constant(0.8)));
        assertFalse(SExpressions.usesClairvoyantTerminal(observedOnly));
        assertTrue(SExpressions.usesClairvoyantTerminal(clairvoyant));
    }

    @Test
    void anIllustrativeCycleScoreHubGenomePrintsAndEvaluates() {
        // A readable, in-range stand-in for the CYCLE_SCORE_HUB intent (not
        // bit-identical to the engine formula — that parity is a later phase).
        ExpressionNode cycleScoreHub = new Product(java.util.List.of(
                new TerminalNode(Terminal.MY_HUB_ACTIVE),
                new Sigmoid(new TerminalNode(Terminal.HELD_RATIO), 6.0, 0.4),
                new Threshold(new TerminalNode(Terminal.DIST_TO_HUB), 0.4, 0.65, 1.0)));
        String text = SExpressions.write(cycleScoreHub);
        assertTrue(text.startsWith("product(my_hub_active, sigmoid(held_ratio"), text);
        assertNotEquals(0.0, cycleScoreHub.evaluate(ctx(false)));
        assertTrue(cycleScoreHub.evaluate(ctx(false)) <= 1.0);
    }
}
