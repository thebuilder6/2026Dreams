package frc.robot.Intelligence.utility.ast;

import java.util.List;

import frc.robot.Intelligence.PolicyWeights;

/**
 * Hand-authored reference genomes for engine objectives, used to prove that an
 * {@link ExpressionNode} can reproduce a Java objective's math before the engine
 * is switched over to it.
 *
 * <p>{@link #cycleScoreHub} is a tree form of {@code JevDecisionEngine}'s
 * {@code CYCLE_SCORE_HUB} raw score (the pre-archetype formula). It matches the
 * engine's value under the conditions the parity test states; shell features
 * that are <b>not</b> part of that formula — archetype zeroing, the
 * harvest-deadline force, the endgame G420 suppression, and the commitment
 * inertia — stay hand-written in the engine and are deliberately not expressible
 * here.
 */
public final class ObjectiveExpressions {

    private ObjectiveExpressions() {}

    /**
     * {@code CYCLE_SCORE_HUB} raw utility as an expression.
     *
     * <p>Terminal conventions this tree assumes (the caller must build the
     * {@link EvalContext} the same way):
     * <ul>
     *   <li>{@code HELD_RATIO} = {@code heldFuel / ballCapacity}</li>
     *   <li>{@code DIST_TO_HUB} = {@code clamp01(1 - distanceMeters / distNormMeters)}</li>
     *   <li>{@code TIME_UNTIL_SHIFT} = {@code clamp01(timeUntilShiftSec / 20)}</li>
     *   <li>{@code SCORE_DIFF} = {@code clamp01(0.5 + differential / 2)} (sign-preserving)</li>
     * </ul>
     *
     * <p>Assumes a fitted shooter ({@code hasShooter = true}); the engine's
     * {@code shooterFactor} is {@code 1} in that case.
     *
     * @param weights      policy weights supplying the score constants
     * @param ballCapacity hopper capacity the normalization divides by
     * @param distNormMeters distance the {@code DIST_TO_HUB} terminal normalizes over
     */
    public static ExpressionNode cycleScoreHub(PolicyWeights weights, int ballCapacity,
            double distNormMeters) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);
        double divisor = Math.min(w.scoreHubCapacityDivisor(), cap);
        double loadFactor = cap / divisor;

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode dist = new ExpressionNode.TerminalNode(Terminal.DIST_TO_HUB);
        ExpressionNode timeUntilShift = new ExpressionNode.TerminalNode(Terminal.TIME_UNTIL_SHIFT);
        ExpressionNode scoreDiff = new ExpressionNode.TerminalNode(Terminal.SCORE_DIFF);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);

        // inShootingRange: distance <= shootingRange
        double rangeThreshold = 1.0 - w.scoreHubShootingRangeMeters() / distNormMeters;
        ExpressionNode inRange = new ExpressionNode.Threshold(dist, rangeThreshold, 0, 1, true);

        // shiftEndingSoon: 0 < timeUntilShift <= window  (0 is the "no flip" sentinel)
        double shiftRatio = w.scoreHubShiftEndingWindowSec() / 20.0;
        ExpressionNode shiftEnding = new ExpressionNode.Product(List.of(
                new ExpressionNode.Threshold(timeUntilShift, 0.0, 0, 1),
                new ExpressionNode.Not(new ExpressionNode.Threshold(timeUntilShift, shiftRatio, 0, 1))));

        // load: full when in range, else min(1, held / capacityDivisor); shaped by the payload exponent
        ExpressionNode loadRatio = new ExpressionNode.IfThenElse(inRange,
                new ExpressionNode.Constant(1.0),
                new ExpressionNode.Clamp(new ExpressionNode.Scale(heldRatio, loadFactor), 0.0, 1.0));
        ExpressionNode shapedLoad = new ExpressionNode.Power(loadRatio, w.scoreHubPayloadExponent());

        // base = scoreHubScale * shapedLoad + scoreHubBase
        ExpressionNode base = new ExpressionNode.Sum(List.of(
                new ExpressionNode.Scale(shapedLoad, w.scoreHubScale()),
                new ExpressionNode.Constant(w.scoreHubBase())));
        // behind bonus: min(behindMax, base + bonus) when differential < 0
        ExpressionNode behind = new ExpressionNode.Not(new ExpressionNode.Threshold(scoreDiff, 0.5, 0, 1, true));
        ExpressionNode baseFinal = new ExpressionNode.IfThenElse(behind,
                new ExpressionNode.Min(List.of(
                        new ExpressionNode.Sum(List.of(base, new ExpressionNode.Constant(w.scoreHubBehindBonus()))),
                        new ExpressionNode.Constant(w.scoreHubBehindMax()))),
                base);

        // min fuel gate: in range -> 1, else shift-ending -> minFuelShiftEnding, else minFuelNormal
        ExpressionNode fuelThreshold = new ExpressionNode.IfThenElse(inRange,
                new ExpressionNode.Threshold(heldRatio, 1.0 / cap, 0, 1, true),
                new ExpressionNode.IfThenElse(shiftEnding,
                        new ExpressionNode.Threshold(heldRatio, w.scoreHubMinFuelShiftEnding() / cap, 0, 1, true),
                        new ExpressionNode.Threshold(heldRatio, w.scoreHubMinFuelNormal() / cap, 0, 1, true)));

        return new ExpressionNode.Product(List.of(baseFinal, hubActive, fuelThreshold));
    }

    /** {@link #cycleScoreHub} with the standard 8 m distance normalization. */
    public static ExpressionNode cycleScoreHub(PolicyWeights weights, int ballCapacity) {
        return cycleScoreHub(weights, ballCapacity, 8.0);
    }
}
