package frc.robot.Intelligence.utility.ast;

import java.util.List;

import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Navigation.FieldMap;

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

    /**
     * {@code STAGE_STANDOFF} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code STAGE_STANDOFF} score
     * (the pre-archetype formula for one archetype branch). The engine blends
     * {@code stageStandoffBase} toward a per-archetype imminent target with a
     * logistic urgency over <b>raw</b> seconds; the {@code TIME_UNTIL_SHIFT}
     * terminal carries {@code clamp01(t / 20)}, so the tree runs the sigmoid
     * with a negated, re-scaled steepness ({@code -k * 20}) and midpoint
     * ({@code mid / 20}) — the sign flip is because engine urgency <i>falls</i>
     * as the countdown grows while the terminal <i>rises</i>. The
     * {@code t > 0} no-flip sentinel is a strict threshold gate, matching the
     * engine's {@code timeUntilHubShift > 0.0} branch.
     *
     * <p>Assumes a fitted shooter ({@code hasShooter = true}); the engine's
     * {@code stageShooterFactor} is {@code 1} in that case. Like
     * {@link #cycleScoreHub}, shell features that are not part of the formula —
     * defensive-archetype zeroing, the harvest-deadline force, G420
     * suppression, and the commitment inertia — stay hand-written in the engine.
     *
     * @param weights      policy weights supplying the stage constants
     * @param ballCapacity hopper capacity the fuel gate normalizes over
     * @param coPilot      true for the co-pilot branch (target + min-fuel 1),
     *                     false for the normal branch
     */
    public static ExpressionNode stageStandoff(PolicyWeights weights, int ballCapacity, boolean coPilot) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);
        double base = w.stageStandoffBase();
        double target = coPilot ? w.stageShiftImminentCoPilot() : w.stageShiftImminentNormal();
        double minFuel = coPilot ? 1 : w.stageMinFuelNormal();

        ExpressionNode timeUntilShift = new ExpressionNode.TerminalNode(Terminal.TIME_UNTIL_SHIFT);
        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);

        // shiftUrgency: 0 when no flip is coming (t == 0 sentinel), else the
        // logistic 1 / (1 + exp(-k * (mid - t))) re-based onto the normalized terminal.
        ExpressionNode hasFlip = new ExpressionNode.Threshold(timeUntilShift, 0.0, 0, 1);
        ExpressionNode logistic = new ExpressionNode.Sigmoid(timeUntilShift,
                -w.shiftUrgencySigmoidSteepness() * WorldStateTerminals.SHIFT_NORM_SEC,
                w.shiftUrgencyMidpointSec() / WorldStateTerminals.SHIFT_NORM_SEC);
        ExpressionNode urgency = new ExpressionNode.Product(List.of(hasFlip, logistic));

        // stageBase = base * (1 - urgency) + target * urgency (stays in [0,1]
        // for weights in [0,1], so the clamping Sum never saturates).
        ExpressionNode stageBase = new ExpressionNode.Sum(List.of(
                new ExpressionNode.Product(List.of(new ExpressionNode.Constant(base),
                        new ExpressionNode.Not(urgency))),
                new ExpressionNode.Product(List.of(new ExpressionNode.Constant(target), urgency))));

        ExpressionNode hubInactive = new ExpressionNode.Not(hubActive);
        ExpressionNode fuelGate = new ExpressionNode.Threshold(heldRatio, minFuel / cap, 0, 1, true);

        return new ExpressionNode.Product(List.of(stageBase, hubInactive, fuelGate));
    }

    /** {@link #stageStandoff(PolicyWeights, int, boolean)} for the normal branch. */
    public static ExpressionNode stageStandoff(PolicyWeights weights, int ballCapacity) {
        return stageStandoff(weights, ballCapacity, false);
    }

    /**
     * {@code VACUUM_MIDFIELD} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code VACUUM_MIDFIELD} score:
     * a hub-gated branch between the inactive harvest ramp and the active batch
     * ramp (batch target picked by the same shift-ending window the score-hub
     * formula uses, capped to hopper capacity; at or above the batch the flat
     * full-cap value applies), vetoed by hopper headroom and midfield fuel
     * presence. Each linear ramp {@code B - s*p} is factored as
     * {@code B * (1 - (s/B)*p)} because the node set has no subtraction.
     *
     * <p>Two boundaries, both documented rather than modelled: (a) the tree
     * reads the <b>clairvoyant</b> fuel path ({@code MIDFIELD_FUEL > 0}) — under
     * the observed tier the engine assumes neutral fuel may exist
     * ({@code midFuelCount = 1}) while the terminal honestly reads 0, and that
     * assume-present default stays engine scaffolding with the other
     * hand-written shell (defensive-archetype zeroing, harvest-deadline force,
     * G420 suppression, commitment inertia); (b) assumes
     * {@code ballCapacity >= 1} (the harvest-capacity factor is 1 there), and
     * the batch-progress {@code held / target} rescale assumes the capped batch
     * target is positive — a zero target takes the full-cap arm at build time.
     * Assumes a fitted shooter only insofar as the engine's vacuum formula does
     * (it does not read {@code hasShooter} at all).
     *
     * @param weights      policy weights supplying the vacuum constants
     * @param ballCapacity hopper capacity normalizations divide by
     */
    public static ExpressionNode vacuumMidfield(PolicyWeights weights, int ballCapacity) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode timeUntilShift = new ExpressionNode.TerminalNode(Terminal.TIME_UNTIL_SHIFT);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);
        ExpressionNode midfieldFuel = new ExpressionNode.TerminalNode(Terminal.MIDFIELD_FUEL);

        // shiftEndingSoon: 0 < t <= scoreHubShiftEndingWindowSec (the 0.0 no-flip sentinel).
        double windowRatio = w.scoreHubShiftEndingWindowSec() / WorldStateTerminals.SHIFT_NORM_SEC;
        ExpressionNode shiftEnding = new ExpressionNode.Product(List.of(
                new ExpressionNode.Threshold(timeUntilShift, 0.0, 0, 1),
                new ExpressionNode.Not(new ExpressionNode.Threshold(timeUntilShift, windowRatio, 0, 1))));

        int normalBatch = Math.min(w.vacuumActiveNormalBatch(), (int) cap);
        int endingBatch = Math.min(w.vacuumActiveShiftEndingBatch(), (int) cap);

        // batchProgress = held / target (curve input clamps to [0,1], as does Scale).
        ExpressionNode batchProgress = new ExpressionNode.IfThenElse(shiftEnding,
                progressFor(heldRatio, cap, endingBatch), progressFor(heldRatio, cap, normalBatch));
        // Full at or above the batch: progress hits exactly 1.0 there.
        ExpressionNode atBatch = new ExpressionNode.Threshold(batchProgress, 1.0, 0, 1, true);

        ExpressionNode activeRaw = new ExpressionNode.IfThenElse(atBatch,
                new ExpressionNode.Constant(w.vacuumActiveFullCap()),
                descendingRamp(batchProgress, w.vacuumActiveBase(), w.vacuumActiveScale()));
        ExpressionNode inactiveRaw =
                descendingRamp(heldRatio, w.vacuumInactiveBase(), w.vacuumInactiveScale());
        ExpressionNode raw = new ExpressionNode.IfThenElse(hubActive, activeRaw, inactiveRaw);

        // Headroom: 1 iff held < cap (HELD_RATIO hits exactly 1.0 at cap).
        ExpressionNode headroom =
                new ExpressionNode.Not(new ExpressionNode.Threshold(heldRatio, 1.0, 0, 1, true));
        ExpressionNode fuelPresent = new ExpressionNode.Threshold(midfieldFuel, 0.0, 0, 1);

        return new ExpressionNode.Product(List.of(raw, headroom, fuelPresent));
    }

    /**
     * {@code SHUTTLE_PASS} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code SHUTTLE_PASS} score: a
     * flat weight vetoed by nine gates — fitted shooter, teleop phase, inactive
     * home hub, opponent hub live now or after the shift (a {@code Max}, exact
     * on 0/1 flags), own bumpers in the alliance zone, own bumpers out of the
     * trench bands, standoff beyond the minimum pass distance, and minimum held
     * fuel. The distance gate mirrors the score-hub range pattern
     * ({@code dist > min} is {@code Not(norm >= 1 - min/distNorm)}, inclusive so
     * the equality case agrees) and likewise requires
     * {@code minDist < distNorm}.
     *
     * <p>Uses the same distance normalization the caller passes to
     * {@link WorldStateTerminals#toContext}. Assumes
     * {@code ballCapacity >= 1} for the fuel gate like the other conversions.
     *
     * @param weights        policy weights supplying the shuttle constants
     * @param ballCapacity   hopper capacity the fuel gate normalizes over
     * @param distNormMeters distance the {@code DIST_TO_HUB} terminal normalizes over
     */
    public static ExpressionNode shuttlePass(PolicyWeights weights, int ballCapacity,
            double distNormMeters) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);
        ExpressionNode dist = new ExpressionNode.TerminalNode(Terminal.DIST_TO_HUB);

        double distThreshold = 1.0 - w.shuttlePassMinDistMeters() / distNormMeters;
        ExpressionNode distGate =
                new ExpressionNode.Not(new ExpressionNode.Threshold(dist, distThreshold, 0, 1, true));
        ExpressionNode fuelGate =
                new ExpressionNode.Threshold(heldRatio, w.shuttlePassMinHeld() / cap, 0, 1, true);
        ExpressionNode oppHubLive = new ExpressionNode.Max(List.of(
                new ExpressionNode.TerminalNode(Terminal.OPP_HUB_ACTIVE),
                new ExpressionNode.TerminalNode(Terminal.OPP_HUB_NEXT)));

        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.shuttlePassUtility()),
                new ExpressionNode.TerminalNode(Terminal.HAS_SHOOTER),
                new ExpressionNode.Not(new ExpressionNode.TerminalNode(Terminal.IS_AUTONOMOUS)),
                new ExpressionNode.Not(hubActive),
                oppHubLive,
                new ExpressionNode.TerminalNode(Terminal.IN_ALLIANCE_ZONE),
                new ExpressionNode.Not(new ExpressionNode.TerminalNode(Terminal.IN_LOW_CLEARANCE)),
                distGate,
                fuelGate));
    }

    /** {@link #shuttlePass(PolicyWeights, int, double)} with the standard 8 m distance normalization. */
    public static ExpressionNode shuttlePass(PolicyWeights weights, int ballCapacity) {
        return shuttlePass(weights, ballCapacity, 8.0);
    }

    /**
     * {@code LONG_RANGE_SNIPE} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code LONG_RANGE_SNIPE}
     * score: a close/far base picked by whether a tracked opponent contests
     * our hub, vetoed by fitted shooter, live home hub, minimum held fuel,
     * alliance zone, and the distance band ({@code minDist <= dist <=
     * SHOOTING_MAX_DISTANCE}). The band edges mirror the engine's mixed
     * inclusivity exactly — inclusive below ({@code >=}, so a <i>strict</i>
     * threshold under the {@code Not}) and inclusive above — and require
     * {@code minDist < distNorm}. Uses the same distance normalization the
     * caller passes to {@link WorldStateTerminals#toContext}; the max-range
     * edge reads {@code FieldMap.Hubs.SHOOTING_MAX_DISTANCE}, single-owned
     * there. Assumes {@code ballCapacity >= 1} like the other conversions.
     *
     * @param weights        policy weights supplying the snipe constants
     * @param ballCapacity   hopper capacity the fuel gate normalizes over
     * @param distNormMeters distance the {@code DIST_TO_HUB} terminal normalizes over
     */
    public static ExpressionNode longRangeSnipe(PolicyWeights weights, int ballCapacity,
            double distNormMeters) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);
        ExpressionNode dist = new ExpressionNode.TerminalNode(Terminal.DIST_TO_HUB);

        ExpressionNode base = new ExpressionNode.IfThenElse(
                new ExpressionNode.TerminalNode(Terminal.OPP_NEAR_MY_HUB),
                new ExpressionNode.Constant(w.snipeCloseUtility()),
                new ExpressionNode.Constant(w.snipeFarUtility()));
        double lowerThreshold = 1.0 - w.snipeMinDistMeters() / distNormMeters;
        ExpressionNode lower =
                new ExpressionNode.Not(new ExpressionNode.Threshold(dist, lowerThreshold, 0, 1));
        double upperThreshold = 1.0 - FieldMap.Hubs.SHOOTING_MAX_DISTANCE / distNormMeters;
        ExpressionNode upper = new ExpressionNode.Threshold(dist, upperThreshold, 0, 1, true);
        ExpressionNode fuelGate =
                new ExpressionNode.Threshold(heldRatio, w.snipeMinHeld() / cap, 0, 1, true);

        return new ExpressionNode.Product(List.of(
                base,
                new ExpressionNode.TerminalNode(Terminal.HAS_SHOOTER),
                hubActive,
                fuelGate,
                new ExpressionNode.TerminalNode(Terminal.IN_ALLIANCE_ZONE),
                lower,
                upper));
    }

    /** {@link #longRangeSnipe(PolicyWeights, int, double)} with the standard 8 m distance normalization. */
    public static ExpressionNode longRangeSnipe(PolicyWeights weights, int ballCapacity) {
        return longRangeSnipe(weights, ballCapacity, 8.0);
    }

    /**
     * {@code DENY_SHOOTING_LANE} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code DENY_SHOOTING_LANE}
     * score: the flat active weight vetoed by opponent tracks present, opponent
     * hub live, and the tracked opponent inside the denial radius (strict, so a
     * strict threshold). Two boundaries, both documented: (a) the defender-role
     * factor is assumed 1 — archetype stays engine scaffolding, so the parity
     * grid runs a defender archetype whose post-processing leaves the lane
     * value untouched ({@code ADAPTIVE_COMPETITOR}); (b) the range terminal
     * reads the nearest listed opponent while the engine reads the tracked
     * pose — identical in production and in the grid, where the two are kept
     * equal. Uses the same distance normalization the caller passes to
     * {@link WorldStateTerminals#toContext}.
     *
     * @param weights        policy weights supplying the denial constants
     * @param distNormMeters distance the {@code OPP_DIST_TO_HUB} terminal normalizes over
     */
    public static ExpressionNode denyShootingLane(PolicyWeights weights, double distNormMeters) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;

        double laneThreshold = 1.0 - w.laneDenialMaxDistMeters() / distNormMeters;
        ExpressionNode inRange = new ExpressionNode.Threshold(
                new ExpressionNode.TerminalNode(Terminal.OPP_DIST_TO_HUB), laneThreshold, 0, 1);

        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.laneDenialActiveUtility()),
                new ExpressionNode.TerminalNode(Terminal.OPPONENT_OBSERVED),
                new ExpressionNode.TerminalNode(Terminal.OPP_HUB_ACTIVE),
                inRange));
    }

    /** {@link #denyShootingLane(PolicyWeights, double)} with the standard 8 m distance normalization. */
    public static ExpressionNode denyShootingLane(PolicyWeights weights) {
        return denyShootingLane(weights, 8.0);
    }

    /**
     * {@code RUSH_CLIMB} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code RUSH_CLIMB} score: an
     * endgame time ramp — flat zero outside {@code 0 < t <= 20} s, a
     * {@code 1 - t/15} progression arm at or under 15 s remaining, and a
     * {@code 1 - (t-15)/5} arm above it — vetoed by fitted climber and teleop
     * phase. Both arms are {@code Affine} nodes: the late arm's intercept
     * (4.0 in time-norm units) is what the node type was added for. Thresholds
     * mirror the engine's inclusivity ({@code <= 15} picks the early arm,
     * {@code <= 20} keeps the window).
     *
     * @param weights policy weights supplying the climb constants
     */
    public static ExpressionNode rushClimb(PolicyWeights weights) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double match = WorldStateTerminals.MATCH_LENGTH_SEC;

        ExpressionNode remaining = new ExpressionNode.TerminalNode(Terminal.REMAINING_TIME);

        ExpressionNode early = new ExpressionNode.Not(
                new ExpressionNode.Threshold(remaining, 15.0 / match, 0, 1));
        ExpressionNode progEarly =
                new ExpressionNode.Not(new ExpressionNode.Scale(remaining, match / 15.0));
        ExpressionNode progLate = new ExpressionNode.Affine(remaining, -match / 5.0, 4.0);

        ExpressionNode raw = new ExpressionNode.IfThenElse(early,
                new ExpressionNode.Affine(progEarly, w.climbScale15(), w.climbBase15()),
                new ExpressionNode.Affine(progLate, w.climbScale20(), w.climbBase20()));

        ExpressionNode role = new ExpressionNode.Product(List.of(
                new ExpressionNode.TerminalNode(Terminal.HAS_CLIMBER),
                new ExpressionNode.Not(new ExpressionNode.TerminalNode(Terminal.IS_AUTONOMOUS))));
        ExpressionNode inWindow = new ExpressionNode.Product(List.of(
                new ExpressionNode.Threshold(remaining, 0.0, 0, 1),
                new ExpressionNode.Not(new ExpressionNode.Threshold(remaining, 20.0 / match, 0, 1))));

        return new ExpressionNode.Product(List.of(raw, role, inWindow));
    }

    /**
     * {@code CHOKE_TRENCH} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code CHOKE_TRENCH} score:
     * the flat weight gated on opponent tracks present, teleop phase, and the
     * tracked opponent sitting in a trench band. Boundaries: the defender role
     * is assumed (grid runs {@code ADAPTIVE_COMPETITOR}, whose
     * post-processing leaves the choke value untouched), and the trench
     * terminal reads the nearest listed opponent while the engine reads the
     * tracked pose — kept equal in the grid, as production does.
     *
     * @param weights policy weights supplying the choke constant
     */
    public static ExpressionNode chokeTrench(PolicyWeights weights) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.chokeTrenchUtility()),
                new ExpressionNode.TerminalNode(Terminal.OPPONENT_OBSERVED),
                new ExpressionNode.Not(new ExpressionNode.TerminalNode(Terminal.IS_AUTONOMOUS)),
                new ExpressionNode.TerminalNode(Terminal.OPP_IN_TRENCH)));
    }

    /**
     * {@code SCREEN_FOR_ALLY} raw utility as an expression, escort override
     * included.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code SCREEN_FOR_ALLY}
     * score: the flat weight gated on opponent tracks, ally-side fuel at or
     * above 12, live home hub, and an ally in distress — maxed with the 0.99
     * emergency escort when the ally side holds 20 or more while self holds 8
     * or fewer (strictly below the escort cap only past it: equality keeps the
     * escort, matching the engine's {@code <= 8}). The ally-fuel thresholds
     * normalize over the shared 30-fuel norm. Assumes
     * {@code ballCapacity >= 1} for the self-hold gate like the other
     * conversions.
     *
     * @param weights      policy weights supplying the screen constants
     * @param ballCapacity hopper capacity the self-hold gate normalizes over
     */
    public static ExpressionNode screenForAlly(PolicyWeights weights, int ballCapacity) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode alliesHeld =
                new ExpressionNode.TerminalNode(Terminal.ALLIES_HELD_FUEL);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);
        ExpressionNode distress = new ExpressionNode.TerminalNode(Terminal.ALLY_IN_DISTRESS);
        ExpressionNode observed = new ExpressionNode.TerminalNode(Terminal.OPPONENT_OBSERVED);

        ExpressionNode base = new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.screenForAllyUtility()),
                observed,
                new ExpressionNode.Threshold(alliesHeld,
                        12.0 / WorldStateTerminals.FUEL_NORM, 0, 1, true),
                hubActive,
                distress));

        ExpressionNode escortCondition = new ExpressionNode.Product(List.of(
                distress,
                new ExpressionNode.Threshold(alliesHeld,
                        20.0 / WorldStateTerminals.FUEL_NORM, 0, 1, true),
                new ExpressionNode.Not(new ExpressionNode.Threshold(
                        new ExpressionNode.TerminalNode(Terminal.HELD_RATIO), 8.0 / cap, 0, 1)),
                observed,
                hubActive));
        ExpressionNode escort = new ExpressionNode.IfThenElse(escortCondition,
                new ExpressionNode.Constant(0.99), new ExpressionNode.Constant(0.0));

        return new ExpressionNode.Max(List.of(base, escort));
    }

    /**
     * {@code SHADOW_MIDLINE} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code SHADOW_MIDLINE} score:
     * the flat base weight gated on opponent tracks present. The
     * defender-role factor is assumed 1 — archetype stays engine scaffolding,
     * so the parity grid runs {@code ADAPTIVE_COMPETITOR}, whose
     * post-processing leaves the shadow value untouched.
     *
     * @param weights policy weights supplying the shadow constant
     */
    public static ExpressionNode shadowMidline(PolicyWeights weights) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.shadowMidlineBaseUtility()),
                new ExpressionNode.TerminalNode(Terminal.OPPONENT_OBSERVED)));
    }

    /**
     * {@code LEAD_INTERCEPT} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code LEAD_INTERCEPT} score:
     * the flat base weight gated on opponent tracks present. Same boundaries
     * as {@link #shadowMidline} — defender role assumed, grid runs
     * {@code ADAPTIVE_COMPETITOR} (and the clock stays out of the endgame
     * window, where G420 zeroes the intercept leg).
     *
     * @param weights policy weights supplying the intercept constant
     */
    public static ExpressionNode leadIntercept(PolicyWeights weights) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.interceptBaseUtility()),
                new ExpressionNode.TerminalNode(Terminal.OPPONENT_OBSERVED)));
    }

    /**
     * {@code POACH_OPPONENT_ZONE} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code POACH_OPPONENT_ZONE}
     * score: a flat weight vetoed by teleop phase, hopper headroom, the shift
     * window ({@code 0 < t <= poachWindow}, same sentinel shape as the other
     * window gates), opponent hub opening after the shift, held fuel below the
     * poach cap (strict — equality vetoes, matching the engine), and opponent
     * zone fuel presence. The fuel gate is tier-clean: the engine reads the
     * opponent count straight off the knowledge, honestly zero under the
     * observed tier. Assumes {@code ballCapacity >= 1} like the other
     * conversions.
     *
     * @param weights      policy weights supplying the poach constants
     * @param ballCapacity hopper capacity the gates normalize over
     */
    @SuppressWarnings("deprecation") // poach weights are deprecated but the formula is live
    public static ExpressionNode poachOpponentZone(PolicyWeights weights, int ballCapacity) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode timeUntilShift = new ExpressionNode.TerminalNode(Terminal.TIME_UNTIL_SHIFT);

        double windowRatio = w.poachOpponentZoneWindowSec() / WorldStateTerminals.SHIFT_NORM_SEC;
        ExpressionNode windowGate = new ExpressionNode.Product(List.of(
                new ExpressionNode.Threshold(timeUntilShift, 0.0, 0, 1),
                new ExpressionNode.Not(new ExpressionNode.Threshold(timeUntilShift, windowRatio, 0, 1))));
        ExpressionNode headroom =
                new ExpressionNode.Not(new ExpressionNode.Threshold(heldRatio, 1.0, 0, 1, true));
        ExpressionNode underCap = new ExpressionNode.Not(new ExpressionNode.Threshold(
                heldRatio, w.poachOpponentZoneMaxHeld() / cap, 0, 1, true));
        ExpressionNode fuelPresent =
                new ExpressionNode.Threshold(new ExpressionNode.TerminalNode(Terminal.OPPONENT_FUEL), 0.0, 0, 1);

        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(w.poachOpponentZoneUtility()),
                new ExpressionNode.Not(new ExpressionNode.TerminalNode(Terminal.IS_AUTONOMOUS)),
                headroom,
                windowGate,
                new ExpressionNode.TerminalNode(Terminal.OPP_HUB_NEXT),
                underCap,
                fuelPresent));
    }

    /**
     * {@code STOCKPILE_DEPOT} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code STOCKPILE_DEPOT} score:
     * the descending restock ramp over hopper fill, vetoed by hopper headroom
     * and an inactive hub. It reads no fuel count and no clock, so it is fully
     * tier-independent (all terminals are observed-tier). Assumes
     * {@code ballCapacity >= 1} like the other harvest conversions.
     *
     * @param weights      policy weights supplying the stockpile constants
     * @param ballCapacity hopper capacity the fill ratio normalizes over
     */
    public static ExpressionNode stockpileDepot(PolicyWeights weights, int ballCapacity) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);

        ExpressionNode raw = descendingRamp(heldRatio,
                w.stockpileDepotBase(), w.stockpileDepotScale());
        ExpressionNode headroom =
                new ExpressionNode.Not(new ExpressionNode.Threshold(heldRatio, 1.0, 0, 1, true));
        ExpressionNode hubInactive = new ExpressionNode.Not(hubActive);

        return new ExpressionNode.Product(List.of(raw, headroom, hubInactive));
    }

    /** {@code held / target} as a clamped node, or constant full progress for a zero target. */
    private static ExpressionNode progressFor(ExpressionNode heldRatio, double cap, int target) {
        if (target <= 0) {
            return new ExpressionNode.Constant(1.0);
        }
        return new ExpressionNode.Scale(heldRatio, cap / target);
    }

    /**
     * {@code SWEEP_ALLIANCE_ZONE} raw utility as an expression.
     *
     * <p>Tree form of {@code JevDecisionEngine}'s {@code SWEEP_ALLIANCE_ZONE}
     * score: the flat active value when the hub is live, else the inactive
     * descending ramp over hopper fill, vetoed by home-zone fuel presence and
     * hopper headroom. Unlike {@link #vacuumMidfield}, the fuel gate needs no
     * tier caveat — the engine reads the home count straight off the knowledge,
     * which is honestly zero under the observed tier, exactly what the
     * {@code ALLIANCE_FUEL} terminal evaluates to there. Assumes
     * {@code ballCapacity >= 1} (harvest-capacity factor is 1) like the other
     * harvest conversions.
     *
     * @param weights      policy weights supplying the sweep constants
     * @param ballCapacity hopper capacity the fill ratio normalizes over
     */
    public static ExpressionNode sweepAllianceZone(PolicyWeights weights, int ballCapacity) {
        PolicyWeights w = weights == null ? PolicyWeights.DEFAULT : weights;
        double cap = Math.max(1, ballCapacity);

        ExpressionNode heldRatio = new ExpressionNode.TerminalNode(Terminal.HELD_RATIO);
        ExpressionNode hubActive = new ExpressionNode.TerminalNode(Terminal.MY_HUB_ACTIVE);
        ExpressionNode homeFuel = new ExpressionNode.TerminalNode(Terminal.ALLIANCE_FUEL);

        ExpressionNode raw = new ExpressionNode.IfThenElse(hubActive,
                new ExpressionNode.Constant(w.sweepAllianceZoneActive()),
                descendingRamp(heldRatio,
                        w.sweepAllianceZoneInactiveBase(), w.sweepAllianceZoneInactiveScale()));

        ExpressionNode fuelPresent = new ExpressionNode.Threshold(homeFuel, 0.0, 0, 1);
        ExpressionNode headroom =
                new ExpressionNode.Not(new ExpressionNode.Threshold(heldRatio, 1.0, 0, 1, true));

        return new ExpressionNode.Product(List.of(raw, fuelPresent, headroom));
    }

    /**
     * Linear IAUS ramp {@code base + scale - scale * x} (engine
     * {@code ResponseCurve.linear(-scale, 0, base + scale)}, clamped to
     * {@code [0,1]}), factored without subtraction. Exact while
     * {@code base + scale <= 1}, which holds for the production weights.
     */
    private static ExpressionNode descendingRamp(ExpressionNode x, double base, double scale) {
        double top = base + scale;
        double k = top > 1e-9 ? scale / top : 0.0;
        return new ExpressionNode.Product(List.of(
                new ExpressionNode.Constant(top), new ExpressionNode.Not(new ExpressionNode.Scale(x, k))));
    }
}
