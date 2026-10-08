package frc.robot.Intelligence;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;

/**
 * One agent's objective commitment (hysteresis latch).
 *
 * <p>The Jev utility matrix is recomputed every cycle and several objectives
 * sit within ~0.02 of each other, so an unlatched engine re-routes a loaded bot
 * every 20 ms - the observed "constantly moving before trying to shoot". This
 * latch keeps the incumbent objective until a challenger is decisively better
 * or the minimum hold has matured.
 *
 * <p>State lives here, on the agent that owns the robot, deliberately
 * <em>not</em> on {@link JevDecisionEngine}. The engine is documented as a
 * stateless System 2 evaluator; a static latch there made every agent share one
 * box, so one bot's decision leaked into another's (and across unit tests in
 * the same JVM).
 *
 * <p>Rules, in the order they are applied:
 * <ol>
 *   <li>no incumbent, or the same objective: adopt and hold,</li>
 *   <li>incumbent utility collapsed to 0 (its hub just went inactive): release
 *       immediately,</li>
 *   <li>challenger ahead by {@link JevDecisionEngine#COMMITMENT_DECISIVE_MARGIN}:
 *       switch at once,</li>
 *   <li>otherwise hold until
 *       {@link JevDecisionEngine#COMMITMENT_MIN_HOLD_SEC} has elapsed
 *       <em>and</em> the challenger leads by
 *       {@link JevDecisionEngine#COMMITMENT_MARGIN}.</li>
 * </ol>
 */
public final class ObjectiveCommitment {
    private StrategicObjective committed;
    private double sinceSeconds;

    /** Currently committed objective, or null before the first decision. */
    public StrategicObjective committed() {
        return committed;
    }

    /** Seconds the current commitment has been held. */
    public double heldSeconds() {
        return sinceSeconds;
    }

    /** Drops the commitment (match reset, new objective set). */
    public void reset() {
        committed = null;
        sinceSeconds = 0.0;
    }

    /**
     * Calculates the active dynamic action inertia bonus at {@code nowSeconds}.
     * Returns 0.0 when no objective is currently committed.
     */
    public double activeInertia(double nowSeconds) {
        return activeInertia(nowSeconds, PolicyWeights.getActive());
    }

    /**
     * Calculates the active dynamic action inertia bonus at {@code nowSeconds} using explicit weights.
     */
    public double activeInertia(double nowSeconds, PolicyWeights weights) {
        if (committed == null) {
            return 0.0;
        }
        PolicyWeights w = (weights != null) ? weights : PolicyWeights.getActive();
        double elapsed = Math.max(0.0, nowSeconds - sinceSeconds);
        return w.inertiaInitialBoost() * Math.exp(-elapsed / w.inertiaTimeConstantSec());
    }

    /**
     * Latches {@code candidate} against the current commitment and returns the
     * objective to pursue. Mutates only this agent's latch.
     */
    public StrategicObjective apply(StrategicObjective candidate,
            Map<StrategicObjective, Double> utilities) {
        return apply(candidate, utilities, PolicyWeights.getActive());
    }

    /**
     * Wall-clock entry point: delegates to {@link #applyAt} with the FPGA
     * timestamp, preserving the production path exactly.
     */
    public StrategicObjective apply(StrategicObjective candidate,
            Map<StrategicObjective, Double> utilities,
            PolicyWeights weights) {
        return applyAt(candidate, utilities, weights, Timer.getFPGATimestamp());
    }

    /**
     * Sim-time entry point for fast-forward runners, whose wall clock does not
     * advance between ticks. Pure function of {@code nowSeconds}: identical
     * rules to the wall-clock path, driven by the caller's match clock.
     *
     * @param nowSeconds caller-owned time base (sim elapsed, seconds)
     */
    public StrategicObjective applyAt(StrategicObjective candidate,
            Map<StrategicObjective, Double> utilities,
            PolicyWeights weights,
            double nowSeconds) {
        if (candidate == null) {
            return committed;
        }
        if (committed == null) {
            committed = candidate;
            sinceSeconds = nowSeconds;
            return committed;
        }

        StrategicObjective resolved = JevDecisionEngine.resolveCommittedObjective(
                candidate, utilities, committed, sinceSeconds, nowSeconds, weights);
        if (resolved != committed) {
            committed = resolved;
            sinceSeconds = nowSeconds;
        }
        return committed;
    }
}

