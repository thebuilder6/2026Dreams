package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import frc.robot.Sim.AIRobotInstance.RecoveryState;
import frc.robot.Sim.BotMatchMetrics.StallCause;

/**
 * Pins the stall-cause taxonomy, including the {@link StallCause#STATIC_UNSTICK}
 * bucket added with the peer-independent unstick path.
 *
 * <p><b>The gap this guards.</b> The unstick recovery was counted in
 * {@code inRecovery} from the day it landed, but had no branch in the attribution
 * chain, so every sample it produced fell through to
 * {@link #classifyUnattributedStall}'s {@code STALLED_OTHER} fallback. The newest
 * recovery was therefore the only one the headless report could not distinguish —
 * and the report is the instrument used to judge whether the recovery helps, so
 * the bucket that most needed measuring was the one being absorbed into
 * "unattributed".
 *
 * <p>The chain is ordered to mirror the drive pipeline, not to group by
 * similarity: the last correction applied to the chassis is the one that explains
 * the stall. That is why the unreachable-target escape outranks the unstick, which
 * outranks the peer corrections.
 */
class StallCauseTaxonomyTest {

    /** All flags off: a bot that is flowing normally. */
    private static RecoveryState flowing() {
        return new RecoveryState(false, false, false, false, false, false);
    }

    @Test
    void flowingAttributesNothing() {
        assertNull(AIRobotInstance.classifyStallCause(flowing(), 5.0, -1.0, 0.0),
                "a bot that is neither stalled nor recovering must attribute nothing, "
                        + "otherwise the report counts normal driving as stall time");
    }

    // ---------------------------------------------------------------------
    // STATIC_UNSTICK: the branch that was missing
    // ---------------------------------------------------------------------

    @Test
    void activeUnstickIsAttributedToItself() {
        RecoveryState s = new RecoveryState(true, false, false, false, false, true);
        assertEquals(StallCause.STATIC_UNSTICK,
                AIRobotInstance.classifyStallCause(s, 5.0, 5.0, 5.0),
                "an active unstick must have its own bucket, not STALLED_OTHER");
    }

    @Test
    void unstickIsAttributedEvenWithoutAnUnattributedStallFlag() {
        // The unstick implies a stall in practice, but the taxonomy must not
        // depend on that coupling: if the two ever diverge the recovery would
        // silently vanish from the report again.
        RecoveryState s = new RecoveryState(true, false, false, false, false, false);
        assertEquals(StallCause.STATIC_UNSTICK,
                AIRobotInstance.classifyStallCause(s, 5.0, 5.0, 5.0),
                "the recovery that is executing owns the tick regardless of the "
                        + "stall flag");
    }

    @Test
    void unstickInATrenchIsNotReportedAsADeadlock() {
        // The unstick and the peer corrections can be flagged in the same tick
        // (the unstick is applied after them), and the taxonomy must report the
        // one the robot actually executed.
        RecoveryState s = new RecoveryState(true, false, true, true, false, true);
        assertEquals(StallCause.STATIC_UNSTICK,
                AIRobotInstance.classifyStallCause(s, 5.0, 5.0, 5.0));
    }

    // ---------------------------------------------------------------------
    // The rest of the chain, and the priority that mirrors the pipeline
    // ---------------------------------------------------------------------

    @Test
    void unreachableTargetOutranksUnstick() {
        // The target watchdog runs after the unstick in the pipeline and overwrites
        // it, so it is the correction that actually moved the robot.
        RecoveryState s = new RecoveryState(true, true, false, false, false, true);
        assertEquals(StallCause.TARGET_UNREACHABLE,
                AIRobotInstance.classifyStallCause(s, 5.0, 5.0, 5.0));
    }

    @Test
    void deadlockIsStillSplitByCorridor() {
        assertEquals(StallCause.DEADLOCK_TRENCH,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, true, true, false, true), 5.0, 5.0, 5.0));
        assertEquals(StallCause.DEADLOCK_OPEN,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, true, false, false, true), 5.0, 5.0, 5.0));
    }

    @Test
    void unstickOutranksEveryPeerCorrection() {
        assertEquals(StallCause.STATIC_UNSTICK,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(true, false, false, false, true, true), 5.0, 5.0, 5.0),
                "the unstick is applied after the peer arbitration, so it wins");
    }

    @Test
    void trenchYieldIsReportedWhenNothingElseFires() {
        assertEquals(StallCause.TRENCH_YIELD,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, false, true, true, true), 5.0, 5.0, 5.0));
    }

    // ---------------------------------------------------------------------
    // Delegation to the unattributed buckets is unchanged
    // ---------------------------------------------------------------------

    @Test
    void bareStallStillDelegatesToTheUnattributedBuckets() {
        assertEquals(StallCause.STALLED_ARRIVED,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, false, false, false, true), 0.3, 5.0, 5.0));
        assertEquals(StallCause.STALLED_CHURN,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, false, false, false, true), 5.0, 1.0, 4.0));
        assertEquals(StallCause.STALLED_OTHER,
                AIRobotInstance.classifyStallCause(
                        new RecoveryState(false, false, false, false, false, true), 5.0, 5.0, 5.0));
    }

    /**
     * Every cause must be reachable. A taxonomy entry nothing can produce is a
     * silent hole, which is exactly the class of defect this file exists to stop.
     */
    @Test
    void everyCauseIsReachable() {
        java.util.EnumSet<StallCause> seen = java.util.EnumSet.noneOf(StallCause.class);
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(true, false, false, false, false, true), 5.0, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, true, false, false, false, true), 5.0, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, true, true, false, true), 5.0, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, true, false, false, true), 5.0, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, false, true, true, true), 5.0, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, false, false, false, true), 0.3, 5.0, 5.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, false, false, false, true), 5.0, 1.0, 4.0));
        seen.add(AIRobotInstance.classifyStallCause(
                new RecoveryState(false, false, false, false, false, true), 5.0, 5.0, 5.0));

        assertEquals(java.util.EnumSet.allOf(StallCause.class), seen,
                "every StallCause must be producible by classifyStallCause; an "
                        + "unreachable bucket is a silent hole in the report");
    }
}
