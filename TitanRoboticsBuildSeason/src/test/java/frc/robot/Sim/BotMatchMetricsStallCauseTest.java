package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Sim.BotMatchMetrics.StallCause;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Phase 0 stall-taxonomy cover: every stalled sample attributes to exactly one
 * cause, flowing samples attribute nothing, and the legacy 5-arg overload
 * keeps reporting the totals it always did.
 */
class BotMatchMetricsStallCauseTest {

    private static final double DT = 0.02;
    private BotMatchMetrics metrics;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        metrics = new BotMatchMetrics();
    }

    private static Pose2d pose(double x) {
        return new Pose2d(x, 4.0, new Rotation2d());
    }

    @Test
    void stalledSamplesAttributeToTheGivenCause() {
        for (int i = 0; i < 10; i++) {
            metrics.sample(pose(i * 0.001), true, true, true, i * DT,
                    StallCause.DEADLOCK_TRENCH);
        }
        assertEquals(10 * DT, metrics.getStallSecByCause(StallCause.DEADLOCK_TRENCH), 1e-9);
        assertEquals(0.0, metrics.getStallSecByCause(StallCause.DEADLOCK_OPEN), 1e-9);
        assertEquals(0.0, metrics.getStallSecByCause(StallCause.STALLED_OTHER), 1e-9);
        // Legacy totals are unchanged by the cause tag.
        assertEquals(10 * DT, metrics.getMaxContiguousStallSec(), 1e-9);
    }

    @Test
    void nullCauseWhileStalledFallsBackToUnattributed() {
        metrics.sample(pose(0), true, false, true, 0.0, null);
        assertEquals(DT, metrics.getStallSecByCause(StallCause.STALLED_OTHER), 1e-9);
    }

    @Test
    void flowingSamplesAttributeNothing() {
        metrics.sample(pose(0), false, false, true, 0.0, StallCause.TRENCH_YIELD);
        for (StallCause cause : StallCause.values()) {
            assertEquals(0.0, metrics.getStallSecByCause(cause), 1e-9);
            assertEquals(0, metrics.getRecoveryEventsByCause(cause));
        }
        assertEquals(0.0, metrics.getMaxContiguousStallSec(), 1e-9);
    }

    @Test
    void recoveryRisingEdgeAttributesToTheCauseAtEventTick() {
        metrics.sample(pose(0), true, true, true, 0.0, StallCause.TARGET_UNREACHABLE);
        // Still recovering: no second edge.
        metrics.sample(pose(0), true, true, true, DT, StallCause.TARGET_UNREACHABLE);
        // Edge cleared, then a different cause fires.
        metrics.sample(pose(0), false, false, true, 2 * DT, null);
        metrics.sample(pose(0), true, true, true, 3 * DT, StallCause.DEADLOCK_OPEN);

        assertEquals(2, metrics.getRecoveryEventCount());
        assertEquals(1, metrics.getRecoveryEventsByCause(StallCause.TARGET_UNREACHABLE));
        assertEquals(1, metrics.getRecoveryEventsByCause(StallCause.DEADLOCK_OPEN));
        assertEquals(0, metrics.getRecoveryEventsByCause(StallCause.TRENCH_YIELD));
    }

    @Test
    void legacyOverloadKeepsWorking() {
        metrics.sample(pose(0), true, true, true, 0.0);
        assertEquals(DT, metrics.getMaxContiguousStallSec(), 1e-9);
        assertEquals(1, metrics.getRecoveryEventCount());
        assertEquals(DT, metrics.getStallSecByCause(StallCause.STALLED_OTHER), 1e-9);
    }

    @Test
    void inactiveSamplesAttributeNothing() {
        metrics.sample(pose(0), true, true, false, 0.0, StallCause.DEADLOCK_TRENCH);
        assertEquals(0.0, metrics.getStallSecByCause(StallCause.DEADLOCK_TRENCH), 1e-9);
        assertEquals(0, metrics.getRecoveryEventCount());
    }

    @Test
    void resetClearsCauseCounters() {
        metrics.sample(pose(0), true, true, true, 0.0, StallCause.TRENCH_YIELD);
        metrics.reset();
        for (StallCause cause : StallCause.values()) {
            assertEquals(0.0, metrics.getStallSecByCause(cause), 1e-9);
            assertEquals(0, metrics.getRecoveryEventsByCause(cause));
        }
    }
}
