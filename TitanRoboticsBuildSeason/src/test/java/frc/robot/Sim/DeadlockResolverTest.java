package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Unit tests for the multi-robot deadlock detector/recovery helper. */
public class DeadlockResolverTest {

    private static final double DT = 0.02;

    private static DeadlockResolver.Resolution tick(DeadlockResolver r, boolean stalled, double dist, int n) {
        DeadlockResolver.Resolution last = null;
        for (int i = 0; i < n; i++) {
            last = r.update(stalled, dist, DT);
        }
        return last;
    }

    @Test
    public void testNoTriggerWhenMoving() {
        DeadlockResolver r = new DeadlockResolver(new java.util.Random(1));
        DeadlockResolver.Resolution res = tick(r, false, 0.5, 200);
        assertFalse(res.recovering(), "Free motion pressed against a peer must not count as deadlock");
        assertEquals(0, r.getRecoveryCount());
    }

    @Test
    public void testNoTriggerWhenPeerFar() {
        DeadlockResolver r = new DeadlockResolver(new java.util.Random(1));
        DeadlockResolver.Resolution res = tick(r, true, 5.0, 200);
        assertFalse(res.recovering(), "Stall with no peer nearby is not a deadlock");
        assertEquals(0, r.getRecoveryCount());
    }

    @Test
    public void testTriggersAfterSustainedStallPressed() {
        DeadlockResolver r = new DeadlockResolver(new java.util.Random(1));
        DeadlockResolver.Resolution res = tick(r, true, 0.6, 49);
        assertFalse(res.recovering(), "49 ticks (0.98s) must not trigger yet");
        res = r.update(true, 0.6, DT);
        assertTrue(res.recovering(), "50 ticks (1.0s) of stall-pressed must trigger recovery");
        assertEquals(DeadlockResolver.FORWARD_SCALE, res.forwardScale(), 1e-9);
        assertEquals(DeadlockResolver.JINK_SPEED, Math.abs(res.lateralJink()), 1e-9);
        assertEquals(1, r.getRecoveryCount());
    }

    @Test
    public void testRecoveryLengthAndCooldown() {
        DeadlockResolver r = new DeadlockResolver(new java.util.Random(1));
        tick(r, true, 0.6, 50);
        // 0.7s recovery = 35 ticks; the 35th tick ends recovery and starts cooldown.
        DeadlockResolver.Resolution res = tick(r, true, 0.6, 34);
        assertTrue(res.recovering(), "Recovery must persist for its full duration");
        res = r.update(true, 0.6, DT);
        assertFalse(res.recovering(), "Recovery must end and enter cooldown");
        res = tick(r, true, 0.6, 100);
        assertFalse(res.recovering(), "Cooldown must suppress immediate re-trigger");
        assertEquals(1, r.getRecoveryCount(), "Only one recovery so far");
        // After 2.0s cooldown (100 ticks) + 1.0s stall (50 ticks), triggers again.
        res = tick(r, true, 0.6, 50);
        assertTrue(res.recovering(), "Must re-trigger after cooldown + fresh stall window");
        assertEquals(2, r.getRecoveryCount());
    }

    @Test
    public void testJinkSignIsSeeded() {
        DeadlockResolver a = new DeadlockResolver(new java.util.Random(42));
        DeadlockResolver b = new DeadlockResolver(new java.util.Random(42));
        tick(a, true, 0.6, 50);
        tick(b, true, 0.6, 50);
        DeadlockResolver.Resolution ra = a.update(true, 0.6, DT);
        DeadlockResolver.Resolution rb = b.update(true, 0.6, DT);
        assertEquals(Math.signum(ra.lateralJink()), Math.signum(rb.lateralJink()),
                "Same seed must produce the same jink direction");
    }

    @Test
    public void testResetClearsState() {
        DeadlockResolver r = new DeadlockResolver(new java.util.Random(1));
        tick(r, true, 0.6, 60);
        assertEquals(1, r.getRecoveryCount());
        r.reset();
        DeadlockResolver.Resolution res = tick(r, true, 0.6, 10);
        assertFalse(res.recovering(), "Reset must clear an in-progress recovery");
    }
}
