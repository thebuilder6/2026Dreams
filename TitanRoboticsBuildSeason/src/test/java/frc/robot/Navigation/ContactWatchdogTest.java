package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Unified tests for ContactWatchdog (merged from LegalPinningWatchdogTest +
 * DeadlockResolverTest): G418 pin timing, deadlock yield-and-jink, and
 * arbitrate priority.
 */
public class ContactWatchdogTest {

    private static final double DT = 0.02;

    private ContactWatchdog watchdog;

    @BeforeEach
    public void setup() {
        watchdog = new ContactWatchdog(new java.util.Random(1));
        watchdog.reset();
    }

    // ── G418 pin timing ──────────────────────────────────────────────

    private void contactTick(Pose2d robotPose, Pose2d oppPose) {
        watchdog.update(
                robotPose,
                new ChassisSpeeds(),
                new ChassisSpeeds(2.0, 0.0, 0.0),
                0.0,
                0.0,
                30.0,
                0.5,
                oppPose,
                DT);
    }

    private void clearTick(Pose2d robotPose, Pose2d oppPose) {
        watchdog.update(
                robotPose,
                new ChassisSpeeds(),
                new ChassisSpeeds(),
                0.0,
                0.0,
                0.0,
                Double.MAX_VALUE,
                oppPose,
                DT);
    }

    @Test
    public void testPinWarningAtThreshold() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        for (int i = 0; i < 50; i++) {
            contactTick(robotPose, oppPose);
        }
        assertFalse(watchdog.isWarningActive());
        assertFalse(watchdog.isForcedBackoffActive());

        for (int i = 0; i < 41; i++) {
            contactTick(robotPose, oppPose);
        }
        assertTrue(watchdog.isWarningActive(), "Driver warning should trigger at 1.8s contact");
        assertFalse(watchdog.isForcedBackoffActive());
    }

    @Test
    public void testForcedBackoffAtMaxPinDuration() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        for (int i = 0; i < 125; i++) {
            contactTick(robotPose, oppPose);
        }

        assertTrue(watchdog.isForcedBackoffActive(), "Forced backoff must trigger at 2.4s max contact hold");
        assertFalse(watchdog.isWarningActive());

        Pose2d backoff = watchdog.getBackOffTarget(robotPose, oppPose);
        assertNotNull(backoff);
        double separation = backoff.getTranslation().getDistance(robotPose.getTranslation());
        assertTrue(separation >= ContactWatchdog.BACKOFF_DISTANCE_METERS,
                "Backoff target must enforce >= 0.9144m clearance");

        Pose2d backedOffPose = new Pose2d(robotPose.getX() - 1.0, robotPose.getY(), new Rotation2d());
        for (int i = 0; i < 155; i++) {
            clearTick(backedOffPose, oppPose);
        }

        assertFalse(watchdog.isForcedBackoffActive(),
                "Forced backoff should clear after 3.0s cooldown and 3-foot backoff");
    }

    @Test
    public void testContactDecayWhenSeparatedEarly() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        for (int i = 0; i < 50; i++) {
            contactTick(robotPose, oppPose);
        }
        assertEquals(1.0, watchdog.getPinDuration(), 0.05);

        for (int i = 0; i < 25; i++) {
            clearTick(robotPose, oppPose);
        }
        assertTrue(watchdog.getPinDuration() < 0.20, "Pin duration should decay rapidly when contact breaks");
    }

    // ── Deadlock yield-and-jink ──────────────────────────────────────

    private static ContactWatchdog.Resolution tick(ContactWatchdog r, boolean stalled, double dist, int n) {
        ContactWatchdog.Resolution last = null;
        for (int i = 0; i < n; i++) {
            last = r.updateDeadlockOnly(stalled, dist, DT);
        }
        return last;
    }

    @Test
    public void testNoTriggerWhenMoving() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        ContactWatchdog.Resolution res = tick(r, false, 0.5, 200);
        assertFalse(res.recovering(), "Free motion pressed against a peer must not count as deadlock");
        assertEquals(0, r.getRecoveryCount());
    }

    @Test
    public void testNoTriggerWhenPeerFar() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        ContactWatchdog.Resolution res = tick(r, true, 5.0, 200);
        assertFalse(res.recovering(), "Stall with no peer nearby is not a deadlock");
        assertEquals(0, r.getRecoveryCount());
    }

    @Test
    public void testTriggersAfterSustainedStallPressed() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        ContactWatchdog.Resolution res = tick(r, true, 0.6, 49);
        assertFalse(res.recovering(), "49 ticks (0.98s) must not trigger yet");
        res = r.updateDeadlockOnly(true, 0.6, DT);
        assertTrue(res.recovering(), "50 ticks (1.0s) of stall-pressed must trigger recovery");
        assertEquals(ContactWatchdog.FORWARD_SCALE, res.forwardScale(), 1e-9);
        assertEquals(ContactWatchdog.JINK_SPEED, Math.abs(res.lateralJink()), 1e-9);
        assertEquals(1, r.getRecoveryCount());
    }

    @Test
    public void testRecoveryLengthAndCooldown() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        tick(r, true, 0.6, 50);
        ContactWatchdog.Resolution res = tick(r, true, 0.6, 34);
        assertTrue(res.recovering(), "Recovery must persist for its full duration");
        res = r.updateDeadlockOnly(true, 0.6, DT);
        assertFalse(res.recovering(), "Recovery must end and enter cooldown");
        res = tick(r, true, 0.6, 100);
        assertFalse(res.recovering(), "Cooldown must suppress immediate re-trigger");
        assertEquals(1, r.getRecoveryCount(), "Only one recovery so far");
        res = tick(r, true, 0.6, 50);
        assertTrue(res.recovering(), "Must re-trigger after cooldown + fresh stall window");
        assertEquals(2, r.getRecoveryCount());
    }

    @Test
    public void testJinkSignIsSeeded() {
        ContactWatchdog a = new ContactWatchdog(new java.util.Random(42));
        ContactWatchdog b = new ContactWatchdog(new java.util.Random(42));
        tick(a, true, 0.6, 50);
        tick(b, true, 0.6, 50);
        ContactWatchdog.Resolution ra = a.updateDeadlockOnly(true, 0.6, DT);
        ContactWatchdog.Resolution rb = b.updateDeadlockOnly(true, 0.6, DT);
        assertEquals(Math.signum(ra.lateralJink()), Math.signum(rb.lateralJink()),
                "Same seed must produce the same jink direction");
    }

    @Test
    public void testTrenchTriggersFasterWithReverse() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        ContactWatchdog.Resolution res = tickTrench(r, true, 0.6, 24);
        assertFalse(res.recovering(), "24 ticks (0.48s) must not trigger yet in trench");
        res = r.updateDeadlockOnly(true, 0.6, DT, true);
        assertTrue(res.recovering(), "25 ticks (0.5s) of trench stall-pressed must trigger yield");
        assertEquals(ContactWatchdog.TRENCH_FORWARD_SCALE, res.forwardScale(), 1e-9);
        assertEquals(0.0, res.lateralJink(), 1e-9);
        assertEquals(1, r.getRecoveryCount());
    }

    @Test
    public void testTrenchArbitrateReversesWithoutLateral() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        tickTrench(r, true, 0.6, 25);
        // Trench pose: Blue bottom corridor (X 3.20-6.10, Y <= 1.28).
        Pose2d trenchPose = new Pose2d(4.5, 0.65, new Rotation2d());
        ChassisSpeeds out = r.arbitrate(new ChassisSpeeds(2.0, 0.0, 0.0), trenchPose, null);
        assertTrue(out.vxMetersPerSecond < -0.5,
                "Trench recovery must reverse out, not jink laterally into the truss");
        assertEquals(0.0, out.vyMetersPerSecond, 1e-9);
    }

    @Test
    public void testTrenchEscalatesConsecutiveRecoveries() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        int d1 = ticksUntilRecoveringEnds(trenchTriggered(r));
        assertTrue(d1 >= 35 && d1 <= 60, "First trench recovery should last 0.7-1.2s, was " + d1 + " ticks");
        assertEquals(1, r.getTrenchEscalation());
        int d2 = ticksUntilRecoveringEnds(retriggerAfterCooldown(r));
        assertEquals(2, r.getTrenchEscalation());
        assertTrue(d2 >= 39 && d2 <= 70, "Escalated trench recovery should last 0.9-1.4s, was " + d2 + " ticks");
        // Stall clearing (after cooldown drains) resets escalation.
        tick(r, false, 5.0, 200);
        assertEquals(0, r.getTrenchEscalation());
    }

    @Test
    public void testTrenchCoolingFlagTracksCooldown() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        tickTrench(r, true, 0.6, 25);
        assertFalse(r.isDeadlockCooling(), "No cooldown while recovering");
        ticksUntilRecoveringEnds(r);
        assertTrue(r.isDeadlockCooling(), "Cooldown must be active right after recovery ends");
        tick(r, false, 5.0, 200);
        assertFalse(r.isDeadlockCooling(), "Cooldown must expire after ~2-3s idle");
    }

    private static ContactWatchdog trenchTriggered(ContactWatchdog r) {
        for (int i = 0; i < 40; i++) {
            ContactWatchdog.Resolution res = r.updateDeadlockOnly(true, 0.6, DT, true);
            if (res.recovering()) return r;
        }
        fail("Trench recovery never triggered");
        return r;
    }

    private static int ticksUntilRecoveringEnds(ContactWatchdog r) {
        int n = 0;
        while (n < 120) {
            ContactWatchdog.Resolution res = r.updateDeadlockOnly(true, 0.6, DT, true);
            n++;
            if (!res.recovering()) return n;
        }
        fail("Trench recovery never ended");
        return n;
    }

    private static ContactWatchdog retriggerAfterCooldown(ContactWatchdog r) {
        for (int i = 0; i < 400; i++) {
            ContactWatchdog.Resolution res = r.updateDeadlockOnly(true, 0.6, DT, true);
            if (res.recovering()) return r;
        }
        fail("Second trench recovery never triggered");
        return r;
    }
    private static ContactWatchdog.Resolution tickTrench(ContactWatchdog r, boolean stalled, double dist, int n) {
        ContactWatchdog.Resolution last = null;
        for (int i = 0; i < n; i++) {
            last = r.updateDeadlockOnly(stalled, dist, DT, true);
        }
        return last;
    }

    @Test
    public void testResetClearsState() {
        ContactWatchdog r = new ContactWatchdog(new java.util.Random(1));
        tick(r, true, 0.6, 60);
        assertEquals(1, r.getRecoveryCount());
        r.reset();
        ContactWatchdog.Resolution res = tick(r, true, 0.6, 10);
        assertFalse(res.recovering(), "Reset must clear an in-progress recovery");
    }

    // ── Arbitrate priority ───────────────────────────────────────────

    @Test
    public void testArbitrateForcedBackoffOverridesCommanded() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());
        for (int i = 0; i < 125; i++) {
            contactTick(robotPose, oppPose);
        }
        assertTrue(watchdog.isForcedBackoffActive());

        ChassisSpeeds out = watchdog.arbitrate(new ChassisSpeeds(2.0, 0.0, 0.0), robotPose, oppPose);
        assertTrue(Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond) > 0.5,
                "Forced backoff must command retreat away from the pin");
        assertEquals(0.0, out.omegaRadiansPerSecond, 1e-9);
    }
}
