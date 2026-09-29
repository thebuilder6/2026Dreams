package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Intelligence.FuelTargetMemory.ScoredTarget;

/**
 * Pins per-agent fuel-target stickiness.
 *
 * <p>The defect: without hysteresis, the cluster-weighted scent is a function of
 * the robot's instantaneous heading (via the {@code cos(heading - delta)}
 * alignment bonus), so two comparable pieces trade places whenever the robot
 * turns its head a few degrees. That oscillating target resets
 * {@code TargetProgressWatchdog}'s no-progress window on every change, so the
 * give-up timer never expires and the bot never abandons fuel it cannot reach
 * ({@code STALLED_CHURN}).
 *
 * <p>The failure mode this class must NOT repeat is the one
 * {@link ObjectiveCommitment} documents: state on the shared engine singleton
 * leaks one bot's decision into another's. {@link #memoriesArePerAgent} is the
 * guard for that.
 */
class FuelTargetMemoryTest {

    private static final double NEAR_A_X = 7.00;
    private static final double NEAR_B_X = 7.60;
    private static final double LANE_Y = 4.00;

    private FuelTargetMemory memory;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        memory = new FuelTargetMemory();
    }

    private static ScoredTarget at(double x, double scent) {
        return new ScoredTarget(new Translation2d(x, LANE_Y), scent);
    }

    private static ScoredTarget at(double x, double y, double scent) {
        return new ScoredTarget(new Translation2d(x, y), scent);
    }

    // ---------------------------------------------------------------------
    // Hysteresis
    // ---------------------------------------------------------------------

    /**
     * The core case. A challenger 10% better must not steal, because in the field
     * a 10% scent difference is well inside the noise the heading bonus injects.
     */
    @Test
    void retainsIncumbentOnNearTie() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 0.95)));

        // Challenger improves by 10%: still the incumbent.
        assertEquals(NEAR_A_X, memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 1.10)))
                .getX(), 1e-9,
                "a 10% improvement is inside heading-bonus noise and must not steal");

        // Challenger improves by 30%: decisively better, takes over.
        assertEquals(NEAR_B_X, memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 1.30)))
                .getX(), 1e-9,
                "a decisive improvement must preempt the latch");
    }

    /**
     * An exactly-equal challenger must never steal. Ties have to resolve toward
     * stability; if they resolved by list order the oscillation would come back
     * whenever two pieces scored identically, which is precisely the
     * equal-density case the selector is designed around.
     */
    @Test
    void exactTieNeverSteals() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 1.00)));

        for (int i = 0; i < 20; i++) {
            Translation2d held = memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 1.00)));
            assertEquals(NEAR_A_X, held.getX(), 1e-9,
                    "identical scents must not alternate across cycles");
        }
    }

    /**
     * The actual oscillation. Two pieces, and a heading term that makes the
     * leader alternate every cycle. Without the latch the winner flips; with it
     * the choice is stable, which is what stops the watchdog window resetting.
     */
    @Test
    void alternatingHeadingNoiseDoesNotChurnTheTarget() {
        // 20 cycles where the raw winner alternates between the two pieces.
        boolean flipped = false;
        Translation2d first = null;

        for (int i = 0; i < 20; i++) {
            double aScent = (i % 2 == 0) ? 1.00 : 0.98;
            double bScent = (i % 2 == 0) ? 0.99 : 1.01;

            // What the unlatched selector would pick this cycle.
            double rawBest = Math.max(aScent, bScent);
            double rawWinnerX = (rawBest == aScent) ? NEAR_A_X : NEAR_B_X;
            if (first == null) {
                first = new Translation2d(rawWinnerX, LANE_Y);
            } else if (Math.abs(first.getX() - rawWinnerX) > 1e-9) {
                flipped = true;
            }

            Translation2d held = memory.resolve(List.of(at(NEAR_A_X, aScent), at(NEAR_B_X, bScent)));
            assertEquals(first.getX(), held.getX(), 1e-9,
                    "the latched target must survive heading-driven raw flips");
        }

        assertTrue(flipped, "the premise must hold: the raw winner did alternate, "
                + "otherwise this test proves nothing");
    }

    // ---------------------------------------------------------------------
    // Release
    // ---------------------------------------------------------------------

    /**
     * The piece left the field (collected by this agent or another). The latch
     * must drop it and adopt what is left, or the bot drives to a phantom.
     */
    @Test
    void releasesWhenLatchedPieceDisappears() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 0.50)));
        assertEquals(NEAR_A_X, memory.latched().getX(), 1e-9);

        Translation2d next = memory.resolve(List.of(at(NEAR_B_X, 0.50)));

        assertEquals(NEAR_B_X, next.getX(), 1e-9,
                "a collected piece must not stay latched");
    }

    /**
     * The piece was blacklisted by the watchdog (unreachable this match) and so
     * no longer appears as a candidate. Same release path as disappearance, and
     * this is the case that matters: keeping the latch would re-target fuel the
     * watchdog already gave up on.
     */
    @Test
    void releasesWhenLatchedPieceIsBlocked() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00), at(NEAR_B_X, 0.50)));

        // Watchdog excludes the unreachable piece; only the other remains.
        Translation2d next = memory.resolve(List.of(at(NEAR_B_X, 0.50)));

        assertEquals(NEAR_B_X, next.getX(), 1e-9,
                "a blacklisted piece must be released so a reachable one is taken");
    }

    @Test
    void emptyCandidateListClearsTheLatch() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00)));
        assertNull(memory.resolve(List.of()), "no candidates means nothing to hold");
        assertNull(memory.latched());
        assertNull(memory.resolve(null), "a null list is treated as no candidates");
        assertNull(memory.latched());
    }

    // ---------------------------------------------------------------------
    // Identity and lifetime
    // ---------------------------------------------------------------------

    /**
     * The regression guard for the bug {@link ObjectiveCommitment} documents. An
     * earlier latch lived in a {@code static} map on the shared singleton, which
     * put every caller into one box; one bot's decision leaked into another's and
     * six tests failed. Two agents here must be fully independent.
     */
    @Test
    void memoriesArePerAgent() {
        FuelTargetMemory a = new FuelTargetMemory();
        FuelTargetMemory b = new FuelTargetMemory();

        a.resolve(List.of(at(NEAR_A_X, 1.00)));
        assertNull(b.latched(), "agent B must not inherit agent A's latch");

        b.resolve(List.of(at(NEAR_B_X, 1.00)));
        assertNotEquals(a.latched().getX(), b.latched().getX(), 1e-9,
                "two agents must be able to hold different pieces simultaneously");

        // Resolving for B must not disturb A's hold.
        b.resolve(List.of(at(NEAR_B_X, 9.00)));
        assertEquals(NEAR_A_X, a.latched().getX(), 1e-9,
                "one agent's decision must not move another agent's target");
    }

    /**
     * Holding one piece for a long time must not read as a fresh commitment each
     * cycle, or a long pursuit would look like continuous churn to any consumer
     * comparing ages.
     */
    @Test
    void heldSecondsDoesNotRestartWhileHoldingTheSamePiece() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00)));
        assertTrue(memory.heldSeconds() >= 0.0, "a held latch reports a real age");
        assertEquals(-1.0, new FuelTargetMemory().heldSeconds(), 1e-9,
                "an empty memory reports -1, matching the watchdog's convention");

        // Re-resolving the same piece must not reset the age.
        for (int i = 0; i < 5; i++) {
            memory.resolve(List.of(at(NEAR_A_X, 1.00 + i * 0.01)));
        }
        assertTrue(memory.heldSeconds() < 1.0,
                "re-affirming the same piece must not reset its age, got "
                        + memory.heldSeconds());
    }

    @Test
    void resetClearsTheLatch() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00)));
        memory.reset();
        assertNull(memory.latched(), "match reset must drop the latch");
    }

    // ---------------------------------------------------------------------
    // Matching mechanics
    // ---------------------------------------------------------------------

    /**
     * The latch is matched by position, not identity, so a piece whose reported
     * position jitters slightly between cycles must still be recognised as the
     * same piece rather than treated as vanished and re-adopted.
     */
    @Test
    void smallPositionJitterStillMatchesTheIncumbent() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00)));

        // 0.2 m of reporting jitter on the same piece: a worse scent, but the
        // same physical piece. The latch must survive it (that is the whole
        // point of the stick radius), and the returned position must be the
        // piece's CURRENT reported location, not the stale latched one -- the bot
        // should track where the fuel actually is.
        Translation2d held = memory.resolve(List.of(at(NEAR_A_X + 0.20, 0.60)));

        assertNotEquals(null, held, "the jittered piece must still be held");
        assertEquals(NEAR_A_X + 0.20, held.getX(), 1e-9,
                "a 0.2 m jitter is the same piece, and the fresh position must win");
        assertEquals(NEAR_A_X + 0.20, memory.latched().getX(), 1e-9,
                "the latch must follow the piece, not the stale first reading");
    }

    /**
     * When two candidates both fall inside the stick radius of the latch, the
     * nearer one wins so the match does not depend on list order.
     */
    @Test
    void nearestCandidateWinsWhenTwoFallInsideStickRadius() {
        memory.resolve(List.of(at(NEAR_A_X, 1.00)));

        List<ScoredTarget> crowded = new ArrayList<>();
        crowded.add(at(NEAR_A_X + 0.25, 1.00));
        crowded.add(at(NEAR_A_X + 0.05, 1.00));

        Translation2d held = memory.resolve(crowded);
        assertEquals(NEAR_A_X + 0.05, held.getX(), 1e-9,
                "the nearest in-radius candidate wins, independent of list order");
    }

    @Test
    void nullEntriesInTheCandidateListAreIgnored() {
        memory.resolve(Arrays.asList(at(NEAR_A_X, 1.00), null, at(NEAR_B_X, 0.10)));
        assertEquals(NEAR_A_X, memory.latched().getX(), 1e-9);

        Translation2d held = memory.resolve(new ArrayList<>(List.of()));
        assertNull(held);
    }
}
