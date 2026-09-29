package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Sim.HubSchedule;
import frc.robot.Sim.HubSchedule.Phase;

/** Unit coverage for the official 6.4 / 6.4.1 hub schedule (Table 6-2, 6-3). */
public class HubScheduleTest {

    private final double[] fakeClock = {1000.0};

    @BeforeEach
    public void setup() {
        HubSchedule.reset();
        HubSchedule.setClockForTests(() -> fakeClock[0]);
    }

    @AfterEach
    public void teardown() {
        HubSchedule.setClockForTests(null);
        HubSchedule.reset();
    }

    // ------------------------------------------------------------------
    // Next-shift derivation. WorldState carries these so the decision layer can
    // tell a shuttle (aims at the opponent's end) from a trip to its own hub.
    // ------------------------------------------------------------------

    @Test
    public void nextPhaseCyclesThroughTheShifts() {
        assertEquals(Phase.SHIFT2, HubSchedule.nextPhase(Phase.SHIFT1));
        assertEquals(Phase.SHIFT3, HubSchedule.nextPhase(Phase.SHIFT2));
        assertEquals(Phase.SHIFT4, HubSchedule.nextPhase(Phase.SHIFT3));
        assertEquals(Phase.ENDGAME, HubSchedule.nextPhase(Phase.SHIFT4));
    }

    @Test
    public void nextPhaseIsSelfForBothActivePhases() {
        // Once both hubs are live for a stretch no later phase turns one off, so
        // there is no meaningful "next".
        assertEquals(Phase.TRANSITION, HubSchedule.nextPhase(Phase.TRANSITION));
        assertEquals(Phase.ENDGAME, HubSchedule.nextPhase(Phase.ENDGAME));
        assertEquals(Phase.AUTO, HubSchedule.nextPhase(Phase.AUTO));
        assertEquals(Phase.DONE, HubSchedule.nextPhase(Phase.DONE));
    }

    /**
     * Pinned against the shift table directly rather than against a hand-written
     * expectation, because the previous version asserted current-phase semantics
     * on an after-next-phase function and so contradicted itself.
     *
     * <p>With seed 'R', Red sits out first, so Blue is live in SHIFT1 and SHIFT3
     * and Red in SHIFT2 and SHIFT4. "After the next phase" therefore alternates:
     * asking what is live after SHIFT1 means asking about SHIFT2, where Red is on.
     */
    @Test
    public void afterNextPhaseAlternatesTheActiveHubAcrossShifts() {
        // SHIFT4 is included deliberately. It is the only phase whose successor
        // is NOT the next shift: per Table 6-2 the match runs SHIFT1..SHIFT4 then
        // END GAME, so nextPhase(SHIFT4) is ENDGAME, not SHIFT1. Omitting it left
        // the one transition that is not a simple alternation completely
        // unasserted, which is how nextPhase(SHIFT4) -> SHIFT1 survived -- that
        // told a bot in SHIFT4 its own hub was about to go dark when Table 6-3
        // makes it live again for the whole 30 s endgame.
        for (Phase p : new Phase[] {Phase.SHIFT1, Phase.SHIFT2, Phase.SHIFT3, Phase.SHIFT4}) {
            Phase successor = HubSchedule.nextPhase(p);
            assertEquals(HubSchedule.isHubActive(false, successor, 'R'),
                    HubSchedule.isHubActiveAfterNextPhase(false, p, 'R'),
                    "Blue after " + p + " must match Blue in " + successor);
            assertEquals(HubSchedule.isHubActive(true, successor, 'R'),
                    HubSchedule.isHubActiveAfterNextPhase(true, p, 'R'),
                    "Red after " + p + " must match Red in " + successor);
        }
        // The alternation itself: the successor is the opposite alliance's turn.
        assertFalse(HubSchedule.isHubActiveAfterNextPhase(false, Phase.SHIFT1, 'R'),
                "Red is live in SHIFT2, so Blue must be out after SHIFT1");
        assertTrue(HubSchedule.isHubActiveAfterNextPhase(true, Phase.SHIFT1, 'R'),
                "Red is live in SHIFT2, so Red must be in after SHIFT1");
        assertTrue(HubSchedule.isHubActiveAfterNextPhase(false, Phase.SHIFT2, 'R'),
                "Blue is live in SHIFT3, so Blue must be in after SHIFT2");
        assertTrue(HubSchedule.isHubActiveAfterNextPhase(true, Phase.SHIFT3, 'R'),
                "Red is live in SHIFT4, so Red must be in after SHIFT3");
    }

    @Test
    public void afterShift4IsEndGameWhereBothHubsAreLiveAgain() {
        // Split out so the load-bearing case is readable on its own: the shift
        // cycle does not wrap, it ends. Both alliances are ACTIVE after SHIFT4,
        // which is what stops a bot in SHIFT4 abandoning its own hub on a stale
        // "my hub is about to go dark" reading.
        assertEquals(Phase.ENDGAME, HubSchedule.nextPhase(Phase.SHIFT4),
                "Table 6-2 puts END GAME after SHIFT4; the shift cycle does not wrap");
        assertTrue(HubSchedule.isHubActiveAfterNextPhase(false, Phase.SHIFT4, 'R'),
                "Blue is live in END GAME (Table 6-3)");
        assertTrue(HubSchedule.isHubActiveAfterNextPhase(true, Phase.SHIFT4, 'R'),
                "Red is live in END GAME too (Table 6-3) -- both hubs return to active");
    }

    @Test
    public void afterNextPhaseIsBothLiveOutsideTheShifts() {
        for (Phase p : new Phase[] {Phase.AUTO, Phase.TRANSITION, Phase.ENDGAME}) {
            assertTrue(HubSchedule.isHubActiveAfterNextPhase(false, p, 'R'), "Blue live in " + p);
            assertTrue(HubSchedule.isHubActiveAfterNextPhase(true, p, 'R'), "Red live in " + p);
        }
    }

    @Test
    public void phaseBoundariesMatchTable62() {
        assertEquals(Phase.TRANSITION, HubSchedule.phaseFor(150.0, false));
        assertEquals(Phase.TRANSITION, HubSchedule.phaseFor(131.0, false));
        assertEquals(Phase.SHIFT1, HubSchedule.phaseFor(130.0, false));
        assertEquals(Phase.SHIFT1, HubSchedule.phaseFor(106.0, false));
        assertEquals(Phase.SHIFT2, HubSchedule.phaseFor(105.0, false));
        assertEquals(Phase.SHIFT2, HubSchedule.phaseFor(81.0, false));
        assertEquals(Phase.SHIFT3, HubSchedule.phaseFor(80.0, false));
        assertEquals(Phase.SHIFT3, HubSchedule.phaseFor(56.0, false));
        assertEquals(Phase.SHIFT4, HubSchedule.phaseFor(55.0, false));
        assertEquals(Phase.SHIFT4, HubSchedule.phaseFor(31.0, false));
        assertEquals(Phase.ENDGAME, HubSchedule.phaseFor(30.0, false));
        assertEquals(Phase.ENDGAME, HubSchedule.phaseFor(1.0, false));
        assertEquals(Phase.ENDGAME, HubSchedule.phaseFor(0.0, false));
        assertEquals(Phase.AUTO, HubSchedule.phaseFor(20.0, true));
        assertEquals(Phase.AUTO, HubSchedule.phaseFor(140.0, true));
    }

    @Test
    public void hubTableMatchesTable63() {
        // Seed 'R': red scored more in AUTO -> red inactive first.
        assertTrue(HubSchedule.isHubActive(true, Phase.AUTO, 'R'));
        assertTrue(HubSchedule.isHubActive(false, Phase.AUTO, 'R'));
        assertTrue(HubSchedule.isHubActive(true, Phase.TRANSITION, 'R'));
        assertTrue(HubSchedule.isHubActive(false, Phase.TRANSITION, 'R'));
        assertFalse(HubSchedule.isHubActive(true, Phase.SHIFT1, 'R'));
        assertTrue(HubSchedule.isHubActive(false, Phase.SHIFT1, 'R'));
        assertTrue(HubSchedule.isHubActive(true, Phase.SHIFT2, 'R'));
        assertFalse(HubSchedule.isHubActive(false, Phase.SHIFT2, 'R'));
        assertFalse(HubSchedule.isHubActive(true, Phase.SHIFT3, 'R'));
        assertTrue(HubSchedule.isHubActive(false, Phase.SHIFT3, 'R'));
        assertTrue(HubSchedule.isHubActive(true, Phase.SHIFT4, 'R'));
        assertFalse(HubSchedule.isHubActive(false, Phase.SHIFT4, 'R'));
        assertTrue(HubSchedule.isHubActive(true, Phase.ENDGAME, 'R'));
        assertTrue(HubSchedule.isHubActive(false, Phase.ENDGAME, 'R'));

        // Seed 'B' mirrors the order.
        assertTrue(HubSchedule.isHubActive(true, Phase.SHIFT1, 'B'));
        assertFalse(HubSchedule.isHubActive(false, Phase.SHIFT1, 'B'));
        assertFalse(HubSchedule.isHubActive(true, Phase.SHIFT2, 'B'));
        assertTrue(HubSchedule.isHubActive(false, Phase.SHIFT2, 'B'));
    }

    @Test
    public void seedDecisionFollowsAutoResult() {
        assertEquals('R', HubSchedule.decideSeed(9, 4));
        assertEquals('B', HubSchedule.decideSeed(3, 8));
        char tied = HubSchedule.decideSeed(5, 5);
        assertTrue(tied == 'R' || tied == 'B', "Tie must randomly seed R or B");
    }

    @Test
    public void currentPhaseTracksTheLatestUpdate() {
        HubSchedule.update(150.0, false);
        assertEquals(Phase.TRANSITION, HubSchedule.currentPhase());
        HubSchedule.update(120.0, false);
        assertEquals(Phase.SHIFT1, HubSchedule.currentPhase());
        HubSchedule.update(10.0, false);
        assertEquals(Phase.ENDGAME, HubSchedule.currentPhase());
        HubSchedule.update(150.0, true);
        assertEquals(Phase.AUTO, HubSchedule.currentPhase());
    }

    @Test
    public void scheduleAdvancesWhileSimClockRunsDown() {
        // Regression: GameSim only advanced the schedule on the branch where
        // the DS reported no valid match time, so a headless match (valid DS
        // time) logged "hub never active" while both hubs were scoring.
        HubSchedule.reset();
        HubSchedule.setShiftSeed('R');
        // Sim clock counting down from 150 through every shift boundary.
        double[] clock = {150.0, 135.0, 120.0, 100.0, 75.0, 50.0, 25.0, 5.0};
        boolean sawBlueActive = false;
        boolean sawRedActive = false;
        boolean sawBothActive = false;
        for (double remaining : clock) {
            HubSchedule.update(remaining, false);
            boolean blue = HubSchedule.isHubActiveNow(false);
            boolean red = HubSchedule.isHubActiveNow(true);
            sawBlueActive |= blue;
            sawRedActive |= red;
            sawBothActive |= (blue && red);
        }
        assertTrue(sawBothActive, "AUTO/TRANSITION/ENDGAME must have both hubs active");
        assertTrue(sawBlueActive && sawRedActive,
                "shifted phases must alternate which hub is active");
    }

    @Test
    public void shiftClockAdvancesAndDrivesHarvestDecisions() {
        // Regression: Jev read timeUntilHubShift from Dashboard, which reports
        // -1 under sim and so stayed 0.0 forever. Every bot then believed its
        // shift was always ending, dumping its hopper on an 8-ball cycle and
        // flipping SCORE <-> VACUUM each volley. The shift clock must count
        // down within a shift and read 0 where no flip is coming.
        HubSchedule.reset();
        HubSchedule.setShiftSeed('R');

        HubSchedule.update(130.0, false);
        assertEquals(Phase.SHIFT1, HubSchedule.currentPhase());
        assertEquals(25.0, HubSchedule.timeUntilShiftEnd(), 0.01,
                "a shift starts with the full 25 s until the flip");

        HubSchedule.update(120.0, false);
        assertEquals(15.0, HubSchedule.timeUntilShiftEnd(), 0.01,
                "the shift clock must count down, not stick at 0 or 25");

        HubSchedule.update(107.0, false);
        assertEquals(2.0, HubSchedule.timeUntilShiftEnd(), 0.01,
                "timeUntilHubShift must approach zero near the boundary");

        // No flip is coming in these phases, so harvest planning must not see
        // a shift deadline.
        HubSchedule.update(140.0, false);
        assertEquals(0.0, HubSchedule.timeUntilShiftEnd(), 0.01,
                "TRANSITION has both hubs active: no shift deadline");
        HubSchedule.update(20.0, false);
        assertEquals(0.0, HubSchedule.timeUntilShiftEnd(), 0.01,
                "ENDGAME has both hubs active: no shift deadline");
    }

    @Test
    public void matchClockIsExposedForDecisionMakers() {
        HubSchedule.reset();
        assertEquals(-1.0, HubSchedule.lastMatchTimeRemaining(), 0.01,
                "before the first update there is no clock");
        HubSchedule.update(95.0, false);
        assertEquals(95.0, HubSchedule.lastMatchTimeRemaining(), 0.01);
        assertEquals(Phase.SHIFT2, HubSchedule.currentPhase());
    }

    @Test
    public void scoringGraceCoversThreeSecondsAfterDeactivation() {
        HubSchedule.setShiftSeed('R');

        // SHIFT 1: red inactive, blue active.
        HubSchedule.update(120.0, false);
        assertFalse(HubSchedule.isHubActiveNow(true));
        assertTrue(HubSchedule.isHubActiveNow(false));
        // Just deactivated: still scoring-active (processing grace).
        assertTrue(HubSchedule.isScoringActive(true));
        assertTrue(HubSchedule.isScoringActive(false));

        // 2.9 s later: grace still holds.
        fakeClock[0] += 2.9;
        HubSchedule.update(117.0, false);
        assertTrue(HubSchedule.isScoringActive(true));

        // Past 3 s: closed.
        fakeClock[0] += 0.2;
        HubSchedule.update(116.0, false);
        assertFalse(HubSchedule.isScoringActive(true));
        assertTrue(HubSchedule.isScoringActive(false));

        // SHIFT 2 flips red back active: grace state clears.
        HubSchedule.update(90.0, false);
        assertTrue(HubSchedule.isHubActiveNow(true));
        assertTrue(HubSchedule.isScoringActive(true));
        assertFalse(HubSchedule.isHubActiveNow(false));
        // Blue just deactivated: its own grace window opens.
        assertTrue(HubSchedule.isScoringActive(false));
    }
}
