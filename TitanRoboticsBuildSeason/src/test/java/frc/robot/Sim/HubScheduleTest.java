package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
        assertEquals(Phase.DONE, HubSchedule.phaseFor(0.0, false));
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
