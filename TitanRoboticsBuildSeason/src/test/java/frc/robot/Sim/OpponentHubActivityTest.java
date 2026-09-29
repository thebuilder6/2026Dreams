package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Sim.HubSchedule.Phase;

/**
 * Pins {@link HubSchedule#isOpponentHubActiveGivenMineIs(boolean, Phase)}.
 *
 * <p>The two hubs are complementary <b>only</b> during SHIFT 1-4. AUTO,
 * TRANSITION and ENDGAME have both hubs live, so any code that infers the
 * opponent hub by inverting its own ({@code !myHubActive}) reports the opponent
 * as dead for all of autonomous and the whole 30 s endgame. That is what
 * {@code WorldStateBuilder} used to do on the real-robot path.
 */
public class OpponentHubActivityTest {

    @Test
    public void bothHubsAreLiveOutsideTheShiftsSoInversionWouldBeWrong() {
        for (Phase bothLive : new Phase[] {Phase.AUTO, Phase.TRANSITION, Phase.ENDGAME}) {
            assertTrue(HubSchedule.isOpponentHubActiveGivenMineIs(true, bothLive),
                    bothLive + ": opponent must be live while ours is");
            assertTrue(HubSchedule.isOpponentHubActiveGivenMineIs(false, bothLive),
                    bothLive + ": opponent must be live even if ours reads inactive");
            // The old behaviour, asserted so the regression cannot come back.
            assertFalse(!HubSchedule.isOpponentHubActiveGivenMineIs(true, bothLive),
                    bothLive + ": inverting would wrongly report the opponent dead");
        }
    }

    @Test
    public void exactlyOneHubIsLiveDuringEveryShiftSoInversionIsCorrect() {
        for (Phase shift : new Phase[] {Phase.SHIFT1, Phase.SHIFT2, Phase.SHIFT3, Phase.SHIFT4}) {
            assertFalse(HubSchedule.isOpponentHubActiveGivenMineIs(true, shift),
                    shift + ": ours live means the opponent is out");
            assertTrue(HubSchedule.isOpponentHubActiveGivenMineIs(false, shift),
                    shift + ": ours out means the opponent is live");
        }
    }

    @Test
    public void bothHubsAreDeadOnceTheMatchIsDone() {
        assertFalse(HubSchedule.isOpponentHubActiveGivenMineIs(true, Phase.DONE));
        assertFalse(HubSchedule.isOpponentHubActiveGivenMineIs(false, Phase.DONE));
    }

    /**
     * The derivation must never disagree with the authoritative shift table, for
     * any phase, seed or alliance. This is the guard against the two drifting
     * apart when someone edits one and not the other.
     */
    @Test
    public void opponentHubDerivedFromOwnHubAgreesWithTheTable() {
        for (Phase phase : Phase.values()) {
            for (char seed : new char[] {'R', 'B'}) {
                for (boolean queryRed : new boolean[] {true, false}) {
                    boolean mine = HubSchedule.isHubActive(queryRed, phase, seed);
                    boolean opponent = HubSchedule.isHubActive(!queryRed, phase, seed);
                    assertEquals(opponent,
                            HubSchedule.isOpponentHubActiveGivenMineIs(mine, phase),
                            "phase=" + phase + " seed=" + seed + " querying "
                                    + (queryRed ? "Red" : "Blue"));
                }
            }
        }
    }
}
