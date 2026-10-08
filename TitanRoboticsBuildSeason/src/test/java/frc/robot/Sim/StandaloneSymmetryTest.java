package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.Intelligence.Archetype;

/**
 * Locks the invariant that the standalone runner is <b>alliance-symmetric</b>.
 *
 * <p>B1 (2026-10-08) chased a standing Blue advantage in {@code default3v3}
 * (Blue won 8/8 at ~2:1). Root cause: {@code default3v3} is deliberately
 * asymmetric — Blue carries {@code TACTICAL_DEFENDER}, Red {@code DEFENSE_BULLY}
 * — and the former suppresses the opponent's harvesters far harder. Swapping the
 * two flips the winner (Red 6/8); replacing both with cyclers gives a near-tie.
 * So the bias is the archetypes, not the engine. The only runner-level artefact
 * was a hard-coded {@code 'R'} shift seed (permanently Blue-first); that is now
 * derived from the AUTO result like the full sim ({@code HubSchedule.decideSeed}).
 *
 * <p>This test uses a mirrored, archetype-identical roster and asserts the
 * aggregate is not lopsided — the regression guard for that invariant.
 */
public class StandaloneSymmetryTest {
    private static final long[] SEEDS = {7, 11, 42, 101, 2026, 3141, 7777, 90210};

    /** Same three archetypes on both alliances, on the mirrored {@code default3v3} spawns. */
    private static StandaloneMatchRunner.Config mirroredCyclers(long seed) {
        TrainingMatchScenario s = TrainingMatchScenario.default3v3(seed);
        List<TrainingMatchScenario.RobotConfig> b = s.blueRobots();
        List<TrainingMatchScenario.RobotConfig> r = s.redRobots();
        List<StandaloneMatchRunner.BotSpec> bots = List.of(
                new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, false, b.get(0).startingPose(), 8),
                new StandaloneMatchRunner.BotSpec(Archetype.ADAPTIVE_COMPETITOR, false, b.get(1).startingPose(), 8),
                new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, false, b.get(2).startingPose(), 8),
                new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, true, r.get(0).startingPose(), 8),
                new StandaloneMatchRunner.BotSpec(Archetype.ADAPTIVE_COMPETITOR, true, r.get(1).startingPose(), 8),
                new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, true, r.get(2).startingPose(), 8));
        return new StandaloneMatchRunner.Config(seed, 150.0, 15.0, 108, 'R', bots);
    }

    @Test
    void mirroredRosterDoesNotFavourEitherAlliance() {
        int blue = 0;
        int red = 0;
        for (long seed : SEEDS) {
            StandaloneMatchRunner.Result res = StandaloneMatchRunner.run(mirroredCyclers(seed));
            blue += res.blueScored();
            red += res.redScored();
        }
        double diff = Math.abs(blue - red) / (double) (blue + red);
        assertTrue(diff < 0.20,
                "mirrored roster must not favour a side: Blue=" + blue + " Red=" + red
                        + " diff=" + String.format("%.1f%%", diff * 100));
    }
}
