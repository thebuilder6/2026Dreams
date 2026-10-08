package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Intelligence.Archetype;

/**
 * Smoke coverage for the standalone (MapleSim-free) runner: deterministic,
 * makes progress, and finishes full matches far faster than wall clock.
 */
public class StandaloneMatchRunnerTest {
    private static StandaloneMatchRunner.Config shortSmoke(long seed) {
        return new StandaloneMatchRunner.Config(seed, 30.0, 5.0, 54, 'R',
                List.of(new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, false,
                        new Pose2d(2.0, 2.25, Rotation2d.fromDegrees(0)), 8)));
    }

    private static StandaloneMatchRunner.Config short3v3(long seed) {
        StandaloneMatchRunner.Config full = StandaloneMatchRunner.default3v3(seed);
        return new StandaloneMatchRunner.Config(seed, 30.0, 5.0,
                full.fuelCount(), full.shiftSeed(), full.bots());
    }

    @Test
    void sameSeedSameResult() {
        StandaloneMatchRunner.Result first = StandaloneMatchRunner.run(shortSmoke(7));
        StandaloneMatchRunner.Result second = StandaloneMatchRunner.run(shortSmoke(7));
        assertEquals(first.blueScored(), second.blueScored());
        assertEquals(first.pickedUp(), second.pickedUp());
        assertEquals(first.fuelRemaining(), second.fuelRemaining());
        assertEquals(first.pathLengthM()[0], second.pathLengthM()[0], 1e-9);
    }

    @Test
    void fullMatchMakesProgressFasterThanRealtime() {
        StandaloneMatchRunner.Result result =
                StandaloneMatchRunner.run(StandaloneMatchRunner.defaultSmoke(2026));
        assertEquals(7_500, result.ticks());
        assertTrue(result.pickedUp() > 0, "bot should collect fuel");
        assertTrue(result.blueScored() > 0, "cycler should score while its hub is active");
        assertTrue(result.pathLengthM()[0] > 5.0, "bot should actually drive");
        assertTrue(result.wallMs() < 15_000,
                "150 s sim must finish much faster than 150 s wall, was " + result.wallMs() + " ms");
    }

    @Test
    void threeVsThreeIsDeterministic() {
        StandaloneMatchRunner.Result first = StandaloneMatchRunner.run(short3v3(42));
        StandaloneMatchRunner.Result second = StandaloneMatchRunner.run(short3v3(42));
        assertEquals(first.blueScored(), second.blueScored());
        assertEquals(first.redScored(), second.redScored());
        assertEquals(first.winner(), second.winner());
    }

    @Test
    void threeVsThreeFullMatchScoresBothSides() {
        StandaloneMatchRunner.Result result =
                StandaloneMatchRunner.run(StandaloneMatchRunner.default3v3(2026));
        assertEquals(7_500, result.ticks());
        assertEquals(6, result.botScored().length);
        assertTrue(result.blueScored() > 0, "blue should score, was " + result.blueScored());
        assertTrue(result.redScored() > 0, "red should score, was " + result.redScored());
        assertTrue(result.wallMs() < 15_000,
                "6-bot 150 s sim must finish far faster than wall, was " + result.wallMs() + " ms");
    }

    @Test
    void rejectsBadConfig() {
        List<StandaloneMatchRunner.BotSpec> solo = List.of(
                new StandaloneMatchRunner.BotSpec(Archetype.AUTONOMOUS_CYCLER, false,
                        new Pose2d(2.0, 2.25, Rotation2d.fromDegrees(0)), 8));
        assertThrows(IllegalArgumentException.class,
                () -> new StandaloneMatchRunner.Config(1, -5.0, 0.0, 54, 'R', solo));
        assertThrows(IllegalArgumentException.class,
                () -> new StandaloneMatchRunner.Config(1, 10.0, 9.5, 54, 'R', solo));
        assertThrows(IllegalArgumentException.class,
                () -> new StandaloneMatchRunner.Config(1, 30.0, 5.0, 54, 'R', List.of()));
    }
}
