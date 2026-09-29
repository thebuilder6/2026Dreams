package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * Reproducibility contracts for seeded training matches.
 *
 * <p>Same seed must produce the same random streams and the same fuel ordering.
 * It cannot promise identical matches: MapleSim owns an unseeded static RNG and
 * its solver is not bit-reproducible. These tests therefore pin the parts we
 * control, which is what turns a seed from decorative into useful.
 */
class MatchDeterminismTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setMatchTime(-1.0);
        SimulatedArena.getInstance().clearGamePieces();
        MatchDeterminism.clearSeed();
    }

    @AfterEach
    void tearDown() {
        SimulatedArena.getInstance().clearGamePieces();
        MatchDeterminism.clearSeed();
    }

    @Test
    void unseededGeneratorsRemainUsable() {
        MatchDeterminism.clearSeed();
        assertFalse(MatchDeterminism.isSeeded());
        Random r = MatchDeterminism.random("shot:Bot0");
        assertNotNull(r);
        // No fixed value asserted: unseeded behavior is intentionally free.
        r.nextDouble();
    }

    @Test
    void sameSeedProducesSameStream() {
        MatchDeterminism.seed(2026L);
        Random a = MatchDeterminism.random("shot:Bot0");
        double[] first = new double[8];
        for (int i = 0; i < first.length; i++) {
            first[i] = a.nextDouble();
        }

        MatchDeterminism.seed(2026L);
        Random b = MatchDeterminism.random("shot:Bot0");
        for (double v : first) {
            assertEquals(v, b.nextDouble(), 0.0,
                    "the same seed must replay the same stream exactly");
        }
    }

    @Test
    void differentSeedProducesDifferentStream() {
        MatchDeterminism.seed(2026L);
        Random a = MatchDeterminism.random("shot:Bot0");
        double firstA = a.nextDouble();

        MatchDeterminism.seed(77L);
        Random b = MatchDeterminism.random("shot:Bot0");
        double firstB = b.nextDouble();

        // Not asserting inequality of a single draw (it could coincide), but
        // the aggregate over many draws must differ.
        MatchDeterminism.seed(2026L);
        Random a2 = MatchDeterminism.random("shot:Bot0");
        MatchDeterminism.seed(77L);
        Random b2 = MatchDeterminism.random("shot:Bot0");
        int differing = 0;
        for (int i = 0; i < 16; i++) {
            if (a2.nextDouble() != b2.nextDouble()) {
                differing++;
            }
        }
        assertTrue(differing > 12, "distinct seeds must yield distinct streams, differing=" + differing);
    }

    @Test
    void streamsAreIndependentPerPurpose() {
        MatchDeterminism.seed(2026L);
        Random bot0 = MatchDeterminism.random("shot:Bot0");
        Random bot1 = MatchDeterminism.random("shot:Bot1");
        assertFalse(bot0 == bot1, "each purpose gets its own generator");
        // Drawing from one must not advance the other.
        double before = bot1.nextDouble();
        bot0.nextDouble();
        MatchDeterminism.seed(2026L);
        assertEquals(before, MatchDeterminism.random("shot:Bot1").nextDouble(), 0.0,
                "one bot's draws must not shift another bot's stream");
    }

    @Test
    void generatorInstanceIsStablePerPurpose() {
        MatchDeterminism.seed(2026L);
        Random first = MatchDeterminism.random("hubShiftTiebreak");
        first.nextDouble();
        Random again = MatchDeterminism.random("hubShiftTiebreak");
        assertTrue(first == again, "a named stream must keep its state across calls");
    }

    @Test
    void fuelOrderIsSortedByPosition() {
        SimulatedArena arena = SimulatedArena.getInstance();
        // Add well out of order; the sorted view must impose x-then-y order.
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(12.0, 2.0)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(3.0, 6.0)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(3.0, 1.0)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(8.0, 4.0)));

        List<?> sorted = MatchDeterminism.fuelOnFieldSorted();
        assertTrue(sorted.size() >= 4, "expected at least the 4 added pieces, got " + sorted.size());

        double lastX = -1.0;
        double lastY = -1.0;
        for (Object o : sorted) {
            Translation2d p = ((swervelib.simulation.ironmaple.simulation.gamepieces
                    .GamePieceOnFieldSimulation) o).getPoseOnField().getTranslation();
            if (p.getX() < lastX - 1e-9) {
                throw new AssertionError("fuel not sorted by x: " + p);
            }
            if (Math.abs(p.getX() - lastX) < 1e-9 && p.getY() < lastY - 1e-9) {
                throw new AssertionError("fuel not sorted by y within same x: " + p);
            }
            lastX = p.getX();
            lastY = p.getY();
        }
    }

    @Test
    void fuelOrderIsStableAcrossRepeatedCalls() {
        SimulatedArena arena = SimulatedArena.getInstance();
        for (int i = 0; i < 12; i++) {
            arena.addGamePiece(new RebuiltFuelOnField(
                    new Translation2d(2.0 + i * 0.3, 1.5 + (i % 4) * 1.1)));
        }
        String first = describe(MatchDeterminism.fuelOnFieldSorted());
        for (int i = 0; i < 5; i++) {
            assertEquals(first, describe(MatchDeterminism.fuelOnFieldSorted()),
                    "repeated reads must return an identical order");
        }
    }

    @Test
    void fuelOrderSurvivesAPieceRemoval() {
        SimulatedArena arena = SimulatedArena.getInstance();
        var removed = new RebuiltFuelOnField(new Translation2d(5.0, 3.0));
        arena.addGamePiece(removed);
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 5.0)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(7.0, 5.0)));

        arena.removeGamePiece(removed);

        // Compare positions, not a rendered string: a naive substring check
        // matches the surviving pieces' y coordinate too.
        for (Object o : MatchDeterminism.fuelOnFieldSorted()) {
            Translation2d p = ((swervelib.simulation.ironmaple.simulation.gamepieces
                    .GamePieceOnFieldSimulation) o).getPoseOnField().getTranslation();
            assertFalse(Math.abs(p.getX() - 5.0) < 1e-6 && Math.abs(p.getY() - 3.0) < 1e-6,
                    "removed piece must not appear, found " + p);
        }
    }

    private static String describe(List<?> pieces) {
        StringBuilder sb = new StringBuilder();
        for (Object o : pieces) {
            Translation2d p = ((swervelib.simulation.ironmaple.simulation.gamepieces
                    .GamePieceOnFieldSimulation) o).getPoseOnField().getTranslation();
            sb.append(String.format("%.2f,%.2f;", p.getX(), p.getY()));
        }
        return sb.toString();
    }
}
