package frc.robot.Sim;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Reproducibility helpers for the AI-vs-AI training match.
 *
 * <p>Two independent sources of run-to-run variation existed in the sim, and
 * only one of them was intentional.
 *
 * <p><b>Accidental (fixed here).</b> {@code SimulatedArena.gamePiecesOnField()}
 * builds a fresh {@link java.util.HashSet} on every call, so iteration order
 * depends on identity hash codes and differs between JVM runs. That mattered in
 * two places: {@code checkProximityPickup} collects whichever overlapping fuel
 * piece the iteration reaches first (one extra ball in a hopper), and the Jev
 * fuel selectors resolve equal-scoring candidates with strict {@code >}, so the
 * chosen target also depended on hash order. Either one re-routes a bot, and
 * the match diverges from there. {@link #fuelOnFieldSorted()} returns a
 * position-sorted snapshot, giving stable order without touching the library.
 *
 * <p><b>Intentional (seeded here).</b> Shot spread, contact-watchdog jink and
 * hub-shift tie-breaks all drew from {@code Math.random()} or an unseeded
 * {@link Random}. Those behaviours are deliberate - they are meant to model
 * spread and desynchronise head-on pairs - but they made a seed meaningless.
 * They now draw from a per-purpose generator derived from the scenario seed.
 *
 * <p><b>What is still not reproducible.</b> MapleSim owns a static unseeded
 * {@code Random} ({@code RebuiltHub.rng}) and its rigid-body solver is not
 * bit-reproducible, so two runs with identical seeds will still diverge
 * slightly. The goal here is low variance, not bit-exact replay: enough
 * repeatability that a single match can distinguish a real behavioural
 * regression from noise.
 */
public final class MatchDeterminism {

    private static final Object LOCK = new Object();
    private static long scenarioSeed = 0L;
    private static boolean seeded;

    private static final java.util.Map<String, Random> GENERATORS = new java.util.LinkedHashMap<>();

    private MatchDeterminism() {}

    /**
     * Installs the scenario seed. Every generator below is derived from it, so
     * one call at match start makes the whole run reproducible-by-seed.
     */
    public static void seed(long scenarioSeed) {
        synchronized (LOCK) {
            MatchDeterminism.scenarioSeed = scenarioSeed;
            seeded = true;
            GENERATORS.clear();
        }
    }

    /** Restores the unseeded fallback. */
    public static void clearSeed() {
        synchronized (LOCK) {
            scenarioSeed = 0L;
            seeded = false;
            GENERATORS.clear();
        }
    }

    public static boolean isSeeded() {
        synchronized (LOCK) {
            return seeded;
        }
    }

    public static long currentSeed() {
        synchronized (LOCK) {
            return scenarioSeed;
        }
    }

    /**
     * Named generator, stable per run. Unseeded callers get a generator seeded
     * from {@link System#nanoTime()} so behaviour is unchanged for anyone not
     * using a scenario.
     *
     * @param purpose stream name, e.g. {@code "shot:Bot0"}
     */
    public static Random random(String purpose) {
        synchronized (LOCK) {
            Random r = GENERATORS.get(purpose);
            if (r == null) {
                r = seeded
                        ? new Random(mix(scenarioSeed, purpose))
                        : new Random();
                GENERATORS.put(purpose, r);
            }
            return r;
        }
    }

    /** Stable string hash (String.hashCode is specified, but mix it anyway). */
    private static long mix(long seed, String purpose) {
        long h = seed;
        for (int i = 0; i < purpose.length(); i++) {
            h = h * 31 + purpose.charAt(i);
        }
        return h;
    }

    /**
     * Position-sorted snapshot of grounded fuel, nearest-first.
     *
     * <p>Sorting by (x, y) - not by distance to any robot - keeps the order a
     * pure function of field state, so two bots in different places still see
     * the same sequence. Nearest-first would be faster to reason about but
     * reintroduces a per-bot difference in tie order.
     */
    public static List<GamePieceOnFieldSimulation> fuelOnFieldSorted() {
        return fuelOnFieldSorted(null, null);
    }

    /**
     * Position-sorted snapshot of grounded fuel, optionally filtered to a
     * rectangular region to avoid copying pieces the caller cannot use.
     *
     * @param minX inclusive lower X bound, or null for no bound
     * @param maxX exclusive upper X bound, or null for no bound
     */
    public static List<GamePieceOnFieldSimulation> fuelOnFieldSorted(Double minX, Double maxX) {
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null) {
            return List.of();
        }
        Set<GamePieceOnFieldSimulation> pieces;
        try {
            pieces = arena.gamePiecesOnField();
        } catch (Exception e) {
            return List.of();
        }
        if (pieces == null || pieces.isEmpty()) {
            return List.of();
        }
        List<GamePieceOnFieldSimulation> out = new ArrayList<>();
        for (GamePieceOnFieldSimulation piece : pieces) {
            if (piece == null || !"Fuel".equals(piece.getType())) {
                continue;
            }
            Translation2d p;
            try {
                p = piece.getPoseOnField().getTranslation();
            } catch (Exception e) {
                continue;
            }
            if (minX != null && p.getX() < minX) {
                continue;
            }
            if (maxX != null && p.getX() >= maxX) {
                continue;
            }
            out.add(piece);
        }
        out.sort(Comparator
                .comparingDouble((GamePieceOnFieldSimulation p) ->
                        p.getPoseOnField().getTranslation().getX())
                .thenComparingDouble(p -> p.getPoseOnField().getTranslation().getY()));
        return out;
    }
}
