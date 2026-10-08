package frc.robot.Sim;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Intelligence.Archetype;
import frc.robot.Navigation.FieldMap;
import frc.robot.Sim.StandaloneBot.BotView;

/**
 * MapleSim-free fixed-step match runner.
 *
 * <p>Runs the real {@code JevDecisionEngine} policy for a full roster against
 * a kinematic model and a list-backed {@link FuelStore} at a fixed 20 ms step
 * ({@code GameSim.Config.SIMULATION_PERIOD}), with no WPILib robot loop, no
 * NetworkTables server, no logging, and no wall-clock sleeps.
 *
 * <p>Each tick every bot sees pre-tick roster views (simultaneous update, fixed
 * roster order, so the run is a pure function of the seed), then the runner
 * applies a positional peer push-out. Hub phase comes from the single owner
 * ({@link HubSchedule}): the runner calls {@code HubSchedule.update} once per
 * tick and reads strict activity plus {@code timeUntilShiftEnd}. The 3 s
 * scoring grace is intentionally unused — its stamps come from the wall clock
 * and would break determinism. Shift order is an explicit config char (ties
 * cannot draw from {@code MatchDeterminism} here because its tiebreak requires
 * sim mode).
 */
public final class StandaloneMatchRunner {
    /** Fixed step matching the full sim's 20 ms loop. */
    public static final double DT_SEC = 0.02;

    /** Minimum center-to-center distance enforced by the peer push-out. */
    public static final double PEER_MIN_DISTANCE_M = 0.90;

    /** One roster slot. */
    public record BotSpec(Archetype archetype, boolean isRed, Pose2d startPose, int preload) {}

    public record Config(
            long seed,
            double durationSec,
            double autoSec,
            int fuelCount,
            char shiftSeed,
            List<BotSpec> bots,
            String logPath) {
        public Config {
            if (!Double.isFinite(durationSec) || durationSec <= 0.0) {
                throw new IllegalArgumentException("durationSec must be finite and positive");
            }
            if (!Double.isFinite(autoSec) || autoSec < 0.0 || durationSec - autoSec < 1.0) {
                throw new IllegalArgumentException("durationSec must exceed autoSec by at least 1 s");
            }
            if (fuelCount < 0 || fuelCount > TrainingMatchScenario.MAX_FIELD_FUEL_COUNT) {
                throw new IllegalArgumentException("fuelCount out of range");
            }
            if (bots == null || bots.isEmpty()) {
                throw new IllegalArgumentException("bots must contain at least one slot");
            }
            bots = List.copyOf(bots);
        }

        /** Same config without replay recording. */
        public Config withoutLog() {
            return new Config(seed, durationSec, autoSec, fuelCount, shiftSeed, bots, null);
        }

        public Config(
                long seed,
                double durationSec,
                double autoSec,
                int fuelCount,
                char shiftSeed,
                List<BotSpec> bots) {
            this(seed, durationSec, autoSec, fuelCount, shiftSeed, bots, null);
        }
    }

    public record Result(
            long ticks,
            int blueScored,
            int redScored,
            String winner,
            int pickedUp,
            int fuelRemaining,
            double[] pathLengthM,
            int[] botScored,
            long wallMs) {}

    private StandaloneMatchRunner() {}

    /** 1-bot blue-cycler smoke: training spawn, 8 preload, 54 fuel. */
    public static Config defaultSmoke(long seed) {
        TrainingMatchScenario scenario = TrainingMatchScenario.default3v3(seed);
        TrainingMatchScenario.RobotConfig bot = scenario.bluePlayerRobot();
        return new Config(seed, scenario.durationSeconds(), 15.0,
                scenario.fieldFuelCount(), 'R',
                List.of(new BotSpec(bot.archetype(), false, bot.startingPose(), bot.preloadFuel())),
                null);
    }

    /** Full 3v3 roster straight from {@code TrainingMatchScenario.default3v3}. */
    public static Config default3v3(long seed) {
        TrainingMatchScenario scenario = TrainingMatchScenario.default3v3(seed);
        List<BotSpec> bots = new ArrayList<>();
        for (TrainingMatchScenario.RobotConfig bot : scenario.blueRobots()) {
            bots.add(new BotSpec(bot.archetype(), false, bot.startingPose(), bot.preloadFuel()));
        }
        for (TrainingMatchScenario.RobotConfig bot : scenario.redRobots()) {
            bots.add(new BotSpec(bot.archetype(), true, bot.startingPose(), bot.preloadFuel()));
        }
        return new Config(seed, scenario.durationSeconds(), 15.0,
                scenario.fieldFuelCount(), 'R', bots);
    }

    public static Result run(Config config) {
        long wallStart = System.currentTimeMillis();
        char savedSeed = HubSchedule.getShiftSeed();
        MatchDeterminism.seed(config.seed());
        try {
            HubSchedule.setShiftSeed(config.shiftSeed());
            FuelStore fuel = FuelStore.scatterSeeded(config.seed(), config.fuelCount());
            List<StandaloneBot> bots = new ArrayList<>();
            for (BotSpec spec : config.bots()) {
                bots.add(new StandaloneBot(
                        spec.startPose(), spec.archetype(), spec.isRed(), spec.preload()));
            }

            long ticks = Math.round(config.durationSec() / DT_SEC);
            for (long tick = 0; tick < ticks; tick++) {
                double elapsed = tick * DT_SEC;
                double remaining = Math.max(0.0, config.durationSec() - elapsed);
                boolean isAuto = elapsed < config.autoSec();
                HubSchedule.update(remaining, isAuto);
                double timeUntilShift = HubSchedule.timeUntilShiftEnd();

                int blue = 0;
                int red = 0;
                for (StandaloneBot bot : bots) {
                    if (bot.isRed()) {
                        red += bot.getScored();
                    } else {
                        blue += bot.getScored();
                    }
                }
                List<BotView> views = new ArrayList<>();
                for (StandaloneBot bot : bots) {
                    views.add(bot.view());
                }
                for (int i = 0; i < bots.size(); i++) {
                    StandaloneBot bot = bots.get(i);
                    List<BotView> allies = new ArrayList<>();
                    List<BotView> opponents = new ArrayList<>();
                    for (int j = 0; j < bots.size(); j++) {
                        if (bots.get(j).isRed() == bot.isRed()) {
                            allies.add(views.get(j));
                        } else {
                            opponents.add(views.get(j));
                        }
                    }
                    int diff = bot.isRed() ? red - blue : blue - red;
                    bot.step(DT_SEC, remaining, isAuto, fuel, config.shiftSeed(),
                            diff, timeUntilShift, allies, opponents,
                            nearestOpponent(bot.getPose().getTranslation(), opponents), elapsed);
                }
                separatePeers(bots);
            }

            int blue = 0;
            int red = 0;
            int pickedUp = 0;
            double[] path = new double[bots.size()];
            int[] scored = new int[bots.size()];
            for (int i = 0; i < bots.size(); i++) {
                StandaloneBot bot = bots.get(i);
                if (bot.isRed()) {
                    red += bot.getScored();
                } else {
                    blue += bot.getScored();
                }
                pickedUp += bot.getPickedUp();
                path[i] = bot.getPathLengthM();
                scored[i] = bot.getScored();
            }
            String winner = blue > red ? "Blue" : red > blue ? "Red" : "Tie";
            long wallMs = System.currentTimeMillis() - wallStart;
            return new Result(ticks, blue, red, winner, pickedUp, fuel.size(), path, scored, wallMs);
        } finally {
            MatchDeterminism.clearSeed();
            HubSchedule.setShiftSeed(savedSeed);
        }
    }

    private static Pose2d nearestOpponent(Translation2d at, List<BotView> opponents) {
        Pose2d best = new Pose2d();
        double bestDist = Double.MAX_VALUE;
        for (BotView o : opponents) {
            double d = at.getDistance(o.pose().getTranslation());
            if (d < bestDist) {
                bestDist = d;
                best = o.pose();
            }
        }
        return best;
    }

    /**
     * Crude positional deconfliction: push overlapping pairs apart along their
     * shared axis, fixed pair order. No deadlock recovery, no trench logic —
     * that arbitration lives in the full sim's watchdogs.
     */
    static void separatePeers(List<StandaloneBot> bots) {
        for (int i = 0; i < bots.size(); i++) {
            for (int j = i + 1; j < bots.size(); j++) {
                Translation2d a = bots.get(i).getPose().getTranslation();
                Translation2d b = bots.get(j).getPose().getTranslation();
                Translation2d delta = b.minus(a);
                double dist = delta.getNorm();
                if (dist >= PEER_MIN_DISTANCE_M || dist < 1e-6) {
                    continue;
                }
                Translation2d push = delta.times((PEER_MIN_DISTANCE_M - dist) / 2.0 / dist);
                bots.get(i).setPose(clampOnField(a.minus(push), bots.get(i).getPose()));
                bots.get(j).setPose(clampOnField(b.plus(push), bots.get(j).getPose()));
            }
        }
    }

    private static Pose2d clampOnField(Translation2d at, Pose2d keepHeading) {
        double x = Math.min(Math.max(at.getX(), 0.3), FieldMap.FIELD_LENGTH - 0.3);
        double y = Math.min(Math.max(at.getY(), 0.3), FieldMap.FIELD_WIDTH - 0.3);
        return new Pose2d(x, y, keepHeading.getRotation());
    }
}
