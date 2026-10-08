package frc.robot.Sim;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Intelligence.StrategicObjective;
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

    /** Climb points per robot; mirrors {@code MatchScoreTracker.POINTS_PER_CLIMB}. */
    private static final int POINTS_PER_CLIMB_MIRROR = 10;

    /** One roster slot. */
    public record BotSpec(Archetype archetype, boolean isRed, Pose2d startPose, int preload,
            boolean hasClimber) {
        /** Legacy 4-arg slot: no climber, preserving prior behaviour. */
        public BotSpec(Archetype archetype, boolean isRed, Pose2d startPose, int preload) {
            this(archetype, isRed, startPose, preload, false);
        }
    }

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

    /**
     * Per-roster-slot telemetry, the input the individual-EPA evaluator
     * ({@code tools/tune/sim_epa.py}) consumes. Roster order is the order of
     * {@link Config#bots()}: Blue slots first, then Red, in {@code default3v3}.
     */
    public record BotTelemetry(
            boolean isRed,
            String archetype,
            int autoScored,
            int teleopScored,
            int scored,
            int pickedUp,
            int attemptedShots,
            int missedShots,
            int escapes,
            double pathLengthM,
            int wastedFuel,
            int shuttledFuel,
            boolean climbed,
            double climbArrivalSec,
            int minorFouls,
            int majorFouls,
            double hubActiveTeleopSec) {}

    public record Result(
            long ticks,
            int blueScored,
            int redScored,
            int blueAutoFuel,
            int redAutoFuel,
            int blueTeleopFuel,
            int redTeleopFuel,
            String winner,
            int pickedUp,
            int fuelRemaining,
            int attemptedShots,
            int missedShots,
            int escapes,
            double[] pathLengthM,
            int[] botScored,
            BotTelemetry[] botTelemetry,
            double[][] markSeconds,
            long wallMs) {
        public Result(
                long ticks,
                int blueScored,
                int redScored,
                String winner,
                int pickedUp,
                int fuelRemaining,
                double[] pathLengthM,
                int[] botScored,
                long wallMs) {
            this(ticks, blueScored, redScored, 0, 0, blueScored, redScored,
                    winner, pickedUp, fuelRemaining, 0, 0, 0,
                    pathLengthM, botScored, new BotTelemetry[0], new double[0][0], wallMs);
        }
    }

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

    /**
     * Full 3v3 roster with an explicit climber flag on every slot. The default
     * roster has no climber, so endgame is otherwise unmeasurable in the
     * standalone; the evolution harness uses this to make Endgame EPA real.
     */
    public static Config default3v3(long seed, boolean withClimbers) {
        TrainingMatchScenario scenario = TrainingMatchScenario.default3v3(seed);
        List<BotSpec> bots = new ArrayList<>();
        for (TrainingMatchScenario.RobotConfig bot : scenario.blueRobots()) {
            bots.add(new BotSpec(bot.archetype(), false, bot.startingPose(), bot.preloadFuel(), withClimbers));
        }
        for (TrainingMatchScenario.RobotConfig bot : scenario.redRobots()) {
            bots.add(new BotSpec(bot.archetype(), true, bot.startingPose(), bot.preloadFuel(), withClimbers));
        }
        return new Config(seed, scenario.durationSeconds(), 15.0,
                scenario.fieldFuelCount(), 'R', bots);
    }

    public static Result run(Config config) {
        long wallStart = System.currentTimeMillis();
        char savedSeed = HubSchedule.getShiftSeed();
        MatchDeterminism.seed(config.seed());
        boolean logging = config.logPath() != null && !config.logPath().isBlank();
        if (logging) {
            StandaloneReplay.start(config.logPath());
        }
        try {
            HubSchedule.setShiftSeed(config.shiftSeed());
            FuelStore fuel = FuelStore.scatterSeeded(config.seed(), config.fuelCount());
            List<StandaloneBot> bots = new ArrayList<>();
            for (int i = 0; i < config.bots().size(); i++) {
                BotSpec spec = config.bots().get(i);
                bots.add(new StandaloneBot(
                        spec.startPose(), spec.archetype(), spec.isRed(), spec.preload(), i,
                        spec.hasClimber()));
            }
            StandaloneReferee referee = new StandaloneReferee();
            double[][] markSeconds = new double[bots.size()][bots.size()];

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
                HubSchedule.Phase phase = HubSchedule.phaseFor(remaining, isAuto);
                for (int i = 0; i < bots.size(); i++) {
                    StandaloneBot bot = bots.get(i);
                    List<BotView> allies = new ArrayList<>();
                    List<BotView> opponents = new ArrayList<>();
                    List<Integer> opponentIndices = new ArrayList<>();
                    for (int j = 0; j < bots.size(); j++) {
                        if (bots.get(j).isRed() == bot.isRed()) {
                            allies.add(views.get(j));
                        } else {
                            opponents.add(views.get(j));
                            opponentIndices.add(j);
                        }
                    }
                    int diff = bot.isRed() ? red - blue : blue - red;
                    int markIdx = nearestOpponentIndex(
                            bot.getPose().getTranslation(), opponents, opponentIndices);
                    Pose2d markPose = markIdx >= 0 ? views.get(markIdx).pose() : bot.getPose();
                    bot.step(DT_SEC, remaining, isAuto, fuel, config.shiftSeed(),
                            diff, timeUntilShift, allies, opponents, markPose, elapsed);

                    // Teleop defensive attribution: credit the defender for the
                    // time it spent marking an opponent whose hub was live — the
                    // window a mark can actually suppress scoring.
                    if (!isAuto && markIdx >= 0 && isMarkingObjective(bot.getObjective())
                            && HubSchedule.isHubActive(bots.get(markIdx).isRed(), phase, config.shiftSeed())) {
                        markSeconds[i][markIdx] += DT_SEC;
                    }
                }
                referee.update(bots, DT_SEC);
                if (logging) {
                    StandaloneReplay.recordTick(remaining, bots, fuel, blue, red);
                }
                separatePeers(bots);
            }

            int blue = 0;
            int red = 0;
            int blueAuto = 0;
            int redAuto = 0;
            int blueTeleop = 0;
            int redTeleop = 0;
            int pickedUp = 0;
            int attemptedShots = 0;
            int missedShots = 0;
            int escapes = 0;
            double[] path = new double[bots.size()];
            int[] scored = new int[bots.size()];
            BotTelemetry[] telemetry = new BotTelemetry[bots.size()];
            for (int i = 0; i < bots.size(); i++) {
                StandaloneBot bot = bots.get(i);
                if (bot.isRed()) {
                    red += bot.getScored();
                    redAuto += bot.getAutoScored();
                    redTeleop += bot.getTeleopScored();
                } else {
                    blue += bot.getScored();
                    blueAuto += bot.getAutoScored();
                    blueTeleop += bot.getTeleopScored();
                }
                pickedUp += bot.getPickedUp();
                attemptedShots += bot.getAttemptedShots();
                missedShots += bot.getMissedShots();
                escapes += bot.getEscapes();
                path[i] = bot.getPathLengthM();
                scored[i] = bot.getScored();
                telemetry[i] = new BotTelemetry(
                        bot.isRed(), bot.getArchetype().name(),
                        bot.getAutoScored(), bot.getTeleopScored(), bot.getScored(),
                        bot.getPickedUp(), bot.getAttemptedShots(), bot.getMissedShots(),
                        bot.getEscapes(), bot.getPathLengthM(),
                        bot.getWastedFuel(), bot.getShuttledFuel(),
                        bot.isClimbed(), bot.getClimbArrivalSec(),
                        bot.getMinorFouls(), bot.getMajorFouls(),
                        bot.getHubActiveTeleopSec());
            }
            String winner = blue > red ? "Blue" : red > blue ? "Red" : "Tie";
            long wallMs = System.currentTimeMillis() - wallStart;
            return new Result(ticks, blue, red, blueAuto, redAuto, blueTeleop, redTeleop,
                    winner, pickedUp, fuel.size(), attemptedShots, missedShots, escapes,
                    path, scored, telemetry, markSeconds, wallMs);
        } finally {
            if (logging) {
                StandaloneReplay.end();
            }
            MatchDeterminism.clearSeed();
            HubSchedule.setShiftSeed(savedSeed);
        }
    }

    /** Roster index of the nearest opponent, or -1 if there are none. */
    private static int nearestOpponentIndex(
            Translation2d at, List<BotView> opponents, List<Integer> opponentIndices) {
        int best = -1;
        double bestDist = Double.MAX_VALUE;
        for (int k = 0; k < opponents.size(); k++) {
            double d = at.getDistance(opponents.get(k).pose().getTranslation());
            if (d < bestDist) {
                bestDist = d;
                best = opponentIndices.get(k);
            }
        }
        return best;
    }

    /** Defensive postures whose intent is to physically mark an opponent. */
    static boolean isMarkingObjective(StrategicObjective objective) {
        return objective == StrategicObjective.DENY_SHOOTING_LANE
                || objective == StrategicObjective.SHADOW_MIDLINE
                || objective == StrategicObjective.LEAD_INTERCEPT
                || objective == StrategicObjective.CHOKE_TRENCH;
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

    public static String toJsonLine(Config config, Result result, String variant, int replica) {
        StringBuilder sb = new StringBuilder(512);
        String sha;
        try {
            sha = frc.robot.BuildConstants.GIT_SHA;
        } catch (Throwable t) {
            sha = "standalone";
        }
        int dirty;
        try {
            dirty = frc.robot.BuildConstants.DIRTY;
        } catch (Throwable t) {
            dirty = 0;
        }

        sb.append('{');
        sb.append("\"schemaVersion\":2");
        sb.append(",\"gitSha\":\"").append(sha).append('"');
        sb.append(",\"dirty\":").append(dirty);
        sb.append(",\"stamp\":\"").append(LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"))).append('"');
        sb.append(",\"seed\":").append(config.seed());
        sb.append(",\"variant\":\"").append(variant).append('"');
        sb.append(",\"replica\":").append(replica);
        sb.append(",\"durationSec\":").append(config.durationSec());
        sb.append(",\"autoSec\":").append(config.autoSec());
        sb.append(",\"fieldFuelCount\":").append(config.fuelCount());
        sb.append(",\"attemptedShots\":").append(result.attemptedShots());
        sb.append(",\"missedShots\":").append(result.missedShots());
        sb.append(",\"escapes\":").append(result.escapes());
        sb.append(",\"winner\":\"").append(result.winner()).append('"');
        sb.append(",\"blueTotal\":").append(result.blueScored());
        sb.append(",\"redTotal\":").append(result.redScored());
        sb.append(",\"margin\":").append(Math.abs(result.blueScored() - result.redScored()));
        sb.append(",\"blueAutoFuel\":").append(result.blueAutoFuel());
        sb.append(",\"redAutoFuel\":").append(result.redAutoFuel());
        sb.append(",\"blueTeleopFuel\":").append(result.blueTeleopFuel());
        sb.append(",\"redTeleopFuel\":").append(result.redTeleopFuel());
        sb.append(",\"blueFuel\":").append(result.blueScored());
        sb.append(",\"redFuel\":").append(result.redScored());

        int blueClimbCount = 0;
        int redClimbCount = 0;
        int blueMinor = 0;
        int blueMajor = 0;
        int redMinor = 0;
        int redMajor = 0;
        int blueWasted = 0;
        int redWasted = 0;
        for (BotTelemetry b : result.botTelemetry()) {
            if (b.isRed()) {
                redMinor += b.minorFouls();
                redMajor += b.majorFouls();
                redWasted += b.wastedFuel();
                if (b.climbed()) {
                    redClimbCount++;
                }
            } else {
                blueMinor += b.minorFouls();
                blueMajor += b.majorFouls();
                blueWasted += b.wastedFuel();
                if (b.climbed()) {
                    blueClimbCount++;
                }
            }
        }
        // Points are mirrored from MatchScoreTracker: a foul credits the opponent.
        int bluePenaltyPoints = 5 * redMinor + 15 * redMajor;
        int redPenaltyPoints = 5 * blueMinor + 15 * blueMajor;

        sb.append(",\"blueClimb\":").append(blueClimbCount * POINTS_PER_CLIMB_MIRROR);
        sb.append(",\"redClimb\":").append(redClimbCount * POINTS_PER_CLIMB_MIRROR);
        sb.append(",\"blueClimbCount\":").append(blueClimbCount);
        sb.append(",\"redClimbCount\":").append(redClimbCount);
        sb.append(",\"blueFouls\":").append(blueMinor + blueMajor);
        sb.append(",\"redFouls\":").append(redMinor + redMajor);
        sb.append(",\"bluePenaltyPoints\":").append(bluePenaltyPoints);
        sb.append(",\"redPenaltyPoints\":").append(redPenaltyPoints);
        sb.append(",\"blueWastedFuel\":").append(blueWasted);
        sb.append(",\"redWastedFuel\":").append(redWasted);
        sb.append(",\"playerBlueFuel\":0");
        sb.append(",\"playerRedFuel\":0");
        sb.append(",\"blueReconciliationResidual\":0");
        sb.append(",\"redReconciliationResidual\":0");
        sb.append(",\"blueUnattributedFuel\":0");
        sb.append(",\"redUnattributedFuel\":0");

        List<String> blueArchetypes = new ArrayList<>();
        List<Double> bluePaths = new ArrayList<>();
        List<String> redArchetypes = new ArrayList<>();
        List<Double> redPaths = new ArrayList<>();
        List<Integer> blueScores = new ArrayList<>();
        List<Integer> redScores = new ArrayList<>();

        for (int i = 0; i < config.bots().size(); i++) {
            BotSpec spec = config.bots().get(i);
            double p = i < result.pathLengthM().length ? Math.round(result.pathLengthM()[i] * 100.0) / 100.0 : 0.0;
            int s = i < result.botScored().length ? result.botScored()[i] : 0;
            if (spec.isRed()) {
                redArchetypes.add(spec.archetype().name());
                redPaths.add(p);
                redScores.add(s);
            } else {
                blueArchetypes.add(spec.archetype().name());
                bluePaths.add(p);
                blueScores.add(s);
            }
        }

        sb.append(",\"botFuelScored\":").append(redScores);
        sb.append(",\"allyFuelScored\":").append(blueScores);

        appendBotsBlock(sb, "redBots", redArchetypes, redPaths);
        appendBotsBlock(sb, "blueBots", blueArchetypes, bluePaths);
        appendBotTelemetry(sb, result.botTelemetry());
        appendMarkSeconds(sb, result.markSeconds());

        sb.append(",\"logPath\":\"").append(config.logPath() == null ? "" : config.logPath()).append('"');
        sb.append(",\"reportPath\":\"\"");
        sb.append(",\"health\":{\"loopOverruns\":0,\"maxRobotPeriodicMs\":0.0}");
        sb.append('}');
        return sb.toString();
    }

    private static void appendBotsBlock(StringBuilder sb, String label,
            List<String> archetypes, List<Double> paths) {
        sb.append(",\"").append(label).append("\":{");
        sb.append("\"archetype\":[");
        for (int i = 0; i < archetypes.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append('"').append(archetypes.get(i)).append('"');
        }
        sb.append("],\"pathLengthM\":[");
        for (int i = 0; i < paths.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(paths.get(i));
        }
        sb.append("],\"maxContiguousStallSec\":[");
        for (int i = 0; i < paths.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append("0.0");
        }
        sb.append("],\"maxConsecutiveRecoveries\":[");
        for (int i = 0; i < paths.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append('0');
        }
        sb.append("],\"recoveryEventCount\":[");
        for (int i = 0; i < paths.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append('0');
        }
        sb.append("]}");
    }

    /**
     * Per-roster-slot block for the individual-EPA evaluator. Field names are
     * lowerCamel and the array is in roster order (see {@link BotTelemetry}).
     */
    private static void appendBotTelemetry(StringBuilder sb, BotTelemetry[] telemetry) {
        sb.append(",\"perBot\":[");
        for (int i = 0; i < telemetry.length; i++) {
            if (i > 0) {
                sb.append(',');
            }
            BotTelemetry b = telemetry[i];
            sb.append('{');
            sb.append("\"roster\":").append(i);
            sb.append(",\"alliance\":\"").append(b.isRed() ? "red" : "blue").append('"');
            sb.append(",\"archetype\":\"").append(b.archetype()).append('"');
            sb.append(",\"autoScored\":").append(b.autoScored());
            sb.append(",\"teleopScored\":").append(b.teleopScored());
            sb.append(",\"scored\":").append(b.scored());
            sb.append(",\"pickedUp\":").append(b.pickedUp());
            sb.append(",\"attemptedShots\":").append(b.attemptedShots());
            sb.append(",\"missedShots\":").append(b.missedShots());
            sb.append(",\"escapes\":").append(b.escapes());
            sb.append(",\"pathLengthM\":").append(b.pathLengthM());
            sb.append(",\"wastedFuel\":").append(b.wastedFuel());
            sb.append(",\"shuttledFuel\":").append(b.shuttledFuel());
            sb.append(",\"climbed\":").append(b.climbed());
            sb.append(",\"climbArrivalSec\":").append(b.climbArrivalSec());
            sb.append(",\"minorFouls\":").append(b.minorFouls());
            sb.append(",\"majorFouls\":").append(b.majorFouls());
            sb.append(",\"hubActiveTeleopSec\":").append(b.hubActiveTeleopSec());
            sb.append('}');
        }
        sb.append(']');
    }

    /**
     * Roster x roster matrix of teleop seconds defender {@code i} spent marking
     * opponent {@code j} while {@code j}'s hub was live. Consumed by the
     * model-based defensive-EPA residual in {@code tools/tune/sim_epa.py}.
     */
    private static void appendMarkSeconds(StringBuilder sb, double[][] markSeconds) {
        sb.append(",\"markSeconds\":[");
        for (int i = 0; i < markSeconds.length; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('[');
            for (int j = 0; j < markSeconds[i].length; j++) {
                if (j > 0) {
                    sb.append(',');
                }
                sb.append(markSeconds[i][j]);
            }
            sb.append(']');
        }
        sb.append(']');
    }

    public static void main(String[] args) {
        long seed = 42;
        String seedsStr = null;
        double durationSec = 150.0;
        double autoSec = 15.0;
        int fuelCount = 54;
        char shiftSeed = 'R';
        String weightsSpec = null;
        String jsonlPath = null;
        String logPath = null;
        String variant = "standalone";
        int replica = 0;
        boolean climb = false;

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--seed" -> { if (i + 1 < args.length) seed = Long.parseLong(args[++i]); }
                case "--seeds" -> { if (i + 1 < args.length) seedsStr = args[++i]; }
                case "--duration" -> { if (i + 1 < args.length) durationSec = Double.parseDouble(args[++i]); }
                case "--auto" -> { if (i + 1 < args.length) autoSec = Double.parseDouble(args[++i]); }
                case "--fuel" -> { if (i + 1 < args.length) fuelCount = Integer.parseInt(args[++i]); }
                case "--shiftSeed" -> { if (i + 1 < args.length) shiftSeed = args[++i].charAt(0); }
                case "--weights" -> { if (i + 1 < args.length) weightsSpec = args[++i]; }
                case "--jsonl" -> { if (i + 1 < args.length) jsonlPath = args[++i]; }
                case "--log" -> { if (i + 1 < args.length) logPath = args[++i]; }
                case "--variant" -> { if (i + 1 < args.length) variant = args[++i]; }
                case "--replica" -> { if (i + 1 < args.length) replica = Integer.parseInt(args[++i]); }
                case "--climb" -> climb = true;
                default -> { /* ignore unrecognized option */ }
            }
        }

        List<Long> seeds = new ArrayList<>();
        if (seedsStr != null && !seedsStr.isBlank()) {
            for (String s : seedsStr.split("[,;]")) {
                s = s.trim();
                if (!s.isEmpty()) {
                    seeds.add(Long.parseLong(s));
                }
            }
        } else {
            seeds.add(seed);
        }

        if (weightsSpec != null && !weightsSpec.isBlank()) {
            PolicyWeights.setActive(PolicyWeights.fromString(weightsSpec));
        }

        try {
            for (long s : seeds) {
                Config full = default3v3(s, climb);
                String matchLog = logPath;
                if (matchLog != null && seeds.size() > 1) {
                    matchLog = matchLog.endsWith(".wpilog")
                            ? matchLog.substring(0, matchLog.length() - 7) + "_seed" + s + ".wpilog"
                            : matchLog + "_seed" + s;
                }
                Config config = new Config(s, durationSec, autoSec, fuelCount, shiftSeed, full.bots(), matchLog);
                Result res = run(config);
                System.out.printf(
                        "[Standalone] Seed %d: Blue %d (auto %d, teleop %d) - Red %d (auto %d, teleop %d)"
                                + " [%s] volleys %d/%d scored (miss %d), escapes %d, in %d ms%n",
                        s, res.blueScored(), res.blueAutoFuel(), res.blueTeleopFuel(),
                        res.redScored(), res.redAutoFuel(), res.redTeleopFuel(),
                        res.winner(), res.blueScored() + res.redScored(), res.attemptedShots(),
                        res.missedShots(), res.escapes(), res.wallMs());

                if (jsonlPath != null && !jsonlPath.isBlank()) {
                    String line = toJsonLine(config, res, variant, replica);
                    Path outP = Paths.get(jsonlPath);
                    if (outP.getParent() != null) {
                        Files.createDirectories(outP.getParent());
                    }
                    Files.writeString(
                            outP,
                            line + System.lineSeparator(),
                            StandardOpenOption.CREATE,
                            StandardOpenOption.APPEND);
                }
            }
        } catch (Exception e) {
            System.err.println("[Standalone] ERROR: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        } finally {
            PolicyWeights.resetToDefault();
        }
    }
}
