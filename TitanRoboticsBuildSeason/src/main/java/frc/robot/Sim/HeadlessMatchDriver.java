package frc.robot.Sim;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Intelligence.Archetype;
import org.littletonrobotics.junction.Logger;

/**
 * Self-driving entry point for headless AI-vs-AI training matches.
 *
 * <p>Enabled only when the {@code frc.headless} system property is set (the
 * {@code simulateJavaRelease -Pheadless} path sets it; see build.gradle).
 * The driver runs on its own thread inside the normal
 * robot process: it applies a {@link TrainingMatchScenario}, sequences the
 * DriverStation through AUTO → disabled → TELEOP exactly as an operator
 * would, then snapshots the rulebook scoreboard, ends the AdvantageKit
 * Logger (the {@code .wpilog} replay), writes a markdown report, and exits.
 *
 * <p>No GUI, gamepad, or operator input is required. The full robot loop
 * ({@code Robot.robotPeriodic/simulationPeriodic}, hub schedule, referee,
 * scoreboard) runs unmodified, so a headless match exercises the same code
 * as a GUI sim match.
 *
 * <p>System properties (all optional, defaults shown):
 * <ul>
 * <li>{@code frc.headless.seed} — scenario seed (default 2026)
 * <li>{@code frc.headless.durationSec} — scenario clock, seconds (default 150)
 * <li>{@code frc.headless.autoSec} — autonomous phase, seconds (default 15)
 * <li>{@code frc.headless.disabledGapSec} — pause between AUTO and TELEOP (default 3)
 * <li>{@code frc.headless.bootWaitSec} — settle time before applying the scenario (default 8)
 * <li>{@code frc.headless.fieldFuelCount} — loose field fuel pieces (default 108)
 * <li>{@code frc.headless.logDir} — wpilog output dir (default {@code logs})
 * <li>{@code frc.headless.reportDir} — markdown report dir (default {@code reports})
 * </ul>
 */
public final class HeadlessMatchDriver {
    private static final String PROP_PREFIX = "frc.headless.";

    private static final long DEFAULT_SEED = 2026L;
    private static final double DEFAULT_DURATION_SEC = 150.0;
    private static final double DEFAULT_AUTO_SEC = 15.0;
    private static final double DEFAULT_DISABLED_GAP_SEC = 3.0;
    private static final double DEFAULT_BOOT_WAIT_SEC = 8.0;
    private static final int DEFAULT_FIELD_FUEL_COUNT = 108;
    private static final String DEFAULT_LOG_DIR = "logs";
    private static final String DEFAULT_REPORT_DIR = "reports";

    private static volatile String cachedLogPath;

    private HeadlessMatchDriver() {}

    /** Immutable, validated headless-match configuration. */
    public record Options(
            long seed,
            double durationSec,
            double autoSec,
            double disabledGapSec,
            double bootWaitSec,
            int fieldFuelCount,
            String logDir,
            String reportDir,
            String resultJsonl,
            String variant,
            int replica) {
        /** Teleop phase length derived from the scenario clock minus autonomous. */
        public double teleopSec() {
            return durationSec - autoSec;
        }
    }

    /**
     * Per-bot score-rig metrics, parallel to the {@code botFuelScored} /
     * {@code allyFuelScored} arrays. Index-aligned with those arrays, so entry
     * {@code i} describes the same robot in both.
     *
     * <p>Read by {@code tools/score/compare.py} to gate on freezes, recovery
     * loops, and defensive engagement rather than on fuel share alone.
     */
    public record BotMetrics(
            String[] archetype,
            double[] pathLengthM,
            double[] maxContiguousStallSec,
            int[] maxConsecutiveRecoveries,
            int[] recoveryEventCount) {}

    /**
     * Per-match loop-timing health, as measured by {@link LoopHealth}.
     *
     * <p>These are measurement-validity fields, not robot behaviour: a worker
     * starved by its siblings does not crash, it quietly produces a different
     * match. Carrying the numbers in the row means a degraded sweep is visible in
     * the data rather than only in a log, so {@code tools/score/compare.py} can
     * refuse to compare a run it knows was perturbed.
     *
     * <p>{@code loopOverruns} and {@code maxRobotPeriodicMs} are -1 / -1.0 when the
     * match did not arm {@link LoopHealth} -- the case for any non-headless caller
     * of {@link #toJsonLine} and for the unit tests.
     */
    public record MatchHealth(int loopOverruns, double maxRobotPeriodicMs) {
        /** Sentinel for "not measured" rather than "measured as zero". */
        public static final MatchHealth UNKNOWN = new MatchHealth(-1, -1.0);

        /** Snapshot of the live counters, or {@link #UNKNOWN} when not armed. */
        public static MatchHealth capture() {
            return LoopHealth.isArmed()
                    ? new MatchHealth(LoopHealth.overrunCount(), LoopHealth.maxEpochSec() * 1000.0)
                    : UNKNOWN;
        }
    }
    /** Final scoreboard snapshot used for the console summary and markdown report. */
    public record MatchResult(
            long seed,
            double durationSec,
            double autoSec,
            int fieldFuelCount,
            String winner,
            int blueTotal,
            int redTotal,
            int margin,
            int blueAutoFuel,
            int redAutoFuel,
            int blueTeleopFuel,
            int redTeleopFuel,
            int blueClimb,
            int redClimb,
            int blueClimbCount,
            int redClimbCount,
            int blueFouls,
            int redFouls,
            int bluePenaltyPoints,
            int redPenaltyPoints,
            int blueWastedFuel,
            int redWastedFuel,
            int[] botFuelScored,
            int[] allyFuelScored,
            // The player's own fuel, attributed separately. Until this existed the
            // player called recordFuelScore directly, so its scores inflated an
            // alliance total that the per-bot table could not account for -- 13 of
            // 20 archived headless reports missed the per-bot sum by 2-10, always
            // on Blue. The two canaries below must be 0.
            int playerBlueFuel,
            int playerRedFuel,
            int blueReconciliationResidual,
            int redReconciliationResidual,
            int blueUnattributedFuel,
            int redUnattributedFuel,
            String variant,
            int replica,
            BotMetrics redBotMetrics,
            BotMetrics blueBotMetrics,
            String logPath,
            String reportPath,
            MatchHealth health) {}

    /** True when the {@code frc.headless} system property is set. */
    public static boolean isHeadless() {
        return System.getProperty("frc.headless") != null;
    }

    /** Parses and validates {@link Options} from system properties. */
    public static Options parseOptions() {
        long seed = getLongProperty("seed", DEFAULT_SEED);
        double durationSec = getDoubleProperty("durationSec", DEFAULT_DURATION_SEC);
        double autoSec = getDoubleProperty("autoSec", DEFAULT_AUTO_SEC);
        double disabledGapSec = getDoubleProperty("disabledGapSec", DEFAULT_DISABLED_GAP_SEC);
        double bootWaitSec = getDoubleProperty("bootWaitSec", DEFAULT_BOOT_WAIT_SEC);
        int fieldFuelCount = getIntProperty("fieldFuelCount", DEFAULT_FIELD_FUEL_COUNT);
        String logDir = System.getProperty(PROP_PREFIX + "logDir", DEFAULT_LOG_DIR);
        String reportDir = System.getProperty(PROP_PREFIX + "reportDir", DEFAULT_REPORT_DIR);
        String resultJsonl = System.getProperty(PROP_PREFIX + "resultJsonl", "").trim();
        // Sweep identity. Stamped into every JSONL row so tools/score/sweep.ps1
        // can resume (skip rows already present) and compare.py can group.
        String variant = System.getProperty(PROP_PREFIX + "variant", "baseline").trim();
        if (variant.isEmpty()) variant = "baseline";
        int replica = getIntProperty("replica", 0);

        if (!Double.isFinite(durationSec) || durationSec <= 0.0) {
            throw new IllegalArgumentException("frc.headless.durationSec must be finite and positive");
        }
        if (!Double.isFinite(autoSec) || autoSec < 0.0) {
            throw new IllegalArgumentException("frc.headless.autoSec must be finite and non-negative");
        }
        if (durationSec - autoSec < 1.0) {
            throw new IllegalArgumentException(
                    "frc.headless.durationSec must exceed frc.headless.autoSec by at least 1 s");
        }
        if (!Double.isFinite(disabledGapSec) || disabledGapSec < 0.0) {
            throw new IllegalArgumentException("frc.headless.disabledGapSec must be finite and non-negative");
        }
        if (!Double.isFinite(bootWaitSec) || bootWaitSec < 0.0) {
            throw new IllegalArgumentException("frc.headless.bootWaitSec must be finite and non-negative");
        }
        if (fieldFuelCount < 0 || fieldFuelCount > TrainingMatchScenario.MAX_FIELD_FUEL_COUNT) {
            throw new IllegalArgumentException("frc.headless.fieldFuelCount must be between 0 and "
                    + TrainingMatchScenario.MAX_FIELD_FUEL_COUNT);
        }
        if (replica < 0) {
            throw new IllegalArgumentException("frc.headless.replica must be non-negative");
        }
        return new Options(seed, durationSec, autoSec, disabledGapSec, bootWaitSec, fieldFuelCount, logDir,
                reportDir, resultJsonl.isEmpty() ? null : resultJsonl, variant, replica);
    }

    /**
     * Builds the training scenario: the standard one-click 3v3 rosters with
     * the configured seed, scenario-clock duration, and field fuel count.
     * Counts above the 54-ball lightweight layout draw from the full-density
     * preplaced positions (see {@code GameSim}).
     */
    public static TrainingMatchScenario buildScenario(Options options) {
        TrainingMatchScenario base = TrainingMatchScenario.default3v3(options.seed());
        return new TrainingMatchScenario(
                base.seed(), options.durationSec(), options.fieldFuelCount(),
                base.blueRobots(), base.redRobots());
    }

    /**
     * Resolves the wpilog path for this run. Cached so {@code Robot}'s
     * constructor (which attaches the {@code WPILOGWriter} before
     * {@code Logger.start()}) and the driver thread report the same file.
     */
    public static synchronized String resolveLogPath() {
        if (cachedLogPath == null) {
            Options options;
            try {
                options = parseOptions();
            } catch (IllegalArgumentException e) {
                options = new Options(DEFAULT_SEED, DEFAULT_DURATION_SEC, DEFAULT_AUTO_SEC,
                        DEFAULT_DISABLED_GAP_SEC, DEFAULT_BOOT_WAIT_SEC, DEFAULT_FIELD_FUEL_COUNT,
                        DEFAULT_LOG_DIR, DEFAULT_REPORT_DIR, null, "baseline", 0);
            }
            String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
            cachedLogPath = options.logDir() + "/headless_3v3" + logTag(options)
                    + "_seed" + options.seed() + "_" + stamp + ".wpilog";
        }
        return cachedLogPath;
    }

    /**
     * Filename discriminator for the replay log and the markdown report.
     *
     * <p>Variant and replica must be in the name. A parallel score-rig sweep
     * ({@code tools/score/sweep.ps1}) starts both replicas of a seed within the
     * same second, so a seed-plus-second stamp alone made them collide on one
     * path and silently overwrite each other's replay — a 16-match baseline
     * wrote only 8 wpilogs, which is how the loss was noticed.
     *
     * <p>The report path needs the same treatment and did not have it: two
     * same-seed workers finished into the same second and produced one file
     * instead of two. Reusing this tag for both keeps one rule, not two.
     */
    static String logTag(Options options) {
        StringBuilder tag = new StringBuilder();
        if (options.variant() != null && !options.variant().isBlank()
                && !options.variant().equals("baseline")) {
            tag.append('_').append(options.variant().replaceAll("[^A-Za-z0-9_.-]", "_"));
        }
        if (options.replica() > 0) {
            tag.append("_r").append(options.replica());
        }
        return tag.toString();
    }

    /** Test-only hook to reset the cached log path between cases. */
    static synchronized void clearCachedLogPathForTests() {
        cachedLogPath = null;
    }

    /** Starts the headless driver thread; no-op unless {@link #isHeadless()}. */
    public static void maybeStartHeadlessMatch() {
        if (!isHeadless()) {
            return;
        }
        // Arm before the match starts so the first epoch is already counted.
        LoopHealth.arm();
        Thread driver = new Thread(HeadlessMatchDriver::runHeadlessMatch, "HeadlessMatchDriver");
        driver.setDaemon(false);
        driver.start();
    }

    private static void runHeadlessMatch() {
        int exitCode = 0;
        try {
            Options options = parseOptions();
            System.out.println("[Headless] boot settle " + options.bootWaitSec() + " s");
            sleepSec(options.bootWaitSec());

            TrainingMatchScenario scenario = buildScenario(options);
            GameSim.getInstance().resetGame(scenario);
            if (!AIRobotSim.getInstance().isTrainingScenarioActive()) {
                throw new IllegalStateException("Training scenario did not activate after resetGame");
            }
            System.out.println("[Headless] 3v3 scenario active (seed=" + options.seed()
                    + ", duration=" + options.durationSec() + " s"
                    + ", fieldFuel=" + options.fieldFuelCount() + ")");

            setDsMode(true, true);
            System.out.println("[Headless] AUTONOMOUS for " + options.autoSec() + " s");
            sleepSec(options.autoSec());

            setDsMode(false, false);
            System.out.println("[Headless] disabled gap " + options.disabledGapSec() + " s");
            sleepSec(options.disabledGapSec());

            setDsMode(false, true);
            System.out.println("[Headless] TELEOP until scenario clock expires");
            double deadline = options.durationSec() + 120.0;
            double waited = 0.0;
            while (GameSim.getInstance().getSimTimeRemainingSec() > 0.0 && waited < deadline) {
                sleepSec(0.5);
                waited += 0.5;
                MatchScoreTracker tracker = MatchScoreTracker.getInstance();
                double remaining = GameSim.getInstance().getSimTimeRemainingSec();
                // Refresh here rather than trusting whichever caller last
                // touched the schedule: this is the authoritative poll for the
                // replay, and the robot loop's own update can be skipped.
                HubSchedule.refreshFromMatchState();
                Logger.recordOutput("Headless/Phase",
                        HubSchedule.currentPhase().name());
                Logger.recordOutput("Headless/TimeRemainingSec", remaining);
                Logger.recordOutput("Headless/BlueTotal", tracker.getBlueTotalScore());
                Logger.recordOutput("Headless/RedTotal", tracker.getRedTotalScore());
                Logger.recordOutput("Headless/BlueHubActive",
                        HubSchedule.isHubActiveNow(false));
                Logger.recordOutput("Headless/RedHubActive",
                        HubSchedule.isHubActiveNow(true));
                Logger.recordOutput("Headless/ShiftSeed",
                        (double) String.valueOf(HubSchedule.getShiftSeed()).charAt(0));
            }
            if (GameSim.getInstance().getSimTimeRemainingSec() > 0.0) {
                System.out.println("[Headless] WARNING: scenario clock did not expire before deadline");
            }

            setDsMode(false, false);
            sleepSec(3.0);

            String reportPath = writeMatchReport(options);
            MatchResult result = snapshotResult(options, reportPath);
            System.out.println(formatConsoleSummary(result));
        } catch (Throwable t) {
            System.out.println("[Headless] FATAL: " + t);
            t.printStackTrace(System.out);
            exitCode = 2;
        } finally {
            try {
                Logger.end();
            } catch (Throwable ignored) {
            }
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            System.exit(exitCode);
        }
    }

    private static void setDsMode(boolean autonomous, boolean enabled) {
        DriverStationSim.setAutonomous(autonomous);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private static void sleepSec(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000.0));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("Headless driver interrupted", e);
        }
    }

    /**
     * Collects score-rig metrics for an ordered roster. Missing bots contribute
     * neutral entries rather than shortening the array, so indices stay aligned
     * with the fuel arrays.
     */
    private static BotMetrics collectBotMetrics(List<Archetype> roster,
            java.util.function.IntFunction<AIRobotInstance> lookup) {
        int n = Math.max(roster.size(), 1);
        String[] archetype = new String[n];
        double[] path = new double[n];
        double[] stall = new double[n];
        int[] consec = new int[n];
        int[] events = new int[n];
        for (int i = 0; i < n; i++) {
            archetype[i] = (i < roster.size() && roster.get(i) != null)
                    ? roster.get(i).name() : "NONE";
            AIRobotInstance bot = lookup.apply(i);
            if (bot == null) continue;
            BotMatchMetrics m = bot.getMatchMetrics();
            path[i] = round2(m.getIntegratedPathLengthM());
            stall[i] = round2(m.getMaxContiguousStallSec());
            consec[i] = m.getMaxConsecutiveRecoveries();
            events[i] = m.getRecoveryEventCount();
        }
        return new BotMetrics(archetype, path, stall, consec, events);
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    /** Reads the rulebook tracker + latched NT result into a {@link MatchResult}. */
    static MatchResult snapshotResult(Options options, String reportPath) {
        MatchScoreTracker tracker = MatchScoreTracker.getInstance();
        int blue = tracker.getBlueTotalScore();
        int red = tracker.getRedTotalScore();
        String winner = blue > red ? "Blue" : (red > blue ? "Red" : "Tie");
        String ntWinner = SmartDashboard.getString("Training/Result/Winner", winner);

        // Rosters come from the live sim so the JSONL is self-describing: the
        // role-aware guardrails in tools/score/ key off the declared archetype,
        // never off observed scoring. A missing bot yields a neutral entry rather
        // than shortening the array, so indices stay aligned with the fuel arrays.
        AIRobotSim sim = AIRobotSim.getInstance();
        List<Archetype> redRoster = new ArrayList<>();
        redRoster.add(sim.getOpponents().isEmpty() ? null : sim.getOpponents().get(0).getArchetype());
        for (int i = 1; i < 3; i++) {
            redRoster.add(i - 1 < sim.getAdditionalBots().size()
                    ? sim.getAdditionalBots().get(i - 1).getArchetype() : null);
        }
        List<Archetype> blueRoster = new ArrayList<>();
        blueRoster.add(sim.getTrainingBluePrimaryBot() == null
                ? null : sim.getTrainingBluePrimaryBot().getArchetype());
        for (int i = 0; i < 2; i++) {
            blueRoster.add(i < sim.getAllyBots().size()
                    ? sim.getAllyBots().get(i).getArchetype() : null);
        }

        BotMetrics redMetrics = collectBotMetrics(redRoster, i -> {
            if (i == 0) return sim.getOpponents().isEmpty() ? null : sim.getOpponents().get(0);
            return i - 1 < sim.getAdditionalBots().size() ? sim.getAdditionalBots().get(i - 1) : null;
        });
        BotMetrics blueMetrics = collectBotMetrics(blueRoster, i -> {
            if (i == 0) return sim.getTrainingBluePrimaryBot();
            return i - 1 < sim.getAllyBots().size() ? sim.getAllyBots().get(i - 1) : null;
        });

        return new MatchResult(
                options.seed(), options.durationSec(), options.autoSec(), options.fieldFuelCount(),
                ntWinner, blue, red, Math.abs(blue - red),
                tracker.getBlueAutoFuelCount(), tracker.getRedAutoFuelCount(),
                tracker.getBlueTeleopFuelCount(), tracker.getRedTeleopFuelCount(),
                tracker.getBlueClimbScore(), tracker.getRedClimbScore(),
                tracker.getBlueClimbCount(), tracker.getRedClimbCount(),
                tracker.getBlueFoulCount(), tracker.getRedFoulCount(),
                tracker.getBluePenaltyScore(), tracker.getRedPenaltyScore(),
                tracker.getBlueWastedFuelCount(), tracker.getRedWastedFuelCount(),
                new int[] {tracker.getBot0FuelScored(), tracker.getBot1FuelScored(),
                        tracker.getBot2FuelScored()},
                new int[] {tracker.getAlly0FuelScored(), tracker.getAlly1FuelScored(),
                        tracker.getAlly2FuelScored()},
                tracker.getPlayerBlueFuelScored(), tracker.getPlayerRedFuelScored(),
                tracker.getBlueReconciliationResidual(), tracker.getRedReconciliationResidual(),
                tracker.getBlueUnattributedFuel(), tracker.getRedUnattributedFuel(),
                options.variant(), options.replica(),
                redMetrics, blueMetrics,
                resolveLogPath(), reportPath,
                MatchHealth.capture());
    }

    /**
     * Bumped whenever the JSONL field set changes shape, so
     * {@code tools/score/compare.py} can refuse a file it does not understand
     * instead of silently reading missing fields as zero.
     *
     * <p>v2 adds the {@code health} block ({@code loopOverruns},
     * {@code maxRobotPeriodicMs}). v1 rows carry no health at all, so they cannot
     * be distinguished from a clean run -- that is exactly why the v1 baseline
     * sweep had to be discarded rather than re-read. See KNOWN_ISSUES.md.
     */
    public static final int JSONL_SCHEMA_VERSION = 2;

    private static String esc(String s) {
        if (s == null) return "";
        return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ");
    }

    private static String jsonInts(int[] a) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < a.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(a[i]);
        }
        return sb.append(']').toString();
    }

    private static String jsonDoubles(double[] a) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < a.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(a[i]);
        }
        return sb.append(']').toString();
    }

    private static String jsonStrings(String[] a) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < a.length; i++) {
            if (i > 0) sb.append(',');
            sb.append('"').append(esc(a[i])).append('"');
        }
        return sb.append(']').toString();
    }

    private static void appendBotMetrics(StringBuilder sb, String label, BotMetrics m) {
        sb.append(",\"").append(label).append("\":{");
        if (m == null) {
            sb.append("\"archetype\":[],\"pathLengthM\":[],\"maxContiguousStallSec\":[],")
                    .append("\"maxConsecutiveRecoveries\":[],\"recoveryEventCount\":[]");
        } else {
            sb.append("\"archetype\":").append(jsonStrings(m.archetype()))
                    .append(",\"pathLengthM\":").append(jsonDoubles(m.pathLengthM()))
                    .append(",\"maxContiguousStallSec\":").append(jsonDoubles(m.maxContiguousStallSec()))
                    .append(",\"maxConsecutiveRecoveries\":").append(jsonInts(m.maxConsecutiveRecoveries()))
                    .append(",\"recoveryEventCount\":").append(jsonInts(m.recoveryEventCount()));
        }
        sb.append('}');
    }

    /**
     * Serialises one match as a single JSONL line.
     *
     * <p>This is the machine-readable counterpart to {@link #formatReport}, and
     * replaces scraping the markdown table (the previous sweep parser split on
     * {@code |} and silently produced plausible-but-wrong columns). Carries the
     * git sha so a score can always be traced to the code that produced it.
     */
    public static String toJsonLine(MatchResult r) {
        String sha;
        try {
            sha = frc.robot.BuildConstants.GIT_SHA;
        } catch (Throwable t) {
            sha = "unknown";
        }
        StringBuilder sb = new StringBuilder(512);
        sb.append('{');
        sb.append("\"schemaVersion\":").append(JSONL_SCHEMA_VERSION);
        sb.append(",\"gitSha\":\"").append(esc(sha)).append('"');
        sb.append(",\"stamp\":\"").append(LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"))).append('"');
        sb.append(",\"seed\":").append(r.seed());
        sb.append(",\"variant\":\"").append(esc(r.variant())).append('"');
        sb.append(",\"replica\":").append(r.replica());
        sb.append(",\"durationSec\":").append(r.durationSec());
        sb.append(",\"autoSec\":").append(r.autoSec());
        sb.append(",\"fieldFuelCount\":").append(r.fieldFuelCount());
        sb.append(",\"winner\":\"").append(esc(r.winner())).append('"');
        sb.append(",\"blueTotal\":").append(r.blueTotal());
        sb.append(",\"redTotal\":").append(r.redTotal());
        sb.append(",\"margin\":").append(r.margin());
        sb.append(",\"blueAutoFuel\":").append(r.blueAutoFuel());
        sb.append(",\"redAutoFuel\":").append(r.redAutoFuel());
        sb.append(",\"blueTeleopFuel\":").append(r.blueTeleopFuel());
        sb.append(",\"redTeleopFuel\":").append(r.redTeleopFuel());
        // Convenience rollup so compare.py does not have to re-derive it; still
        // decomposed above so a gain can be attributed to fuel vs penalty.
        sb.append(",\"blueFuel\":").append(r.blueAutoFuel() + r.blueTeleopFuel());
        sb.append(",\"redFuel\":").append(r.redAutoFuel() + r.redTeleopFuel());
        sb.append(",\"blueClimb\":").append(r.blueClimb());
        sb.append(",\"redClimb\":").append(r.redClimb());
        sb.append(",\"blueClimbCount\":").append(r.blueClimbCount());
        sb.append(",\"redClimbCount\":").append(r.redClimbCount());
        sb.append(",\"blueFouls\":").append(r.blueFouls());
        sb.append(",\"redFouls\":").append(r.redFouls());
        // NOTE: bluePenaltyPoints is what BLUE received (Red fouled), and is
        // inside blueTotal. redPenaltyPoints is what RED received (Blue fouled).
        sb.append(",\"bluePenaltyPoints\":").append(r.bluePenaltyPoints());
        sb.append(",\"redPenaltyPoints\":").append(r.redPenaltyPoints());
        sb.append(",\"blueWastedFuel\":").append(r.blueWastedFuel());
        sb.append(",\"redWastedFuel\":").append(r.redWastedFuel());
        sb.append(",\"playerBlueFuel\":").append(r.playerBlueFuel());
        sb.append(",\"playerRedFuel\":").append(r.playerRedFuel());
        sb.append(",\"blueReconciliationResidual\":").append(r.blueReconciliationResidual());
        sb.append(",\"redReconciliationResidual\":").append(r.redReconciliationResidual());
        sb.append(",\"blueUnattributedFuel\":").append(r.blueUnattributedFuel());
        sb.append(",\"redUnattributedFuel\":").append(r.redUnattributedFuel());
        sb.append(",\"botFuelScored\":").append(jsonInts(r.botFuelScored()));
        sb.append(",\"allyFuelScored\":").append(jsonInts(r.allyFuelScored()));
        appendBotMetrics(sb, "redBots", r.redBotMetrics());
        appendBotMetrics(sb, "blueBots", r.blueBotMetrics());
        sb.append(",\"logPath\":\"").append(esc(r.logPath())).append('"');
        sb.append(",\"reportPath\":\"").append(esc(r.reportPath())).append('"');
        // Measurement-validity block, not robot behaviour. A row with overruns or
        // a runaway epoch is a match that was perturbed by machine load, and must
        // not be compared against a clean one as if it were a policy difference.
        MatchHealth h = r.health() == null ? MatchHealth.UNKNOWN : r.health();
        sb.append(",\"health\":{")
                .append("\"loopOverruns\":").append(h.loopOverruns())
                .append(",\"maxRobotPeriodicMs\":").append(h.maxRobotPeriodicMs())
                .append("}");
        sb.append('}');
        return sb.toString();
    }

    /** Appends one JSONL row when {@code -PresultJsonl} was supplied. */
    private static void writeResultJsonl(Options options, MatchResult result) {
        if (options == null || options.resultJsonl() == null || options.resultJsonl().isBlank()) {
            return;
        }
        try {
            Path out = Paths.get(options.resultJsonl());
            if (out.getParent() != null) {
                Files.createDirectories(out.getParent());
            }
            // Append: the sweep runs N workers against one file.
            Files.writeString(out, toJsonLine(result) + System.lineSeparator(),
                    java.nio.file.StandardOpenOption.CREATE,
                    java.nio.file.StandardOpenOption.APPEND);
            System.out.println("[Headless] jsonl: appended to " + out);
        } catch (IOException e) {
            System.err.println("[Headless] failed to append JSONL result: " + e.getMessage());
        }
    }

    /**
     * Markdown report filename. Carries the same {@link #logTag} discriminator as
     * the replay log, for the same reason: two replicas of one seed finish into
     * the same second under a parallel sweep, and an untagged name made them
     * overwrite each other's report.
     */
    static String reportFileName(Options options, String stamp) {
        return "headless_match_seed" + options.seed() + logTag(options) + "_" + stamp + ".md";
    }

    private static String writeMatchReport(Options options) throws IOException {
        Path dir = Paths.get(options.reportDir());
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        Path report = dir.resolve(reportFileName(options, stamp));
        MatchResult result = snapshotResult(options, report.toString());
        Files.writeString(report, formatReport(result));
        writeResultJsonl(options, result);
        System.out.println("[Headless] report: " + report);
        System.out.println("[Headless] replay log: " + result.logPath());
        return report.toString();
    }

    /** Renders the markdown match report from a {@link MatchResult}. */
    public static String formatReport(MatchResult r) {
        StringBuilder sb = new StringBuilder();
        sb.append("# Headless 3v3 Match Report\n\n");
        sb.append("- Seed: ").append(r.seed()).append("\n");
        sb.append("- Scenario clock: ").append(r.durationSec()).append(" s (AUTO ")
                .append(r.autoSec()).append(" s)\n");
        sb.append("- Field fuel: ").append(r.fieldFuelCount()).append("\n");
        sb.append("- Winner: **").append(r.winner()).append("** (Blue ").append(r.blueTotal())
                .append(" - Red ").append(r.redTotal()).append(", margin ").append(r.margin())
                .append(")\n\n");
        sb.append("## Scoreboard\n\n");
        sb.append("| | Blue | Red |\n|---|---|---|\n");
        sb.append("| Total | ").append(r.blueTotal()).append(" | ").append(r.redTotal()).append(" |\n");
        sb.append("| Auto fuel | ").append(r.blueAutoFuel()).append(" | ").append(r.redAutoFuel())
                .append(" |\n");
        sb.append("| Teleop fuel | ").append(r.blueTeleopFuel()).append(" | ")
                .append(r.redTeleopFuel()).append(" |\n");
        sb.append("| Wasted fuel (inactive hub) | ").append(r.blueWastedFuel()).append(" | ")
                .append(r.redWastedFuel()).append(" |\n");
        sb.append("| Climb points | ").append(r.blueClimb()).append(" | ").append(r.redClimb())
                .append(" |\n");
        sb.append("| Climbs (robots) | ").append(r.blueClimbCount()).append(" | ")
                .append(r.redClimbCount()).append(" |\n");
        sb.append("| Fouls committed | ").append(r.blueFouls()).append(" | ").append(r.redFouls())
                .append(" |\n");
        sb.append("| Penalty points conceded | ").append(r.bluePenaltyPoints()).append(" | ")
                .append(r.redPenaltyPoints()).append(" |\n\n");

        // Per-bot table. The Player row is what makes the column sum to the Total
        // row: player scores used to reach the alliance total with no per-slot
        // attribution, so 13 of 20 archived reports missed the sum by 2-10.
        sb.append("## Per-bot fuel scored\n\n");
        sb.append("| Bot | Fuel | Archetype | Path (m) | Max stall (s) | Max consec. recoveries |\n");
        sb.append("|---|---|---|---|---|---|\n");
        for (int i = 0; i < r.botFuelScored().length; i++) {
            sb.append("| Red Bot").append(i).append(" | ").append(r.botFuelScored()[i]).append(" | ")
                    .append(metric(r.redBotMetrics(), i, 0)).append(" | ")
                    .append(metric(r.redBotMetrics(), i, 1)).append(" | ")
                    .append(metric(r.redBotMetrics(), i, 2)).append(" | ")
                    .append(metric(r.redBotMetrics(), i, 3)).append(" |\n");
        }
        for (int i = 0; i < r.allyFuelScored().length; i++) {
            sb.append("| Blue Ally").append(i).append(" | ").append(r.allyFuelScored()[i])
                    .append(" | ")
                    .append(metric(r.blueBotMetrics(), i, 0)).append(" | ")
                    .append(metric(r.blueBotMetrics(), i, 1)).append(" | ")
                    .append(metric(r.blueBotMetrics(), i, 2)).append(" | ")
                    .append(metric(r.blueBotMetrics(), i, 3)).append(" |\n");
        }
        // The player's own row. Zero in a training match (the SwerveBase is parked),
        // non-zero in an interactive or P2 headless-player match.
        sb.append("| Player | ").append(r.playerBlueFuel()).append(" | n/a | n/a | n/a | n/a |\n");

        int blueSum = 0;
        for (int v : r.allyFuelScored()) blueSum += v;
        blueSum += r.playerBlueFuel();
        int redSum = 0;
        for (int v : r.botFuelScored()) redSum += v;
        redSum += r.playerRedFuel();
        sb.append("\nReconciliation: Blue per-bot ").append(blueSum).append(" vs total ")
                .append(r.blueTotal()).append(" (residual ").append(r.blueReconciliationResidual())
                .append(", unattributed ").append(r.blueUnattributedFuel()).append(") | Red per-bot ")
                .append(redSum).append(" vs total ").append(r.redTotal())
                .append(" (residual ").append(r.redReconciliationResidual())
                .append(", unattributed ").append(r.redUnattributedFuel()).append(")\n");
        if (r.blueReconciliationResidual() != 0 || r.redReconciliationResidual() != 0) {
            sb.append("\n**RECONCILIATION FAILED** - a scoring path is bypassing attribution.\n");
        }

        // Loop health, so "this match was starved by its sibling workers" is
        // readable from the report alone and not only from the JSONL.
        sb.append("\n## Loop health\n\n");
        MatchHealth h = r.health() == null ? MatchHealth.UNKNOWN : r.health();
        if (h.loopOverruns() < 0) {
            sb.append("Not measured (no headless match ran).\n");
        } else {
            sb.append("- Main-loop overruns (>").append(Math.round(LoopHealth.PERIOD_SEC * 1000.0))
                    .append(" ms): **").append(h.loopOverruns()).append("**\n");
            sb.append("- Slowest `robotPeriodic()`: **")
                    .append(Math.round(h.maxRobotPeriodicMs())).append(" ms**\n");
            sb.append("\nA match with a high overrun count was perturbed by machine load, not by\n"
                    + "robot policy. `tools/score/compare.py` refuses to compare it.\n");
        }
        sb.append("\n## Replay\n\n");
        sb.append("AdvantageScope -> File -> Open Log -> `").append(r.logPath()).append("`\n");
        sb.append("Logged topics include per-bot `ActualPose`/`TargetPose`/`Objective`,\n");
        sb.append("`FieldSimulation/Fuel`, `Odometry/RobotPose`, `Headless/*` phase/scoreboard,\n");
        sb.append("and the existing `JevAI/*`, `Trajectory/*`, and `Scoreboard/*` outputs.\n");
        return sb.toString();
    }

    /**
     * Reads one field out of a {@link BotMetrics} block, tolerating a null block
     * or a short array so the report renders even for a partially-built fleet.
     *
     * @param which 0 archetype, 1 path length, 2 max stall, 3 max consecutive
     */
    private static String metric(BotMetrics m, int bot, int which) {
        if (m == null) return "n/a";
        try {
            return switch (which) {
                case 0 -> m.archetype()[bot];
                case 1 -> String.valueOf(m.pathLengthM()[bot]);
                case 2 -> String.valueOf(m.maxContiguousStallSec()[bot]);
                default -> String.valueOf(m.maxConsecutiveRecoveries()[bot]);
            };
        } catch (ArrayIndexOutOfBoundsException e) {
            return "n/a";
        }
    }

    private static String formatConsoleSummary(MatchResult r) {
        return "[Headless] FINAL winner=" + r.winner() + " Blue=" + r.blueTotal() + " Red=" + r.redTotal()
                + " (auto " + r.blueAutoFuel() + "/" + r.redAutoFuel()
                + ", teleop " + r.blueTeleopFuel() + "/" + r.redTeleopFuel()
                + ", fieldFuel " + r.fieldFuelCount()
                + ", climb " + r.blueClimbCount() + "/" + r.redClimbCount()
                + ", fouls " + r.blueFouls() + "/" + r.redFouls() + ")"
                + " " + formatHealth(r.health())
                + " report=" + r.reportPath();
    }

    /**
     * One-line loop-health summary, printed on the console line the rig greps for
     * and rendered into the report so a human reading a single match can see
     * whether it was load-perturbed.
     */
    private static String formatHealth(MatchHealth h) {
        if (h == null || h.loopOverruns() < 0) {
            return "[health unmeasured]";
        }
        return "[health overruns=" + h.loopOverruns()
                + " maxRobotPeriodicMs=" + Math.round(h.maxRobotPeriodicMs()) + "]";
    }

    private static int getIntProperty(String key, int defaultValue) {
        String raw = System.getProperty(PROP_PREFIX + key);
        if (raw == null || raw.isBlank()) {
            return defaultValue;
        }
        try {
            return Integer.parseInt(raw.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("frc.headless." + key + " must be an integer", e);
        }
    }

    private static long getLongProperty(String key, long defaultValue) {
        String raw = System.getProperty(PROP_PREFIX + key);
        if (raw == null || raw.isBlank()) {
            return defaultValue;
        }
        try {
            return Long.parseLong(raw.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("frc.headless." + key + " must be an integer", e);
        }
    }

    private static double getDoubleProperty(String key, double defaultValue) {
        String raw = System.getProperty(PROP_PREFIX + key);
        if (raw == null || raw.isBlank()) {
            return defaultValue;
        }
        try {
            return Double.parseDouble(raw.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("frc.headless." + key + " must be a number", e);
        }
    }

}
