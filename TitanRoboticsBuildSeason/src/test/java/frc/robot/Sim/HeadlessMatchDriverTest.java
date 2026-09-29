package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Intelligence.Archetype;

class HeadlessMatchDriverTest {
    private static final String[] PROP_KEYS = {
            "seed", "durationSec", "autoSec", "disabledGapSec", "bootWaitSec", "fieldFuelCount", "logDir",
            "reportDir", "resultJsonl", "variant", "replica"
    };

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        clearHeadlessProperties();
        HeadlessMatchDriver.clearCachedLogPathForTests();
        GameSim.getInstance().resetGame(null);
    }

    @AfterEach
    void tearDown() {
        clearHeadlessProperties();
        HeadlessMatchDriver.clearCachedLogPathForTests();
        GameSim.getInstance().resetGame(null);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void parseOptionsDefaults() {
        HeadlessMatchDriver.Options options = HeadlessMatchDriver.parseOptions();
        assertEquals(2026L, options.seed());
        assertEquals(150.0, options.durationSec());
        assertEquals(15.0, options.autoSec());
        assertEquals(135.0, options.teleopSec());
        assertEquals(108, options.fieldFuelCount());
    }

    @Test
    void parseOptionsFieldFuelOverride() {
        System.setProperty("frc.headless.fieldFuelCount", "200");
        HeadlessMatchDriver.Options options = HeadlessMatchDriver.parseOptions();
        assertEquals(200, options.fieldFuelCount());
        clearHeadlessProperties();
        System.setProperty("frc.headless.fieldFuelCount", "-1");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
        clearHeadlessProperties();
        System.setProperty("frc.headless.fieldFuelCount", "385");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
        clearHeadlessProperties();
        System.setProperty("frc.headless.fieldFuelCount", "abc");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
    }

    @Test
    void parseOptionsOverrides() {
        System.setProperty("frc.headless.seed", "7");
        System.setProperty("frc.headless.durationSec", "60");
        System.setProperty("frc.headless.autoSec", "10");
        HeadlessMatchDriver.Options options = HeadlessMatchDriver.parseOptions();
        assertEquals(7L, options.seed());
        assertEquals(60.0, options.durationSec());
        assertEquals(50.0, options.teleopSec());
    }

    @Test
    void parseOptionsRejectsBadPhases() {
        System.setProperty("frc.headless.durationSec", "10");
        System.setProperty("frc.headless.autoSec", "15");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
        clearHeadlessProperties();
        System.setProperty("frc.headless.autoSec", "-1");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
        clearHeadlessProperties();
        System.setProperty("frc.headless.durationSec", "abc");
        assertThrows(IllegalArgumentException.class, HeadlessMatchDriver::parseOptions);
    }

    @Test
    void buildScenarioKeepsRostersWithDurationOverride() {
        System.setProperty("frc.headless.seed", "99");
        System.setProperty("frc.headless.durationSec", "45");
        TrainingMatchScenario scenario =
                HeadlessMatchDriver.buildScenario(HeadlessMatchDriver.parseOptions());
        assertEquals(99L, scenario.seed());
        assertEquals(45.0, scenario.durationSeconds());
        assertEquals(108, scenario.fieldFuelCount());
        assertEquals(3, scenario.blueRobots().size());
        assertEquals(3, scenario.redRobots().size());
        assertEquals(8, scenario.redRobots().get(0).preloadFuel());
    }

    @Test
    void buildScenarioUsesFieldFuelOverride() {
        System.setProperty("frc.headless.fieldFuelCount", "200");
        TrainingMatchScenario scenario =
                HeadlessMatchDriver.buildScenario(HeadlessMatchDriver.parseOptions());
        assertEquals(200, scenario.fieldFuelCount());
    }

    @Test
    void resolveLogPathIsStableAndSeeded() {
        System.setProperty("frc.headless.seed", "42");
        String first = HeadlessMatchDriver.resolveLogPath();
        String second = HeadlessMatchDriver.resolveLogPath();
        assertEquals(first, second);
        assertTrue(first.endsWith(".wpilog"));
        assertTrue(first.contains("42"));
    }

    @Test
    void formatReportContainsScoreboardAndReplay() {
        HeadlessMatchDriver.MatchResult result = sampleResult();
        String report = HeadlessMatchDriver.formatReport(result);
        assertTrue(report.contains("Blue"));
        assertTrue(report.contains("12"));
        assertTrue(report.contains("108"));
        assertTrue(report.contains("logs/headless_3v3_seed2026_x.wpilog"));
        assertTrue(report.contains("AdvantageScope"));
    }

    /**
     * A deliberately INCONSISTENT result, so the report's reconciliation-failure
     * path is exercised. Blue: per-bot 4+1+1 = 6, player 0, but auto 6 + teleop 6
     * = 12 fuel, so the residual is 6. Red: per-bot 3+2+0 = 5 against 4 + 5 = 9,
     * residual 4. A real match has both residuals at 0; this fixture does not, and
     * the report must say so rather than printing a total nobody can account for.
     */
    private static HeadlessMatchDriver.MatchResult sampleResult() {
        return new HeadlessMatchDriver.MatchResult(
                2026L, 150.0, 15.0, 108, "Blue", 12, 9, 3,
                6, 4, 6, 5, 0, 0, 0, 0, 1, 0, 5, 0, 2, 3,
                new int[] {3, 2, 0}, new int[] {4, 1, 1},
                0, 0,
                // residuals: Blue 12 - (4+1+1+0) = 6 ; Red 9 - (3+2+0+0) = 4
                6, 4, 0, 0,
                "baseline", 0,
                new HeadlessMatchDriver.BotMetrics(
                        new String[] {"AUTONOMOUS_CYCLER", "ADAPTIVE_COMPETITOR", "DEFENSE_BULLY"},
                        new double[] {41.2, 22.8, 33.0},
                        new double[] {1.4, 0.8, 0.0},
                        new int[] {1, 0, 2}, new int[] {1, 0, 3}),
                new HeadlessMatchDriver.BotMetrics(
                        new String[] {"AUTONOMOUS_CYCLER", "ADAPTIVE_COMPETITOR", "TACTICAL_DEFENDER"},
                        new double[] {38.5, 44.1, 29.7},
                        new double[] {0.6, 1.1, 0.0},
                        new int[] {0, 1, 0}, new int[] {0, 1, 0}),
                "logs/headless_3v3_seed2026_x.wpilog", "reports/headless_match_seed2026_x.md",
                new HeadlessMatchDriver.MatchHealth(4, 63.0));
    }

    @Test
    void reportShowsReconciliationFailureWhenResidualIsNonZero() {
        String report = HeadlessMatchDriver.formatReport(sampleResult());
        assertTrue(report.contains("RECONCILIATION FAILED"),
                "a non-zero residual must be surfaced, not silently reported");
        assertTrue(report.contains("residual 6"));
    }

    @Test
    void reportCarriesPerBotGuardrailMetrics() {
        String report = HeadlessMatchDriver.formatReport(sampleResult());
        // Declared archetype per bot: the role-aware guardrails key off these.
        assertTrue(report.contains("DEFENSE_BULLY"));
        assertTrue(report.contains("TACTICAL_DEFENDER"));
        // Path length, longest stall, and consecutive-recovery count.
        assertTrue(report.contains("41.2"));
        assertTrue(report.contains("Max consec. recoveries"));
    }

    @Test
    void jsonlLineIsWellFormedAndCarriesProvenance() {
        String line = HeadlessMatchDriver.toJsonLine(sampleResult());
        assertTrue(line.startsWith("{"));
        assertTrue(line.endsWith("}"));
        assertTrue(line.contains("\"schemaVersion\":" + HeadlessMatchDriver.JSONL_SCHEMA_VERSION));
        assertTrue(line.contains("\"gitSha\":"));
        // Dirty-tree provenance. A gitSha alone does not identify the code that
        // ran: a build from a modified working tree reports the HEAD sha, so a
        // sweep of uncommitted edits was indistinguishable from a sweep of that
        // commit. -1 means BuildConstants was unreadable, which is itself a
        // provenance failure and must not read as clean (0).
        assertTrue(line.contains("\"dirty\":"),
                "every row must record whether the tree was dirty when it was built");
        // find(), not matches(): matches() anchors the whole string and this is a
        // substring probe. -1 = BuildConstants unreadable, which is a provenance
        // failure and must not read as clean (0).
        assertTrue(java.util.regex.Pattern.compile("\"dirty\":(-1|0|1)[,}]").matcher(line).find(),
                "dirty must be -1, 0, or 1 followed by a delimiter, got: " + line);
        assertTrue(line.contains("\"seed\":2026"));
        // Sweep identity, used for resume + grouping.
        assertTrue(line.contains("\"variant\":\"baseline\""));
        assertTrue(line.contains("\"replica\":0"));
        assertTrue(line.contains("\"blueTotal\":12"));
        // Decomposed so a gain can be attributed to fuel rather than penalty.
        assertTrue(line.contains("\"blueFuel\":12"));
        assertTrue(line.contains("\"bluePenaltyPoints\":5"));
        // Canaries for the attribution bug.
        assertTrue(line.contains("\"blueReconciliationResidual\":6"));
        assertTrue(line.contains("\"blueUnattributedFuel\":0"));
        // Per-bot metrics, self-describing.
        assertTrue(line.contains("\"redBots\":"));
        assertTrue(line.contains("\"blueBots\":"));
        assertTrue(line.contains("\"maxContiguousStallSec\":[1.4,0.8,0.0]"));
        assertTrue(line.contains("\"archetype\":[\"AUTONOMOUS_CYCLER\",\"ADAPTIVE_COMPETITOR\",\"DEFENSE_BULLY\"]"));
        // Loop health, so a load-perturbed match is visible in the data.
        assertTrue(line.contains("\"health\":{"));
        assertTrue(line.contains("\"loopOverruns\":4"));
        assertTrue(line.contains("\"maxRobotPeriodicMs\":63.0"));
        // Single line: a JSONL row must never contain a raw newline.
        assertFalse(line.contains("\n"));
    }

    @Test
    void unmeasuredHealthIsSentinelledNotZeroed() {
        // A match that never armed LoopHealth must report -1, not 0. Reading a
        // missing measurement as "zero overruns" is how a v1 row would have been
        // mistaken for a clean one.
        String line = HeadlessMatchDriver.toJsonLine(withHealth(
                HeadlessMatchDriver.MatchHealth.UNKNOWN));
        assertTrue(line.contains("\"loopOverruns\":-1"));
        assertTrue(line.contains("\"maxRobotPeriodicMs\":-1.0"));
    }

    @Test
    void reportShowsLoopHealth() {
        String report = HeadlessMatchDriver.formatReport(sampleResult());
        assertTrue(report.contains("Loop health"));
        assertTrue(report.contains("63 ms"));
    }

    @Test
    void reportSaysUnmeasuredRatherThanShowingZero() {
        String report = HeadlessMatchDriver.formatReport(withHealth(
                HeadlessMatchDriver.MatchHealth.UNKNOWN));
        assertTrue(report.contains("Not measured"));
        assertFalse(report.contains("Not measured") && report.contains("0 ms"));
    }

    /** The same fixture with a different health block. */
    private static HeadlessMatchDriver.MatchResult withHealth(
            HeadlessMatchDriver.MatchHealth health) {
        HeadlessMatchDriver.MatchResult r = sampleResult();
        return new HeadlessMatchDriver.MatchResult(
                r.seed(), r.durationSec(), r.autoSec(), r.fieldFuelCount(), r.winner(),
                r.blueTotal(), r.redTotal(), r.margin(), r.blueAutoFuel(), r.redAutoFuel(),
                r.blueTeleopFuel(), r.redTeleopFuel(), r.blueClimb(), r.redClimb(),
                r.blueClimbCount(), r.redClimbCount(), r.blueFouls(), r.redFouls(),
                r.bluePenaltyPoints(), r.redPenaltyPoints(), r.blueWastedFuel(), r.redWastedFuel(),
                r.botFuelScored(), r.allyFuelScored(), r.playerBlueFuel(), r.playerRedFuel(),
                r.blueReconciliationResidual(), r.redReconciliationResidual(),
                r.blueUnattributedFuel(), r.redUnattributedFuel(), r.variant(), r.replica(),
                r.redBotMetrics(), r.blueBotMetrics(), r.logPath(), r.reportPath(), health);
    }

    @Test
    void resultJsonlOptionDefaultsToDisabled() {
        assertNull(HeadlessMatchDriver.parseOptions().resultJsonl(),
                "JSONL output is opt-in so existing headless runs are unchanged");
    }

    @Test
    void resultJsonlOptionIsParsed() {
        System.setProperty("frc.headless.resultJsonl", "results/run.jsonl");
        assertEquals("results/run.jsonl", HeadlessMatchDriver.parseOptions().resultJsonl());
    }

    @Test
    void variantAndReplicaDefaultForSingleRuns() {
        HeadlessMatchDriver.Options o = HeadlessMatchDriver.parseOptions();
        assertEquals("baseline", o.variant());
        assertEquals(0, o.replica());
    }

    @Test
    void variantAndReplicaAreParsed() {
        System.setProperty("frc.headless.variant", "batch18");
        System.setProperty("frc.headless.replica", "2");
        HeadlessMatchDriver.Options o = HeadlessMatchDriver.parseOptions();
        assertEquals("batch18", o.variant());
        assertEquals(2, o.replica());
    }

    @Test
    void blankVariantFallsBackToBaseline() {
        System.setProperty("frc.headless.variant", "   ");
        assertEquals("baseline", HeadlessMatchDriver.parseOptions().variant());
    }

    @Test
    void negativeReplicaIsRejected() {
        System.setProperty("frc.headless.replica", "-1");
        assertThrows(IllegalArgumentException.class, () -> HeadlessMatchDriver.parseOptions());
    }

    @Test
    void logTagDistinguishesVariantsAndReplicas() {
        // A parallel sweep starts both replicas of a seed in the same second, so
        // the replay filename must differ or they overwrite each other.
        assertEquals("", HeadlessMatchDriver.logTag(
                HeadlessMatchDriver.parseOptions()));

        System.setProperty("frc.headless.variant", "batch18");
        assertEquals("_batch18", HeadlessMatchDriver.logTag(
                HeadlessMatchDriver.parseOptions()));

        System.setProperty("frc.headless.replica", "1");
        assertEquals("_batch18_r1", HeadlessMatchDriver.logTag(
                HeadlessMatchDriver.parseOptions()));

        System.setProperty("frc.headless.variant", "baseline");
        assertEquals("_r1", HeadlessMatchDriver.logTag(
                HeadlessMatchDriver.parseOptions()));
    }

    @Test
    void logTagSanitisesVariantNames() {
        System.setProperty("frc.headless.variant", "a/b c");
        assertEquals("_a_b_c", HeadlessMatchDriver.logTag(
                HeadlessMatchDriver.parseOptions()));
    }

    @Test
    void reportNameCarriesTheSameTagAsTheReplayLog() {
        // The replay log was tagged after a 16-match baseline wrote 8 wpilogs; the
        // report was missed and two same-seed replicas still overwrote each other.
        System.setProperty("frc.headless.variant", "batch18");
        System.setProperty("frc.headless.replica", "1");
        HeadlessMatchDriver.Options o = HeadlessMatchDriver.parseOptions();
        assertEquals("headless_match_seed2026_batch18_r1_20260928-190000.md",
                HeadlessMatchDriver.reportFileName(o, "20260928-190000"));
    }

    @Test
    void reportNamesOfReplicasDiffer() {
        System.setProperty("frc.headless.replica", "0");
        String r0 = HeadlessMatchDriver.reportFileName(
                HeadlessMatchDriver.parseOptions(), "20260928-190000");
        System.setProperty("frc.headless.replica", "1");
        String r1 = HeadlessMatchDriver.reportFileName(
                HeadlessMatchDriver.parseOptions(), "20260928-190000");
        assertNotEquals(r0, r1,
                "two replicas finishing in the same second must not share a report path");
    }

    @Test
    void loopHealthIsInertUntilArmed() {
        LoopHealth.disarm();
        assertFalse(LoopHealth.isArmed());
        assertEquals(-1, LoopHealth.overrunCount());
        assertEquals(-1.0, LoopHealth.maxEpochSec(), 1e-9);
        // A disarmed begin()/end() pair must not arm anything or count anything.
        LoopHealth.end(LoopHealth.begin());
        assertFalse(LoopHealth.isArmed());
        assertEquals(-1, LoopHealth.overrunCount());
    }

    @Test
    void loopHealthDiscardsStartupWarmUp() {
        // The first second of a match runs one-time init inside robotPeriodic
        // (AprilTag layout, MapleSim arena, AdvantageKit structs, JIT). That epoch
        // measured 0.81 s at 12-wide and 0.32-0.42 s at 6-wide while every later
        // epoch was far smaller, so counting it would fail the gate even on an
        // idle machine. Warm-up epochs must not register.
        LoopHealth.arm();
        try {
            for (int i = 0; i < LoopHealth.WARMUP_EPOCHS; i++) {
                long t = LoopHealth.begin();
                Thread.sleep(2);
                LoopHealth.end(t);
            }
            assertEquals(0, LoopHealth.measuredEpochs());
            assertEquals(0, LoopHealth.overrunCount());
            assertEquals(0.0, LoopHealth.maxEpochSec(), 1e-9,
                    "a slow warm-up epoch must not become the reported maximum");

            // Past warm-up, epochs are counted again.
            long t = LoopHealth.begin();
            Thread.sleep(2);
            LoopHealth.end(t);
            assertEquals(1, LoopHealth.measuredEpochs());
            assertTrue(LoopHealth.maxEpochSec() > 0.0);
            assertTrue(LoopHealth.maxEpochSec() < LoopHealth.PERIOD_SEC,
                    "a sub-period epoch must not register as an overrun");
            assertEquals(0, LoopHealth.overrunCount());
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new AssertionError("interrupted", e);
        } finally {
            LoopHealth.disarm();
        }
    }

    @Test
    void loopHealthArmResetsCounters() {
        LoopHealth.arm();
        try {
            for (int i = 0; i < LoopHealth.WARMUP_EPOCHS + 5; i++) {
                LoopHealth.end(LoopHealth.begin());
            }
            assertEquals(5, LoopHealth.measuredEpochs());
            LoopHealth.arm();
            assertEquals(0, LoopHealth.measuredEpochs());
            assertEquals(0, LoopHealth.overrunCount());
        } finally {
            LoopHealth.disarm();
        }
    }

    @Test
    void matchHealthCaptureReflectsArmedState() {
        LoopHealth.disarm();
        assertEquals(HeadlessMatchDriver.MatchHealth.UNKNOWN,
                HeadlessMatchDriver.MatchHealth.capture());
        LoopHealth.arm();
        try {
            HeadlessMatchDriver.MatchHealth captured =
                    HeadlessMatchDriver.MatchHealth.capture();
            assertEquals(0, captured.loopOverruns());
            assertEquals(0.0, captured.maxRobotPeriodicMs(), 1e-9);
        } finally {
            LoopHealth.disarm();
        }
    }

    @Test
    void resetForMatchStartPreservesScenarioSpawns() {
        Pose2d redSpawn = pose(14.5, 1.2, 180.0);
        TrainingMatchScenario scenario = new TrainingMatchScenario(
                23L, 90.0, 54,
                List.of(robot(Archetype.AUTONOMOUS_CYCLER, pose(2.0, 2.25, 0.0), 8)),
                List.of(robot(Archetype.AUTONOMOUS_CYCLER, redSpawn, 5)));
        AIRobotSim aiSim = AIRobotSim.getInstance();
        GameSim.getInstance().resetGame(scenario);

        // Autonomous-enable sequence: resetGame() re-applies the scenario, then
        // mid-match drift, then the scenario-preserving reset must restore it.
        // (Bare reset() alone returns bots to queuing poses and would wipe the
        // scenario here, which is why lifecycle paths use resetForMatchStart.)
        GameSim.getInstance().resetGame();
        aiSim.reset();
        assertPoseNear(AIRobotSim.ROBOT_QUEUING_POSITIONS[0],
                aiSim.getOpponents().get(0).getActualPose());

        aiSim.setRobotPose(pose(8.0, 4.0, 90.0));
        aiSim.resetForMatchStart();

        assertPoseNear(redSpawn, aiSim.getOpponents().get(0).getActualPose());
        assertEquals(5, aiSim.getOpponents().get(0).getFuelCount());
        assertTrue(aiSim.isTrainingScenarioActive());
    }

    @Test
    void resetForMatchStartWithoutScenarioMatchesReset() {
        AIRobotSim aiSim = AIRobotSim.getInstance();
        aiSim.setRobotPose(pose(8.0, 4.0, 90.0));
        aiSim.resetForMatchStart();
        assertPoseNear(AIRobotSim.ROBOT_QUEUING_POSITIONS[0],
                aiSim.getOpponents().get(0).getActualPose());
    }

    private static void clearHeadlessProperties() {
        for (String key : PROP_KEYS) {
            System.clearProperty("frc.headless." + key);
        }
    }

    private static TrainingMatchScenario.RobotConfig robot(
            Archetype archetype, Pose2d pose, int preload) {
        return new TrainingMatchScenario.RobotConfig(archetype, pose, preload);
    }

    private static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
    }

    private static void assertPoseNear(Pose2d expected, Pose2d actual) {
        assertTrue(actual.getTranslation().getDistance(expected.getTranslation()) < 0.15);
        assertTrue(Math.abs(actual.getRotation().minus(expected.getRotation()).getRadians()) < 0.2);
    }
}
