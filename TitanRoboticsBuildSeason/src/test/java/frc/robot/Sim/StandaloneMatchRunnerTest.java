package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Intelligence.Archetype;
import frc.robot.Navigation.FieldMap;

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

    @Test
    void autoAndTeleopFuelSumMatchesTotal() {
        StandaloneMatchRunner.Result result =
                StandaloneMatchRunner.run(StandaloneMatchRunner.default3v3(42));
        assertEquals(result.blueScored(), result.blueAutoFuel() + result.blueTeleopFuel());
        assertEquals(result.redScored(), result.redAutoFuel() + result.redTeleopFuel());
    }

    @Test
    void volleyTotalsReconcileWithScoredFuel() {
        StandaloneMatchRunner.Result result =
                StandaloneMatchRunner.run(StandaloneMatchRunner.default3v3(42));
        assertTrue(result.attemptedShots() > 0, "bots should fire in a full 3v3");
        assertTrue(result.missedShots() <= result.attemptedShots());
        assertEquals(result.blueScored() + result.redScored(),
                result.attemptedShots() - result.missedShots(),
                "every fired ball scores or misses");
    }

    @Test
    void shotModelIsDeterministicAndCanMiss() {
        // Every gated volley used to convert fully. With the spread model a
        // stationary, well-aimed shot still makes (the 70 deg arc drops through
        // the funnel), but shots taken while moving can miss — the robot
        // velocity is added to the launch vector.
        Pose2d pose = new Pose2d(FieldMap.Hubs.BLUE_HUB_X - 2.0, FieldMap.Hubs.HUB_Y,
                Rotation2d.fromDegrees(0));
        Random probe = new Random(3);
        boolean anyMiss = false;
        boolean anyMake = false;
        for (int trial = 0; trial < 2000 && !(anyMiss && anyMake); trial++) {
            double vx = (probe.nextDouble() - 0.5) * 4.0;
            double vy = (probe.nextDouble() - 0.5) * 4.0;
            if (StandaloneBot.shotMakes(pose, new ChassisSpeeds(vx, vy, 0), false, probe)) {
                anyMake = true;
            } else {
                anyMiss = true;
            }
        }
        assertTrue(anyMake, "some shots must make");
        assertTrue(anyMiss, "some moving shots must miss");

        // Same seed, same shot outcomes.
        Random a = new Random(7);
        Random b = new Random(7);
        for (int i = 0; i < 50; i++) {
            assertEquals(StandaloneBot.shotMakes(pose, new ChassisSpeeds(1.0, -1.0, 0), false, a),
                    StandaloneBot.shotMakes(pose, new ChassisSpeeds(1.0, -1.0, 0), false, b));
        }
    }

    @Test
    void watchdogArbitrationFiresInAFullMatch() {
        StandaloneMatchRunner.Result result =
                StandaloneMatchRunner.run(StandaloneMatchRunner.default3v3(7));
        assertTrue(result.escapes() > 0,
                "target-progress watchdog should arbitrate in a full 3v3");
    }

    @Disabled("Native DataLogWriter crashes JVM in headless test runner")
    @Test
    void testStandaloneLoggingCreatesWpilog() throws Exception {
        java.io.File tempLog = java.io.File.createTempFile("standalone_test_", ".wpilog");
        tempLog.deleteOnExit();
        try {
            StandaloneMatchRunner.Config base = short3v3(7);
            StandaloneMatchRunner.Config config = new StandaloneMatchRunner.Config(
                    base.seed(), base.durationSec(), base.autoSec(),
                    base.fuelCount(), base.shiftSeed(), base.bots(), tempLog.getAbsolutePath());
            StandaloneMatchRunner.Result result = StandaloneMatchRunner.run(config);
            assertTrue(result.ticks() > 0);
            assertTrue(tempLog.exists(), "wpilog file should exist");
            assertTrue(tempLog.length() > 0, "wpilog should not be empty, was " + tempLog.length() + " bytes");
        } finally {
            tempLog.delete();
        }
    }

    @Test
    void testJsonLineFormat() {
        StandaloneMatchRunner.Config config = short3v3(7);
        StandaloneMatchRunner.Result result = StandaloneMatchRunner.run(config);
        String json = StandaloneMatchRunner.toJsonLine(config, result, "candidate", 0);
        assertTrue(json.startsWith("{\"schemaVersion\":2,"));
        assertTrue(json.contains("\"seed\":7,"));
        assertTrue(json.contains("\"variant\":\"candidate\","));
        assertTrue(json.contains("\"blueBots\":{"));
        assertTrue(json.contains("\"redBots\":{"));
        assertTrue(json.contains("\"health\":{\"loopOverruns\":0,\"maxRobotPeriodicMs\":0.0}"));
    }

    @Test
    void testMainExecutionWithJsonl() throws Exception {
        java.io.File tempJsonl = java.io.File.createTempFile("standalone_cli_", ".jsonl");
        tempJsonl.deleteOnExit();
        try {
            String[] args = new String[] {
                "--seeds", "7,11",
                "--duration", "10.0",
                "--auto", "3.0",
                "--jsonl", tempJsonl.getAbsolutePath(),
                "--weights", "scoreHubBase=0.75"
            };
            StandaloneMatchRunner.main(args);
            List<String> lines = java.nio.file.Files.readAllLines(tempJsonl.toPath());
            assertEquals(2, lines.size(), "Should have written 2 JSONL lines for 2 seeds");
            assertTrue(lines.get(0).contains("\"seed\":7"));
            assertTrue(lines.get(1).contains("\"seed\":11"));
        } finally {
            tempJsonl.delete();
        }
    }

    @Test
    void perBotTelemetryReconcilesWithAllianceTotalsAndPreservesRosterOrder() {
        StandaloneMatchRunner.Config config = StandaloneMatchRunner.default3v3(42);
        StandaloneMatchRunner.Result result = StandaloneMatchRunner.run(config);
        StandaloneMatchRunner.BotTelemetry[] t = result.botTelemetry();
        assertEquals(config.bots().size(), t.length);

        int blueAuto = 0;
        int blueTeleop = 0;
        int blueScored = 0;
        int redAuto = 0;
        int redTeleop = 0;
        int redScored = 0;
        for (int i = 0; i < t.length; i++) {
            assertEquals(config.bots().get(i).isRed(), t[i].isRed(), "roster order preserved");
            if (t[i].isRed()) {
                redAuto += t[i].autoScored();
                redTeleop += t[i].teleopScored();
                redScored += t[i].scored();
            } else {
                blueAuto += t[i].autoScored();
                blueTeleop += t[i].teleopScored();
                blueScored += t[i].scored();
            }
            assertTrue(t[i].hubActiveTeleopSec() >= 0.0);
            assertTrue(t[i].hubActiveTeleopSec() <= config.durationSec());
        }
        assertEquals(result.blueAutoFuel(), blueAuto);
        assertEquals(result.blueTeleopFuel(), blueTeleop);
        assertEquals(result.blueScored(), blueScored);
        assertEquals(result.redAutoFuel(), redAuto);
        assertEquals(result.redTeleopFuel(), redTeleop);
        assertEquals(result.redScored(), redScored);
    }

    @Test
    void climberOnTheTowerInsideTheEndgameWindowIsCredited() {
        Pose2d onTower = new Pose2d(FieldMap.ClimbingTowers.BLUE_TOWER_POLE, Rotation2d.fromDegrees(0));
        StandaloneMatchRunner.BotSpec spec = new StandaloneMatchRunner.BotSpec(
                Archetype.AUTONOMOUS_CYCLER, false, onTower, 0, true);
        StandaloneMatchRunner.Config config = new StandaloneMatchRunner.Config(
                11, 18.0, 0.0, 10, 'R', List.of(spec));
        StandaloneMatchRunner.Result result = StandaloneMatchRunner.run(config);
        assertTrue(result.botTelemetry()[0].climbed(),
                "a climber sitting on the tower inside the endgame window must be credited");
        assertTrue(result.botTelemetry()[0].climbArrivalSec() >= 0.0);
    }

    @Test
    void jsonlCarriesPerBotAndDefensiveMarkMatrix() {
        StandaloneMatchRunner.Config config = StandaloneMatchRunner.default3v3(7);
        StandaloneMatchRunner.Result result = StandaloneMatchRunner.run(config);
        String json = StandaloneMatchRunner.toJsonLine(config, result, "candidate", 0);
        assertTrue(json.contains("\"perBot\":["), "per-bot telemetry block present");
        assertTrue(json.contains("\"markSeconds\":["), "defensive mark matrix present");
        assertTrue(json.contains("\"climbed\":"), "climb flag present");
        assertTrue(json.contains("\"minorFouls\":"), "foul telemetry present");
        assertTrue(json.contains("\"hubActiveTeleopSec\":"));
    }
}
