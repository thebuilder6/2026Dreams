package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Sim.BotMatchMetrics.StallCause;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Phase 0 report cover: the stall-cause table renders per-cause seconds from
 * live bot metrics without touching the JSONL schema or {@link HeadlessMatchDriver.MatchResult}.
 */
class HeadlessMatchDriverStallCauseTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
    }

    @Test
    void rowShowsEachCauseBucket() {
        BotMatchMetrics m = new BotMatchMetrics();
        for (int i = 0; i < 5; i++) {
            m.sample(new Pose2d(i * 0.001, 4.0, new Rotation2d()), true, true, true,
                    i * 0.02, StallCause.DEADLOCK_TRENCH);
        }
        for (int i = 0; i < 3; i++) {
            m.sample(new Pose2d(i * 0.001, 4.0, new Rotation2d()), true, true, true,
                    1.0 + i * 0.02, StallCause.TRENCH_YIELD);
        }

        String row = HeadlessMatchDriver.stallCauseRow("Red Bot0", m);
        assertTrue(row.contains("Red Bot0"));
        assertTrue(row.contains("0.10"), "5 ticks at 20 ms = 0.10 s trench deadlock");
        assertTrue(row.contains("0.06"), "3 ticks at 20 ms = 0.06 s trench yield");
        assertTrue(row.startsWith("|"));
    }

    @Test
    void rowShowsArrivedAndChurnBuckets() {
        BotMatchMetrics m = new BotMatchMetrics();
        for (int i = 0; i < 4; i++) {
            m.sample(new Pose2d(i * 0.001, 4.0, new Rotation2d()), true, true, true,
                    i * 0.02, StallCause.STALLED_ARRIVED);
        }
        m.sample(new Pose2d(0, 4.0, new Rotation2d()), true, true, true,
                0.08, StallCause.STALLED_CHURN);

        String row = HeadlessMatchDriver.stallCauseRow("Blue Ally1", m);
        assertTrue(row.contains("Blue Ally1"));
        assertTrue(row.contains("0.08"), "4 ticks at 20 ms = 0.08 s arrived");
        assertTrue(row.contains("0.02"), "1 tick at 20 ms = 0.02 s churn");
    }

    @Test
    void sectionNeverThrowsWithoutASim() {
        // Either "" (no sim live) or a table (a leaked singleton from another
        // test) — both are acceptable; what matters is no throw and no schema
        // coupling. toJsonLine must not mention the cause table either.
        String section = HeadlessMatchDriver.formatStallCauseSection();
        assertTrue(section.isEmpty() || section.contains("Stall cause breakdown"));
    }
}
