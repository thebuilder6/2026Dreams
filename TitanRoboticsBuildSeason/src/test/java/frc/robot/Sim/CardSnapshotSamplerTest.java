package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.WorldState;

class CardSnapshotSamplerTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        CardSnapshotSampler.disarmForTests();
    }

    @AfterEach
    void tearDown() {
        CardSnapshotSampler.disarmForTests();
    }

    private static WorldState world(Pose2d self, int held, Pose2d mark) {
        return new WorldState(
                self, new ChassisSpeeds(), held,
                mark, new ChassisSpeeds(),
                108.0, true, false, 3.0, false, false,
                true, true, true, 30, false);
    }

    private static MatchKnowledge knowledge(List<Pose2d> allies, List<Pose2d> opponents,
            int allianceFuel, int midfieldFuel, int opponentFuel) {
        return new ClairvoyantKnowledge(0, 25, 0, 0, 0,
                allies, opponents, List.of(), List.of(),
                allianceFuel, midfieldFuel, opponentFuel, List.of());
    }

    @Test
    void rowHas27TabSeparatedColumns() {
        WorldState world = world(new Pose2d(2.0, 4.03, new Rotation2d()),
                20, new Pose2d(12.0, 4.0, new Rotation2d()));
        MatchKnowledge knowledge = knowledge(
                List.of(new Pose2d(3.0, 2.0, new Rotation2d())),
                List.of(new Pose2d(12.0, 4.0, new Rotation2d())),
                4, 9, 0);
        CardSnapshotSampler.arm(null, 7, 0, "baseline");
        String row = CardSnapshotSampler.toRow("Bot1", Archetype.TACTICAL_DEFENDER,
                world, knowledge, "LEAD_INTERCEPT", false, 20);
        String[] f = row.split("\t", -1);
        assertEquals(27, f.length);
    }

    @Test
    void rowCarriesIdProvenanceAndBlankExpected() {
        WorldState world = world(new Pose2d(2.0, 4.03, new Rotation2d()),
                20, new Pose2d(12.0, 4.0, new Rotation2d()));
        MatchKnowledge knowledge = knowledge(List.of(), List.of(), 0, 0, 0);
        CardSnapshotSampler.arm(null, 7, 0, "baseline");
        String row = CardSnapshotSampler.toRow("Bot1", Archetype.TACTICAL_DEFENDER,
                world, knowledge, "LEAD_INTERCEPT", false, 20);
        String[] f = row.split("\t", -1);
        assertEquals("S7r0-Bot1-108s", f[0]);
        assertEquals("TACTICAL_DEFENDER", f[1]);
        assertEquals("20", f[4]);
        assertEquals("", f[17]); // expected stays blank for human review
        assertTrue(f[16].contains("S7r0-Bot1-108s") || f[16].contains("seed 7"));
        assertTrue(f[16].contains("LEAD_INTERCEPT"));
    }

    @Test
    void trackedMarkExcludedFromExtras() {
        Pose2d mark = new Pose2d(12.0, 4.0, new Rotation2d());
        WorldState world = world(new Pose2d(2.0, 4.03, new Rotation2d()), 0, mark);
        MatchKnowledge knowledge = knowledge(
                List.of(),
                List.of(mark, new Pose2d(13.8, 4.0, new Rotation2d())),
                0, 0, 16);
        CardSnapshotSampler.arm(null, 7, 1, "baseline");
        String row = CardSnapshotSampler.toRow("Bot1", Archetype.ADAPTIVE_COMPETITOR,
                world, knowledge, "VACUUM_MIDFIELD", false, 0);
        String[] f = row.split("\t", -1);
        assertEquals("12.00,4.00", f[9] + "," + f[10]);
        assertEquals("0", f[14]); // oppZonePieces yields to the explicit list
        assertEquals("13.80,4.00", f[26]); // mark de-duplicated out
        assertEquals("0", f[19]);
        assertEquals("16", f[21]);
    }

    @Test
    void allyPosesAndHardwareFlowThrough() {
        WorldState world = world(new Pose2d(2.5, 3.0, new Rotation2d()),
                4, new Pose2d(12.0, 4.0, new Rotation2d()));
        MatchKnowledge knowledge = knowledge(
                List.of(new Pose2d(3.0, 2.0, new Rotation2d()),
                        new Pose2d(2.0, 5.5, new Rotation2d())),
                List.of(), 5, 6, 7);
        CardSnapshotSampler.arm(null, 42, 2, "baseline");
        String row = CardSnapshotSampler.toRow("Ally0", Archetype.ADAPTIVE_COMPETITOR,
                world, knowledge, "SCREEN_FOR_ALLY", false, 4);
        String[] f = row.split("\t", -1);
        assertEquals("S42r2-Ally0-108s", f[0]);
        assertEquals("3.00,2.00;2.00,5.50", f[25]);
        assertEquals("5", f[19]);
        assertEquals("6", f[20]);
        assertEquals("7", f[21]);
        assertEquals("true", f[22]);
        assertEquals("30", f[23]);
        assertEquals("false", f[24]);
    }

    @Test
    void bucketBoundariesMirrorCardSweep() {
        assertEquals(0, CardSnapshotSampler.bucket(0));
        assertEquals(1, CardSnapshotSampler.bucket(7));
        assertEquals(2, CardSnapshotSampler.bucket(8));
        assertEquals(2, CardSnapshotSampler.bucket(17));
        assertEquals(3, CardSnapshotSampler.bucket(18));
        assertEquals(3, CardSnapshotSampler.bucket(29));
        assertEquals(4, CardSnapshotSampler.bucket(30));
    }

    @Test
    void disarmedSamplerAcceptsNullsWithoutThrowing() {
        // Disarmed (null path): maybeSample is one volatile read and a return.
        CardSnapshotSampler.maybeSample(null, null, null, null, null, false, 0);
    }

    @Test
    void flushWhenDisarmedIsNoOp() {
        CardSnapshotSampler.flush();
    }

    @Test
    void extrasExcludingTrackedHandlesNulls() {
        assertTrue(CardSnapshotSampler.extrasExcludingTracked(null, new Pose2d()).isEmpty());
    }
}
