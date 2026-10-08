package frc.robot.Intelligence.spatial;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;

class StandoffGeneratorsTest {

    private static final Translation2d BLUE_HUB = FieldMap.Hubs.getHubLocation2d(false);

    @Test
    void testStandoffArcStaysInZoneAtRadiusFacingHub() {
        List<Pose2d> poses = StandoffGenerators.standoffArc(BLUE_HUB, 2.8, 16, false)
                .generate(new PointGenerator.QueryContext(null, null, false));
        assertFalse(poses.isEmpty());
        for (Pose2d pose : poses) {
            assertEquals(2.8, pose.getTranslation().getDistance(BLUE_HUB), 1e-9);
            assertTrue(FieldMap.AllianceZones.isInAllianceZone(pose, false));
            double expected = BLUE_HUB.minus(pose.getTranslation()).getAngle().getRadians();
            assertEquals(expected, pose.getRotation().getRadians(), 1e-9);
        }
    }

    @Test
    void testStandoffArcRejectsDegenerateInput() {
        assertTrue(StandoffGenerators.standoffArc(null, 2.8, 8, false)
                .generate(new PointGenerator.QueryContext(null, null, false)).isEmpty());
        assertTrue(StandoffGenerators.standoffArc(BLUE_HUB, 0.0, 8, false)
                .generate(new PointGenerator.QueryContext(null, null, false)).isEmpty());
    }

    @Test
    void testDefenseBarrierSpansCorridorFacingOpponent() {
        Translation2d opponent = new Translation2d(10.0, 4.0);
        Translation2d goal = FieldMap.Hubs.getHubLocation2d(true);
        List<Pose2d> poses = StandoffGenerators.defenseBarrier(opponent, goal, 7, 0.6)
                .generate(new PointGenerator.QueryContext(null, null, true));
        assertEquals(7, poses.size());
        for (Pose2d pose : poses) {
            double expected = opponent.minus(pose.getTranslation()).getAngle().getRadians();
            assertEquals(expected, pose.getRotation().getRadians(), 1e-9);
            assertTrue(FieldMap.isWithinField(pose.getTranslation(), 0.45));
        }
    }

    @Test
    void testFuelCandidatesCullOutOfBounds() {
        List<Translation2d> pieces = List.of(
                new Translation2d(8.0, 4.0),
                new Translation2d(-5.0, 4.0),
                new Translation2d(20.0, 4.0));
        List<Pose2d> poses = StandoffGenerators.fuelCandidates(pieces, 0.45)
                .generate(new PointGenerator.QueryContext(null, null, false));
        assertEquals(1, poses.size());
        assertEquals(8.0, poses.get(0).getX(), 1e-9);
    }

    @Test
    void testStandoffDistancePeaksAtMidpoint() {
        PointGenerator.QueryContext ctx = new PointGenerator.QueryContext(null, null, false);
        PointTest test = PointTest.standoffDistance(BLUE_HUB, 2.8, 0.5);
        Pose2d atMid = new Pose2d(BLUE_HUB.plus(new Translation2d(2.8, 0.0)), new edu.wpi.first.math.geometry.Rotation2d());
        Pose2d off = new Pose2d(BLUE_HUB.plus(new Translation2d(1.0, 0.0)), new edu.wpi.first.math.geometry.Rotation2d());
        assertEquals(1.0, test.score(atMid, ctx), 1e-9);
        double offScore = test.score(off, ctx);
        assertTrue(offScore > 0.0 && offScore < 1.0);
        assertEquals(Math.exp(-(1.8 * 1.8) / (2 * 0.5 * 0.5)), offScore, 1e-9);
    }

    @Test
    void testHubShellExclusionVetoesInsideShell() {
        PointGenerator.QueryContext ctx = new PointGenerator.QueryContext(null, null, false);
        PointTest exclusion = PointTest.hubShellExclusion(BLUE_HUB, 2.05);
        Pose2d inside = new Pose2d(BLUE_HUB.plus(new Translation2d(1.0, 0.0)), new edu.wpi.first.math.geometry.Rotation2d());
        Pose2d outside = new Pose2d(BLUE_HUB.plus(new Translation2d(3.0, 0.0)), new edu.wpi.first.math.geometry.Rotation2d());
        assertEquals(0.0, exclusion.score(inside, ctx), 1e-9);
        assertEquals(1.0, exclusion.score(outside, ctx), 1e-9);

        TacticalPointQuery query = new TacticalPointQuery(
                StandoffGenerators.standoffArc(BLUE_HUB, 1.0, 8, false),
                List.of(exclusion));
        assertEquals(TacticalPointQuery.QueryResult.EMPTY, query.execute(ctx));
    }
}
