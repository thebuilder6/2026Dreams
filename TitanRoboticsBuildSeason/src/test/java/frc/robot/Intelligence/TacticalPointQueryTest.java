package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Intelligence.spatial.PointGenerator;
import frc.robot.Intelligence.spatial.PointTest;
import frc.robot.Intelligence.spatial.TacticalPointQuery;
import frc.robot.Intelligence.utility.ResponseCurve;

public class TacticalPointQueryTest {

    @Test
    public void testCandidateSelectionAndScoring() {
        // Generator creates 3 points along a line
        PointGenerator generator = context -> List.of(
                new Pose2d(1.0, 4.0, new Rotation2d()),
                new Pose2d(2.5, 4.0, new Rotation2d()),
                new Pose2d(5.0, 4.0, new Rotation2d())
        );

        Translation2d hubTarget = new Translation2d(2.5, 4.0);

        // Distance test prefers point at 2.5m (diff = 0)
        PointTest distTest = PointTest.distanceToTarget(
                hubTarget, 0.0, 5.0, ResponseCurve.reverseRamp(0.0, 1.0));

        TacticalPointQuery query = new TacticalPointQuery(generator, List.of(distTest));
        TacticalPointQuery.QueryResult result = query.execute(new PointGenerator.QueryContext(null, null, false));

        assertNotNull(result.pose());
        assertEquals(2.5, result.pose().getX(), 1e-6);
        assertEquals(4.0, result.pose().getY(), 1e-6);
        assertEquals(1.0, result.score(), 1e-6);
    }

    @Test
    public void testNaturalVetoOnFailingTest() {
        PointGenerator generator = context -> List.of(
                new Pose2d(1.0, 1.0, new Rotation2d()),
                new Pose2d(2.0, 2.0, new Rotation2d())
        );

        // Test that vetoes everything
        PointTest vetoTest = (candidate, context) -> 0.0;

        TacticalPointQuery query = new TacticalPointQuery(generator, List.of(vetoTest));
        TacticalPointQuery.QueryResult result = query.execute(new PointGenerator.QueryContext(null, null, false));

        assertEquals(TacticalPointQuery.QueryResult.EMPTY, result);
    }
}
