package frc.robot.Intelligence.spatial;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.WorldState;

/**
 * Functional generator creating candidate spatial points for environmental queries.
 */
@FunctionalInterface
public interface PointGenerator {

    /**
     * Context passed to point generators and tests during query execution.
     */
    record QueryContext(
            WorldState world,
            MatchKnowledge knowledge,
            boolean isRedAlliance
    ) {}

    /**
     * Generates a list of candidate poses to be tested and scored.
     *
     * @param context query context containing world and match knowledge
     * @return list of candidate poses
     */
    List<Pose2d> generate(QueryContext context);
}
