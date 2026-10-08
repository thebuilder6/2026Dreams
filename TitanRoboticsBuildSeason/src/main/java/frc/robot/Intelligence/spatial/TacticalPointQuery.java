package frc.robot.Intelligence.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Unreal EQS / CryEngine TPS-style Tactical Environmental Point Query.
 *
 * <p>Pipeline:
 * <ol>
 *   <li>Runs a {@link PointGenerator} to generate candidate spatial poses.</li>
 *   <li>Evaluates a pipeline of multiplicative {@link PointTest} response curves against each candidate.</li>
 *   <li>Applies natural vetoes (early exit on 0.0) when an obstacle or boundary rule fails.</li>
 *   <li>Selects the highest scoring candidate pose.</li>
 * </ol>
 */
public final class TacticalPointQuery {

    public record QueryResult(Pose2d pose, double score) {
        public static final QueryResult EMPTY = new QueryResult(null, 0.0);
    }

    private final PointGenerator generator;
    private final List<PointTest> tests;

    public TacticalPointQuery(PointGenerator generator, List<PointTest> tests) {
        this.generator = generator;
        this.tests = Collections.unmodifiableList(new ArrayList<>(tests));
    }

    /**
     * Executes the query and returns the optimal pose.
     *
     * @param context query context
     * @return the highest scoring candidate pose, or fallback if none valid
     */
    public QueryResult execute(PointGenerator.QueryContext context) {
        List<Pose2d> candidates = generator.generate(context);
        if (candidates == null || candidates.isEmpty()) {
            return QueryResult.EMPTY;
        }

        Pose2d bestPose = null;
        double bestScore = -1.0;

        for (Pose2d candidate : candidates) {
            if (candidate == null) {
                continue;
            }

            double totalScore = 1.0;
            for (PointTest test : tests) {
                double score = test.score(candidate, context);
                if (Double.isNaN(score) || score <= 1e-6) {
                    totalScore = 0.0;
                    break; // Natural Veto
                }
                totalScore *= score;
            }

            if (totalScore > bestScore && totalScore > 1e-6) {
                bestScore = totalScore;
                bestPose = candidate;
            }
        }

        if (bestPose == null) {
            return QueryResult.EMPTY;
        }
        return new QueryResult(bestPose, bestScore);
    }
}
