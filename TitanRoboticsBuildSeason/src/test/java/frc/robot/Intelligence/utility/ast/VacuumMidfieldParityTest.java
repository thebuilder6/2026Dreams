package frc.robot.Intelligence.utility.ast;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.PolicyWeights;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import frc.robot.Navigation.FieldMap;

/**
 * Proves that {@link ObjectiveExpressions#vacuumMidfield} reproduces the live
 * engine's {@code VACUUM_MIDFIELD} value across a state grid — the third
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>States that would trip the engine's hand-written post-processing (the
 * harvest-deadline force, which zeroes vacuum) are skipped, and the grid runs
 * clairvoyant only: the observed-tier assume-present fuel default is engine
 * scaffolding the expression deliberately does not model.
 */
public class VacuumMidfieldParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;
    private static final double DIST_M = 1.0;

    private static Pose2d poseAtDistance(double distanceM) {
        return new Pose2d(
                FieldMap.Hubs.getHubLocation2d(false).plus(new edu.wpi.first.math.geometry.Translation2d(distanceM, 0.0)),
                Rotation2d.fromDegrees(180));
    }

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.vacuumMidfield(weights, BALL_CAPACITY);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        for (int held : new int[] {0, 5, 6, 7, 17, 18, 19, 30}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                for (double tus : new double[] {0.0, 3.0, 4.0, 10.0}) {
                    for (int mid : new int[] {0, 5}) {
                        // Skip the harvest-deadline force: engine zeroes vacuum when
                        // hub live, held >= 8, and no time left to harvest.
                        double timeLeftToHarvest = tus - DIST_M / 3.2;
                        if (hubActive && held >= 8 && timeLeftToHarvest <= 0.0) {
                            continue;
                        }
                        WorldState world = new WorldState(
                                poseAtDistance(DIST_M), new ChassisSpeeds(), held,
                                new Pose2d(), new ChassisSpeeds(), 100.0,
                                hubActive, !hubActive, tus, false, false);
                        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                                0, held, 0, held, 0,
                                List.of(), List.of(), List.of(), List.of(), 0, mid, 0);

                        double engineValue = engine.evaluateUtilityScores(
                                world, Collections.emptySet(), knowledge,
                                Archetype.AUTONOMOUS_CYCLER, weights)
                                .get(StrategicObjective.VACUUM_MIDFIELD);
                        double expressionValue = expression.evaluate(
                                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

                        String label = "held=" + held + " hub=" + hubActive
                                + " tus=" + tus + " mid=" + mid;
                        assertEquals(engineValue, expressionValue, 1e-9, label);
                        checked++;
                    }
                }
            }
        }
        assertTrue(checked > 90, "grid should cover many states, was " + checked);
    }

    @Test
    void hubBranchAndBatchCodecRoundTrip() {
        ExpressionNode genome = ObjectiveExpressions.vacuumMidfield(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("if("), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));

        WorldState world = new WorldState(
                poseAtDistance(DIST_M), new ChassisSpeeds(), 10,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                true, false, 10.0, false, false);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, 10, 0, 10, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 5, 0);
        EvalContext ctx = WorldStateTerminals.toContext(world, knowledge, DIST_NORM);
        // Codec prints 4 decimals, so repeating constants (e.g. 0.14/0.96) lose
        // ~1e-5 through a write/read cycle; the 1e-9 engine parity above uses
        // the unserialized tree and is the exact guarantee.
        assertEquals(genome.evaluate(ctx), parsed.evaluate(ctx), 1e-3, "codec preserves the tree");
    }
}
