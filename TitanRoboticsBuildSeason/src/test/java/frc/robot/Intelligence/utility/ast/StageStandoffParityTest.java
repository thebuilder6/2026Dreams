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

/**
 * Proves that {@link ObjectiveExpressions#stageStandoff} reproduces the live
 * engine's {@code STAGE_STANDOFF} value across a state grid — the second
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>Defensive archetypes zero the stage score in engine post-processing, so
 * the grid covers only the branches the expression models (normal +
 * co-pilot). {@code t = 25 s} pins the beyond-normalization path
 * ({@code TIME_UNTIL_SHIFT} clamps at 20 s while the engine reads raw
 * seconds).
 */
public class StageStandoffParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        // Normal branch (AUTONOMOUS_CYCLER keeps stage untouched in post-processing).
        ExpressionNode normal = ObjectiveExpressions.stageStandoff(weights, BALL_CAPACITY, false);
        for (int held : new int[] {0, 1, 17, 18, 19, 30}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                for (double tus : new double[] {0.0, 0.5, 3.5, 10.0, 25.0}) {
                    checked += check(engine, normal, Archetype.AUTONOMOUS_CYCLER,
                            held, hubActive, tus, weights);
                }
            }
        }
        // Co-pilot branch (target 0.99, min-fuel 1).
        ExpressionNode coPilot = ObjectiveExpressions.stageStandoff(weights, BALL_CAPACITY, true);
        for (int held : new int[] {0, 1, 2}) {
            for (boolean hubActive : new boolean[] {true, false}) {
                for (double tus : new double[] {0.0, 3.5, 25.0}) {
                    checked += check(engine, coPilot, Archetype.CO_PILOT,
                            held, hubActive, tus, weights);
                }
            }
        }
        assertTrue(checked > 70, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression,
            Archetype archetype, int held, boolean hubActive, double tus, PolicyWeights weights) {
        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                hubActive, !hubActive, tus, false, false);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, held, 0, held, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 0, 0);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge, archetype, weights)
                .get(StrategicObjective.STAGE_STANDOFF);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        String label = "arch=" + archetype + " held=" + held + " hub=" + hubActive + " tus=" + tus;
        assertEquals(engineValue, expressionValue, 1e-9, label);
        return 1;
    }

    @Test
    void negativeSteepnessSigmoidRoundTripsThroughCodec() {
        ExpressionNode genome = ObjectiveExpressions.stageStandoff(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("sigmoid("), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));

        WorldState world = new WorldState(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)), new ChassisSpeeds(), 30,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                false, true, 3.5, false, false);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, 30, 0, 30, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 0, 0);
        EvalContext ctx = WorldStateTerminals.toContext(world, knowledge, DIST_NORM);
        assertEquals(genome.evaluate(ctx), parsed.evaluate(ctx), 1e-12, "codec preserves the tree");
    }
}
