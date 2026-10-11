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
 * Proves that {@link ObjectiveExpressions#shuttlePass} reproduces the live
 * engine's {@code SHUTTLE_PASS} value across a state grid — the sixth
 * behaviour-preserving AST conversion, without touching the engine.
 *
 * <p>Under default weights the distance gate (6 m) kills every in-zone pose,
 * so the default-weights grid pins the gates at zero while an override grid
 * with a reachable minimum distance pins the flat 0.87 value. No
 * post-processing touches shuttle on the covered archetype, so nothing is
 * skipped.
 */
public class ShuttlePassParityTest {

    private static final int BALL_CAPACITY = 30;
    private static final double DIST_NORM = 8.0;

    // A/B: in-zone, clear of the trench bands, inside/outside default pass range.
    // C: out of zone. D: genuinely low-clearance (trench X band, south of y 1.279).
    private static final Pose2d POSE_A = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d POSE_B = new Pose2d(0.5, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d POSE_C = new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(0));
    private static final Pose2d POSE_D = new Pose2d(4.0, 0.65, Rotation2d.fromDegrees(0));

    @Test
    void expressionMatchesTheEngineAcrossAGrid() {
        PolicyWeights weights = PolicyWeights.DEFAULT;
        ExpressionNode expression = ObjectiveExpressions.shuttlePass(weights, BALL_CAPACITY, DIST_NORM);
        JevDecisionEngine engine = JevDecisionEngine.getInstance();

        int checked = 0;
        // Default weights: every cell is vetoed (distance, zone, ceiling, hub, or fuel).
        for (Pose2d pose : new Pose2d[] {POSE_A, POSE_B, POSE_C, POSE_D}) {
            for (int held : new int[] {0, 15, 16, 30}) {
                for (boolean hubActive : new boolean[] {true, false}) {
                    checked += check(engine, expression, weights, pose, held,
                            hubActive, true, false, false, true);
                }
            }
        }
        // Opponent-hub gate variants (live-next vs fully dark).
        checked += check(engine, expression, weights, POSE_A, 16, false, false, true, false, true);
        checked += check(engine, expression, weights, POSE_A, 16, false, false, false, false, true);
        // Teleop gate: autonomous kills it.
        checked += check(engine, expression, weights, POSE_A, 16, false, true, false, true, true);
        // Shooter gate: no fitted shooter kills it.
        checked += check(engine, expression, weights, POSE_A, 16, false, true, false, false, false);

        // Reachable minimum distance: the open cells read the flat weight.
        PolicyWeights close = PolicyWeights.fromString("shuttlePassMinDistMeters=2.0");
        ExpressionNode closeExpression = ObjectiveExpressions.shuttlePass(close, BALL_CAPACITY, DIST_NORM);
        for (Pose2d pose : new Pose2d[] {POSE_A, POSE_B}) {
            for (int held : new int[] {16, 30}) {
                for (boolean oppNext : new boolean[] {false, true}) {
                    checked += check(engine, closeExpression, close, pose, held,
                            false, true, oppNext, false, true);
                }
            }
        }
        // Trench ceiling still vetoes under the reachable weights (dist passes here).
        checked += check(engine, closeExpression, close, POSE_D, 16, false, true, false, false, true);

        assertTrue(checked > 35, "grid should cover many states, was " + checked);
    }

    private static int check(JevDecisionEngine engine, ExpressionNode expression, PolicyWeights weights,
            Pose2d pose, int held, boolean hubActive, boolean oppActive, boolean oppNext,
            boolean autonomous, boolean hasShooter) {
        WorldState world = new WorldState(
                pose, new ChassisSpeeds(), held,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                hubActive, oppActive, 10.0, false, autonomous,
                hubActive, oppNext)
                .withHardware(hasShooter, BALL_CAPACITY, false);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, held, 0, held, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 0, 0);

        double engineValue = engine.evaluateUtilityScores(
                world, Collections.emptySet(), knowledge,
                Archetype.AUTONOMOUS_CYCLER, weights)
                .get(StrategicObjective.SHUTTLE_PASS);
        double expressionValue = expression.evaluate(
                WorldStateTerminals.toContext(world, knowledge, DIST_NORM));

        String label = "pose=" + pose.getTranslation() + " held=" + held + " hub=" + hubActive
                + " opp=" + oppActive + "/" + oppNext + " auto=" + autonomous + " shooter=" + hasShooter;
        assertEquals(engineValue, expressionValue, 1e-9, label);
        return 1;
    }

    @Test
    void newTerminalsAreObservedTierAndRoundTripThroughCodec() {
        assertEquals(Terminal.Tier.OBSERVED, Terminal.HAS_SHOOTER.tier());
        assertEquals(Terminal.Tier.OBSERVED, Terminal.IS_AUTONOMOUS.tier());
        assertEquals(Terminal.Tier.OBSERVED, Terminal.IN_LOW_CLEARANCE.tier());

        ExpressionNode genome = ObjectiveExpressions.shuttlePass(PolicyWeights.DEFAULT, BALL_CAPACITY);
        String text = SExpressions.write(genome);
        assertTrue(text.contains("has_shooter"), text);
        assertTrue(text.contains("in_low_clearance"), text);
        ExpressionNode parsed = SExpressions.read(text);
        assertEquals(text, SExpressions.write(parsed));

        // Spot-check the pinned flat value through the codec path inputs.
        WorldState world = new WorldState(
                POSE_A, new ChassisSpeeds(), 16,
                new Pose2d(), new ChassisSpeeds(), 100.0,
                false, true, 10.0, false, false, false, true);
        ClairvoyantKnowledge knowledge = new ClairvoyantKnowledge(
                0, 16, 0, 16, 0,
                List.of(), List.of(), List.of(), List.of(), 0, 0, 0);
        PolicyWeights close = PolicyWeights.fromString("shuttlePassMinDistMeters=2.0");
        ExpressionNode closeGenome = ObjectiveExpressions.shuttlePass(close, BALL_CAPACITY, DIST_NORM);
        assertEquals(0.87,
                closeGenome.evaluate(WorldStateTerminals.toContext(world, knowledge, DIST_NORM)), 1e-9,
                "open shuttle reads the flat weight");
    }
}
