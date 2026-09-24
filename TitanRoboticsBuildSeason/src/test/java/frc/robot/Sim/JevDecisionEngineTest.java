package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Sim.JevDecisionEngine.DecisionResult;
import frc.robot.Sim.JevDecisionEngine.TacticalAction;

public class JevDecisionEngineTest {

    private JevDecisionEngine engine;

    @BeforeEach
    public void setup() {
        engine = JevDecisionEngine.getInstance();
    }

    @Test
    public void testBlockShootingLaneWhenCloseToHub() {
        // Player is near Blue Hub (4.597, 4.035) with active Hub
        Pose2d playerNearHub = new Pose2d(4.0, 4.0, new Rotation2d());
        Pose2d opponentPose = new Pose2d(8.0, 4.0, new Rotation2d());

        DecisionResult result = engine.evaluate(playerNearHub, opponentPose, 120.0, true, false);

        assertNotNull(result);
        assertEquals(TacticalAction.BLOCK_SHOOTING_LANE, result.action);
        assertTrue(result.confidence > 0.70);
        assertTrue(result.latencyMs < 20.0, "Jev AI latency must be sub-20ms");
        assertTrue(result.targetPose.getX() > 0 && result.targetPose.getX() < 16.5);
    }

    @Test
    public void testContestDepotWhenPlayerAtDepot() {
        // Player is near Blue Depot (1.5, 6.5)
        Pose2d playerAtDepot = new Pose2d(1.8, 6.2, new Rotation2d());
        Pose2d opponentPose = new Pose2d(5.0, 5.0, new Rotation2d());

        DecisionResult result = engine.evaluate(playerAtDepot, opponentPose, 120.0, true, false);

        assertNotNull(result);
        assertEquals(TacticalAction.CONTEST_DEPOT, result.action);
        assertTrue(result.confidence > 0.65);
    }

    @Test
    public void testShadowPlayerAtMidfield() {
        // Player is in midfield transition zone around X=8.0, far from hub and depot
        Pose2d playerMidfield = new Pose2d(8.27, 2.5, new Rotation2d());
        Pose2d opponentPose = new Pose2d(9.0, 4.0, new Rotation2d());

        DecisionResult result = engine.evaluate(playerMidfield, opponentPose, 100.0, true, false);

        assertNotNull(result);
        assertEquals(TacticalAction.SHADOW_PLAYER, result.action);
        assertTrue(result.confidence > 0.60);
    }

    @Test
    public void testRetreatDefenseWhenHubInactive() {
        // Player is far, Hub is inactive
        Pose2d playerPose = new Pose2d(10.0, 1.0, new Rotation2d());
        Pose2d opponentPose = new Pose2d(8.0, 4.0, new Rotation2d());

        DecisionResult result = engine.evaluate(playerPose, opponentPose, 60.0, false, false);

        assertNotNull(result);
        assertEquals(TacticalAction.RETREAT_DEFENSE, result.action);
    }

    @Test
    public void testSchemaJsonOutput() {
        Pose2d playerPose = new Pose2d(4.0, 4.0, new Rotation2d());
        Pose2d opponentPose = new Pose2d(7.0, 4.0, new Rotation2d());

        DecisionResult result = engine.evaluate(playerPose, opponentPose, 90.0, true, false);
        String json = result.toSchemaJson();

        assertNotNull(json);
        assertTrue(json.contains("\"selected_action\""));
        assertTrue(json.contains("\"confidence\""));
        assertTrue(json.contains("\"latency_ms\""));
        assertTrue(json.contains("\"target_pose\""));
        assertTrue(json.contains("\"utility_scores\""));
        assertTrue(json.startsWith("{") && json.endsWith("}"));
    }

    @Test
    public void testOffensiveStrategyArbitration() {
        // When near Hub and Hub is active -> SCORE_HUB_HIGH
        Pose2d playerNearHub = new Pose2d(4.0, 4.0, new Rotation2d());
        var adviceActive = engine.evaluateOffensiveStrategy(playerNearHub, 120.0, true, false);
        assertEquals(JevDecisionEngine.OffensiveStrategy.SCORE_HUB_HIGH, adviceActive.strategy);
        assertTrue(adviceActive.utility > 0.80);

        // When Hub is inactive and near depot -> FEED_DEPOT
        Pose2d playerNearDepot = new Pose2d(2.0, 6.0, new Rotation2d());
        var adviceDepot = engine.evaluateOffensiveStrategy(playerNearDepot, 120.0, false, false);
        assertEquals(JevDecisionEngine.OffensiveStrategy.FEED_DEPOT, adviceDepot.strategy);

        // When Hub is inactive and midfield -> DEFEND_TRANSITION
        Pose2d playerMidfield = new Pose2d(8.0, 3.0, new Rotation2d());
        var adviceDefend = engine.evaluateOffensiveStrategy(playerMidfield, 120.0, false, false);
        assertEquals(JevDecisionEngine.OffensiveStrategy.DEFEND_TRANSITION, adviceDefend.strategy);
    }
}
