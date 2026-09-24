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

    @Test
    public void testSmartGlideArbitration() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());

        // 1. Holding fuel + Active Hub -> Hub shooting pose
        Pose2d hubTarget = engine.getSmartGlideTarget(robotPose, true, true, false);
        assertNotNull(hubTarget);
        assertEquals(5.60, hubTarget.getX(), 0.1);
        assertEquals(4.10, hubTarget.getY(), 0.1);

        // 2. Empty + Active Hub -> Neutral Midfield ball hunt
        Pose2d huntTarget = engine.getSmartGlideTarget(robotPose, false, true, false);
        assertNotNull(huntTarget);
        assertEquals(8.27, huntTarget.getX(), 0.1);

        // 3. Inactive Hub -> Feeder / Depot reload
        Pose2d depotTarget = engine.getSmartGlideTarget(robotPose, true, false, false);
        assertNotNull(depotTarget);
        assertEquals(1.50, depotTarget.getX(), 0.1);
    }

    @Test
    public void testLeadPursuitInterception() {
        Pose2d robotPose = new Pose2d(2.0, 2.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(6.0, 2.0, new Rotation2d());
        edu.wpi.first.math.geometry.Translation2d oppVel = new edu.wpi.first.math.geometry.Translation2d(2.0, 0.0);

        // Opponent moving in +X direction at 2.0 m/s; robot max speed 4.5 m/s
        Pose2d interceptTarget = engine.solveLeadPursuitIntercept(robotPose, oppPose, oppVel, 4.5);

        assertNotNull(interceptTarget);
        // Intercept X must be ahead of opponent initial position (X > 6.0)
        assertTrue(interceptTarget.getX() > 6.0, "Intercept point must lead moving opponent along trajectory");
        assertEquals(2.0, interceptTarget.getY(), 0.05);

        // Stationary opponent test
        Pose2d stationaryTarget = engine.solveLeadPursuitIntercept(robotPose, oppPose, new edu.wpi.first.math.geometry.Translation2d(0, 0), 4.5);
        assertNotNull(stationaryTarget);
        assertEquals(6.0, stationaryTarget.getX(), 0.1);
        assertEquals(2.0, stationaryTarget.getY(), 0.1);
    }

    @Test
    public void testDynamicTrenchCorridorSelection() {
        frc.robot.Auto.DynamicRouter.clearObstacles();
        Pose2d robotPose = new Pose2d(2.0, 4.0, new Rotation2d());

        // Place obstacle blocking Top Trench (X=4.5, Y=7.4)
        frc.robot.Auto.DynamicRouter.registerObstacle(
                new edu.wpi.first.math.geometry.Translation2d(4.5, 7.4),
                new edu.wpi.first.math.geometry.Translation2d(),
                0.60,
                5.0,
                false
        );

        // Should dynamically select Bottom Trench
        var chosen = engine.selectOptimalTrenchCorridor(robotPose, true, false);
        assertNotNull(chosen);
        assertEquals("Blue Bottom Trench", chosen.name);

        // Clear and place obstacle in Bottom Trench (X=4.5, Y=0.65)
        frc.robot.Auto.DynamicRouter.clearObstacles();
        frc.robot.Auto.DynamicRouter.registerObstacle(
                new edu.wpi.first.math.geometry.Translation2d(4.5, 0.65),
                new edu.wpi.first.math.geometry.Translation2d(),
                0.60,
                5.0,
                false
        );

        // Should dynamically switch to Top Trench
        chosen = engine.selectOptimalTrenchCorridor(robotPose, true, false);
        assertNotNull(chosen);
        assertEquals("Blue Top Trench", chosen.name);
    }
}
