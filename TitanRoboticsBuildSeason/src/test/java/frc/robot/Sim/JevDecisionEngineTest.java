package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Data.Constants;
import frc.robot.Data.FieldMap;
import frc.robot.Sim.JevDecisionEngine.DecisionResult;
import frc.robot.Sim.JevDecisionEngine.TacticalAction;
import frc.robot.Subsystems.Shooter.ShooterState;

public class JevDecisionEngineTest {

    private JevDecisionEngine engine;

    @BeforeEach
    public void setup() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime(-1.0);
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

    @Test
    public void testSystem2UtilityScoringLatency() {
        WorldState state = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                100.0,
                true,
                false,
                15.0,
                false
        );

        // Warm up JIT
        for (int i = 0; i < 50; i++) {
            engine.evaluatePolicy(state, Archetype.AUTONOMOUS_CYCLER);
        }

        long start = System.nanoTime();
        final int iterations = 1000;
        for (int i = 0; i < iterations; i++) {
            AIActionIntent intent = engine.evaluatePolicy(state, Archetype.AUTONOMOUS_CYCLER);
            assertNotNull(intent);
        }
        long duration = System.nanoTime() - start;
        double avgLatencyMs = (duration / (double) iterations) / 1_000_000.0;
        assertTrue(avgLatencyMs < 0.50, "Average latency must be sub-millisecond, actual: " + avgLatencyMs + " ms");
    }

    @Test
    public void testArchetypePolicyDifferentiation() {
        WorldState state = new WorldState(
                new Pose2d(8.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                0,
                new Pose2d(4.0, 4.0, new Rotation2d()), // opponent near their blue hub
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                false,
                true, // opponent hub active
                15.0,
                true // robot is red
        );

        AIActionIntent cyclerIntent = engine.evaluatePolicy(state, Archetype.AUTONOMOUS_CYCLER);
        AIActionIntent bullyIntent = engine.evaluatePolicy(state, Archetype.DEFENSE_BULLY);

        assertNotEquals(cyclerIntent.objective(), bullyIntent.objective());
        assertTrue(cyclerIntent.objective().isOffensive(), "Cycler should pursue offensive objective");
        assertTrue(bullyIntent.objective().isDefensive(), "Bully should pursue defensive objective");
    }

    @Test
    public void testEndgameRushClimbTransition() {
        WorldState normalState = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                60.0,
                true,
                false,
                20.0,
                false
        );
        AIActionIntent normalIntent = engine.evaluatePolicy(normalState, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, normalIntent.objective());

        WorldState endgameState = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10.0, // t <= 20s
                true,
                false,
                5.0,
                false
        );
        AIActionIntent endgameIntent = engine.evaluatePolicy(endgameState, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, endgameIntent.objective(),
                "Sim bots have no climber: even in endgame they must keep playing, not rush climb");
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, endgameIntent.objective(),
                "Endgame bot with fuel and active hub must keep cycling");

        // The player-facing co-pilot (which advises a robot WITH a climber) still climbs.
        AIActionIntent coPilotEndgame = engine.evaluatePolicy(endgameState, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.RUSH_CLIMB, coPilotEndgame.objective(),
                "CO_PILOT advises the real robot, which has a climber");
    }

    @Test
    public void testGameAgnosticNomenclatureAndAliases() {
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, StrategicObjective.SCORE_GOAL);
        assertEquals(StrategicObjective.STOCKPILE_DEPOT, StrategicObjective.HARVEST_FEEDER);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, StrategicObjective.HARVEST_FIELD_PIECES);
        assertEquals(StrategicObjective.STAGE_STANDOFF, StrategicObjective.STAGE_SCORING_WINDOW);
        assertEquals(StrategicObjective.DENY_SHOOTING_LANE, StrategicObjective.DENY_SCORING_LANE);
        assertEquals(StrategicObjective.RUSH_CLIMB, StrategicObjective.RUSH_ENDGAME);

        WorldState state = new WorldState(
                new Pose2d(), new edu.wpi.first.math.kinematics.ChassisSpeeds(), 5, new Pose2d(), new edu.wpi.first.math.kinematics.ChassisSpeeds(), 100.0, true, false, 12.0, false
        );
        assertEquals(state.heldFuelCount(), state.heldGamePieceCount());
        assertEquals(state.isAllianceHubActive(), state.isAllianceGoalActive());
        assertEquals(state.isOpponentHubActive(), state.isOpponentGoalActive());
        assertEquals(state.timeUntilHubShift(), state.timeUntilGoalShift());
    }

    @Test
    public void testMultiBotSimultaneousExecution() {
        AIRobotSim sim = AIRobotSim.getInstance();
        sim.reset();

        assertNotNull(sim.getDriveSimulation());
        assertEquals(AIRobotSim.INITIAL_HELD_BALLS, sim.getFuelCount());

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Features/Opponent Robot", true);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Simulation/OpponentCount", 3);
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().flush();

        sim.simulationUpdate();

        java.util.List<AIRobotInstance> bots = sim.getAdditionalBots();
        assertEquals(2, bots.size(), "Should have 2 additional bots for opponentCount=3");

        assertNotSame(sim.getDriveSimulation(), bots.get(0).getDriveSimulation());
        assertNotSame(bots.get(0).getDriveSimulation(), bots.get(1).getDriveSimulation());
        assertEquals(Archetype.DEFENSE_BULLY, bots.get(0).getArchetype());
        assertEquals(Archetype.ADAPTIVE_COMPETITOR, bots.get(1).getArchetype());

        sim.reset();
        assertEquals(AIRobotSim.INITIAL_HELD_BALLS, sim.getFuelCount());
        assertEquals(AIRobotSim.INITIAL_HELD_BALLS, bots.get(0).getFuelCount());
    }

    @Test
    public void testClusterWeightedBallScentEvaluator() {
        Pose2d robotPose = new Pose2d(8.27, 4.0, Rotation2d.fromDegrees(0));
        Pose2d target = engine.findClusterWeightedFuelTarget(robotPose, false);
        assertNotNull(target);
        assertTrue(target.getX() > 0.0 && target.getX() < 16.5);
        assertTrue(target.getY() > 0.0 && target.getY() < 8.1);
    }

    @Test
    public void testHighVolumeHarvestAndShootingToLastBall() {
        // Red Hub location is ~ (11.94, 4.035)
        Translation2d redHub = FieldMap.Hubs.getHubLocation2d(true);

        // Case 1: Midfield bot (dist > 4.0m) with 10 balls (under 16 batch threshold)
        // Hub active, shift not ending soon (> 4.5s)
        Pose2d midfieldPose = new Pose2d(redHub.getX() - 5.0, redHub.getY(), new Rotation2d());
        WorldState midfieldHarvestState = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10, // 10 balls: should keep vacuuming, not cycle early!
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                true, // Alliance Hub active
                false,
                10.0, // Shift not ending soon
                true  // Red
        );
        AIActionIntent harvestIntent = engine.evaluatePolicy(midfieldHarvestState, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, harvestIntent.objective(),
                "Midfield bot with 10 balls should keep harvesting large batch instead of abandoning depot early");

        // Case 2: Midfield bot with 18 balls (>= 16 batch threshold)
        WorldState midfieldFullState = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                18, // 18 balls
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                true,
                false,
                10.0,
                true
        );
        AIActionIntent cycleIntent = engine.evaluatePolicy(midfieldFullState, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, cycleIntent.objective(),
                "Midfield bot with 18 balls should initiate scoring run");

        // Case 3: Bot in shooting range (dist <= 4.0m) with only 1 ball left
        // Should keep firing down to the very last ball!
        Pose2d shootingRangePose = new Pose2d(redHub.getX() - 2.5, redHub.getY(), new Rotation2d());
        WorldState lastBallState = new WorldState(
                shootingRangePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                1, // Only 1 ball left
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                true,
                false,
                10.0,
                true
        );
        AIActionIntent lastBallIntent = engine.evaluatePolicy(lastBallState, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, lastBallIntent.objective(),
                "Bot within shooting range should keep cycling/shooting down to the 1st ball");

        // Case 4: Hub inactive staging threshold
        // When shift is not imminent (> 3.5s) and inventory is not full (< 30), bot keeps harvesting/stockpiling
        WorldState inactiveHubShiftNotImminent = new WorldState(
                shootingRangePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                20,
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                false, // Hub inactive
                true,
                10.0, // Shift not imminent
                true
        );
        AIActionIntent keepStockpilingIntent = engine.evaluatePolicy(inactiveHubShiftNotImminent, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, keepStockpilingIntent.objective(),
                "Bot with 20 balls should continue stockpiling to full capacity while hub is inactive and shift not imminent");

        // When shift is imminent (<= 3.5s) and hopper has >= 18 balls, bot stages at standoff arc
        WorldState inactiveHubImminentShift = new WorldState(
                shootingRangePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                20,
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                false, // Hub inactive
                true,
                2.5, // Shift imminent (<= 3.5s)
                true
        );
        AIActionIntent imminentShiftIntent = engine.evaluatePolicy(inactiveHubImminentShift, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.STAGE_STANDOFF, imminentShiftIntent.objective(),
                "Bot with 20 balls should stage at standoff when hub shift is imminent");

        // When inventory is full (30 balls), bot stages at standoff regardless of shift timer
        WorldState inactiveHubFullHopper = new WorldState(
                shootingRangePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                30, // Full hopper
                new Pose2d(4.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                false, // Hub inactive
                true,
                10.0,
                true
        );
        AIActionIntent fullHopperIntent = engine.evaluatePolicy(inactiveHubFullHopper, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.STAGE_STANDOFF, fullHopperIntent.objective(),
                "Bot with full hopper (30 balls) should stage at standoff when hub is inactive");
    }

    @Test
    public void testCapacitiesAndCadenceConstants() {
        assertEquals(30, WorldState.CO_PILOT_CAPACITY, "Co-pilot capacity should be 30");
        assertEquals(30, WorldState.DEFAULT_MAX_CAPACITY, "Default max capacity should be 30");
        assertEquals(0.12, Constants.ShooterConstants.BALL_SPAWN_INTERVAL, 1e-4, "Ball spawn interval should be 0.12s");

        assertTrue(Archetype.DEFENSE_BULLY.isDefensive());
        assertTrue(Archetype.TACTICAL_DEFENDER.isDefensive());
        assertTrue(Archetype.LEAD_PURSUIT_INTERCEPTOR.isDefensive());
        assertFalse(Archetype.AUTONOMOUS_CYCLER.isDefensive());
        assertFalse(Archetype.CO_PILOT.isDefensive());
    }

    @Test
    public void testAutonomousModePreventsPrematureRushClimb() {
        Pose2d botPose = new Pose2d(3.0, 4.0, new Rotation2d());
        // Autonomous world: 12 seconds remaining in auto, bot has 18 fuel, active hub, isAutonomous = true
        WorldState autoWorld = new WorldState(
                botPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                18,
                new Pose2d(8.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12.0,
                true,
                true,
                15.0,
                false, // Blue alliance
                true   // Autonomous mode active!
        );
        AIActionIntent autoIntent = engine.evaluatePolicy(autoWorld, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, autoIntent.objective(),
                "In autonomous mode, bot must NOT rush to climb even when match time is <= 15 seconds");
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, autoIntent.objective(),
                "In autonomous mode, bot with 18 fuel should prioritize cycling and scoring fuel");

        // Teleop endgame world: 12 seconds remaining in teleop, isAutonomous = false
        WorldState teleopEndgameWorld = new WorldState(
                botPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                18,
                new Pose2d(8.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12.0,
                true,
                true,
                15.0,
                false, // Blue alliance
                false  // Teleop mode
        );
        AIActionIntent endgameIntent = engine.evaluatePolicy(teleopEndgameWorld, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, endgameIntent.objective(),
                "Sim bots have no climber: teleop endgame must not select rush climb either");
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, endgameIntent.objective(),
                "In teleop endgame (<= 15s), bot with fuel and active hub keeps cycling");
    }

    @Test
    public void testShootingRejectionOutsideAllianceZone() {
        // Blue Hub is at (4.597, 4.035). Place robot in midfield at (6.0, 4.035).
        // Distance is ~1.403m (well within 1.4m - 3.6m shooting distance), but outside Blue Alliance Zone (X <= 4.5974)
        Pose2d midfieldPose = new Pose2d(6.0, 4.035, new Rotation2d(Math.PI)); // Facing Hub
        WorldState midfieldWorld = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                20,
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0,
                true,
                false,
                15.0,
                false, // Blue alliance
                false
        );

        AIActionIntent intent = engine.evaluatePolicy(midfieldWorld, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(ShooterState.SHOOTING, intent.shooterCommand(),
                "Bot outside Alliance Zone must not command SHOOTING");
        assertFalse(intent.triggerFeedKicker(),
                "Bot outside Alliance Zone must not trigger feed kicker");
    }

    @Test
    public void testAutonomousDefenseSuppressionAndCenterlineBoundary() {
        Pose2d botPose = new Pose2d(3.0, 4.0, new Rotation2d());
        Pose2d playerPose = new Pose2d(10.0, 4.0, new Rotation2d());

        // Auto world with 10 balls preloaded, bot is TACTICAL_DEFENDER
        WorldState autoWithPreload = new WorldState(
                botPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10,
                playerPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                15.0,
                true,
                false,
                15.0,
                false, // Blue alliance
                true   // Autonomous mode
        );

        AIActionIntent autoDefenderIntent = engine.evaluatePolicy(autoWithPreload, Archetype.TACTICAL_DEFENDER);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, autoDefenderIntent.objective(),
                "In autonomous mode, even defender archetypes must score their preloaded fuel instead of illegal defense");

        AIActionIntent autoBullyIntent = engine.evaluatePolicy(autoWithPreload, Archetype.DEFENSE_BULLY);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, autoBullyIntent.objective(),
                "In autonomous mode, bully archetypes must score preload rather than cross-field intercept");

        // Blue alliance fuel search in auto must not cross centerline (X=8.27m)
        Pose2d blueAutoFuelTarget = engine.findClusterWeightedFuelTarget(botPose, false, true);
        assertNotNull(blueAutoFuelTarget);
        assertTrue(blueAutoFuelTarget.getX() <= FieldMap.CENTERLINE_X + 0.05,
                "Blue alliance auto fuel target must not cross centerline into opponent territory: " + blueAutoFuelTarget.getX());

        // Red alliance fuel search in auto must not cross centerline (X=8.27m)
        Pose2d redAutoFuelTarget = engine.findClusterWeightedFuelTarget(playerPose, true, true);
        assertNotNull(redAutoFuelTarget);
        assertTrue(redAutoFuelTarget.getX() >= FieldMap.CENTERLINE_X - 0.05,
                "Red alliance auto fuel target must not cross centerline into opponent territory: " + redAutoFuelTarget.getX());
    }

    @Test
    public void testAutoFillThenVolleyBatching() {
        Pose2d midfieldPose = new Pose2d(10.0, 4.0, new Rotation2d()); // ~5.4m from hub: out of range
        Pose2d inRangePose = new Pose2d(3.0, 4.0, new Rotation2d()); // ~1.6m from hub: in range
        Pose2d oppPose = new Pose2d(13.0, 4.0, new Rotation2d());

        // Partial batch (3 fuel), plenty of clock: keep harvesting, don't cycle one ball at a time.
        WorldState lightLoad = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                3,
                oppPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12.0, true, true, 15.0, false, true);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                engine.evaluatePolicy(lightLoad, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Auto bot with a partial batch out of range must harvest, not cycle");

        // Full batch (8 fuel): commit to the scoring trip.
        WorldState fullBatch = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                8,
                oppPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12.0, true, true, 15.0, false, true);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                engine.evaluatePolicy(fullBatch, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Auto bot with a full batch must commit to scoring");

        // Already in range with a partial batch: finish the volley, don't drive away.
        WorldState inRange = new WorldState(
                inRangePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                3,
                oppPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                12.0, true, true, 15.0, false, true);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                engine.evaluatePolicy(inRange, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Auto bot already in range must fire its partial batch");

        // Clock nearly out: dump the hopper rather than carrying balls home.
        WorldState clockLow = new WorldState(
                midfieldPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                3,
                oppPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                4.0, true, true, 15.0, false, true);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                engine.evaluatePolicy(clockLow, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Auto bot with expiring clock must dump its partial batch");
    }
}
