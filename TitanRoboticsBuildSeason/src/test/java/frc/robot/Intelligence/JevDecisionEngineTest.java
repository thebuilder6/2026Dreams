package frc.robot.Intelligence;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Data.Constants;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.GlidePoints;
import frc.robot.Navigation.StaticPathfinder;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Sim.AIRobotInstance;
import frc.robot.Sim.AIRobotSim;
import frc.robot.Sim.HubSchedule;
import frc.robot.Intelligence.JevDecisionEngine.DecisionResult;
import frc.robot.Intelligence.JevDecisionEngine.TacticalAction;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class JevDecisionEngineTest {

    /**
     * The phases {@code HubSchedule.phaseFor} can return. {@code DONE} is
     * deliberately absent: the countdown runs out into {@code ENDGAME}, so DONE
     * is only reachable if a caller passes it by hand.
     */
    private static final HubSchedule.Phase[] REACHABLE_PHASES = {
        HubSchedule.Phase.AUTO, HubSchedule.Phase.TRANSITION,
        HubSchedule.Phase.SHIFT1, HubSchedule.Phase.SHIFT2,
        HubSchedule.Phase.SHIFT3, HubSchedule.Phase.SHIFT4,
        HubSchedule.Phase.ENDGAME
    };

    private JevDecisionEngine engine;

    @BeforeEach
    public void setup() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime(-1.0);
        SimulatedArena.getInstance().clearGamePieces();
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

        // 1. Holding fuel + Active Hub -> Hub shooting pose.
        // Assert against the GlidePoints entry rather than a bare literal: the old
        // literal (5.60, 4.10) was the pre-GlidePoints hardcoded fallback and drifted
        // out of sync with the waypoint map.
        Pose2d hubTarget = engine.getSmartGlideTarget(robotPose, true, true, false);
        assertNotNull(hubTarget);
        Pose2d expectedHub = GlidePoints.GLIDE_POINTS.get("Blue Hub Front").pose();
        assertEquals(expectedHub.getX(), hubTarget.getX(), 1e-6);
        assertEquals(expectedHub.getY(), hubTarget.getY(), 1e-6);

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
        frc.robot.Navigation.DynamicRouter.clearObstacles();
        Pose2d robotPose = new Pose2d(2.0, 4.0, new Rotation2d());

        // Place obstacle blocking Top Trench (X=4.5, Y=7.4)
        frc.robot.Navigation.DynamicRouter.registerObstacle(
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
        frc.robot.Navigation.DynamicRouter.clearObstacles();
        frc.robot.Navigation.DynamicRouter.registerObstacle(
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
            engine.evaluatePolicy(state, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        }

        long start = System.nanoTime();
        final int iterations = 1000;
        for (int i = 0; i < iterations; i++) {
            AIActionIntent intent = engine.evaluatePolicy(state, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
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

        AIActionIntent cyclerIntent = engine.evaluatePolicy(state, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        AIActionIntent bullyIntent = engine.evaluatePolicy(state, fuel(0, 0, 0, true), Archetype.DEFENSE_BULLY);

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

        // Without a climber fitted (default false), even co-pilot keeps playing.
        AIActionIntent coPilotNoClimber = engine.evaluatePolicy(endgameState, Archetype.CO_PILOT);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, coPilotNoClimber.objective(),
                "Robot without climber fitted must keep playing, not rush climb");

        // When a climber IS fitted, the robot climbs in endgame.
        WorldState endgameStateWithClimber = endgameState.withHardware(true, 30, true);
        AIActionIntent coPilotWithClimber = engine.evaluatePolicy(endgameStateWithClimber, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.RUSH_CLIMB, coPilotWithClimber.objective(),
                "Robot equipped with climber rushes to climb in endgame");
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
        assertEquals(0.12, frc.robot.Subsystems.shooter.ShooterConstants.BALL_SPAWN_INTERVAL, 1e-4, "Ball spawn interval should be 0.12s");

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
        // Blue Hub is at (4.6256, 4.035). Place robot in midfield at (6.0, 4.035).
        // Distance is ~1.374m (well within 1.4m - 3.6m shooting distance), but outside Blue Alliance Zone (X <= 4.6256)
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

    @Test
    public void testObjectiveCategoriesAndPlanSchema() {
        assertEquals(16, StrategicObjective.values().length,
                "Seven requested objectives extend the existing nine objective values");
        assertTrue(StrategicObjective.SWEEP_ALLIANCE_ZONE.isOffensive());
        assertTrue(StrategicObjective.POACH_OPPONENT_ZONE.isOffensive());
        assertTrue(StrategicObjective.SHUTTLE_PASS.isTeamwork());
        assertTrue(StrategicObjective.LONG_RANGE_SNIPE.isOffensive());
        assertTrue(StrategicObjective.CHOKE_TRENCH.isDefensive());
        assertTrue(StrategicObjective.SCREEN_FOR_ALLY.isTeamwork());
        assertTrue(StrategicObjective.BAIT_PIN_FOUL.isDefensive());

        WorldState world = new WorldState(new Pose2d(8.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 0,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, false, false, 10.0, false);
        AIActionIntent intent = engine.evaluatePolicy(world, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        assertNotNull(intent.plan());
        assertEquals(intent.objective(), intent.plan().currentObjective());
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.plan().nextObjective());
    }

    @Test
    public void testSweepAllianceZoneOutprioritizesMidfield() {
        SimulatedArena arena = SimulatedArena.getInstance();
        arena.clearGamePieces();
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(2.0, 4.0)));
        arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(8.0, 4.0)));

        WorldState world = new WorldState(new Pose2d(9.5, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 1,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, true, false, 15.0, false);
        AIActionIntent intent = engine.evaluatePolicy(world, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.SWEEP_ALLIANCE_ZONE, intent.objective());
        Pose2d target = engine.findAllianceZoneFuelTarget(world.selfPose(), false);
        assertTrue(FieldMap.AllianceZones.isInAllianceZone(new Translation2d(2.0, 4.0), false));
        assertNotNull(target);
        assertTrue(target.getX() <= FieldMap.AllianceZones.BLUE_ZONE_MAX_X,
                "Zone-restricted targeting must keep its target on our side of the Hub");
        arena.clearGamePieces();
    }

    @Test
    public void testOpportunisticCowcatcherIntakeDuringDefense() {
        WorldState world = new WorldState(new Pose2d(2.0, 2.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 5,
                new Pose2d(3.0, 2.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, false, true, 10.0, false);
        AIActionIntent intent = engine.evaluatePolicy(world, fuel(0, 0, 0, true), Archetype.LEAD_PURSUIT_INTERCEPTOR);
        assertEquals(StrategicObjective.LEAD_INTERCEPT, intent.objective());
        assertEquals(IntakeState.INTAKING, intent.intakeCommand());
    }

    @Test
    public void testTransitionBudgetCutsHarvestBeforeHubShift() {
        WorldState world = new WorldState(new Pose2d(10.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 10,
                new Pose2d(13.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, true, false, 1.0, false);
        AIActionIntent intent = engine.evaluatePolicy(world, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective());
        assertEquals(0.0, intent.plan().timeToTransitionSec(), 1e-9);
    }

    @Test
    public void testDirectionalHarvestBias() {
        SimulatedArena arena = SimulatedArena.getInstance();
        arena.clearGamePieces();
        for (int i = 0; i < 3; i++) {
            arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(7.0, 3.8 + i * 0.2)));
            arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(11.0, 3.8 + i * 0.2)));
        }

        Pose2d target = engine.findClusterWeightedFuelTarget(
                new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(180)), false);
        assertTrue(target.getX() < 9.0,
                "Equal-density clusters should favor travel toward the Blue home zone");
        arena.clearGamePieces();
    }

    @Test
    public void testShuttlePassDoesNotLaunchOutsideAllianceZone() {
        WorldState world = new WorldState(new Pose2d(10.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 18,
                new Pose2d(13.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, false, false, 5.0, false);
        AIActionIntent intent = engine.evaluatePolicy(world, homeFuel(), Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.SHUTTLE_PASS, intent.objective());
        assertNotEquals(ShooterState.SHOOTING, intent.shooterCommand());
    }

    // ------------------------------------------------------------------
    // Next-shift awareness (WorldState.isAllianceHubActiveAfterShift /
    // isOpponentHubActiveAfterShift)
    //
    // The engine used to know only whether each hub is live NOW and how long
    // until the next flip. That is not enough to tell a shuttle (which lobs at
    // the opponent's end) from a trip to its own hub, so both fired at the wrong
    // time. These pin the two gates that fix it.
    // ------------------------------------------------------------------

    /** Convenience: a Blue bot at (2.0, 4.03) holding `held`, 6 m from its hub. */
    /**
     * Clairvoyant knowledge with a known amount of field fuel.
     *
     * <p>Zone counts used to be invisible: the engine read them from
     * {@code SimulatedArena}, so a card or test asserting on
     * {@code SWEEP_ALLIANCE_ZONE} / {@code POACH_OPPONENT_ZONE} depended on
     * whatever fuel happened to be in the live arena. Since the
     * clairvoyant/observed split the counts are an explicit input, so a test that
     * wants fuel in a zone must say so.
     *
     * @param allianceZone fuel in our own zone
     * @param opponentZone fuel in the opponent's zone
     * @param observed    whether an opponent robot is known
     */
    private static MatchKnowledge fuel(int allianceZone, int opponentZone,
            int midfield, boolean observed) {
        return new ClairvoyantKnowledge(
                0, 0, 0, 0, 0,
                List.of(), observed ? List.of(new Pose2d(12.0, 4.0, new Rotation2d())) : List.of(),
                List.of(), List.of(),
                allianceZone, midfield, opponentZone);
    }

    /** Plenty of fuel in our own zone, nothing elsewhere: a harvest card's default. */
    private static MatchKnowledge homeFuel() {
        return fuel(12, 0, 0, false);
    }

    private WorldState shiftState(boolean mineActive, boolean theirsActive,
            boolean mineAfter, boolean theirsAfter, int held, double timeToShift) {
        return new WorldState(
                new Pose2d(2.0, 4.03, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), held,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, mineActive, theirsActive, timeToShift,
                /* isRedAlliance */ false, /* isAutonomous */ false,
                mineAfter, theirsAfter);
    }

    @Test
    public void shuttlePassIsRefusedWhenNeitherHubWillBeLive() {
        // NOTE: unreachable in a real match. Both hubs are never dark at the same
        // time -- SHIFT1-4 have exactly one live, AUTO/TRANSITION/ENDGAME have both.
        // So this asserts the gate's invariant defensively rather than a bug fix.
        WorldState bothDark = shiftState(false, false, false, false, 20, 5.0);
        AIActionIntent intent = engine.evaluatePolicy(bothDark, Archetype.AUTONOMOUS_CYCLER);
        assertNotEquals(StrategicObjective.SHUTTLE_PASS, intent.objective(),
                "must not lob 20 balls at a hub that is dark now and after the shift");
    }

    @Test
    public void hubScheduleNeverLeavesBothHubsDark() {
        // The invariant the shuttle gate leans on. If this ever fails, the
        // explicit opponent-hub check in SHUTTLE_PASS stops being a no-op.
        //
        // Scoped to the phases HubSchedule.phaseFor can actually return
        // (AUTO, TRANSITION, SHIFT1-4, ENDGAME). DONE is excluded and pinned
        // separately below, because it is the one phase where the invariant
        // genuinely does not hold -- iterating Phase.values() as the original
        // version of this test did asserted a premise the schedule never made.
        for (HubSchedule.Phase p : REACHABLE_PHASES) {
            boolean blue = HubSchedule.isHubActive(false, p, 'R');
            boolean red = HubSchedule.isHubActive(true, p, 'R');
            assertTrue(blue || red, "both hubs dark in phase " + p);
        }
    }

    @Test
    public void doneIsTheOnePhaseWhereBothHubsAreDark() {
        // Records the exception the narrowed invariant above cannot cover, so
        // widening the reachable set later is a deliberate act rather than a
        // silent one. phaseFor() never returns DONE -- the match clock is
        // exhausted into ENDGAME (HubSchedule.phaseFor falls through to it) --
        // so this state is only reachable if a caller passes DONE by hand.
        assertFalse(HubSchedule.isHubActive(false, HubSchedule.Phase.DONE, 'R'));
        assertFalse(HubSchedule.isHubActive(true, HubSchedule.Phase.DONE, 'R'));
        assertFalse(HubSchedule.phaseFor(-1.0, false) == HubSchedule.Phase.DONE,
                "phaseFor must never produce DONE; the clock runs out into ENDGAME");
    }

    @Test
    public void shuttlePassIsGeometricallyUnreachable() {
        // Replaces two tests that asserted
        //     assertNotEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective())
        // while their own comment said what mattered was "that the shuttle is
        // reachable rather than gated to zero". A ranking assertion tests
        // reachability only incidentally, and in the passing case it was
        // measuring which harvest objective won -- not whether the shuttle
        // could ever be selected at all.
        //
        // The shuttle gate (JevDecisionEngine) requires ALL of:
        //     in our own alliance zone  AND  distToSelfHub > 6.0
        // Our own zone is the half-plane 0 <= x <= BLUE_HUB_X, and the hub sits
        // at (BLUE_HUB_X, FIELD_WIDTH/2) -- i.e. on the zone's own far edge.
        // The farthest point of the zone from that hub is the wall corner
        // (0, 0), at 6.138 m, which clears the 6.0 m bar. But a robot centre
        // cannot stand on the wall: it is held off by BUMPER_MARGIN (0.45 m),
        // which caps the achievable distance at 5.504 m. The gate is therefore
        // unsatisfiable everywhere the robot can actually be, so
        // shuttleUtility is a structural 0.0 and SHUTTLE_PASS is dead.
        //
        // This is a geometric proof, not an argmax observation, so it does not
        // depend on any utility weighting and cannot be made to pass by
        // rebalancing. It is a tripwire: if the zone geometry, the hub
        // position, or BUMPER_MARGIN ever changes so the gate becomes
        // satisfiable, this fails and the objective deserves a real test.
        double hubX = FieldMap.Hubs.BLUE_HUB_X;
        double hubY = FieldMap.Hubs.HUB_Y;
        double margin = StaticPathfinder.BUMPER_MARGIN;

        double bestAchievable = -1.0;
        for (boolean isRed : new boolean[] {false, true}) {
            // Sweep the drivable footprint of our own zone: centre held one
            // bumper half-width off every wall it would otherwise be against.
            double xMin = isRed ? FieldMap.AllianceZones.RED_ZONE_MIN_X + margin : margin;
            double xMax = isRed ? FieldMap.FIELD_LENGTH - margin : hubX;
            for (double x = xMin; x <= xMax + 1e-9; x += 0.05) {
                for (double y = margin; y <= FieldMap.FIELD_WIDTH - margin + 1e-9; y += 0.05) {
                    Translation2d at = new Translation2d(x, y);
                    if (!FieldMap.AllianceZones.isInAllianceZone(at, isRed)) {
                        continue;
                    }
                    double hub = isRed ? FieldMap.FIELD_LENGTH - hubX : hubX;
                    bestAchievable = Math.max(bestAchievable, at.getDistance(new Translation2d(hub, hubY)));
                }
            }
        }

        assertTrue(bestAchievable > 0.0, "sweep found no drivable pose in either alliance zone");
        assertTrue(bestAchievable <= 6.0,
                "SHUTTLE_PASS requires distToSelfHub > 6.0 but the best drivable pose is "
                        + String.format("%.3f", bestAchievable)
                        + " m away. The objective is geometrically reachable again, so it needs a"
                        + " real behavioural test and the finding in KNOWN_ISSUES.md can be closed.");
    }

    @Test
    public void shuttlePassIsNeverSelectedBecauseItsGateIsUnsatisfiable() {
        // The behavioural consequence of the geometric result above, asserted
        // over the hub states the gate cares about, so the dead objective is
        // pinned from both sides: provably unreachable geometrically, and
        // never actually selected.
        for (boolean theirsActive : new boolean[] {false, true}) {
            for (boolean theirsAfter : new boolean[] {false, true}) {
                WorldState openingWindow =
                        shiftState(false, theirsActive, false, theirsAfter, 20, 5.0);
                AIActionIntent intent = engine.evaluatePolicy(
                        openingWindow, fuel(14, 8, 0, true), Archetype.AUTONOMOUS_CYCLER);
                assertNotEquals(StrategicObjective.SHUTTLE_PASS, intent.objective(),
                        "SHUTTLE_PASS should be unreachable; if it now wins, its gate changed");
            }
        }
    }

    @Test
    public void poachIsRefusedWhenTheOpponentHubIsTheOneGoingDark() {
        // SHIFT2/SHIFT4 shape: OUR hub is dark now and about to open, the opponent's
        // is live now and about to close. Sprinting across for their loose fuel would
        // abandon our own scoring window the moment it opens. The gate now requires
        // the opponent's hub to be the one that opens next.
        WorldState ownHubOpening = shiftState(false, true, true, false, 10, 3.0);
        AIActionIntent intent = engine.evaluatePolicy(ownHubOpening, fuel(14, 8, 0, true), Archetype.AUTONOMOUS_CYCLER);
        // VACUOUS TODAY. POACH_OPPONENT_ZONE is a flat 0.78 and every competitor in
        // this state outranks it, so this assertion passes with or without the gate.
        // It is kept to document the intent, but it is NOT evidence the gate works.
        // See KNOWN_ISSUES.md: POACH may be structurally unreachable, in which case
        // the gate is untestable until the utility table is fixed. A real test needs
        // a utility-map dump, which DecisionCards does not currently provide.
        assertNotEquals(StrategicObjective.POACH_OPPONENT_ZONE, intent.objective(),
                "must not poach away from our own hub opening window");
    }

    @Test
    public void poachIsAllowedWhenTheOpponentHubIsTheOneOpening() {
        // SHIFT1/SHIFT3 shape: our hub is live and about to close, theirs is dark and
        // about to open. Poaching towards the opening window is the intended play.
        WorldState theirsOpening = shiftState(true, false, false, true, 10, 3.0);
        AIActionIntent intent = engine.evaluatePolicy(theirsOpening, fuel(14, 8, 0, true), Archetype.AUTONOMOUS_CYCLER);
        // The objective itself may still lose the utility race; what matters is that
        // poach is reachable rather than gated to zero.
        assertNotEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective(),
                "with an opponent window opening, harvesting midfield should not be the plan");
    }

    @Test
    public void worldStateExposesShiftTransitionHelpers() {
        WorldState lastWindow = shiftState(true, false, false, true, 12, 4.0);
        assertTrue(lastWindow.willOwnHubDeactivate(),
                "own hub live now and dark next means this is the last window");
        assertTrue(lastWindow.willOpponentHubActivate(),
                "opponent hub dark now and live next means an opening");

        WorldState stable = shiftState(true, true, true, true, 12, 0.0);
        assertFalse(stable.willOwnHubDeactivate());
        assertFalse(stable.willOpponentHubActivate());
    }

    @Test
    public void legacyWorldStateConstructorsMirrorCurrentHubState() {
        // The 10- and 11-arg constructors must describe "no pending flip" so every
        // pre-existing caller stays behaviourally identical.
        WorldState legacy = new WorldState(new Pose2d(2.0, 4.03, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 5,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, true, false, 5.0, false);
        assertEquals(legacy.isAllianceHubActive(), legacy.isAllianceHubActiveAfterShift());
        assertEquals(legacy.isOpponentHubActive(), legacy.isOpponentHubActiveAfterShift());
        assertFalse(legacy.willOwnHubDeactivate());
        assertFalse(legacy.willOpponentHubActivate());
    }

    @Test
    public void testLongRangeSnipeUsesSharedShotEnvelope() {
        assertEquals(4.20, FieldMap.Hubs.SHOOTING_MAX_DISTANCE, 1e-9,
                "Shot envelope has a single owner; decision and sim must read FieldMap.Hubs.SHOOTING_MAX_DISTANCE");
        Translation2d hub = FieldMap.Hubs.BLUE_HUB_2D;

        // 4.1 m: inside the shared envelope (old sim limit 4.0 would reject, old
        // snipe limit 4.3 allowed) — must snipe with a live feed request.
        Pose2d outerEdgePose = new Pose2d(hub.getX() - 4.1, hub.getY(), new Rotation2d());
        WorldState outerEdge = new WorldState(outerEdgePose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 15,
                new Pose2d(13.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, true, false, 20.0, false);
        AIActionIntent snipe = engine.evaluatePolicy(outerEdge, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.LONG_RANGE_SNIPE, snipe.objective(),
                "Bot at 4.1 m with hub active and 15 fuel must take the outer-perimeter shot");
        assertEquals(ShooterState.SHOOTING, snipe.shooterCommand());
        assertTrue(snipe.triggerFeedKicker(), "Aimed snipe (0 deg error) must request the feed");

        // 4.4 m: beyond the shared envelope — must not snipe.
        Pose2d beyondPose = new Pose2d(hub.getX() - 4.4, hub.getY(), new Rotation2d());
        WorldState beyond = new WorldState(beyondPose,
                new edu.wpi.first.math.kinematics.ChassisSpeeds(), 15,
                new Pose2d(13.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                90.0, true, false, 20.0, false);
        assertNotEquals(StrategicObjective.LONG_RANGE_SNIPE,
                engine.evaluatePolicy(beyond, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Bot at 4.4 m is beyond the shared envelope and must not snipe");
    }

    @Test
    public void testChokeTrenchRequiresOccupiedTrench() {
        Pose2d selfPose = new Pose2d(8.0, 4.0, new Rotation2d());
        edu.wpi.first.math.kinematics.ChassisSpeeds zero = new edu.wpi.first.math.kinematics.ChassisSpeeds();

        // Opponent poaching in our (Blue) top trench: contest the midfield exit.
        WorldState ownTrench = new WorldState(selfPose, zero, 30,
                new Pose2d(4.5, 7.42, new Rotation2d()), zero,
                90.0, false, false, 20.0, false);
        AIActionIntent own = engine.evaluatePolicy(ownTrench, fuel(0, 0, 0, true), Archetype.TACTICAL_DEFENDER);
        assertEquals(StrategicObjective.CHOKE_TRENCH, own.objective());
        Translation2d ownExit = new Translation2d(FieldMap.Trenches.BLUE_TRENCH_MAX_X + 0.9,
                FieldMap.Trenches.TOP_CORRIDOR_Y);
        assertTrue(own.navigationTarget().getTranslation().getDistance(ownExit) < 1.2,
                "Choke must stage at the occupied trench's midfield exit: " + own.navigationTarget());
        assertFalse(StaticPathfinder.isPointInHardObstacle(own.navigationTarget().getTranslation()),
                "Choke staging point must be reachable, not inside an obstacle");

        // Opponent in the far-side (Red) top trench: target must stay on the
        // occupied side, never mirror onto ours.
        WorldState farTrench = new WorldState(selfPose, zero, 30,
                new Pose2d(12.0, 7.42, new Rotation2d()), zero,
                90.0, false, false, 20.0, false);
        AIActionIntent far = engine.evaluatePolicy(farTrench, fuel(0, 0, 0, true), Archetype.TACTICAL_DEFENDER);
        assertEquals(StrategicObjective.CHOKE_TRENCH, far.objective());
        assertTrue(far.navigationTarget().getX() > FieldMap.CENTERLINE_X,
                "Far-side choke target must stay on the occupied (Red) half: " + far.navigationTarget());
        Translation2d farExit = new Translation2d(FieldMap.Trenches.RED_TRENCH_MIN_X - 0.9,
                FieldMap.Trenches.TOP_CORRIDOR_Y);
        assertTrue(far.navigationTarget().getTranslation().getDistance(farExit) < 1.2,
                "Far-side choke must stage at the Red trench's midfield exit: " + far.navigationTarget());
        assertFalse(StaticPathfinder.isPointInHardObstacle(far.navigationTarget().getTranslation()),
                "Far-side choke staging point must be reachable, not inside an obstacle");

        // Opponent midfield at trench X but center field: not a trench
        // occupant — must not pull the defender to a trench mouth.
        WorldState midfieldX = new WorldState(selfPose, zero, 30,
                new Pose2d(4.5, 4.0, new Rotation2d()), zero,
                90.0, false, false, 20.0, false);
        assertNotEquals(StrategicObjective.CHOKE_TRENCH,
                engine.evaluatePolicy(midfieldX, Archetype.TACTICAL_DEFENDER).objective(),
                "Midfield opponent at trench X is not a trench occupant");
    }

    @Test
    public void testDenyShootingLaneStagesOutsideHubFootprint() {
        Translation2d redHub = FieldMap.Hubs.RED_HUB_2D;
        Pose2d selfPose = new Pose2d(8.0, 4.0, new Rotation2d());
        edu.wpi.first.math.kinematics.ChassisSpeeds zero = new edu.wpi.first.math.kinematics.ChassisSpeeds();

        // Shooter 2.9 m out: block between shooter and Hub, outside the shell.
        WorldState approach = new WorldState(selfPose, zero, 30,
                new Pose2d(9.0, 4.0, new Rotation2d()), zero,
                90.0, false, true, 20.0, false);
        AIActionIntent deny = engine.evaluatePolicy(approach, fuel(0, 0, 0, true), Archetype.TACTICAL_DEFENDER);
        assertEquals(StrategicObjective.DENY_SHOOTING_LANE, deny.objective());
        double approachDist = deny.navigationTarget().getTranslation().getDistance(redHub);
        assertTrue(approachDist >= 1.9,
                "Lane block must stage outside the Hub safety shell, dist: " + approachDist);
        assertFalse(StaticPathfinder.isPointInHardObstacle(deny.navigationTarget().getTranslation()),
                "Lane block must be reachable, not inside an obstacle");

        // Shooter already inside the shell (1.4 m): stage off to the side,
        // never between shooter and Hub.
        WorldState inside = new WorldState(selfPose, zero, 30,
                new Pose2d(10.5, 4.04, new Rotation2d()), zero,
                90.0, false, true, 20.0, false);
        AIActionIntent side = engine.evaluatePolicy(inside, fuel(0, 0, 0, true), Archetype.TACTICAL_DEFENDER);
        assertEquals(StrategicObjective.DENY_SHOOTING_LANE, side.objective());
        double sideDist = side.navigationTarget().getTranslation().getDistance(redHub);
        assertTrue(sideDist >= 1.9,
                "Close-range lane block must stage laterally outside the shell, dist: " + sideDist);
        assertFalse(StaticPathfinder.isPointInHardObstacle(side.navigationTarget().getTranslation()),
                "Lateral lane block must be reachable, not inside an obstacle");
    }

    // ── Shift-clock semantics ────────────────────────────────────────────────
    //
    // The sim shift clock used to be pinned at exactly 0.0 (WorldStateBuilder
    // read Dashboard.getTimeUntilSwitch(), which the DS reports as 0 in sim).
    // These pin the three places that a 0.0 clock distorts, plus the two places
    // it does not. See KNOWN_ISSUES.md §A "Jev engine has no correct hub-shift
    // clock in simulation".

    private static final edu.wpi.first.math.kinematics.ChassisSpeeds ZERO_VEL =
            new edu.wpi.first.math.kinematics.ChassisSpeeds();

    @Test
    public void testZeroShiftClockForcesScoringOverHarvesting() {
        // A 0.0 clock is not neutral: timeLeftToHarvest = 0 - transit is always
        // <= 0, which fires the "cannot harvest in time, go score" override and
        // pins scoreUtility to 0.98 while zeroing vacuum and sweep. A hub-active
        // bot holding a partial load therefore scores instead of batching up.
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(false);
        Pose2d midfield = new Pose2d(hub.getX() + 5.0, hub.getY(), new Rotation2d());

        WorldState frozen = new WorldState(midfield, ZERO_VEL, 10,
                new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                90.0, true, false, 0.0, false);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB,
                engine.evaluatePolicy(frozen, Archetype.AUTONOMOUS_CYCLER).objective(),
                "With no shift clock (t=0, i.e. transition/endgame) a loaded bot must score, not harvest");

        // Same field state, but a real mid-shift clock: 20s of shift left, so
        // timeLeftToHarvest is comfortably positive and the override must not
        // fire. 10 fuel is under minFuelToScore 16, so the bot keeps batching.
        WorldState midShift = new WorldState(midfield, ZERO_VEL, 10,
                new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                90.0, true, false, 20.0, false);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                engine.evaluatePolicy(midShift, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Mid-shift a partially loaded bot must keep harvesting, not dump a partial load");
    }

    @Test
    public void testStagingBeatsHarvestingOnlyNearShiftBoundary() {
        // Pins that the stageUtility/vacuumUtility balance is ALREADY correct
        // once the clock works, so no constant retune is warranted. Hub inactive,
        // hopper stocked to the 18-ball staging threshold.
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(false);
        Pose2d loaded = new Pose2d(hub.getX() + 5.0, hub.getY(), new Rotation2d());

        WorldState nearFlip = new WorldState(loaded, ZERO_VEL, 18,
                new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                90.0, false, false, 2.0, false);
        assertEquals(StrategicObjective.STAGE_STANDOFF,
                engine.evaluatePolicy(nearFlip, Archetype.AUTONOMOUS_CYCLER).objective(),
                "2s before the flip a stocked bot must stage, not keep harvesting (stage 0.95 > vacuum 0.92)");

        // Control: same state, mid-shift. Staging drops to its 0.80 base and
        // harvesting (0.92) must win again.
        WorldState midShift = new WorldState(loaded, ZERO_VEL, 18,
                new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                90.0, false, false, 20.0, false);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD,
                engine.evaluatePolicy(midShift, Archetype.AUTONOMOUS_CYCLER).objective(),
                "Mid-shift with the hub locked, harvesting must still outrank staging");
    }

    @Test
    public void testPoachSuppressedWithoutAnApproachingShift() {
        // POACH_OPPONENT_ZONE's guard is now `0 < t <= 6`. The 0.0 floor matters
        // because HubSchedule.timeUntilShiftEnd() returns 0.0 by design through
        // AUTO/TRANSITION/ENDGAME/DONE, where no flip is coming -- a bare
        // `<= 6.0` stayed true for the entire endgame.
        SimulatedArena arena = SimulatedArena.getInstance();
        arena.clearGamePieces();
        // Opponent (Red) half fuel, so the zone-count precondition can be met.
        for (int i = 0; i < 4; i++) {
            arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(11.0, 3.6 + i * 0.3)));
        }
        try {
            Translation2d hub = FieldMap.Hubs.getHubLocation2d(false);
            Pose2d ours = new Pose2d(hub.getX() + 3.0, hub.getY(), new Rotation2d());
            // held < 20 and not inventory-full, so only the clock can gate poach.
            for (double t : new double[] {0.0, 20.0, 3.0, 6.0}) {
                WorldState w = new WorldState(ours, ZERO_VEL, 8,
                        new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                        90.0, true, false, t, false);
                assertNotEquals(StrategicObjective.POACH_OPPONENT_ZONE,
                        engine.evaluatePolicy(w, Archetype.AUTONOMOUS_CYCLER).objective(),
                        "Poaching the opponent zone must never be selected at t=" + t);
            }
        } finally {
            arena.clearGamePieces();
        }
    }

    @Test
    public void testPoachUtilityIsCurrentlyUnreachable() {
        // Characterization, and the reason the guard fix above has no behavioural
        // effect today: POACH scores a flat 0.78, but whenever poach is eligible
        // (!autonomous, !inventory-full) VACUUM_MIDFIELD is at least 0.82 and
        // STOCKPILE_DEPOT at least 0.86, so poach can never win the argmax. The
        // `0 < t <= 6` guard is a latent-defect fix that matters the moment poach
        // is ever worth more than ~0.88 -- this test is the tripwire for that.
        assertNotEquals(StrategicObjective.POACH_OPPONENT_ZONE,
                engine.evaluatePolicy(
                        new WorldState(new Pose2d(6.0, 4.04, new Rotation2d()), ZERO_VEL, 8,
                                new Pose2d(12.0, 4.04, new Rotation2d()), ZERO_VEL,
                                90.0, true, false, 5.0, false),
                        Archetype.AUTONOMOUS_CYCLER).objective(),
                "If poach ever becomes selectable, this characterization must be revisited with the guard");
    }

    @Test
    public void testEmptyKnowledgeDefendersDegradeSafelyWithoutClimbing() {
        Pose2d selfPose = new Pose2d(6.0, 4.0, new Rotation2d());
        Pose2d placeholderOpp = new Pose2d(0.0, 0.0, new Rotation2d());
        // Verify across all inventory levels (empty 0, partial 8, 15, and full 30)
        // and across both active and inactive hub states.
        int[] fuelLevels = {0, 8, 15, 30};
        boolean[] hubStates = {false, true};

        Archetype[] defenders = {
                Archetype.TACTICAL_DEFENDER,
                Archetype.DEFENSE_BULLY,
                Archetype.LEAD_PURSUIT_INTERCEPTOR
        };

        for (int fuel : fuelLevels) {
            for (boolean hubActive : hubStates) {
                WorldState defWorld = new WorldState(selfPose, ZERO_VEL, fuel,
                        placeholderOpp, ZERO_VEL, 90.0, hubActive, false, 0.0, false);

                for (Archetype defender : defenders) {
                    // Test both explicit ObservedKnowledge.selfOnly() and the 2-arg overload
                    AIActionIntent explicitIntent = engine.evaluatePolicy(defWorld, ObservedKnowledge.selfOnly(), defender);
                    AIActionIntent defaultIntent = engine.evaluatePolicy(defWorld, defender);

                    for (AIActionIntent intent : new AIActionIntent[] { explicitIntent, defaultIntent }) {
                        assertNotEquals(StrategicObjective.RUSH_CLIMB, intent.objective(),
                                defender + " (fuel=" + fuel + ", hub=" + hubActive + ") with empty knowledge must never select RUSH_CLIMB");
                        assertNotEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective(),
                                defender + " (fuel=" + fuel + ", hub=" + hubActive + ") with empty knowledge must not try to score at hub");
                        assertNotEquals(StrategicObjective.VACUUM_MIDFIELD, intent.objective(),
                                defender + " (fuel=" + fuel + ", hub=" + hubActive + ") with empty knowledge must not vacuum fuel");
                        assertEquals(StrategicObjective.SHADOW_MIDLINE, intent.objective(),
                                defender + " (fuel=" + fuel + ", hub=" + hubActive + ") with zero information must degrade safely to midline zone defense");

                        // Target must be safely positioned at the midline center, not clamped to corner (0,0)
                        Pose2d navTarget = intent.navigationTarget();
                        assertEquals(FieldMap.FIELD_WIDTH / 2.0, navTarget.getY(), 1e-4,
                                "Midline patrol Y must be centered on the field");
                        assertEquals(FieldMap.CENTERLINE_X - 0.8, navTarget.getX(), 1e-4,
                                "Blue defender must patrol 0.8m on its own side of the centerline");
                        assertEquals(0.0, navTarget.getRotation().getDegrees(), 1e-4,
                                "Blue defender must face toward opponent half");
                    }
                }
            }
        }
    }

    @Test
    public void testDefenderWithCowcatcherFuelNeverScoresAtActiveHub() {
        Pose2d selfPose = new Pose2d(6.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(3.0, 4.0, new Rotation2d());
        // Hub active, shift timer 0.0 (timeLeftToHarvest <= 0.0), held fuel = 12 (>= 8 threshold).
        // Opponent is observed near our hub.
        WorldState loadedDefWorld = new WorldState(selfPose, ZERO_VEL, 12,
                oppPose, ZERO_VEL, 90.0, true, false, 0.0, false);

        Archetype[] defenders = {
                Archetype.TACTICAL_DEFENDER,
                Archetype.DEFENSE_BULLY,
                Archetype.LEAD_PURSUIT_INTERCEPTOR
        };

        for (Archetype defender : defenders) {
            AIActionIntent intent = engine.evaluatePolicy(loadedDefWorld, fuel(0, 0, 0, true), defender);
            assertNotEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective(),
                    defender + " holding cowcatcher fuel must never abandon defense to score at hub");
            assertTrue(intent.objective().isDefensive(),
                    defender + " holding cowcatcher fuel must pursue defensive objective, was: " + intent.objective());
        }
    }

    @Test
    public void testDefenderNeverSelectsRushClimbEvenInEndgame() {
        Pose2d selfPose = new Pose2d(6.0, 4.0, new Rotation2d());
        // Match time = 5.0s (deep endgame). Simulated defenders have no climber fitted.
        WorldState endgameWorld = new WorldState(selfPose, ZERO_VEL, 30,
                new Pose2d(12.0, 4.0, new Rotation2d()), ZERO_VEL, 5.0, true, true, 0.0, false);

        AIActionIntent intent = engine.evaluatePolicy(endgameWorld, Archetype.TACTICAL_DEFENDER);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, intent.objective(),
                "Sparring defenders do not have climbers fitted and must not climb in endgame");
    }

    @Test
    public void testZeroInformationOffensiveBotDegradesSafelyWithoutClimbing() {
        Pose2d selfPose = new Pose2d(6.0, 4.0, new Rotation2d());
        Pose2d placeholderOpp = new Pose2d(0.0, 0.0, new Rotation2d());

        // Empty hopper (0 held), teleop: cycler degrades to VACUUM_MIDFIELD
        WorldState emptyCyclerWorld = new WorldState(selfPose, ZERO_VEL, 0,
                placeholderOpp, ZERO_VEL, 90.0, false, false, 20.0, false);
        AIActionIntent emptyCycler = engine.evaluatePolicy(emptyCyclerWorld, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, emptyCycler.objective(),
                "Empty hopper cycler with zero information must harvest midfield");

        // Partial hopper (10 held), hub inactive, shift not imminent (20s): keeps harvesting to fill batch
        WorldState partialInactiveWorld = new WorldState(selfPose, ZERO_VEL, 10,
                placeholderOpp, ZERO_VEL, 90.0, false, false, 20.0, false);
        AIActionIntent partialInactive = engine.evaluatePolicy(partialInactiveWorld, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.VACUUM_MIDFIELD, partialInactive.objective(),
                "Partial hopper cycler while hub is inactive and shift not imminent must keep stockpiling");

        // Full inventory (30 held), hub inactive: all harvesting collapses, cycler stages standoff
        WorldState fullCyclerWorld = new WorldState(selfPose, ZERO_VEL, 30,
                placeholderOpp, ZERO_VEL, 90.0, false, false, 20.0, false);
        AIActionIntent fullCycler = engine.evaluatePolicy(fullCyclerWorld, Archetype.AUTONOMOUS_CYCLER);
        assertEquals(StrategicObjective.STAGE_STANDOFF, fullCycler.objective(),
                "Full hopper cycler while hub is inactive must stage standoff");

        // Full inventory (30 held), hub inactive, co-pilot: stages standoff, never climbs outside endgame
        WorldState fullOffWorld = new WorldState(selfPose, ZERO_VEL, 30,
                placeholderOpp, ZERO_VEL, 90.0, false, false, 20.0, false);
        AIActionIntent coPilot = engine.evaluatePolicy(fullOffWorld, Archetype.CO_PILOT);
        assertNotEquals(StrategicObjective.RUSH_CLIMB, coPilot.objective(),
                "Co-pilot outside endgame must not select RUSH_CLIMB");
        assertEquals(StrategicObjective.STAGE_STANDOFF, coPilot.objective(),
                "Full hopper while hub is inactive must stage standoff");
    }

    @Test
    public void testEvaluateUtilityScoresAndPoachReachabilityAnalysis() {
        Pose2d selfPose = new Pose2d(6.0, 4.0, new Rotation2d());
        Pose2d placeholderOpp = new Pose2d(12.0, 4.0, new Rotation2d());

        // 1. Verify utility evaluation for AUTONOMOUS_CYCLER with an approaching shift flip
        // (SHIFT1/SHIFT3: our hub active now and going dark, opponent hub dark now and opening next)
        WorldState flipApproaching = shiftState(true, false, false, true, 10, 4.0);
        MatchKnowledge knowledgeWithFuel = fuel(0, 15, 20, true);

        Map<StrategicObjective, Double> cyclerUtils =
                engine.evaluateUtilityScores(flipApproaching, Collections.emptySet(), knowledgeWithFuel, Archetype.AUTONOMOUS_CYCLER);

        assertNotNull(cyclerUtils);
        assertTrue(cyclerUtils.containsKey(StrategicObjective.POACH_OPPONENT_ZONE));
        assertTrue(cyclerUtils.containsKey(StrategicObjective.VACUUM_MIDFIELD));
        assertTrue(cyclerUtils.containsKey(StrategicObjective.CYCLE_SCORE_HUB));
        assertTrue(cyclerUtils.containsKey(StrategicObjective.STOCKPILE_DEPOT));

        // In this state, POACH_OPPONENT_ZONE evaluates to 0.78
        double poachScore = cyclerUtils.get(StrategicObjective.POACH_OPPONENT_ZONE);
        assertEquals(0.78, poachScore, 0.001, "POACH utility should evaluate to 0.78 when eligible");

        // Verify mathematical domination when our hub is active: CYCLE_SCORE_HUB (0.85) strictly beats POACH (0.78)
        double scoreUtility = cyclerUtils.get(StrategicObjective.CYCLE_SCORE_HUB);
        assertTrue(scoreUtility > poachScore,
                "CYCLE_SCORE_HUB (" + scoreUtility + ") strictly beats POACH (" + poachScore + ") when hub is active");

        // Verify mathematical domination when our hub is inactive: VACUUM_MIDFIELD (>= 0.88) and STOCKPILE_DEPOT (>= 0.86) beat POACH (0.78)
        WorldState inactiveHub = shiftState(false, true, true, false, 10, 4.0);
        Map<StrategicObjective, Double> inactiveUtils =
                engine.evaluateUtilityScores(inactiveHub, Collections.emptySet(), knowledgeWithFuel, Archetype.AUTONOMOUS_CYCLER);
        double vacuumInactive = inactiveUtils.get(StrategicObjective.VACUUM_MIDFIELD);
        double stockpileInactive = inactiveUtils.get(StrategicObjective.STOCKPILE_DEPOT);
        assertTrue(vacuumInactive > 0.78,
                "VACUUM_MIDFIELD (" + vacuumInactive + ") strictly exceeds POACH (0.78) when hub is inactive");
        assertTrue(stockpileInactive > 0.78,
                "STOCKPILE_DEPOT (" + stockpileInactive + ") strictly exceeds POACH (0.78) when hub is inactive");

        // 2. Verify utility evaluation for defensive archetypes: offensive scores must all be zero
        Map<StrategicObjective, Double> defenderUtils =
                engine.evaluateUtilityScores(flipApproaching, Collections.emptySet(), knowledgeWithFuel, Archetype.TACTICAL_DEFENDER);

        assertEquals(0.0, defenderUtils.get(StrategicObjective.CYCLE_SCORE_HUB));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.STAGE_STANDOFF));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.VACUUM_MIDFIELD));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.STOCKPILE_DEPOT));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.SWEEP_ALLIANCE_ZONE));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.POACH_OPPONENT_ZONE));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.SHUTTLE_PASS));
        assertEquals(0.0, defenderUtils.get(StrategicObjective.LONG_RANGE_SNIPE));
        assertTrue(defenderUtils.get(StrategicObjective.SHADOW_MIDLINE) > 0.0);
    }

    @Test
    public void testShooterHardwareConstraintSuppressesScoringAndStaging() {
        WorldState shooterlessWorld = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                15,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                80.0,
                true, // active hub
                false,
                15.0,
                false
        ).withHardware(false, 30, false); // hasShooter = false

        Map<StrategicObjective, Double> scores = engine.evaluateUtilityScores(
                shooterlessWorld, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.CO_PILOT, PolicyWeights.getActive());

        assertEquals(0.0, scores.get(StrategicObjective.CYCLE_SCORE_HUB),
                "Robot without shooter must have 0 utility for CYCLE_SCORE_HUB");
        assertEquals(0.0, scores.get(StrategicObjective.STAGE_STANDOFF),
                "Robot without shooter must have 0 utility for STAGE_STANDOFF");
        assertEquals(0.0, scores.get(StrategicObjective.SHUTTLE_PASS),
                "Robot without shooter must have 0 utility for SHUTTLE_PASS");
        assertEquals(0.0, scores.get(StrategicObjective.LONG_RANGE_SNIPE),
                "Robot without shooter must have 0 utility for LONG_RANGE_SNIPE");

        AIActionIntent intent = engine.evaluatePolicy(shooterlessWorld, Archetype.CO_PILOT);
        assertNotEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective(),
                "Robot without shooter must never cycle score hub");
        assertNotEquals(StrategicObjective.STAGE_STANDOFF, intent.objective(),
                "Robot without shooter must never stage standoff");
    }

    @Test
    public void testBallCapacityHardwareConstraintControlsHarvestingAndFullness() {
        WorldState zeroCapacityWorld = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                0,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                80.0,
                false, // inactive hub
                false,
                15.0,
                false
        ).withHardware(true, 0, false); // ballCapacity = 0

        assertTrue(zeroCapacityWorld.isInventoryFull(),
                "Zero ball capacity robot is always considered inventory full");

        Map<StrategicObjective, Double> zeroCapScores = engine.evaluateUtilityScores(
                zeroCapacityWorld, Collections.emptySet(), ObservedKnowledge.selfOnly(), Archetype.AUTONOMOUS_CYCLER, PolicyWeights.getActive());

        assertEquals(0.0, zeroCapScores.get(StrategicObjective.VACUUM_MIDFIELD),
                "Zero ball capacity robot must have 0 utility for VACUUM_MIDFIELD");
        assertEquals(0.0, zeroCapScores.get(StrategicObjective.STOCKPILE_DEPOT),
                "Zero ball capacity robot must have 0 utility for STOCKPILE_DEPOT");
        assertEquals(0.0, zeroCapScores.get(StrategicObjective.SWEEP_ALLIANCE_ZONE),
                "Zero ball capacity robot must have 0 utility for SWEEP_ALLIANCE_ZONE");

        // Custom capacity robot (e.g. 10 balls): full when held >= 10
        WorldState customCapacityWorld = new WorldState(
                new Pose2d(3.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                10,
                new Pose2d(12.0, 4.0, new Rotation2d()),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(),
                80.0,
                false,
                false,
                15.0,
                false
        ).withHardware(true, 10, false);

        assertTrue(customCapacityWorld.isInventoryFull(),
                "Custom capacity 10 robot holding 10 is inventory full");
        assertEquals(1.0, customCapacityWorld.inventoryRatio(), 1e-6);

        WorldState partialCapacityWorld = customCapacityWorld.withHeldFuelCount(5);
        assertFalse(partialCapacityWorld.isInventoryFull(),
                "Custom capacity 10 robot holding 5 is not inventory full");
        assertEquals(0.5, partialCapacityWorld.inventoryRatio(), 1e-6);
    }
}

