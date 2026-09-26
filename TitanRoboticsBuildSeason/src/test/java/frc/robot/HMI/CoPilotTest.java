package frc.robot.HMI;

import frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.SwerveBase;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.Actions.DriveToPoseAction;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.State.WorldState;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Teleop;
import frc.robot.Utils.AllianceFlipUtil;

public class CoPilotTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        CoPilot.getInstance().resetBallCount();
        CoPilot.getInstance().stopAssist();
    }

    @Test
    public void testTeleopInputShapingCurve() {
        assertEquals(0.0, Teleop.shapeInput(0.0), 1e-4);
        assertEquals(1.0, Teleop.shapeInput(1.0), 1e-4);
        assertEquals(-1.0, Teleop.shapeInput(-1.0), 1e-4);

        // Cubic curve provides precision around center
        double shapedHalf = Teleop.shapeInput(0.5);
        assertTrue(shapedHalf < 0.5, "Shaped half-throttle should be lower than linear 0.5 for fine precision");
        assertTrue(shapedHalf > 0.0, "Shaped output must remain positive for positive input");
    }

    @Test
    public void testDriveToPoseSharedAuthorityDeadband() {
        Pose2d target = new Pose2d(5.0, 4.0, new Rotation2d());
        DriveToPoseAction action = new DriveToPoseAction(target);
        action.start();

        // Very small driver stick input (< 0.10)
        action.setDriverInput(0.1, 0.1, 0.0);
        action.update();

        assertFalse(action.isBreakoutRequested(), "Small input should not trigger breakout");
        assertFalse(action.isFinished(), "Action should not finish immediately from small stick input");
    }

    @Test
    public void testDriveToPoseSharedAuthorityBreakout() {
        Pose2d target = new Pose2d(5.0, 4.0, new Rotation2d());
        DriveToPoseAction action = new DriveToPoseAction(target);
        action.start();

        // Hard driver stick deflection (> 0.65 * MAX_SPEED)
        double hardThrottle = Constants.MAX_SPEED * 0.85;
        action.setDriverInput(hardThrottle, 0.0, 0.0);
        action.update();

        assertTrue(action.isBreakoutRequested(), "Hard driver input must trigger breakout");
        assertTrue(action.isFinished(), "DriveToPoseAction must finish immediately upon breakout");
    }

    @Test
    public void testDriveToPoseSharedAuthorityRotationBreakout() {
        Pose2d target = new Pose2d(5.0, 4.0, new Rotation2d());
        DriveToPoseAction action = new DriveToPoseAction(target);
        action.start();

        // Hard rotation deflection (> 0.60 * MAX_ROTATION_SPEED)
        double hardRotation = Constants.MAX_ROTATION_SPEED * 0.75;
        action.setDriverInput(0.0, 0.0, hardRotation);
        action.update();

        assertTrue(action.isBreakoutRequested(), "Hard rotation input must trigger breakout");
        assertTrue(action.isFinished(), "DriveToPoseAction must finish immediately upon rotation breakout");
    }

    @Test
    public void testCoPilotEndgameParkingSelection() {
        // Create an endgame WorldState with 15s remaining
        WorldState endgameWorld = new WorldState(
                new Pose2d(5.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                5,
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                15.0, // 15s remaining <= 20s
                true,
                false,
                10.0,
                false // Blue alliance
        );

        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(endgameWorld, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.RUSH_CLIMB, intent.objective(), 
                "Endgame with 15s remaining must prioritize RUSH_CLIMB / Alliance Parking");
        
        // Target pose must be near Blue Parking (X ~ 1.05m, Y ~ 2.88m)
        Pose2d target = intent.navigationTarget();
        assertNotNull(target);
        assertTrue(target.getX() < 3.0, "Blue parking target must be near alliance driver wall (X < 3.0m)");
        assertTrue(intent.rationale().contains("Alliance Parking"), "Rationale should indicate alliance parking");
    }

    @Test
    public void testCoPilotEndgameParkingRedAlliance() {
        // Red alliance endgame WorldState
        WorldState redEndgameWorld = new WorldState(
                new Pose2d(12.0, 4.0, Rotation2d.fromDegrees(180)),
                new ChassisSpeeds(),
                4,
                new Pose2d(6.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                12.0, // 12s remaining <= 20s
                true,
                false,
                10.0,
                true // Red alliance
        );

        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(redEndgameWorld, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.RUSH_CLIMB, intent.objective());

        Pose2d target = intent.navigationTarget();
        assertNotNull(target);
        // Red wall is at X ~ 16.54m, so Red parking must be at X > 13.5m
        assertTrue(target.getX() > 13.5, "Red parking target must be near Red alliance driver wall (X > 13.5m)");
    }

    @Test
    public void testCoPilotLifecycle() {
        CoPilot agent = CoPilot.getInstance();
        assertNotNull(agent);

        // Start assist
        agent.startSmartAssist();
        assertTrue(agent.isAssistActive(), "Assist must be active after startSmartAssist()");

        // Normal gentle update
        boolean running = agent.updateSmartAssist(0.5, 0.0, 0.0);
        assertTrue(running, "Gentle assist update should continue running");

        // Hard breakout command
        double breakoutSpeed = Constants.MAX_SPEED * 0.90;
        boolean stillRunning = agent.updateSmartAssist(breakoutSpeed, 0.0, 0.0);
        assertFalse(stillRunning, "Breakout stick deflection must return false from updateSmartAssist()");
        assertFalse(agent.isAssistActive(), "Assist must be deactivated after breakout");
        assertTrue(agent.checkAndClearBreakout(), "Breakout flag must be set and cleared");
        assertFalse(agent.checkAndClearBreakout(), "Breakout flag must now be false after clearing");
    }

    @Test
    public void testSmartGlideTargetEndgameRouting() {
        // Verify getSmartGlideTarget directs to parking when matchTime <= 20s
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime(18.0);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();

        Pose2d blueGlide = JevDecisionEngine.getInstance().getSmartGlideTarget(
                new Pose2d(5.0, 4.0, new Rotation2d()),
                true,
                true,
                false // Blue
        );

        assertTrue(blueGlide.getX() < 3.0, "Smart Glide in endgame must route to Blue park zone (X < 3.0m)");

        Pose2d redGlide = JevDecisionEngine.getInstance().getSmartGlideTarget(
                new Pose2d(11.0, 4.0, new Rotation2d()),
                true,
                true,
                true // Red
        );

        assertTrue(redGlide.getX() > 13.5, "Smart Glide in endgame must route to Red park zone (X > 13.5m)");
    }

    @Test
    public void testCoPilotScoringWithSingleFuel() {
        // Co-Pilot should initiate hub scoring as soon as it has even 1 fuel piece
        WorldState activeHubWithFuel = new WorldState(
                new Pose2d(5.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                1, // 1 fuel piece
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                90.0, // Mid-match
                true, // Hub Active
                false,
                15.0,
                false // Blue
        );

        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(activeHubWithFuel, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, intent.objective(),
                "Co-Pilot must select CYCLE_SCORE_HUB when Hub is active and robot has fuel (even 1 piece)");
    }

    @Test
    public void testCoPilotEndgameOverridesScoringEvenWithFullHopper() {
        // Even if robot has a full hopper of 14 balls, in endgame (<= 15s) climb must strictly dominate
        WorldState endgameFullHopper = new WorldState(
                new Pose2d(5.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                14, // Full hopper
                new Pose2d(10.0, 4.0, new Rotation2d()),
                new ChassisSpeeds(),
                12.0, // 12s remaining <= 15s
                true, // Hub Active
                false,
                10.0,
                false
        );

        AIActionIntent intent = JevDecisionEngine.getInstance().evaluatePolicy(endgameFullHopper, Archetype.CO_PILOT);
        assertEquals(StrategicObjective.RUSH_CLIMB, intent.objective(),
                "In final 15s, RUSH_CLIMB must override CYCLE_SCORE_HUB even when holding full 14 fuel pieces");
    }

    @Test
    public void testDriveToPoseActionDynamicPathfindingAroundHub() {
        // Robot starts west of Blue Hub, target is east of Blue Hub
        Pose2d startPose = new Pose2d(2.0, 4.035, Rotation2d.fromDegrees(0));
        Pose2d targetPose = new Pose2d(6.0, 4.035, Rotation2d.fromDegrees(0));
        frc.robot.Subsystems.SwerveBase.getInstance().resetOdometry(startPose);

        DriveToPoseAction action = new DriveToPoseAction(targetPose);
        assertFalse(action.isTunnelTransit(), "Path across midfield should not be flagged as a tunnel transit");

        action.start();
        action.update(); // Triggers TrajectoryController.calculate() which calls StaticPathfinder

        var waypoints = action.getWaypoints();
        assertNotNull(waypoints);
        assertTrue(waypoints.size() > 1, "Pathfinder must generate multiple detour waypoints around the Hub");

        // Verify that intermediate waypoints route above or below the Hub (Hub Y is 4.035m, width is ~1.4m)
        boolean routedAroundHub = false;
        for (Pose2d wp : waypoints) {
            double dy = Math.abs(wp.getY() - 4.035);
            if (dy > 0.8) {
                routedAroundHub = true;
                break;
            }
        }
        assertTrue(routedAroundHub, "Pathfinder waypoints must divert laterally to bypass the Hub");
    }

    @Test
    public void testSmartAssistMidfieldFuelNavigation() {
        CoPilot agent = CoPilot.getInstance();
        agent.stopAssist();

        // Start Smart Assist with 0 balls held -> Should navigate to fuel cluster
        agent.startSmartAssist();
        agent.updateSmartAssist(0.0, 0.0, 0.0);

        assertEquals(StrategicObjective.VACUUM_MIDFIELD, agent.getActiveObjective(),
                "With 0 balls held, Smart Assist must seek fuel via VACUUM_MIDFIELD");
        assertTrue(agent.getCurrentAction() instanceof DriveToPoseAction,
                "Smart Assist must use DriveToPoseAction with StaticPathfinder to navigate across field to fuel");
        assertEquals(frc.robot.Subsystems.Intake.IntakeState.INTAKING, frc.robot.Subsystems.Intake.getInstance().getState(),
                "Intake must be active during VACUUM_MIDFIELD");

        agent.stopAssist();
    }

    @Test
    public void testSmartAssistStandoffHoldPositionWithoutRestartStutter() {
        CoPilot agent = CoPilot.getInstance();
        agent.stopAssist();

        // Give robot 1 ball so it wants to score at active hub
        agent.incrementBallCount();
        agent.startSmartAssist();
        agent.updateSmartAssist(0.0, 0.0, 0.0);

        assertEquals(StrategicObjective.CYCLE_SCORE_HUB, agent.getActiveObjective(),
                "With fuel held and active hub, Smart Assist must enter CYCLE_SCORE_HUB");
        assertNotNull(agent.getCurrentAction(), "Current action must be instantiated");

        // Simulate reaching goal
        frc.robot.Interfaces.Actions action = agent.getCurrentAction();
        assertTrue(action instanceof DriveToPoseAction);

        // Update when near target should NOT destroy the action and cause stop-start stutter
        agent.updateSmartAssist(0.0, 0.0, 0.0);
        assertNotNull(agent.getCurrentAction(), "Current action must NOT be destroyed when holding standoff");
        assertTrue(agent.isAssistActive(), "Smart Assist must remain active while driver holds button");

        agent.stopAssist();
    }
}
