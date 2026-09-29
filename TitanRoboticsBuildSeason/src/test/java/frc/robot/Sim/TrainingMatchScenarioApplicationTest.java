package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Intelligence.Archetype;
import frc.robot.Subsystems.SwerveBase;

class TrainingMatchScenarioApplicationTest {
    private AIRobotSim aiSim;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        aiSim = AIRobotSim.getInstance();
        GameSim.getInstance().resetGame(null);
    }

    @AfterEach
    void tearDown() {
        GameSim.getInstance().resetGame(null);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void resetAppliesBlueRosterAndOpponentRobotConfigs() {
        Pose2d playerPose = pose(1.0, 1.0, 0.0);
        Pose2d allyPose = pose(2.2, 2.0, 25.0);
        Pose2d redPose = pose(14.5, 1.2, 180.0);
        Pose2d redSecondPose = pose(13.4, 2.2, 180.0);
        TrainingMatchScenario scenario = new TrainingMatchScenario(
                23L,
                90.0,
                54,
                List.of(
                        robot(Archetype.CO_PILOT, playerPose, 6),
                        robot(Archetype.DEFENSE_BULLY, allyPose, 3)),
                List.of(
                        robot(Archetype.AUTONOMOUS_CYCLER, redPose, 5),
                        robot(Archetype.ADAPTIVE_COMPETITOR, redSecondPose, 7)));

        GameSim.getInstance().resetGame(scenario);
        Pose2d actualPrimaryPose = aiSim.getTrainingBluePrimaryBot().getActualPose();
        assertPoseNear(playerPose, actualPrimaryPose);
        assertTrue(SwerveBase.getInstance().getSimulationPose().getY() < 0.0);
        assertEquals(Archetype.CO_PILOT, aiSim.getTrainingBluePrimaryBot().getArchetype());
        assertEquals(6, aiSim.getTrainingBluePrimaryBot().getFuelCount());

        assertEquals(0, GameSim.getInstance().getHeldBalls());
        assertEquals(2, aiSim.getOpponents().size());
        assertEquals(1, aiSim.getAllyBots().size());
        assertPoseNear(redPose, aiSim.getOpponents().get(0).getActualPose());
        assertEquals(Archetype.AUTONOMOUS_CYCLER, aiSim.getOpponents().get(0).getArchetype());
        assertEquals(5, aiSim.getOpponents().get(0).getFuelCount());
        assertPoseNear(redSecondPose, aiSim.getAdditionalBots().get(0).getActualPose());
        assertEquals(7, aiSim.getAdditionalBots().get(0).getFuelCount());
        assertPoseNear(allyPose, aiSim.getAllyBots().get(0).getActualPose());
        assertEquals(Archetype.DEFENSE_BULLY, aiSim.getAllyBots().get(0).getArchetype());
        assertEquals(3, aiSim.getAllyBots().get(0).getFuelCount());
        assertTrue(aiSim.getAllyBots().get(0).isAlly());

        aiSim.simulationUpdate();
    }

    @Test
    void trainingBotsHoldWhileDriverStationDisabled() {
        // 2v2: matches the roster sizes other application tests assume, since
        // sim bot pools are grow-only within a shared JVM.
        TrainingMatchScenario scenario = new TrainingMatchScenario(
                11L,
                90.0,
                54,
                List.of(
                        robot(Archetype.CO_PILOT, pose(1.0, 1.0, 0.0), 6),
                        robot(Archetype.DEFENSE_BULLY, pose(2.2, 2.0, 25.0), 3)),
                List.of(
                        robot(Archetype.AUTONOMOUS_CYCLER, pose(14.5, 1.2, 180.0), 5),
                        robot(Archetype.ADAPTIVE_COMPETITOR, pose(13.4, 2.2, 180.0), 7)));
        GameSim.getInstance().resetGame(scenario);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        aiSim.simulationUpdate();
        Pose2d primaryBefore = aiSim.getTrainingBluePrimaryBot().getActualPose();
        Pose2d oppBefore = aiSim.getOpponents().get(0).getActualPose();

        for (int i = 0; i < 5; i++) {
            aiSim.simulationUpdate();
        }

        assertPoseNear(primaryBefore, aiSim.getTrainingBluePrimaryBot().getActualPose());
        assertPoseNear(oppBefore, aiSim.getOpponents().get(0).getActualPose());

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @Test
    void trainingResultLatchesWhenClockExpires() {
        TrainingMatchScenario scenario = new TrainingMatchScenario(
                13L,
                90.0,
                54,
                List.of(robot(Archetype.CO_PILOT, pose(1.0, 1.0, 0.0), 6)),
                List.of(robot(Archetype.AUTONOMOUS_CYCLER, pose(14.5, 1.2, 180.0), 5)));
        GameSim.getInstance().resetGame(scenario);
        GameSim.getInstance().setSimTimeRemainingSec(0.05);

        for (int i = 0; i < 5; i++) {
            GameSim.getInstance().simulationUpdate();
        }

        assertEquals("Tie", edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getString("Training/Result/Winner", ""));
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getNumber("Training/Result/BlueScore", -1.0));
        assertEquals(0.0, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getNumber("Training/Result/RedScore", -1.0));
    }

    @Test
    void trainingPublishesAllyFieldObjectsForAdvantageScope() {
        // 2v2: pool-size neutral for sibling tests (sim bot pools are grow-only).
        TrainingMatchScenario scenario = new TrainingMatchScenario(
                17L,
                90.0,
                54,
                List.of(
                        robot(Archetype.CO_PILOT, pose(1.0, 1.0, 0.0), 6),
                        robot(Archetype.DEFENSE_BULLY, pose(2.2, 2.0, 25.0), 3)),
                List.of(
                        robot(Archetype.AUTONOMOUS_CYCLER, pose(14.5, 1.2, 180.0), 5),
                        robot(Archetype.ADAPTIVE_COMPETITOR, pose(13.4, 2.2, 180.0), 7)));
        GameSim.getInstance().resetGame(scenario);

        aiSim.simulationUpdate();

        edu.wpi.first.wpilibj.smartdashboard.Field2d field = SwerveBase.getInstance().getField();
        assertTrue(field.getObject("AllyBot0").getPoses().size() > 0,
                "Training primary must publish AllyBot0 for advantagescope-layout.json");
        assertTrue(field.getObject("AllyBot1").getPoses().size() > 0,
                "Ally must publish AllyBot1 for advantagescope-layout.json");
        assertTrue(field.getObject("OpponentBot0").getPoses().size() > 0,
                "Bot0 must publish OpponentBot0 for advantagescope-layout.json");

        // Ally 0 dashboard fields (training primary): archetype display and
        // mark published per tick for the Elastic Simulation tab.
        assertTrue(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getString(SimDashboardKeys.ALLY0_ARCHETYPE, "").length() > 0,
                "Training primary must publish Simulation/Ally0/Archetype");
        assertTrue(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                .getString(SimDashboardKeys.ALLY0_MARK, "").length() > 0,
                "Training primary must publish Simulation/Ally0/Mark");
        assertTrue(SimDashboardKeys.allKeys().contains("Simulation/Ally0/Pose"),
                "Ally0 per-bot keys must be in the dashboard key contract");
    }

    private static TrainingMatchScenario.RobotConfig robot(Archetype archetype, Pose2d pose, int preload) {
        return new TrainingMatchScenario.RobotConfig(archetype, pose, preload);
    }

    private static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
    }

    private static void assertPoseNear(Pose2d expected, Pose2d actual) {
        assertTrue(actual.getTranslation().getDistance(expected.getTranslation()) < 0.1);
        assertTrue(Math.abs(actual.getRotation().minus(expected.getRotation()).getRadians()) < 0.2);
    }
}
