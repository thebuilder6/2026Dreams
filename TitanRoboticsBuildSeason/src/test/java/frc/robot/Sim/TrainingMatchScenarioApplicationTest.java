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
