package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Data.Constants;
import frc.robot.Intelligence.Archetype;
import frc.robot.Navigation.FieldMap;

class TrainingMatchScenarioTest {

    @Test
    void acceptsAndDefensivelyCopiesScenarioRosters() {
        List<TrainingMatchScenario.RobotConfig> blue = new ArrayList<>(List.of(
                new TrainingMatchScenario.RobotConfig(
                        Archetype.AUTONOMOUS_CYCLER,
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        8)));
        List<TrainingMatchScenario.RobotConfig> red = List.of(
                new TrainingMatchScenario.RobotConfig(
                        Archetype.ADAPTIVE_COMPETITOR,
                        new Pose2d(15.0, 2.0, Rotation2d.fromDegrees(180)),
                        4));

        TrainingMatchScenario scenario = new TrainingMatchScenario(42L, 150.0, 54, blue, red);
        blue.clear();

        assertEquals(42L, scenario.seed());
        assertEquals(150.0, scenario.durationSeconds());
        assertEquals(54, scenario.fieldFuelCount());
        assertEquals(1, scenario.blueRobots().size());
        assertEquals(1, scenario.redRobots().size());
        assertEquals(scenario.blueRobots().get(0), scenario.bluePlayerRobot());
        assertTrue(scenario.blueAllyRobots().isEmpty());
        assertEquals(scenario.redRobots(), scenario.redOpponentRobots());
        assertThrows(UnsupportedOperationException.class, () -> scenario.blueRobots().clear());
    }

    @Test
    void rejectsInvalidMatchDurationAndFuelCount() {
        assertThrows(IllegalArgumentException.class,
                () -> scenario(Double.NaN, 54));
        assertThrows(IllegalArgumentException.class,
                () -> scenario(150.0, -1));
        assertThrows(IllegalArgumentException.class,
                () -> scenario(150.0, TrainingMatchScenario.MAX_FIELD_FUEL_COUNT + 1));
    }

    @Test
    void rejectsEmptyOrOversizedRosters() {
        TrainingMatchScenario.RobotConfig robot = robot(Archetype.AUTONOMOUS_CYCLER, 1.0, 2.0, 0);
        List<TrainingMatchScenario.RobotConfig> threeRobots = List.of(robot, robot, robot);
        List<TrainingMatchScenario.RobotConfig> fourRobots = List.of(robot, robot, robot, robot);

        assertThrows(IllegalArgumentException.class,
                () -> new TrainingMatchScenario(0L, 150.0, 54, List.of(), List.of(robot)));
        assertThrows(IllegalArgumentException.class,
                () -> new TrainingMatchScenario(0L, 150.0, 54, fourRobots, List.of(robot)));
        assertEquals(3, new TrainingMatchScenario(0L, 150.0, 54, threeRobots, List.of(robot))
                .blueRobots().size());
    }

    @Test
    void rejectsOutOfFieldPoseAndInvalidPreload() {
        assertThrows(IllegalArgumentException.class,
                () -> robot(Archetype.AUTONOMOUS_CYCLER, FieldMap.FIELD_LENGTH + 0.1, 2.0, 0));
        assertThrows(IllegalArgumentException.class,
                () -> robot(Archetype.AUTONOMOUS_CYCLER, 1.0, 2.0,
                        Constants.IntakeConstants.MAX_HELD_BALLS + 1));
    }

    @Test
    void scenarioFuelUsesSeededSubsetsOfPreplacedLayouts() {
        List<Translation2d> strategicLayout = GameSim.getPreplacedFuelPositions(false);
        List<Translation2d> fullLayout = GameSim.getPreplacedFuelPositions(true);
        List<Translation2d> first = GameSim.selectScenarioFuelPositions(20, new Random(42L));
        List<Translation2d> repeated = GameSim.selectScenarioFuelPositions(20, new Random(42L));
        List<Translation2d> fullMatchSubset = GameSim.selectScenarioFuelPositions(100, new Random(42L));

        assertEquals(54, strategicLayout.size());
        assertEquals(408, fullLayout.size());
        assertEquals(first, repeated);
        assertEquals(20, first.size());
        assertTrue(strategicLayout.containsAll(first));
        assertTrue(fullLayout.containsAll(fullMatchSubset));
        assertEquals(new HashSet<>(strategicLayout),
                new HashSet<>(GameSim.selectScenarioFuelPositions(54, new Random(7L))));
    }

    private static TrainingMatchScenario scenario(double durationSeconds, int fieldFuelCount) {
        List<TrainingMatchScenario.RobotConfig> roster = List.of(robot(
                Archetype.AUTONOMOUS_CYCLER, 1.0, 2.0, 0));
        return new TrainingMatchScenario(0L, durationSeconds, fieldFuelCount, roster, roster);
    }

    private static TrainingMatchScenario.RobotConfig robot(
            Archetype archetype, double x, double y, int preloadFuel) {
        return new TrainingMatchScenario.RobotConfig(
                archetype, new Pose2d(x, y, new Rotation2d()), preloadFuel);
    }
}
