package frc.robot.Sim;

import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Data.Constants;
import frc.robot.Intelligence.Archetype;
import frc.robot.Navigation.FieldMap;

/**
 * Immutable inputs for a reproducible AI-vs-AI training match.
 *
 * <p>The scenario is deliberately independent of NetworkTables and DriverStation
 * controls so simulation setup can consume it consistently. All configured robot
 * slots are applied in simulation; automated match lifecycle and result reporting
 * are separate, unfinished work.
 *
 * @param seed seed used by scenario-controlled random choices
 * @param durationSeconds total simulated match duration, in seconds
 * @param fieldFuelCount number of loose field fuel pieces to place
 * @param blueRobots Blue alliance robot configurations, in spawn order
 * @param redRobots Red alliance robot configurations, in spawn order
 */
public record TrainingMatchScenario(
        long seed,
        double durationSeconds,
        int fieldFuelCount,
        List<RobotConfig> blueRobots,
        List<RobotConfig> redRobots) {

    /** Maximum number of robots allowed on an FRC alliance in one match. */
    public static final int MAX_ROBOTS_PER_ALLIANCE = 3;

    /** Maximum field-piece count supported by the 2026 arena layout. */
    public static final int MAX_FIELD_FUEL_COUNT = 384;

    public TrainingMatchScenario {
        if (!Double.isFinite(durationSeconds) || durationSeconds <= 0.0) {
            throw new IllegalArgumentException("durationSeconds must be finite and greater than zero");
        }
        if (fieldFuelCount < 0 || fieldFuelCount > MAX_FIELD_FUEL_COUNT) {
            throw new IllegalArgumentException(
                    "fieldFuelCount must be between 0 and " + MAX_FIELD_FUEL_COUNT);
        }
        blueRobots = validatedRoster(blueRobots, "blueRobots");
        redRobots = validatedRoster(redRobots, "redRobots");
    }

    private static List<RobotConfig> validatedRoster(List<RobotConfig> roster, String name) {
        Objects.requireNonNull(roster, name + " must not be null");
        if (roster.isEmpty() || roster.size() > MAX_ROBOTS_PER_ALLIANCE) {
            throw new IllegalArgumentException(name + " must contain 1 to " + MAX_ROBOTS_PER_ALLIANCE + " robots");
        }
        return List.copyOf(roster);
    }

    /** Primary Blue robot; maps to the training-only Blue 0 AI instance. */
    public RobotConfig bluePlayerRobot() {
        return blueRobots.get(0);
    }

    /** Additional Blue robots; maps to the existing ally-bot pool after Blue 0. */
    public List<RobotConfig> blueAllyRobots() {
        return blueRobots.subList(1, blueRobots.size());
    }

    /** Red robots; maps to the existing opponent-bot pool. */
    public List<RobotConfig> redOpponentRobots() {
        return redRobots;
    }

    /** Configuration for one simulated training robot. */
    public record RobotConfig(Archetype archetype, Pose2d startingPose, int preloadFuel) {
        public RobotConfig {
            Objects.requireNonNull(archetype, "archetype must not be null");
            Objects.requireNonNull(startingPose, "startingPose must not be null");

            double x = startingPose.getX();
            double y = startingPose.getY();
            double heading = startingPose.getRotation().getRadians();
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                    || x < 0.0 || x > FieldMap.FIELD_LENGTH
                    || y < 0.0 || y > FieldMap.FIELD_WIDTH) {
                throw new IllegalArgumentException("startingPose must be finite and inside the field");
            }
            if (preloadFuel < 0 || preloadFuel > Constants.IntakeConstants.MAX_HELD_BALLS) {
                throw new IllegalArgumentException("preloadFuel is outside the robot hopper capacity");
            }
        }
    }
}
