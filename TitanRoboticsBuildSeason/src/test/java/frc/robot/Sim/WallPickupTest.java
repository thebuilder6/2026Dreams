package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;

/**
 * Fuel tight to the perimeter walls must be targetable and collectable.
 * Regression: the fuel-targeting filters treated the whole 0.45 m wall safety
 * band as an obstacle, so balls near walls were never hunted.
 */
public class WallPickupTest {

    private static final Translation2d BLUE_WALL_BALL = new Translation2d(0.20, 4.03);
    private static final Translation2d RED_WALL_BALL = new Translation2d(16.33, 4.03);
    private static final Translation2d BOTTOM_WALL_BALL = new Translation2d(8.27, 0.20);
    private static final Translation2d TOP_WALL_BALL = new Translation2d(8.27, 7.85);

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();

        SwerveBase.getInstance();
        Shooter.getInstance();
        Intake.getInstance();
        Dashboard.getInstance();
        GameSim.getInstance();
        AIRobotSim.getInstance();
        MatchScoreTracker.getInstance();
        SubsystemManager.initializeSubsystems();
        GameSim.getInstance().resetGame();
        SimulatedArena.getInstance().clearGamePieces();
    }

    private static void spawnBall(Translation2d pos) {
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(pos));
    }

    private static void assertTargetsBall(Pose2d hunter, boolean hunterIsRed, Translation2d ball,
            Translation2d decoy) {
        spawnBall(ball);
        spawnBall(decoy);
        Pose2d target = JevDecisionEngine.getInstance()
                .findClusterWeightedFuelTarget(hunter, hunterIsRed);
        double distToBall = target.getTranslation().getDistance(ball);
        assertTrue(distToBall < 1.2,
                "Hunter at " + hunter.getTranslation() + " must target wall ball at " + ball
                        + ", got approach " + target.getTranslation());
        SimulatedArena.getInstance().clearGamePieces();
    }

    @Test
    public void blueWallBallIsTargeted() {
        assertTargetsBall(new Pose2d(1.70, 4.03, Rotation2d.fromDegrees(180)), false,
                BLUE_WALL_BALL, new Translation2d(9.50, 1.80));
    }

    @Test
    public void redWallBallIsTargeted() {
        assertTargetsBall(new Pose2d(14.80, 4.03, Rotation2d.fromDegrees(0)), true,
                RED_WALL_BALL, new Translation2d(7.00, 6.80));
    }

    @Test
    public void bottomWallBallIsTargeted() {
        assertTargetsBall(new Pose2d(8.27, 1.70, Rotation2d.fromDegrees(-90)), false,
                BOTTOM_WALL_BALL, new Translation2d(9.50, 6.80));
    }

    @Test
    public void topWallBallIsTargeted() {
        assertTargetsBall(new Pose2d(8.27, 6.35, Rotation2d.fromDegrees(90)), false,
                TOP_WALL_BALL, new Translation2d(12.50, 1.50));
    }

    @Test
    public void bot0CollectsWallBallsAtStandoff() {
        spawnBall(BLUE_WALL_BALL);
        spawnBall(RED_WALL_BALL);
        spawnBall(BOTTOM_WALL_BALL);
        spawnBall(TOP_WALL_BALL);

        AIRobotSim aiSim = AIRobotSim.getInstance();
        aiSim.setFuelCount(0);
        aiSim.getIntakeSimulation().startIntake();

        aiSim.checkProximityPickup(new Pose2d(0.68, 4.03, Rotation2d.fromDegrees(180)));
        aiSim.checkProximityPickup(new Pose2d(15.85, 4.03, Rotation2d.fromDegrees(0)));
        aiSim.checkProximityPickup(new Pose2d(8.27, 0.68, Rotation2d.fromDegrees(-90)));
        aiSim.checkProximityPickup(new Pose2d(8.27, 7.37, Rotation2d.fromDegrees(90)));

        assertEquals(4, aiSim.getFuelCount(), "Bot 0 must collect all four wall balls");
        aiSim.getIntakeSimulation().stopIntake();
    }

    @Test
    public void botInstanceCollectsWallBallsAtStandoff() {
        spawnBall(BLUE_WALL_BALL);
        spawnBall(RED_WALL_BALL);
        spawnBall(BOTTOM_WALL_BALL);
        spawnBall(TOP_WALL_BALL);

        AIRobotInstance bot = new AIRobotInstance(1,
                new Pose2d(1.5, -5, new Rotation2d()), Archetype.AUTONOMOUS_CYCLER);
        bot.getIntakeSimulation().setGamePiecesCount(0);
        bot.getIntakeSimulation().startIntake();

        bot.checkProximityPickup(new Pose2d(0.68, 4.03, Rotation2d.fromDegrees(180)));
        bot.checkProximityPickup(new Pose2d(15.85, 4.03, Rotation2d.fromDegrees(0)));
        bot.checkProximityPickup(new Pose2d(8.27, 0.68, Rotation2d.fromDegrees(-90)));
        bot.checkProximityPickup(new Pose2d(8.27, 7.37, Rotation2d.fromDegrees(90)));

        assertEquals(4, bot.getFuelCount(), "Bot instance must collect all four wall balls");
        bot.getIntakeSimulation().stopIntake();
        bot.reset();
    }
}
