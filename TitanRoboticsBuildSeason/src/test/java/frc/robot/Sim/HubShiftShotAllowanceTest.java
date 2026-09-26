package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Navigation.FieldMap;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import frc.robot.Intelligence.Archetype;

/**
 * Proves shot allowance follows the official 6.4 hub schedule for every
 * shooter: Bot 0, AI instances (opponents + allies share the path), and the
 * player robot. The library arena clock is frozen with both hubs physically
 * capturable, so the schedule alone decides legality and scoring.
 */
public class HubShiftShotAllowanceTest {

    private MatchScoreTracker tracker;
    private AIRobotSim aiSim;

    private static Pose2d legalStandoff(boolean forRed) {
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(forRed);
        double dx = forRed ? 1.75 : -1.75;
        Translation2d p = new Translation2d(hub.getX() + dx, hub.getY() + 1.75);
        return new Pose2d(p, hub.minus(p).getAngle());
    }

    /** Teleop countdown seconds placing the match in the named segment. */
    private static void setMatchTime(double remainingSec) {
        GameSim.getInstance().setSimTimeRemainingSec(remainingSec);
        HubSchedule.refreshFromMatchState();
    }

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();

        SwerveBase.getInstance();
        Shooter.getInstance();
        Intake.getInstance();
        Dashboard.getInstance();
        GameSim.getInstance();
        aiSim = AIRobotSim.getInstance();
        tracker = MatchScoreTracker.getInstance();
        RefereeSim.getInstance();
        SubsystemManager.initializeSubsystems();
        GameSim.getInstance().resetGame();
        tracker.reset();
        ShotTracker.clear();
        HubSchedule.setClockForTests(null);

        // Park the player far away so shooting lanes stay clear.
        SwerveBase.getInstance().resetOdometry(new Pose2d(14.5, 1.5, new Rotation2d(0)));

        // Deterministic start: seed 'R', SHIFT 1 window (blue active).
        HubSchedule.setShiftSeed('R');
        setMatchTime(120.0);
        assertTrue(HubSchedule.isHubActiveNow(false));
        assertFalse(HubSchedule.isHubActiveNow(true));
    }

    @Test
    public void bot0FiresOnlyOnActiveHub() {
        Pose2d bluePose = legalStandoff(false);
        Pose2d redPose = legalStandoff(true);
        aiSim.setFuelCount(8);

        // SHIFT 1, seed R: blue active, red inactive.
        assertTrue(aiSim.canShootNow(bluePose, false), "Bot 0 must shoot its active hub");
        assertFalse(aiSim.canShootNow(redPose, true), "Bot 0 must hold fire on inactive hub");

        // SHIFT 2: order flips.
        setMatchTime(90.0);
        assertFalse(aiSim.canShootNow(bluePose, false), "Shift must revoke blue shooting");
        assertTrue(aiSim.canShootNow(redPose, true), "Shift must allow red shooting");

        // END GAME: both active.
        setMatchTime(20.0);
        assertTrue(aiSim.canShootNow(bluePose, false), "Endgame must allow blue shooting");
        assertTrue(aiSim.canShootNow(redPose, true), "Endgame must allow red shooting");
    }

    @Test
    public void aiInstanceFiresOnlyOnActiveHub() {
        AIRobotInstance bot = new AIRobotInstance(1,
                new Pose2d(1.5, -5, new Rotation2d(0)), Archetype.AUTONOMOUS_CYCLER);
        bot.getIntakeSimulation().setGamePiecesCount(8);
        SwerveBase.getInstance().resetOdometry(new Pose2d(14.5, 1.5, new Rotation2d(0)));

        Pose2d bluePose = legalStandoff(false);
        Pose2d redPose = legalStandoff(true);

        assertTrue(bot.canShootNow(bluePose, false), "Instance must shoot its active hub");
        assertFalse(bot.canShootNow(redPose, true), "Instance must hold fire on inactive hub");

        setMatchTime(90.0);
        assertFalse(bot.canShootNow(bluePose, false), "Shift must revoke blue shooting");
        assertTrue(bot.canShootNow(redPose, true), "Shift must allow red shooting");

        bot.reset();
    }

    @Test
    public void playerScoresActiveHubAndWastesInactiveHub() {
        Translation2d hub = FieldMap.Hubs.getHubLocation2d(false);
        Translation2d stand = new Translation2d(hub.getX() - 1.75, hub.getY() + 1.75);
        Pose2d pose = new Pose2d(stand, hub.minus(stand).getAngle());
        SwerveBase.getInstance().resetOdometry(pose);
        var solution = Shooter.getInstance().calculateShootingSolution(
                SwerveBase.getInstance().getPose());
        assertTrue(solution.possible(), "Standoff must yield a valid solution");
        double fireRpm = solution.flywheelRPM();
        ShooterSim shooterSim = new ShooterSim();
        SimulatedArena arena = SimulatedArena.getInstance();

        // Phase 1: SHIFT 1, blue hub active -> balls score.
        setMatchTime(120.0);
        int fuelBefore = tracker.getBlueFuelCount();
        fireBalls(shooterSim, arena, fireRpm, 5);
        assertTrue(tracker.getBlueFuelCount() > fuelBefore,
                "Active-hub shots must score");
        assertEquals(0, tracker.getBlueWastedFuelCount(), "No waste while hub is active");
        assertEquals(0, ShotTracker.getPendingCount(), "Every ball must resolve");

        // Phase 2: SHIFT 2 flips blue inactive -> same shots waste, never score.
        // NOTE: rule 6.5 grants a 3 s processing grace after deactivation, so
        // the (test-induced) deactivation edge must age past the grace window
        // before firing; otherwise the balls legitimately still score.
        GameSim.getInstance().resetGame(); // refill held balls
        tracker.reset();
        ShotTracker.clear();
        double[] clock = {edu.wpi.first.wpilibj.Timer.getFPGATimestamp()};
        HubSchedule.setClockForTests(() -> clock[0]);
        try {
            setMatchTime(90.0);
            assertFalse(HubSchedule.isHubActiveNow(false));
            clock[0] += 3.5; // let the deactivation grace expire
            int scoredBefore = tracker.getBlueFuelCount();
            int wastedBefore = tracker.getBlueWastedFuelCount();
            fireBalls(shooterSim, arena, fireRpm, 5);
            assertEquals(scoredBefore, tracker.getBlueFuelCount(),
                    "Inactive-hub shots must never score");
            assertTrue(tracker.getBlueWastedFuelCount() > wastedBefore,
                    "Inactive-hub captures must resolve as wasted");
            assertEquals(0, ShotTracker.getPendingCount(), "Every ball must resolve");
        } finally {
            HubSchedule.setClockForTests(null);
        }
    }

    private static void fireBalls(ShooterSim shooterSim, SimulatedArena arena, double rpm, int volleys) {
        MatchScoreTracker tracker = MatchScoreTracker.getInstance();
        for (int i = 0; i < volleys * 12; i++) {
            if (i % 12 == 0) {
                shooterSim.updateBallSimulation(1.0, rpm, rpm, 12.0);
            }
            try {
                Thread.sleep(25);
            } catch (InterruptedException ignored) {
            }
            arena.simulationPeriodic();
            tracker.simulationUpdate();
        }
        // Drain: let in-flight balls finish so every launch resolves.
        for (int i = 0; i < 120 && ShotTracker.getPendingCount() > 0; i++) {
            try {
                Thread.sleep(25);
            } catch (InterruptedException ignored) {
            }
            arena.simulationPeriodic();
            tracker.simulationUpdate();
        }
    }
}
