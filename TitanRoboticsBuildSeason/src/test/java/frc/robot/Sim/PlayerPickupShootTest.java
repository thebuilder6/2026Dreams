package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;

/**
 * Regression coverage for the player pickup-to-shoot pipeline: boots the real
 * sim subsystem loop, verifies no subsystem throws, balls are collected from
 * the field, and the shooter consumes held balls when the kicker runs.
 */
public class PlayerPickupShootTest {

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
        RefereeSim.getInstance();
        SubsystemManager.initializeSubsystems();
        GameSim.getInstance().resetGame();
    }

    private static void runLoopTicks(int ticks, StringBuilder failures) {
        for (int tick = 0; tick < ticks; tick++) {
            for (Subsystem s : SubsystemManager.getSubsystems()) {
                try {
                    s.update();
                } catch (Throwable t) {
                    failures.append("UPDATE-THROWER ").append(s.getName()).append(": ").append(t).append("\n");
                }
                try {
                    s.simulationUpdate();
                } catch (Throwable t) {
                    failures.append("SIM-THROWER ").append(s.getName()).append(": ").append(t).append("\n");
                }
            }
            try {
                swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance().simulationPeriodic();
            } catch (Throwable t) {
                failures.append("ARENA: ").append(t).append("\n");
            }
        }
    }

    @Test
    public void simLoopHasNoThrowingSubsystems() {
        StringBuilder failures = new StringBuilder();
        runLoopTicks(100, failures);
        assertEquals("", failures.toString(), "No subsystem may throw in the sim loop");
    }

    @Test
    public void playerPicksUpBallsFromField() {
        GameSim game = GameSim.getInstance();
        SwerveBase.getInstance().resetOdometry(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0)));
        Translation2d nose = SwerveBase.getInstance().getPose().getTranslation();

        var arena = swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance();
        for (int i = 0; i < 6; i++) {
            arena.addGamePiece(
                    new swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField(
                            nose.plus(new Translation2d(0.55, -0.25 + i * 0.1))));
        }

        int startHeld = game.getHeldBalls();
        Intake.getInstance().setState(Intake.IntakeState.INTAKING);

        StringBuilder failures = new StringBuilder();
        runLoopTicks(120, failures);

        assertEquals("", failures.toString(), "No subsystem may throw during pickup");
        assertTrue(game.getHeldBalls() > startHeld,
                "Intake should collect field balls: start=" + startHeld + " end=" + game.getHeldBalls());
    }

    @Test
    public void scoredShotsResolveToAllianceTotal() {
        // The hub physically captures balls before the projectile's analytic
        // hit-time, so counting must resolve disappearances (ShotTracker), not
        // rely on the hit callback. Fire solution-RPM shots at the active hub
        // and require them to show up in the alliance total.
        var arena = swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance();
        boolean clocked = false;
        if (arena instanceof swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
                arena2026) {
            // Freeze hub switching with both hubs active for a deterministic test.
            arena2026.setShouldRunClock(false);
            clocked = true;
            assertTrue(arena2026.isActive(true), "Blue hub must be active for this test");
        }

        try {
            MatchScoreTracker tracker = MatchScoreTracker.getInstance();
            Translation2d hub = frc.robot.Navigation.FieldMap.Hubs.getHubLocation2d(false);
            Translation2d stand = new Translation2d(hub.getX() - 1.75, hub.getY() + 1.75);
            Pose2d pose = new Pose2d(stand, hub.minus(stand).getAngle());
            SwerveBase.getInstance().resetOdometry(pose);

            var solution = Shooter.getInstance().calculateShootingSolution(
                    SwerveBase.getInstance().getPose());
            assertTrue(solution.possible(), "Standoff pose must yield a valid shooting solution");
            double fireRpm = solution.flywheelRPM();

            ShooterSim shooterSim = new ShooterSim();
            int startFuel = tracker.getBlueFuelCount();
            for (int i = 0; i < 140; i++) {
                if (i % 12 == 0) {
                    shooterSim.updateBallSimulation(1.0, fireRpm, fireRpm, 12.0);
                }
                try {
                    Thread.sleep(25);
                } catch (InterruptedException ignored) {
                }
                arena.simulationPeriodic();
                tracker.simulationUpdate();
            }

            assertTrue(shooterSim.getSimShotCount() > 0, "Shooter should have fired");
            assertEquals(0, ShotTracker.getPendingCount(), "Every launched ball must resolve");
            assertTrue(tracker.getBlueFuelCount() > startFuel,
                    "Balls entering the active hub must count: start=" + startFuel
                            + " end=" + tracker.getBlueFuelCount());
            assertTrue(tracker.getPlayerShotsScored() > 0, "Player attribution must increment");
        } finally {
            if (arena instanceof swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
                    arena2026 && clocked) {
                arena2026.setShouldRunClock(true);
            }
        }
    }

    @Test
    public void playerShooterConsumesBallsWhenKickerRuns() {        GameSim game = GameSim.getInstance();
        SwerveBase.getInstance().resetOdometry(new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)));
        assertTrue(game.getHeldBalls() > 0, "Precondition: robot starts holding balls");

        ShooterSim shooterSim = new ShooterSim();
        int beforeShoot = game.getHeldBalls();
        for (int i = 0; i < 6; i++) {
            shooterSim.updateBallSimulation(1.0, 4500, 4500, 12.0);
            try {
                Thread.sleep(250);
            } catch (InterruptedException ignored) {
            }
            swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance().simulationPeriodic();
        }

        assertTrue(shooterSim.getSimShotCount() > 0, "Shooter should fire while kicker runs");
        assertTrue(game.getHeldBalls() < beforeShoot, "Firing should consume held balls");
    }
}
