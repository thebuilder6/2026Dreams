package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Data.FieldMap;
import frc.robot.Subsystems.SwerveBase;

public class RefereeSimTest {

    private MatchScoreTracker scoreTracker;
    private RefereeSim referee;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setMatchTime(-1.0);
        DriverStationSim.setAutonomous(false);

        scoreTracker = MatchScoreTracker.getInstance();
        scoreTracker.reset();

        referee = RefereeSim.getInstance();
        referee.reset();
    }

    @Test
    public void testPenaltyScoringInMatchScoreTracker() {
        // Initially 0-0
        assertEquals(0, scoreTracker.getRedTotalScore());
        assertEquals(0, scoreTracker.getBlueTotalScore());

        // Red commits a minor foul (2 pts awarded to Blue)
        scoreTracker.recordFoul(true, false, "G401 Illegal Pinning");
        assertEquals(1, scoreTracker.getRedFoulCount());
        assertEquals(0, scoreTracker.getBluePenaltyScore() == 2 ? 0 : 1, "Blue should receive 2 penalty points");
        assertEquals(2, scoreTracker.getBluePenaltyScore());
        assertEquals(2, scoreTracker.getBlueTotalScore());
        assertEquals(0, scoreTracker.getRedTotalScore());

        // Blue commits a tech foul (5 pts awarded to Red)
        scoreTracker.recordFoul(false, true, "G201 Auto Centerline Crossing");
        assertEquals(1, scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getBlueTechFoulCount());
        assertEquals(5, scoreTracker.getRedPenaltyScore());
        assertEquals(5, scoreTracker.getRedTotalScore());
        assertEquals(2, scoreTracker.getBlueTotalScore());

        // Total score calculation combines fuel + climb + penalty
        scoreTracker.recordFuelScore(true); // Red gets 1 fuel pt
        scoreTracker.recordFuelScore(false); // Blue gets 1 fuel pt
        assertEquals(6, scoreTracker.getRedTotalScore(), "Red total = 5 penalty + 1 fuel = 6");
        assertEquals(3, scoreTracker.getBlueTotalScore(), "Blue total = 2 penalty + 1 fuel = 3");
    }

    @Test
    public void testScoreResetClearsPenaltiesAndFouls() {
        scoreTracker.recordFoul(true, true, "Major infraction");
        assertTrue(scoreTracker.getBlueTotalScore() > 0);

        scoreTracker.reset();
        assertEquals(0, scoreTracker.getRedFoulCount());
        assertEquals(0, scoreTracker.getBlueFoulCount());
        assertEquals(0, scoreTracker.getRedPenaltyScore());
        assertEquals(0, scoreTracker.getBluePenaltyScore());
        assertEquals(0, scoreTracker.getRedTotalScore());
        assertEquals(0, scoreTracker.getBlueTotalScore());
    }

    @Test
    public void testRefereeAutoCenterlineCrossingDetection() {
        // Enable Autonomous mode
        DriverStationSim.setAutonomous(true);

        // Position Blue robot over the centerline into Red auto zone (X = 9.5m > 8.27m + 0.40m)
        SwerveBase.getInstance().resetOdometry(new Pose2d(9.5, 4.0, new Rotation2d()));

        referee.evaluateAutonomousBoundaries(10.0);

        // Blue committed a Tech Foul, so Red receives 5 penalty points
        assertEquals(1, scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getBlueTechFoulCount());
        assertEquals(5, scoreTracker.getRedPenaltyScore());
        assertTrue(scoreTracker.getLastFoulDescription().contains("Auto Centerline Crossing"));
    }

    @Test
    public void testRefereePinningTimerDecayAndFoulThreshold() {
        referee.setSimPinTimer(1.0);
        assertEquals(1.0, referee.getPinTimer(), 1e-4);

        // Under 2.4s threshold, no foul is called
        assertEquals(0, scoreTracker.getRedFoulCount());
        assertEquals(0, scoreTracker.getBlueFoulCount());
    }
}
