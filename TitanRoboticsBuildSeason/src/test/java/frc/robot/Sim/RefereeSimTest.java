package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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

        // Red commits a minor foul (5 pts awarded to Blue)
        scoreTracker.recordFoul(true, false, "G418 Illegal Pinning");
        assertEquals(1, scoreTracker.getRedFoulCount());
        assertEquals(0, scoreTracker.getRedMajorFoulCount());
        assertEquals(5, scoreTracker.getBluePenaltyScore());
        assertEquals(5, scoreTracker.getBlueTotalScore());
        assertEquals(0, scoreTracker.getRedTotalScore());

        // Blue commits a major foul (15 pts awarded to Red)
        scoreTracker.recordFoul(false, true, "AUTO Centerline Contact");
        assertEquals(1, scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getBlueMajorFoulCount());
        assertEquals(15, scoreTracker.getRedPenaltyScore());
        assertEquals(15, scoreTracker.getRedTotalScore());
        assertEquals(5, scoreTracker.getBlueTotalScore());

        // Total score calculation combines fuel + climb + penalty
        scoreTracker.recordFuelScore(true); // Red gets 1 fuel pt
        scoreTracker.recordFuelScore(false); // Blue gets 1 fuel pt
        assertEquals(16, scoreTracker.getRedTotalScore(), "Red total = 15 penalty + 1 fuel = 16");
        assertEquals(6, scoreTracker.getBlueTotalScore(), "Blue total = 5 penalty + 1 fuel = 6");
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
    public void testRefereeAutoCenterlineContactIsMajor() {
        // Enable Autonomous mode
        DriverStationSim.setAutonomous(true);

        // Blue player fully across the centerline into the Red half (X = 9.5m > 8.27m + 0.50m),
        // with Red Bot 0 in contact range -> MAJOR FOUL on Blue (+15 to Red)
        SwerveBase.getInstance().resetOdometry(new Pose2d(9.5, 4.0, new Rotation2d()));
        AIRobotSim.getInstance().setRobotPose(new Pose2d(9.0, 4.0, new Rotation2d()));

        referee.evaluateAutonomousBoundaries(10.0);

        assertEquals(1, scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getBlueMajorFoulCount());
        assertEquals(15, scoreTracker.getRedPenaltyScore());
        assertTrue(scoreTracker.getLastFoulDescription().contains("Centerline Contact"));
    }

    @Test
    public void testRefereeAutoAcrossWithoutContactIsClean() {
        DriverStationSim.setAutonomous(true);

        // Blue player across the line but no opponent nearby -> no foul.
        // (Park Bot 0 deep in its own half; the singleton persists across tests.)
        SwerveBase.getInstance().resetOdometry(new Pose2d(9.5, 4.0, new Rotation2d()));
        AIRobotSim.getInstance().setRobotPose(new Pose2d(14.5, 4.0, new Rotation2d()));

        referee.evaluateAutonomousBoundaries(20.0);

        assertEquals(0, scoreTracker.getRedFoulCount());
        assertEquals(0, scoreTracker.getBlueFoulCount());
        assertEquals(0, scoreTracker.getRedPenaltyScore());
        assertEquals(0, scoreTracker.getBluePenaltyScore());
    }

    @Test
    public void testG407ShotOutsideAllianceZoneIsMajor() {
        // Blue player shooting from midfield (X = 8.0m, outside Blue zone X <= 4.60m)
        Pose2d illegalPose = new Pose2d(8.0, 4.0, new Rotation2d());
        RefereeSim.checkShotLegality(illegalPose, false, "Player");

        assertEquals(1, scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getBlueMajorFoulCount());
        assertEquals(15, scoreTracker.getRedPenaltyScore());

        // Legal shot inside the zone draws no foul
        Pose2d legalPose = new Pose2d(2.0, 4.0, new Rotation2d());
        RefereeSim.checkShotLegality(legalPose, false, "Player");
        assertEquals(1, scoreTracker.getBlueFoulCount());
    }

    @Test
    public void testG418PinEscalatesMinorThenMajor() {
        // Player and Bot 0 held in contact
        SwerveBase.getInstance().resetOdometry(new Pose2d(5.0, 4.0, new Rotation2d()));
        AIRobotSim.getInstance().setRobotPose(new Pose2d(5.5, 4.0, new Rotation2d()));

        // Hold contact past the 3 s pin limit -> first violation is MINOR (5 pts)
        for (int i = 0; i < 160; i++) {
            referee.evaluatePinningRule(100.0 + i * 0.02);
        }
        assertEquals(1, scoreTracker.getRedFoulCount() + scoreTracker.getBlueFoulCount());
        assertEquals(5, scoreTracker.getRedPenaltyScore() + scoreTracker.getBluePenaltyScore());

        // Hold uncorrected for another 3 s -> MAJOR (15 pts)
        for (int i = 0; i < 160; i++) {
            referee.evaluatePinningRule(200.0 + i * 0.02);
        }
        assertEquals(2, scoreTracker.getRedFoulCount() + scoreTracker.getBlueFoulCount());
        assertEquals(1, scoreTracker.getRedMajorFoulCount() + scoreTracker.getBlueMajorFoulCount());
        assertEquals(20, scoreTracker.getRedPenaltyScore() + scoreTracker.getBluePenaltyScore());
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
