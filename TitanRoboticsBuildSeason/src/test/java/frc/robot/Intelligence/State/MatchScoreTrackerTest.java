package frc.robot.Intelligence.State;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class MatchScoreTrackerTest {

    private MatchScoreTracker tracker;

    @BeforeEach
    public void setup() {
        tracker = MatchScoreTracker.getInstance();
        tracker.reset();
    }

    @Test
    public void testInitialStateIsZero() {
        assertEquals(0, tracker.getRedTotalScore());
        assertEquals(0, tracker.getBlueTotalScore());
        assertEquals(0, tracker.getPlayerScore());
        assertEquals(0, tracker.getOpponentScore());
        assertEquals(0, tracker.getLeadMargin());
        assertTrue(tracker.getLeader().contains("TIED"));
        assertEquals(0.0, tracker.getPlayerAccuracyPercent(), 1e-4);
    }

    @Test
    public void testFuelScoringAndLeader() {
        // Red scores 5 fuel
        for (int i = 0; i < 5; i++) {
            tracker.recordFuelScore(true);
        }
        assertEquals(5, tracker.getRedFuelScore());
        assertEquals(5, tracker.getRedTotalScore());
        assertEquals(0, tracker.getBlueTotalScore());
        assertEquals(5, tracker.getLeadMargin());
        assertTrue(tracker.getLeader().contains("RED (+5)"));

        // Blue scores 8 fuel
        for (int i = 0; i < 8; i++) {
            tracker.recordFuelScore(false);
        }
        assertEquals(8, tracker.getBlueFuelScore());
        assertEquals(8, tracker.getBlueTotalScore());
        assertEquals(3, tracker.getLeadMargin());
        assertTrue(tracker.getLeader().contains("BLUE (+3)"));
    }

    @Test
    public void testPlayerShotAccuracy() {
        // Player attempts 10 shots, scores 7 (into Blue goal)
        tracker.recordPlayerShotAttempt(10);
        for (int i = 0; i < 7; i++) {
            tracker.recordPlayerScore(false);
        }
        assertEquals(10, tracker.getPlayerShotsAttempted());
        assertEquals(7, tracker.getPlayerShotsScored());
        assertEquals(70.0, tracker.getPlayerAccuracyPercent(), 0.1);
        assertEquals(7, tracker.getBlueTotalScore());
    }

    @Test
    public void testBotAttribution() {
        // Bot 0 scores 4 for Red
        for (int i = 0; i < 4; i++) {
            tracker.recordBotScore(0, true);
        }
        // Bot 1 scores 3 for Red
        for (int i = 0; i < 3; i++) {
            tracker.recordBotScore(1, true);
        }
        // Bot 2 scores 2 for Red
        for (int i = 0; i < 2; i++) {
            tracker.recordBotScore(2, true);
        }

        assertEquals(4, tracker.getBot0FuelScored());
        assertEquals(3, tracker.getBot1FuelScored());
        assertEquals(2, tracker.getBot2FuelScored());
        assertEquals(9, tracker.getRedTotalScore());
    }

    @Test
    public void testRankingPointsCalculation() {
        // Red wins by 45 to 20
        for (int i = 0; i < 45; i++) {
            tracker.recordFuelScore(true); // Red fuel >= 40 earns Fuel RP
        }
        for (int i = 0; i < 20; i++) {
            tracker.recordFuelScore(false);
        }

        assertTrue(tracker.isRedFuelRpAchieved(), "Red should achieve Fuel RP with 45 balls");
        assertFalse(tracker.isBlueFuelRpAchieved(), "Blue should not achieve Fuel RP with 20 balls");

        // Win RP (2) + Fuel RP (1) = 3 RP
        assertEquals(3, tracker.getRedRankingPoints());
        // Loss RP (0) = 0 RP
        assertEquals(0, tracker.getBlueRankingPoints());
    }

    @Test
    public void testAllyAttribution() {
        // Ally 1 scores 3 for Red, Ally 2 scores 2 for Blue
        for (int i = 0; i < 3; i++) {
            tracker.recordBotScore(101, true);
        }
        for (int i = 0; i < 2; i++) {
            tracker.recordBotScore(102, false);
        }

        assertEquals(3, tracker.getAlly1FuelScored());
        assertEquals(2, tracker.getAlly2FuelScored());
        assertEquals(3, tracker.getRedTotalScore());
        assertEquals(2, tracker.getBlueTotalScore());
    }

    @Test
    public void testMinorAndMajorFoulValues() {
        // MINOR FOUL credits 5 pts to the opponent
        tracker.recordMinorFoul(true, "G418 Pin");
        assertEquals(5, tracker.getBluePenaltyScore());
        assertEquals(5, tracker.getBlueTotalScore());

        // MAJOR FOUL credits 15 pts to the opponent
        tracker.recordMajorFoul(false, "G407 Zone Shot");
        assertEquals(15, tracker.getRedPenaltyScore());
        assertEquals(15, tracker.getRedTotalScore());
        assertEquals(1, tracker.getBlueMajorFoulCount());
    }

    @Test
    public void testAutoTeleopFuelSplit() {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        tracker.recordFuelScore(true);
        assertEquals(1, tracker.getRedAutoFuelCount());
        assertEquals(0, tracker.getRedTeleopFuelCount());

        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        tracker.recordFuelScore(true);
        tracker.recordFuelScore(false);
        assertEquals(1, tracker.getRedAutoFuelCount());
        assertEquals(1, tracker.getRedTeleopFuelCount());
        assertEquals(1, tracker.getBlueTeleopFuelCount());
        assertEquals(0, tracker.getBlueAutoFuelCount());

        // Totals still combine both periods
        assertEquals(2, tracker.getRedFuelScore());
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        DriverStationSim.setEnabled(false);
    }

    @Test
    public void testResetClearsScores() {
        tracker.recordFuelScore(true);
        tracker.recordFuelScore(false);
        tracker.recordPlayerShotAttempt(5);
        tracker.recordBotScore(0, true);

        tracker.reset();

        assertEquals(0, tracker.getRedTotalScore());
        assertEquals(0, tracker.getBlueTotalScore());
        assertEquals(0, tracker.getPlayerShotsAttempted());
        assertEquals(0, tracker.getBot0FuelScored());
    }
}
