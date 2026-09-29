package frc.robot.Sim;

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

    // ---------------------------------------------------------------------
    // Per-slot attribution (F2)
    //
    // The player used to call recordFuelScore directly, so every player score
    // landed in the alliance total while the per-bot table showed nothing. That
    // made 13 of 20 archived headless reports fail the per-bot sum against the
    // alliance total, short by 2-10, always on Blue -- structurally one-sided
    // because the player always shoots at its own hub. These pin the fix.
    // ---------------------------------------------------------------------

    @Test
    public void playerScoreIsAttributedNotJustTotalled() {
        // 4 player scores into the Blue hub (the player's own alliance).
        for (int i = 0; i < 4; i++) {
            tracker.recordPlayerScore(false);
        }
        assertEquals(4, tracker.getPlayerShotsScored());
        assertEquals(4, tracker.getPlayerBlueFuelScored());
        assertEquals(0, tracker.getPlayerRedFuelScored());
        // Alliance total moved...
        assertEquals(4, tracker.getBlueFuelScore());
        // ...and it is fully explained by the player's own row.
        assertEquals(4, tracker.getPlayerBlueFuelScored());
        assertEquals(0, tracker.getBlueReconciliationResidual());
        assertEquals(0, tracker.getBlueUnattributedFuel());
        // No bot row was invented for it.
        assertEquals(0, tracker.getAlly0FuelScored());
        assertEquals(0, tracker.getAlly1FuelScored());
        assertEquals(0, tracker.getAlly2FuelScored());
    }

    @Test
    public void playerScoreIntoRedHubAttributesToRed() {
        tracker.recordPlayerScore(true);
        assertEquals(1, tracker.getPlayerRedFuelScored());
        assertEquals(0, tracker.getPlayerBlueFuelScored());
        assertEquals(1, tracker.getRedFuelScore());
        assertEquals(0, tracker.getRedReconciliationResidual());
        assertEquals(0, tracker.getBlueFuelScore());
        assertEquals(0, tracker.getBlueReconciliationResidual());
    }

    @Test
    public void unknownBotIdIsCountedAsUnattributedNotDropped() {
        // 7 is not a valid bot id (Red 0-2, Blue 100-102).
        tracker.recordBotScore(7, true);
        assertEquals(1, tracker.getRedUnattributedFuel());
        assertEquals(0, tracker.getBlueUnattributedFuel());
        // It still reached the alliance total...
        assertEquals(1, tracker.getRedFuelScore());
        // ...and the residual stays 0 because the canary accounts for it.
        assertEquals(0, tracker.getRedReconciliationResidual());
        // No bot row was touched.
        assertEquals(0, tracker.getBot0FuelScored());
        assertEquals(0, tracker.getBot1FuelScored());
        assertEquals(0, tracker.getBot2FuelScored());
    }

    @Test
    public void allScoringPathsReconcileAgainstAllianceTotals() {
        // A mixed sequence across all three entry points, in the id order the
        // headless 3v3 actually uses.
        tracker.recordBotScore(0, true);      // Red Bot0
        tracker.recordBotScore(1, true);      // Red Bot1
        tracker.recordBotScore(2, true);      // Red Bot2
        tracker.recordBotScore(100, false);   // Blue training primary
        tracker.recordBotScore(101, false);   // Blue Ally1
        tracker.recordBotScore(102, false);   // Blue Ally2
        tracker.recordPlayerScore(false);     // player -> Blue hub
        tracker.recordPlayerScore(false);
        tracker.recordBotScore(99, false);     // unknown -> Blue canary

        assertEquals(3, tracker.getRedFuelScore());
        // Blue: 3 bots (100/101/102) + 2 player + 1 unknown-id = 6
        assertEquals(6, tracker.getBlueFuelScore());
        assertEquals(0, tracker.getRedReconciliationResidual());
        assertEquals(0, tracker.getBlueReconciliationResidual());
        assertEquals(1, tracker.getBlueUnattributedFuel());
        assertEquals(0, tracker.getRedUnattributedFuel());
    }

    @Test
    public void resetClearsAttributionAndCanaries() {
        tracker.recordPlayerScore(false);
        tracker.recordBotScore(101, false);
        tracker.recordBotScore(7, false);
        assertNotEquals(0, tracker.getBlueUnattributedFuel());

        tracker.reset();

        assertEquals(0, tracker.getPlayerBlueFuelScored());
        assertEquals(0, tracker.getPlayerRedFuelScored());
        assertEquals(0, tracker.getRedUnattributedFuel());
        assertEquals(0, tracker.getBlueUnattributedFuel());
        assertEquals(0, tracker.getBlueReconciliationResidual());
        assertEquals(0, tracker.getRedReconciliationResidual());
        assertEquals(0, tracker.getAlly1FuelScored());
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
