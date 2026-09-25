package frc.robot.Intelligence;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.State.WorldState;
import frc.robot.Intelligence.State.WorldStateBuilder;
import frc.robot.Intelligence.State.MatchScoreTracker;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.Sparring.AIRobotSim;
import frc.robot.Intelligence.Sparring.AIRobotInstance;
import frc.robot.Intelligence.MatchCoach.DrillMode;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MatchCoachTest {

    @BeforeAll
    public static void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testMatchCoachInitializationAndDefaults() {
        MatchCoach coach = MatchCoach.getInstance();
        assertNotNull(coach);

        coach.resetSessionStats();
        assertEquals(0, coach.getTotalShotsAttempted());
        assertEquals(0, coach.getCompletedCyclesCount());
        assertEquals(100.0, coach.getShootingAccuracyPercent(), 1e-3);
        assertEquals("A", coach.getDriverGrade());
        assertNotNull(coach.getActiveCoachingTip());
        assertFalse(coach.getActiveCoachingTip().isEmpty());
    }

    @Test
    public void testDrillModeParsing() {
        assertEquals(DrillMode.FREE_PLAY, DrillMode.fromString("FREE_PLAY"));
        assertEquals(DrillMode.RAPID_CYCLING, DrillMode.fromString("RAPID_CYCLING"));
        assertEquals(DrillMode.TRENCH_DEFENSE, DrillMode.fromString("TRENCH_DEFENSE"));
        assertEquals(DrillMode.ANTI_DEFENSE_SHOOTING, DrillMode.fromString("ANTI_DEFENSE_SHOOTING"));
        assertEquals(DrillMode.FREE_PLAY, DrillMode.fromString("NON_EXISTENT_MODE"));
    }

    @Test
    public void testMatchCoachUpdateAndLog() {
        MatchCoach coach = MatchCoach.getInstance();
        assertDoesNotThrow(coach::update);
        assertDoesNotThrow(coach::log);

        assertTrue(SmartDashboard.containsKey("Coaching/Recommendation"));
        assertTrue(SmartDashboard.containsKey("Coaching/DriverGrade"));
        assertTrue(SmartDashboard.containsKey("Coaching/ShootingAccuracyPercent"));
        assertTrue(SmartDashboard.containsKey("Coaching/DrillMode"));
    }
}
