package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;
import java.util.Set;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.Missions.MissionBase;

public class AutoMissionChooserTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
        AutoMissionChooser.setDelay(0.0);
    }

    @Test
    public void testAutoMissionChooserDefaultsAndRegistration() {
        AutoMissionChooser chooser = new AutoMissionChooser();
        assertNotNull(chooser.getRawChooser());
        assertEquals("Do Nothing", chooser.getSelected());
        assertTrue(chooser.getAutoMission().isEmpty());

        Set<String> registered = chooser.getRegisteredMissionNames();
        assertNotNull(registered);
        assertTrue(registered.contains("DepotShootMission"));
        assertTrue(registered.contains("ShooterMission"));
        assertTrue(registered.contains("ExampleMission"));
        assertTrue(registered.contains("Advanced Choreo Shot"));
        assertTrue(registered.contains("Mobility (Drive Forward)"));
        assertTrue(registered.contains("Subwoofer Shoot & Leave"));
        assertTrue(registered.contains("Fast Depot Cycle"));
        assertTrue(registered.contains("Adaptive Depot Sweep"));
        assertTrue(registered.contains("Delayed Partner Shoot"));
        assertTrue(registered.contains("Shoot & Trench Disruption"));
        assertTrue(registered.contains("Trench Midfield Disruptor"));
        assertTrue(registered.contains("Centerline Sweep & Leave"));
    }

    @Test
    public void testDelayAccessorsAndClamping() {
        AutoMissionChooser chooser = new AutoMissionChooser();

        AutoMissionChooser.setDelay(3.5);
        assertEquals(3.5, AutoMissionChooser.getDelay(), 1e-4);

        // Negative delay must clamp safely to 0.0
        AutoMissionChooser.setDelay(-2.0);
        assertEquals(0.0, AutoMissionChooser.getDelay(), 1e-4);

        // Verify updateMissionCreator pulls delay from SmartDashboard
        SmartDashboard.putNumber("Auto Delay (seconds)", 4.25);
        chooser.updateMissionCreator();
        assertEquals(4.25, AutoMissionChooser.getDelay(), 1e-4);

        AutoMissionChooser.setDelay(0.0);
    }

    @Test
    public void testMissionResolutionAndReset() {
        AutoMissionChooser chooser = new AutoMissionChooser();

        assertTrue(chooser.getAutoMissionForParams("Do Nothing").isEmpty());
        assertTrue(chooser.getAutoMissionForParams(null).isEmpty());
        assertTrue(chooser.getAutoMissionForParams("ImaginaryMission").isEmpty());

        String[] expectedMissions = {
            "ExampleMission",
            "DepotShootMission",
            "ShooterMission",
            "Advanced Choreo Shot",
            "Mobility (Drive Forward)",
            "Subwoofer Shoot & Leave",
            "Fast Depot Cycle",
            "Adaptive Depot Sweep",
            "Delayed Partner Shoot",
            "Shoot & Trench Disruption",
            "Trench Midfield Disruptor",
            "Centerline Sweep & Leave"
        };

        for (String missionName : expectedMissions) {
            Optional<MissionBase> mission = chooser.getAutoMissionForParams(missionName);
            assertTrue(mission.isPresent(), "Expected mission to resolve: " + missionName);
            assertNotNull(mission.get());
        }

        assertDoesNotThrow(chooser::reset);
        assertTrue(chooser.getAutoMission().isEmpty());
        assertDoesNotThrow(chooser::outputToSmartDashboard);
    }
}
