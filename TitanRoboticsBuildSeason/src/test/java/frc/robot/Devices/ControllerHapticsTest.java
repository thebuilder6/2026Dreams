package frc.robot.Devices;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.Devices.Controller.RumblePattern;

public class ControllerHapticsTest {

    @Test
    public void testControllerRumblePatternsEnum() {
        assertNotNull(RumblePattern.NONE);
        assertNotNull(RumblePattern.TARGET_LOCKED);
        assertNotNull(RumblePattern.BALL_ACQUIRED);
        assertNotNull(RumblePattern.HARDWARE_WARNING);
        assertNotNull(RumblePattern.MATCH_TIME_WARNING);
    }
}
