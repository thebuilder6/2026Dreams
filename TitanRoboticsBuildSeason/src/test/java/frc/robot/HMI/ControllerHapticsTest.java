package frc.robot.HMI;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.HMI.Controller.RumblePattern;

public class ControllerHapticsTest {

    @Test
    public void testControllerRumblePatternsEnum() {
        assertNotNull(RumblePattern.NONE);
        assertNotNull(RumblePattern.TARGET_LOCKED);
        assertNotNull(RumblePattern.BALL_ACQUIRED);
        assertNotNull(RumblePattern.HARDWARE_WARNING);
        assertNotNull(RumblePattern.MATCH_TIME_WARNING);
        assertNotNull(RumblePattern.PIN_WARNING);
        assertNotNull(RumblePattern.HUB_PHASE_SHIFT);
        assertNotNull(RumblePattern.COLLISION_IMPACT);
    }
}
