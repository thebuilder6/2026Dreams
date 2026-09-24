package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LegalPinningWatchdogTest {

    private LegalPinningWatchdog watchdog;

    @BeforeEach
    public void setup() {
        watchdog = new LegalPinningWatchdog();
        watchdog.reset();
    }

    @Test
    public void testPinWarningAtThreshold() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        // Under 1.8s: no warning
        for (int i = 0; i < 50; i++) { // 1.0s
            watchdog.update(true, robotPose, oppPose, 0.02);
        }
        assertFalse(watchdog.isWarningActive());
        assertFalse(watchdog.isForcedBackoffActive());

        // Reach 1.8s: warning becomes active
        for (int i = 0; i < 41; i++) { // +0.82s = 1.82s
            watchdog.update(true, robotPose, oppPose, 0.02);
        }
        assertTrue(watchdog.isWarningActive(), "Driver warning should trigger at 1.8s contact");
        assertFalse(watchdog.isForcedBackoffActive());
    }

    @Test
    public void testForcedBackoffAtMaxPinDuration() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        // Reach 2.4s of continuous contact
        for (int i = 0; i < 125; i++) { // 2.5s
            watchdog.update(true, robotPose, oppPose, 0.02);
        }

        assertTrue(watchdog.isForcedBackoffActive(), "Forced backoff must trigger at 2.4s max contact hold");
        assertFalse(watchdog.isWarningActive());

        // Calculate backoff target: must be >= 0.9144m (3 ft) away from contact
        Pose2d backoff = watchdog.getBackOffTarget(robotPose, oppPose);
        assertNotNull(backoff);
        double separation = backoff.getTranslation().getDistance(robotPose.getTranslation());
        assertTrue(separation >= LegalPinningWatchdog.BACKOFF_DISTANCE_METERS,
                "Backoff target must enforce >= 0.9144m clearance");

        // Simulate 3.0s cooldown while backed off
        Pose2d backedOffPose = new Pose2d(robotPose.getX() - 1.0, robotPose.getY(), new Rotation2d());
        for (int i = 0; i < 155; i++) { // 3.1s
            watchdog.update(false, backedOffPose, oppPose, 0.02);
        }

        assertFalse(watchdog.isForcedBackoffActive(), "Forced backoff should clear after 3.0s cooldown and 3-foot backoff");
    }

    @Test
    public void testContactDecayWhenSeparatedEarly() {
        Pose2d robotPose = new Pose2d(5.0, 4.0, new Rotation2d());
        Pose2d oppPose = new Pose2d(5.5, 4.0, new Rotation2d());

        // Contact for 1.0s
        for (int i = 0; i < 50; i++) {
            watchdog.update(true, robotPose, oppPose, 0.02);
        }
        assertEquals(1.0, watchdog.getPinDuration(), 0.05);

        // Separate for 0.5s -> pin duration decays
        for (int i = 0; i < 25; i++) {
            watchdog.update(false, robotPose, oppPose, 0.02);
        }
        assertTrue(watchdog.getPinDuration() < 0.20, "Pin duration should decay rapidly when contact breaks");
    }
}
