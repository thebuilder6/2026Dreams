package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Phase 0b cover: the watchdog reports how old the currently-tracked target
 * is, so a stall can be told apart as arrived / churning / genuinely stuck.
 */
class TargetProgressWatchdogAgeTest {

    private static final double DT = 0.02;
    private static final Pose2d ROBOT = new Pose2d(0.0, 4.0, new Rotation2d());
    private static final ChassisSpeeds DRIVING = new ChassisSpeeds(1.0, 0.0, 0.0);

    private TargetProgressWatchdog watchdog;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        watchdog = new TargetProgressWatchdog();
    }

    private static Pose2d target(double x, double y) {
        return new Pose2d(new Translation2d(x, y), new Rotation2d());
    }

    @Test
    void untrackedAgeIsNegativeOne() {
        assertEquals(-1.0, watchdog.getTrackedTargetAgeSec(), 1e-9);
    }

    @Test
    void ageAccumulatesWhileTrackingOneTarget() {
        Pose2d goal = target(5.0, 4.0);
        for (int i = 0; i < 5; i++) {
            watchdog.update(ROBOT, DRIVING, goal, DT);
        }
        assertEquals(5 * DT, watchdog.getTrackedTargetAgeSec(), 1e-9);
    }

    @Test
    void ageResetsWhenTheTargetChanges() {
        for (int i = 0; i < 10; i++) {
            watchdog.update(ROBOT, DRIVING, target(5.0, 4.0), DT);
        }
        watchdog.update(ROBOT, DRIVING, target(8.0, 4.0), DT);
        assertEquals(DT, watchdog.getTrackedTargetAgeSec(), 1e-9);
    }

    /**
     * Arrived and holding station goes idle. (Arrived but still commanding
     * now keeps tracking — see `arrivedButPushingIsAbandoned` in
     * {@link TargetProgressWatchdogTest} — so this uses a hold command.)
     */
    @Test
    void ageIsNegativeOneOnceArrivedAndHolding() {
        watchdog.update(ROBOT, DRIVING, target(5.0, 4.0), DT);
        // 0.3 m away, commanding nothing: the watchdog goes idle.
        watchdog.update(ROBOT, new ChassisSpeeds(), target(0.3, 4.0), DT);
        assertEquals(-1.0, watchdog.getTrackedTargetAgeSec(), 1e-9);
    }

    @Test
    void ageKeepsTrackingWhenArrivedButPushing() {
        watchdog.update(ROBOT, DRIVING, target(5.0, 4.0), DT);
        watchdog.update(ROBOT, DRIVING, target(0.3, 4.0), DT);
        assertTrue(watchdog.getTrackedTargetAgeSec() >= 0.0,
                "arrived-but-commanding must keep the target tracked, not go idle");
    }

    @Test
    void ageIsNegativeOneWithNoTarget() {
        watchdog.update(ROBOT, DRIVING, target(5.0, 4.0), DT);
        watchdog.update(ROBOT, DRIVING, null, DT);
        assertEquals(-1.0, watchdog.getTrackedTargetAgeSec(), 1e-9);
    }
}
