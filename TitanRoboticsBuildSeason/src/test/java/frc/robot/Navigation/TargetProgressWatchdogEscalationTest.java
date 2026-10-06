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
 * Loop-breaker cover: escape-without-blacklist re-drives into the same trap
 * (30-row set `nav-joint30`: max-consecutive 10-14 recoveries per 20 s,
 * worst stall 143.8 s), so the third consecutive churn escape must escalate
 * to blacklisting. Real progress or a stable give-up restarts the count.
 *
 * <p>Note the loop mechanics these tests pin: {@code pinnedSec} deliberately
 * survives an escape (it must, to catch churn across target changes), so after
 * a failed escape the next fire comes on the first target change rather than
 * after a full new window. A passing-through assertion on "fires at window
 * end every cycle" would encode the opposite of the production loop.
 */
class TargetProgressWatchdogEscalationTest {

    private static final double DT = 0.02;
    private static final Pose2d ROBOT = new Pose2d(5.0, 4.0, new Rotation2d());
    private static final ChassisSpeeds DRIVE = new ChassisSpeeds(1.5, 0.0, 0.0);
    private static final ChassisSpeeds PINNED = new ChassisSpeeds(0.05, 0.0, 0.0);

    private TargetProgressWatchdog watchdog;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        watchdog = new TargetProgressWatchdog();
    }

    private static Pose2d target(double x, double y) {
        return new Pose2d(new Translation2d(x, y), new Rotation2d());
    }

    /**
     * Drives one churn fire: guarantees a target change, then pins until the
     * watchdog fires (a fresh stall needs the full window; a still-pinned one
     * refires on the change). Runs out the escape with a failed-escape model
     * (still pinned throughout) so consecutive fires stay consecutive.
     *
     * <p>Adoption ticks can fire themselves: with a hot pinnedSec the very
     * first target change is already a rescue, so their results are captured
     * rather than discarded — otherwise the observed result would be the
     * subsequent escape-latch tick (recovering with an empty list) instead of
     * the fire that blacklisted.
     */
    private TargetProgressWatchdog.Result fireOnce(Pose2d a, Pose2d b) {
        TargetProgressWatchdog.Result r = watchdog.update(ROBOT, DRIVE, PINNED, a, DT);
        if (!r.recovering()) {
            r = watchdog.update(ROBOT, DRIVE, PINNED, b, DT);
        }
        for (int i = 0; i < 400 && !r.recovering(); i++) {
            r = watchdog.update(ROBOT, DRIVE, PINNED, b, DT);
        }
        assertTrue(r.recovering(), "a pinned churn cycle must fire");
        for (int i = 0; i < (int) (TargetProgressWatchdog.ESCAPE_SEC / DT) + 5; i++) {
            watchdog.update(ROBOT, DRIVE, PINNED, b, DT);
        }
        return r;
    }

    @Test
    void thirdConsecutiveChurnEscapeBlacklists() {
        TargetProgressWatchdog.Result first = fireOnce(target(8.0, 4.0), target(8.0, 6.0));
        assertTrue(first.newlyBlacklisted().isEmpty(), "early escapes must not starve the selector");
        assertEquals(1, watchdog.getConsecutiveStaticEscapes());

        TargetProgressWatchdog.Result second = fireOnce(target(3.0, 4.0), target(3.0, 6.0));
        assertTrue(second.newlyBlacklisted().isEmpty());
        assertEquals(2, watchdog.getConsecutiveStaticEscapes());

        TargetProgressWatchdog.Result third = fireOnce(target(8.0, 2.0), target(6.0, 2.0));
        assertEquals(1, third.newlyBlacklisted().size(),
                "the third consecutive escape must escalate to blacklisting");
        assertEquals(1, watchdog.blockedPoints().size());
        assertEquals(0, watchdog.getConsecutiveStaticEscapes(),
                "escalation restarts the count");
    }

    @Test
    void realProgressRestartsTheCount() {
        fireOnce(target(8.0, 4.0), target(8.0, 6.0));
        assertEquals(1, watchdog.getConsecutiveStaticEscapes());

        // Successful escape: actually move (drains pinnedSec) and close on a
        // stable target (resets the escalation count via the progress branch).
        Pose2d goal = target(8.0, 6.0);
        for (int i = 0; i < 7; i++) {
            Pose2d pose = new Pose2d(
                    ROBOT.getX() + 0.05 * (i + 1), ROBOT.getY(), new Rotation2d());
            watchdog.update(pose, DRIVE, DRIVE, goal, DT);
        }
        assertEquals(0, watchdog.getConsecutiveStaticEscapes(),
                "real progress must restart the escalation count");

        // Two more churn fires after the restart still must not blacklist.
        fireOnce(target(3.0, 4.0), target(3.0, 6.0));
        TargetProgressWatchdog.Result r = fireOnce(target(8.0, 2.0), target(6.0, 2.0));
        assertTrue(r.newlyBlacklisted().isEmpty());
        assertEquals(0, watchdog.blockedPoints().size());
    }
}
