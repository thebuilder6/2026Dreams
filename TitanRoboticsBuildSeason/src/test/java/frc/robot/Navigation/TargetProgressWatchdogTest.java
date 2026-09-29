package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Covers the peer-independent unreachable-target recovery that unsticks bots
 * driving alone into fuel they cannot reach (the headless 3v3 wall-band
 * freeze).
 */
class TargetProgressWatchdogTest {
    private static final double DT = 0.02;
    private static final Pose2d START = new Pose2d(5.26, 7.17, Rotation2d.fromDegrees(0));
    private static final Pose2d TARGET = new Pose2d(5.20, 6.40, Rotation2d.fromDegrees(0));

    private TargetProgressWatchdog watchdog;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        watchdog = new TargetProgressWatchdog();
    }

    private static ChassisSpeeds drive(double vx) {
        return new ChassisSpeeds(vx, 0.0, 0.0);
    }

    /** Feeds N cycles at a fixed pose (bot not moving) while commanding motion. */
    private void runStuck(int cycles, ChassisSpeeds commanded) {
        for (int i = 0; i < cycles; i++) {
            watchdog.update(START, commanded, TARGET, DT);
        }
    }

    @Test
    void givesUpAndEscapesAfterNoProgressWindow() {
        // One cycle short of the window: still working the target.
        runStuck((int) (TargetProgressWatchdog.GIVEUP_SEC / DT) - 1, drive(1.5));
        assertFalse(watchdog.isRecovering(), "must not escape before the window expires");
        assertTrue(watchdog.getNoProgressSec() < TargetProgressWatchdog.GIVEUP_SEC);

        // The next cycle exhausts the window.
        TargetProgressWatchdog.Result result = watchdog.update(START, drive(1.5), TARGET, DT);
        assertTrue(result.recovering(), "no-progress window must trigger recovery");
        assertEquals(1, result.newlyBlacklisted().size(),
                "the abandoned target must be reported for exclusion");
        assertTrue(watchdog.isRecovering());
        assertTrue(result.escapeVector().getNorm() > 0.5, "escape must be a real command");
    }

    @Test
    void escapeIsLatchedForFullDuration() {
        giveUp(watchdog, START);
        assertTrue(watchdog.isRecovering());

        // A bot that now moves must still be driven away for ESCAPE_SEC.
        for (double elapsed = 0.0; elapsed < TargetProgressWatchdog.ESCAPE_SEC; elapsed += DT) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(START, drive(1.5), TARGET, DT);
            if (elapsed < TargetProgressWatchdog.ESCAPE_SEC - 2 * DT) {
                assertTrue(r.recovering(), "escape must hold for its full duration");
            }
        }
        assertFalse(watchdog.isRecovering(), "escape must release after ESCAPE_SEC");
    }

    @Test
    void realProgressPreventsGiveUp() {
        // Robot closes on the target every cycle and stops on arrival: a
        // correctly-approaching bot must never be abandoned.
        double stepM = 0.02; // 1.0 m/s of closing
        for (int i = 0; i < 40; i++) {
            Pose2d pose = new Pose2d(
                    START.getX(),
                    Math.max(TARGET.getY(), START.getY() - i * stepM),
                    Rotation2d.fromDegrees(0));
            TargetProgressWatchdog.Result r =
                    watchdog.update(pose, drive(1.5), TARGET, DT);
            assertFalse(r.recovering(), "a closing robot must never be abandoned");
        }
        assertEquals(0, watchdog.blockedPoints().size());
    }

    @Test
    void robotDrivingPastItsTargetIsAbandoned() {
        // No new objective and closing speed maintained well past the target:
        // the bot is genuinely not converging, so the give-up must still fire.
        // (Passing the target briefly counts as arrival and resets the window,
        // so this needs more than one full window of overshoot.)
        for (int i = 0; i < 400; i++) {
            Pose2d pose = new Pose2d(START.getX(), START.getY() - i * 0.02,
                    Rotation2d.fromDegrees(0));
            watchdog.update(pose, drive(1.5), TARGET, DT);
        }
        assertTrue(watchdog.blockedPoints().size() > 0,
                "overshooting without a new objective must trigger recovery");
    }

    @Test
    void arrivingAtTargetResetsProgress() {
        // Commanded motion but already inside ARRIVED_M: intentional hold, no give-up.
        Pose2d arrived = new Pose2d(TARGET.getX() + 0.1, TARGET.getY(), Rotation2d.fromDegrees(0));
        for (int i = 0; i < 400; i++) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(arrived, drive(1.5), TARGET, DT);
            assertFalse(r.recovering());
        }
        assertEquals(0, watchdog.blockedPoints().size());
    }

    @Test
    void lowCommandedSpeedIsNotAStall() {
        // Staging / planting to shoot commands near zero: must decay, not trigger.
        for (int i = 0; i < 400; i++) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(START, drive(0.10), TARGET, DT);
            assertFalse(r.recovering(), "an intentional hold must not trigger recovery");
        }
    }

    @Test
    void newObjectiveRestartsProgressWindow() {
        runStuck((int) (TargetProgressWatchdog.GIVEUP_SEC / DT) - 3, drive(1.5));
        assertTrue(watchdog.getNoProgressSec() > 1.0);

        // A different objective well away from the tracked one resets the window.
        Pose2d newTarget = new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0));
        TargetProgressWatchdog.Result r = watchdog.update(START, drive(1.5), newTarget, DT);
        assertFalse(r.recovering());
        assertTrue(r.noProgressSec() <= DT,
                "a new objective must restart the window, got " + r.noProgressSec());
    }

    @Test
    void blacklistedFuelIsSkippedWithinRadiusAndExpires() {
        runStuck((int) (TargetProgressWatchdog.GIVEUP_SEC / DT) + 1, drive(1.5));
        assertEquals(1, watchdog.blockedPoints().size());

        Translation2d blocked = watchdog.blockedPoints().iterator().next();
        assertTrue(watchdog.isFuelBlocked(blocked));
        assertTrue(watchdog.isFuelBlocked(blocked.plus(new Translation2d(
                TargetProgressWatchdog.BLACKLIST_RADIUS_M * 0.9, 0.0))),
                "pieces within the blocked radius must be skipped");
        assertFalse(watchdog.isFuelBlocked(blocked.plus(new Translation2d(
                        TargetProgressWatchdog.BLACKLIST_RADIUS_M * 1.5, 0.0))),
                "pieces outside the blocked radius must stay eligible");

        watchdog.reset();
        assertEquals(0, watchdog.blockedPoints().size());
        assertFalse(watchdog.isFuelBlocked(blocked));
    }

    @Test
    void escapeSlidesAlongWallWhenPieceBlocksTheWay() {
        // The frozen-in-scenario: a bot pinned at y = 7.4 (0.65 m off the far
        // wall) with the unreachable piece at y = 6.4, i.e. the piece sits
        // between the bot and the wall. Backing off would re-drive it into the
        // wall, so the escape must slide along the wall toward open field.
        Pose2d wallPinned = new Pose2d(5.26, 7.40, Rotation2d.fromDegrees(0));
        TargetProgressWatchdog.Result r = giveUp(watchdog, wallPinned);

        Translation2d toCentre = new Translation2d(
                frc.robot.Navigation.FieldMap.FIELD_LENGTH / 2.0,
                frc.robot.Navigation.FieldMap.FIELD_WIDTH / 2.0)
                .minus(wallPinned.getTranslation());
        toCentre = toCentre.div(toCentre.getNorm());
        assertTrue(r.escapeVector().dot(toCentre) > 0.0,
                "escape must bias toward open field, not back into the wall");
    }

    @Test
    void escapeBacksOffThePieceInOpenField() {
        // Open field, piece directly behind: the escape must back away from it.
        TargetProgressWatchdog open = new TargetProgressWatchdog();
        Pose2d openPose = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(0));
        Pose2d pieceBehind = new Pose2d(6.0, 5.50, Rotation2d.fromDegrees(0));
        TargetProgressWatchdog.Result r = TargetProgressWatchdog.Result.IDLE;
        for (int i = 0; i <= (int) (TargetProgressWatchdog.GIVEUP_SEC / DT); i++) {
            r = open.update(openPose, drive(1.5), pieceBehind, DT);
        }
        assertTrue(r.recovering());
        Translation2d awayFromPiece = openPose.getTranslation()
                .minus(pieceBehind.getTranslation());
        awayFromPiece = awayFromPiece.div(awayFromPiece.getNorm());
        assertTrue(r.escapeVector().dot(awayFromPiece) > 0.0,
                "escape must back away from the abandoned piece");
    }

    private static TargetProgressWatchdog.Result giveUp(
            TargetProgressWatchdog w, Pose2d pose) {
        TargetProgressWatchdog.Result last = TargetProgressWatchdog.Result.IDLE;
        for (int i = 0; i <= (int) (TargetProgressWatchdog.GIVEUP_SEC / DT); i++) {
            last = w.update(pose, new ChassisSpeeds(1.5, 0.0, 0.0), TARGET, DT);
        }
        assertTrue(last.recovering());
        return last;
    }
}
