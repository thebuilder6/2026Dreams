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

    /**
     * Reached AND holding (commanded ~0, e.g. staging or planting to shoot)
     * stays idle forever. This is the half of the old
     * `arrivingAtTargetResetsProgress` that remains true.
     */
    @Test
    void arrivedHoldStaysIdle() {
        Pose2d arrived = new Pose2d(TARGET.getX() + 0.1, TARGET.getY(), Rotation2d.fromDegrees(0));
        for (int i = 0; i < 400; i++) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(arrived, drive(0.0), TARGET, DT);
            assertFalse(r.recovering());
        }
        assertEquals(0, watchdog.blockedPoints().size());
    }

    /**
     * The churn case: the target keeps changing while the robot is physically
     * pinned, and the give-up timer must still fire.
     *
     * <p>This is the hole the whole class was written for. The watchdog tracked
     * <i>commanded</i> speed only, so "no progress" meant "the commanded target
     * got no closer" -- and a selector that re-targets every few ticks resets
     * that window on every change. A robot pinned against a static obstacle
     * therefore never accumulated GIVEUP_SEC: the stall was classified
     * {@code STALLED_CHURN} and nothing ever rescued it.
     *
     * <p>Passing the measured velocity is what makes "pinned" observable
     * independently of what is being aimed at.
     */
    @Test
    void pinnedChurnTriggersEscapeWithoutBlacklistingOnePiece() {
        Pose2d pinned = new Pose2d(5.26, 7.17, Rotation2d.fromDegrees(0));
        ChassisSpeeds pushing = new ChassisSpeeds(1.5, 0.0, 0.0);
        ChassisSpeeds notMoving = new ChassisSpeeds(0.0, 0.0, 0.0);

        // Command real speed, move not at all, and hand it a different
        // unreachable piece every cycle for longer than the give-up window.
        TargetProgressWatchdog.Result last = TargetProgressWatchdog.Result.IDLE;
        int cycles = (int) (TargetProgressWatchdog.GIVEUP_SEC / DT) * 3;
        for (int i = 0; i < cycles; i++) {
            // > NEW_TARGET_RESET_M apart each cycle, so every update is a new target.
            double y = 6.40 - (i % 5) * 0.75;
            Pose2d shifting = new Pose2d(5.20, y, Rotation2d.fromDegrees(0));
            last = watchdog.update(pinned, pushing, notMoving, shifting, DT);
            if (last.recovering()) {
                break;
            }
        }

        assertTrue(last.recovering(),
                "a pinned robot whose target keeps changing must still be rescued");
        assertTrue(last.escapeVector().getNorm() > 0.5, "escape must be a real command");
        assertEquals(0, watchdog.blockedPoints().size(),
                "churn must not blacklist: no single piece caused a stall, so "
                        + "excluding one would just starve the bot of options");
    }

    /**
     * The counterpart guard: when the robot really is moving, an oscillating
     * target must NOT trigger recovery. Without this, the churn escape above
     * would fire on every harvest tick.
     */
    @Test
    void movingTargetChangesDoNotTriggerPinnedEscape() {
        ChassisSpeeds driving = new ChassisSpeeds(1.0, 0.0, 0.0);

        for (int i = 0; i < (int) (TargetProgressWatchdog.GIVEUP_SEC / DT) * 3; i++) {
            // Genuinely travelling: 1 m/s along +x, tracked from a moving pose.
            Pose2d moving = new Pose2d(5.0 + i * 0.02, 4.0, Rotation2d.fromDegrees(0));
            Pose2d shifting = new Pose2d(5.20, 6.40 - (i % 5) * 0.75, Rotation2d.fromDegrees(0));
            TargetProgressWatchdog.Result r = watchdog.update(moving, driving, driving, shifting, DT);
            assertFalse(r.recovering(),
                    "a moving robot must never be rescued out from under itself");
        }
        assertEquals(0, watchdog.blockedPoints().size());
    }

    /**
     * A stable unreachable target must still blacklist. The churn escape above is
     * additive; it must not have displaced the original, working recovery.
     */
    @Test
    void stableUnreachableTargetStillBlacklistsWithActualVelocity() {
        Pose2d pinned = new Pose2d(5.26, 7.17, Rotation2d.fromDegrees(0));
        ChassisSpeeds pushing = new ChassisSpeeds(1.5, 0.0, 0.0);
        ChassisSpeeds notMoving = new ChassisSpeeds();

        for (int i = 0; i <= (int) (TargetProgressWatchdog.GIVEUP_SEC / DT); i++) {
            watchdog.update(pinned, pushing, notMoving, TARGET, DT);
        }

        assertEquals(1, watchdog.blockedPoints().size(),
                "a stable unreachable target must still be excluded from selection");
    }

    /**
     * Reached but still pushing is abandoned after the window.
     *
     * <p>This deliberately reverses the other half of the old
     * `arrivingAtTargetResetsProgress` (arrived + commanding 1.5 never gave
     * up), whose premise Phase 0b disproved on the current binary: seed 7,
     * variant `nav-phase0b`, measured two bots stalled 15.9/16.2 s inside this
     * radius with no recovery firing. An unconditional arrived-idle is the
     * arrived hole, not a hold protection — holds command ~0 and stay idle via
     * the test above.
     */
    @Test
    void arrivedButPushingIsAbandoned() {
        Pose2d arrived = new Pose2d(TARGET.getX() + 0.1, TARGET.getY(), Rotation2d.fromDegrees(0));
        TargetProgressWatchdog.Result r = TargetProgressWatchdog.Result.IDLE;
        for (int i = 0; i <= (int) (TargetProgressWatchdog.GIVEUP_SEC / DT); i++) {
            r = watchdog.update(arrived, drive(1.5), TARGET, DT);
        }
        assertTrue(r.recovering(), "pushing from inside ARRIVED_M must still give up");
        assertEquals(1, watchdog.blockedPoints().size());
    }

    /**
     * A slow final approach inside ARRIVED_M still converges and is never
     * abandoned: every 0.25 m of closing resets the window.
     */
    @Test
    void slowConvergingApproachNearTargetIsNotAbandoned() {
        // 0.55 m out, closing 0.02 m/cycle at full command: inside ARRIVED_M
        // the whole way, but always making progress.
        for (int i = 0; i < 20; i++) {
            Pose2d pose = new Pose2d(
                    TARGET.getX() + 0.55 - i * 0.02, TARGET.getY(), Rotation2d.fromDegrees(0));
            TargetProgressWatchdog.Result r =
                    watchdog.update(pose, drive(1.5), TARGET, DT);
            assertFalse(r.recovering(), "a closing approach must never be abandoned");
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

    @Test
    void suppressedPinnedBotEscalatesAfterCumulativeStall() {
        // Regression for the 42r0 Ally1 wedge: contact-recovery suppression kept
        // the watchdog at IDLE for 115 s because it zeroed every accumulator.
        // The suppression-proof fuse must escalate at 9 s of cumulative pinning.
        Pose2d pinned = new Pose2d(4.41, 7.30, Rotation2d.fromDegrees(0));
        ChassisSpeeds pushing = new ChassisSpeeds(3.5, 0.0, 0.0);
        ChassisSpeeds notMoving = new ChassisSpeeds();
        Pose2d target = new Pose2d(6.40, 6.08, Rotation2d.fromDegrees(0));

        TargetProgressWatchdog.Result last = TargetProgressWatchdog.Result.IDLE;
        int steps = (int) (TargetProgressWatchdog.SUPPRESSED_GIVEUP_SEC / DT) + 10;
        for (int i = 0; i < steps; i++) {
            last = watchdog.update(pinned, pushing, notMoving, target, true, DT);
            if (last.recovering()) {
                break;
            }
        }
        assertTrue(last.recovering(),
                "suppressed-but-pinned bot must escalate after 9 s cumulative");
        assertTrue(last.escapeVector().getNorm() > 0.5,
                "escalation must produce a real escape command");
    }

    @Test
    void suppressedMovingBotDoesNotEscalate() {
        // A legitimate contact recovery (backing off, pirouette) is suppressed
        // but the bot is actually moving: the 9 s fuse must not fire.
        Pose2d pose = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0));
        ChassisSpeeds pushing = new ChassisSpeeds(2.0, 0.0, 0.0);
        ChassisSpeeds moving = new ChassisSpeeds(2.0, 0.0, 0.0);
        Pose2d target = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(0));

        for (int i = 0; i < (int) (TargetProgressWatchdog.SUPPRESSED_GIVEUP_SEC / DT) + 50; i++) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(pose, pushing, moving, target, true, DT);
            assertFalse(r.recovering(),
                    "a suppressed-but-moving bot must not trip the fuse at " + (i * DT) + " s");
        }
        assertEquals(0, watchdog.blockedPoints().size(),
                "moving through suppression must not blacklist anything");
    }

    @Test
    void suppressedChurningTargetStillEscalates() {
        // The wedge target churns (RETARGET every few ticks in the log) but the
        // robot stays pinned. The fuse is velocity-based, so churn cannot reset it.
        Pose2d pinned = new Pose2d(4.97, 7.30, Rotation2d.fromDegrees(0));
        ChassisSpeeds pushing = new ChassisSpeeds(3.5, 0.0, 0.0);
        ChassisSpeeds notMoving = new ChassisSpeeds();

        TargetProgressWatchdog.Result last = TargetProgressWatchdog.Result.IDLE;
        int steps = (int) (TargetProgressWatchdog.SUPPRESSED_GIVEUP_SEC / DT) + 10;
        for (int i = 0; i < steps; i++) {
            // Target moves > NEW_TARGET_RESET_M every cycle to force re-evaluation.
            Pose2d shifting = new Pose2d(6.40 - (i % 3) * 0.8, 0.68 + (i % 4) * 0.3,
                    Rotation2d.fromDegrees(0));
            last = watchdog.update(pinned, pushing, notMoving, shifting, true, DT);
            if (last.recovering()) {
                break;
            }
        }
        assertTrue(last.recovering(),
                "churning target must not reset the suppressed-pinned fuse");
    }

    @Test
    void leavingSuppressionResetsFuse() {
        // 4 s suppressed-pinned (under the 9 s fuse) then unsuppress: the
        // accumulator must be cleared so the bot starts a fresh budget.
        Pose2d pinned = START;
        ChassisSpeeds pushing = new ChassisSpeeds(1.5, 0.0, 0.0);
        ChassisSpeeds notMoving = new ChassisSpeeds();

        for (int i = 0; i < (int) (4.0 / DT); i++) {
            TargetProgressWatchdog.Result r =
                    watchdog.update(pinned, pushing, notMoving, TARGET, true, DT);
            assertFalse(r.recovering(), "must not fire before 9 s");
        }
        // Unsuppress for a tick: suppressedPinnedSec zeroes.
        watchdog.update(pinned, pushing, notMoving, TARGET, false, DT);
        assertFalse(watchdog.isRecovering(), "un-suppressed tick must reset the fuse");
        assertEquals(0, watchdog.blockedPoints().size());
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

    @Test
    void explicitAbandonTargetBlacklistsAndEscapes() {
        TargetProgressWatchdog.Result r = watchdog.abandonTarget(TARGET.getTranslation(), START);
        assertTrue(r.recovering(), "explicit abandon must trigger recovery");
        assertEquals(1, r.newlyBlacklisted().size(), "abandoned target must be blacklisted");
        assertTrue(watchdog.isFuelBlocked(TARGET.getTranslation()), "target must be blocked");
        assertTrue(watchdog.isRecovering(), "watchdog must be in recovering state");
        assertTrue(r.escapeVector().getNorm() > 0.5, "escape vector must command motion");
    }

    @Test
    void explicitAbandonTargetNullSafety() {
        TargetProgressWatchdog.Result r1 = watchdog.abandonTarget(null, START);
        assertFalse(r1.recovering());
        TargetProgressWatchdog.Result r2 = watchdog.abandonTarget(TARGET.getTranslation(), null);
        assertFalse(r2.recovering());
    }
}
