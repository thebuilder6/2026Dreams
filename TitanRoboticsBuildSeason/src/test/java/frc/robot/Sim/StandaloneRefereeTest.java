package frc.robot.Sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Intelligence.Archetype;

/**
 * G418 pin accounting for the MapleSim-free runner. The referee is stateful and
 * wall-clock-free, so it can be driven at a fixed step directly.
 */
public class StandaloneRefereeTest {

    private static StandaloneBot bullyAt(double x) {
        return new StandaloneBot(new Pose2d(x, 4.0, Rotation2d.fromDegrees(0)),
                Archetype.DEFENSE_BULLY, false, 0, 0, false);
    }

    private static StandaloneBot cyclerAt(double x) {
        return new StandaloneBot(new Pose2d(x, 4.0, Rotation2d.fromDegrees(180)),
                Archetype.AUTONOMOUS_CYCLER, true, 0, 1, false);
    }

    @Test
    void sustainedContactChargesAMinorThenEscalatesToMajor() {
        StandaloneBot a = bullyAt(5.0);
        StandaloneBot b = cyclerAt(5.5); // 0.5 m apart, inside CONTACT_DIST_M
        a.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0)); // actively driving into b
        StandaloneReferee ref = new StandaloneReferee();
        // 7 s of active contact: minor at 3 s, first major at 6 s.
        for (int i = 0; i < 350; i++) {
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(1, a.getMinorFouls(), "first 3 s draws a minor foul");
        assertEquals(1, a.getMajorFouls(), "the next uncorrected 3 s draws a major foul");
        assertEquals(0, b.getMinorFouls(), "only the aggressor is charged");
        assertEquals(0, b.getMajorFouls());
    }

    @Test
    void passiveProximityIsNotAPin() {
        StandaloneBot a = bullyAt(5.0);
        StandaloneBot b = cyclerAt(5.5); // touching, both stationary
        StandaloneReferee ref = new StandaloneReferee();
        for (int i = 0; i < 500; i++) { // 10 s of coexistence
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "coexisting in the contact band is not a PIN");
        assertEquals(0, a.getMajorFouls());
    }

    @Test
    void escalationsAreBoundedPerEngagement() {
        StandaloneBot a = bullyAt(5.0);
        StandaloneBot b = cyclerAt(5.5);
        a.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0));
        StandaloneReferee ref = new StandaloneReferee();
        for (int i = 0; i < 2000; i++) { // 40 s of sustained active pinning
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(1, a.getMinorFouls());
        assertEquals(StandaloneReferee.MAX_PIN_VIOLATIONS - 1, a.getMajorFouls(),
                "escalation is capped until separation resets it");
    }

    @Test
    void separationEndsThePinCountBeforeAFoul() {        StandaloneBot a = bullyAt(5.0);
        StandaloneBot b = cyclerAt(5.5);
        a.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0));
        StandaloneReferee ref = new StandaloneReferee();
        for (int i = 0; i < 100; i++) { // 2 s, under the 3 s threshold
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "2 s of contact is legal");
        b.setPose(new Pose2d(12.0, 4.0, Rotation2d.fromDegrees(180))); // separate
        ref.update(List.of(a, b), 0.02);
        for (int i = 0; i < 100; i++) {
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "separation clears the accumulated pin time");
    }

    @Test
    void restingAtThePushOutFloorNeverAccrues() {        StandaloneBot a = bullyAt(5.0);
        // Exactly the push-out equilibrium: contact, but no overlap.
        StandaloneBot b = cyclerAt(5.0 + StandaloneMatchRunner.PEER_MIN_DISTANCE_M);
        a.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0)); // still driving in
        StandaloneReferee ref = new StandaloneReferee();
        for (int i = 0; i < 500; i++) { // 10 s of floor contact with closing speed
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "floor contact without overlap is not a PIN");
        assertEquals(0, a.getMajorFouls());
    }

    @Test
    void fleeingVictimIsAChaseNotAPin() {
        StandaloneBot a = bullyAt(5.0);
        StandaloneBot b = cyclerAt(5.5); // overlapped, but both at full speed
        a.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0)); // chasing into b
        b.setVelocity(new ChassisSpeeds(1.5, 0.0, 0.0)); // ...who is getting away
        StandaloneReferee ref = new StandaloneReferee();
        for (int i = 0; i < 500; i++) { // 10 s of glued chase
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "a victim that keeps moving is not trapped");
        assertEquals(0, a.getMajorFouls());
    }

    @Test
    void scriptedTrapAccruesAcrossMovingTicksAndHoldsThroughAGap() {
        // The frozen-pose tests never move: here the aggressor creeps into a
        // stationary victim tick-by-tick (realistic speeds, varying poses), with
        // a 1 s mid-gap inside separation but outside contact — the count must
        // hold, not reset, and the fouls must still land on schedule.
        // Midfield open water (far from the inflated hub/ramp/trench
        // footprints `placePose` projects out of), so setPose never fires
        // mid-script.
        StandaloneBot a = new StandaloneBot(new Pose2d(7.8, 4.0, Rotation2d.fromDegrees(0)),
                Archetype.DEFENSE_BULLY, false, 0, 0, false);
        StandaloneBot b = new StandaloneBot(new Pose2d(8.3, 4.0, Rotation2d.fromDegrees(180)),
                Archetype.AUTONOMOUS_CYCLER, true, 0, 1, false);
        b.setVelocity(new ChassisSpeeds()); // trapped: not getting away
        StandaloneReferee ref = new StandaloneReferee();
        double ax = 7.8;
        for (int i = 0; i < 100; i++) { // 2 s of closing contact
            ax = Math.min(ax + 0.005, 8.15);
            a.setPose(new Pose2d(ax, 4.0, Rotation2d.fromDegrees(0)));
            a.setVelocity(new ChassisSpeeds(0.25, 0.0, 0.0));
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "2 s of contact is legal");
        a.setPose(new Pose2d(6.6, 4.0, Rotation2d.fromDegrees(0))); // 1.7 m gap
        for (int i = 0; i < 50; i++) { // 1 s apart: hold (decay), not reset
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(0, a.getMinorFouls(), "the gap must hold the count, not foul");
        ax = 7.8;
        for (int i = 0; i < 100; i++) { // re-close: 2.0 accrued - 1.0 decayed + 2.0
            ax = Math.min(ax + 0.005, 8.15);
            a.setPose(new Pose2d(ax, 4.0, Rotation2d.fromDegrees(0)));
            a.setVelocity(new ChassisSpeeds(0.25, 0.0, 0.0));
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(1, a.getMinorFouls(), "3 s net contact draws a minor foul");
        for (int i = 0; i < 160; i++) { // 3+ s uncorrected (margin past float dust at 6.0)
            ax = Math.min(ax + 0.005, 8.15);
            a.setPose(new Pose2d(ax, 4.0, Rotation2d.fromDegrees(0)));
            a.setVelocity(new ChassisSpeeds(0.25, 0.0, 0.0));
            ref.update(List.of(a, b), 0.02);
        }
        assertEquals(1, a.getMajorFouls(), "the next uncorrected 3 s draws a major foul");
        assertEquals(0, b.getMinorFouls(), "only the aggressor is charged");
        assertEquals(0, b.getMajorFouls());
    }
}
