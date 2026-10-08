package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * A consumed LOCAL_RECOVERY plan must replan, never park.
 *
 * <p>Regression for the seed-2026 headless sit: Blue Ally0 parked 62 s at
 * (0.67, 2.73) commanding 0.000 with its goal 0.84 m away. The planner had
 * returned a single LOCAL_RECOVERY shake-loose step, whose status the
 * controller discarded; arrival at the step looked exactly like arrival at the
 * goal (remaining &lt; 0.05), the static target never retriggered a plan,
 * NoRoute stayed false, and every stall detector gates on commanded speed, so
 * the whole safety stack was blind.
 */
class LocalRecoveryReplanTest {
    private static final Pose2d SIT_POSE = new Pose2d(0.67, 2.73, Rotation2d.fromDegrees(180));
    private static final Pose2d SIT_TARGET = new Pose2d(0.45, 3.54, Rotation2d.fromDegrees(180));

    private static double trans(ChassisSpeeds s) {
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    /** Pins the trigger geometry: pinched start/goal must report LOCAL_RECOVERY. */
    @Test
    void recoveryStatusIsReportedForPinchedTarget() {
        StaticPathfinder.PathResult plan = StaticPathfinder.findPathWithStatus(SIT_POSE, SIT_TARGET);
        assertEquals(StaticPathfinder.PathStatus.LOCAL_RECOVERY, plan.status(),
                "pinched tower-adjacent target must plan as LOCAL_RECOVERY, not a route");
        assertTrue(!plan.waypoints().isEmpty(), "recovery must carry its shake-loose step");
    }

    /**
     * Executing the recovery step and arriving with the goal still far must
     * produce a fresh plan on the next tick — never a held zero.
     */
    @Test
    void consumedRecoveryReplansInsteadOfParking() {
        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));

        // Tick 1: fresh plan, drive the recovery step.
        ChassisSpeeds out1 = tc.calculate(SIT_POSE, new ChassisSpeeds(), SIT_TARGET, 3.4, false, false);
        assertTrue(trans(out1) >= 0.25, "must drive the recovery step, got " + trans(out1));

        // Tick 2: arrive at the step end with the static goal 0.84 m away.
        // This is the arrival tick; the replan fires on the next tick.
        List<Pose2d> step = StaticPathfinder.findPath(SIT_POSE, SIT_TARGET);
        Pose2d atEnd = new Pose2d(
                step.get(step.size() - 1).getX(), step.get(step.size() - 1).getY(),
                Rotation2d.fromDegrees(180));
        tc.calculate(atEnd, out1, SIT_TARGET, 3.4, false, false);

        // Tick 3: the consumed recovery must have replanned from the new
        // vantage — a nonzero command toward a fresh step or a route.
        ChassisSpeeds out3 = tc.calculate(atEnd, out1, SIT_TARGET, 3.4, false, false);
        assertTrue(trans(out3) >= 0.25,
                "consumed recovery with goal 0.84 m away must replan, not park; got " + trans(out3));
    }

    /** Normal routes are untouched: open-field arrival still holds at zero. */
    @Test
    void directRouteArrivalStillHolds() {
        TrajectoryController tc = new TrajectoryController(new PIDController(1.0, 0.0, 0.0));
        Pose2d goal = new Pose2d(9.00, 4.035, new Rotation2d());
        ChassisSpeeds out1 =
                tc.calculate(new Pose2d(8.00, 4.035, new Rotation2d()), new ChassisSpeeds(), goal, 3.0, false, false);
        assertTrue(trans(out1) >= 0.25, "must drive toward an open-field goal");

        ChassisSpeeds out2 =
                tc.calculate(new Pose2d(8.99, 4.035, new Rotation2d()), out1, goal, 3.0, false, false);
        assertTrue(trans(out2) < 0.25, "must hold once genuinely arrived, got " + trans(out2));
    }
}
