package frc.robot.Navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * High-performance holonomic trajectory controller for swerve drive.
 * 
 * Features:
 * - Dynamic velocity-dependent lookahead vectoring
 * - Kinematic deceleration curve: v = min(v_max, sqrt(2 * a * d))
 * - Traction-preserving acceleration ramping
 * - Cross-plane waypoint progression constrained by cross-track error
 * - Trench virtual rail damper for constrained corridors
 * - Independent rotation override for Shooting-On-The-Fly (SOTF)
 * - Stall escape delegated to ContactWatchdog (see arbitrate())
 */
public class TrajectoryController {

    private final PIDController headingController;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;
    private Pose2d lastPathTarget = new Pose2d(-999, -999, new Rotation2d());
    private double lastPlanTimestamp = -1.0;
    private Translation2d pathStartTranslation = new Translation2d();
    private boolean isExplicitPath = false;
    /**
     * Status of the active waypoint plan. The planner reports it via
     * {@link StaticPathfinder#findPathWithStatus}; this controller must not
     * discard it. A consumed LOCAL_RECOVERY plan is a shake-loose step, not
     * arrival at the goal (see the replan trigger in {@link #calculate}).
     */
    private StaticPathfinder.PathStatus lastPathStatus = null;
    /**
     * A consumed LOCAL_RECOVERY plan must be replaced, not held. Set when the
     * recovery step is executed with the goal still far; consumed by the
     * replan branch on the next tick.
     */
    private boolean recoveryReplanDue = false;

    // Kinematics & Speed profile
    private double currentCommandedSpeed = 0.0;
    private double lastCalculationTime = -1.0;
    private static final double MAX_ACCELERATION_MPS2 = 4.25; // Traction-limited acceleration
    private static final double MAX_DECELERATION_MPS2 = 3.50; // Smooth deceleration approaching target

    /**
     * Cross-track error a robot may have before a waypoint-plane crossing counts as
     * real progress, in open field.
     */
    public static final double CROSS_TRACK_GATE_M = 0.45;

    /**
     * Tighter cross-track gate inside a trench corridor. The drivable centre band
     * there is only ~0.3787 m wide (see {@code TrenchCorridorClearanceTest}), so the
     * open-field 0.45 m gate was <i>wider than the corridor the robot may legally
     * occupy</i>: a robot shoved 0.18-0.45 m off the lane centreline passed the gate
     * and advanced a waypoint while its centre was already inside the inflated
     * footprint. The planner then prepended an escape waypoint and the robot
     * re-entered the lane, so the visible symptom was detour and lane re-entry churn
     * rather than a hard stall.
     *
     * <p>Derived, not guessed: the lane centreline sits 0.1797 m clear of the inflated
     * wall, so half the drivable band is the most lateral error the robot can carry
     * before it is illegal. Rounded up slightly to absorb odometry noise without
     * returning to the full-band width.
     */
    public static final double TRENCH_CROSS_TRACK_GATE_M = 0.22;

    /**
     * Retry cadence for a plan that came back empty. See the no-route branch in
     * {@link #calculate}.
     */
    public static final double REPLAN_RETRY_SEC = 0.25;

    /**
     * Goal distance above which a consumed LOCAL_RECOVERY plan triggers a
     * replan instead of a hold. At or below this the step's endpoint counts as
     * arrived and the §9 turn-in-place stance owns the tick.
     */
    public static final double RECOVERY_REPLAN_GOAL_MIN_M = 0.30;

    /**
     * Path-remaining radius below which a LOCAL_RECOVERY plan counts as
     * consumed. Matches the §9 turn-in-place stance so the trigger and the
     * zeroing branch agree on what "executed" means.
     */
    public static final double RECOVERY_CONSUMED_M = 0.05;

    // Rotation override (e.g. SOTF auto-aiming at Hub while moving)
    private Supplier<Rotation2d> rotationOverride = null;

    // Phase 2: pirouette escape lives in ContactWatchdog; this controller stays
    // pure path tracking.

    public TrajectoryController(PIDController headingController) {
        this.headingController = headingController;
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Resets internal path progress and controller state.
     */
    public void reset() {
        waypoints.clear();
        currentWaypointIndex = 0;
        lastPathTarget = new Pose2d(-999, -999, new Rotation2d());
        lastPlanTimestamp = -1.0;
        lastCalculationTime = -1.0;
        currentCommandedSpeed = 0.0;
        isExplicitPath = false;
        lastPathStatus = null;
        recoveryReplanDue = false;
        rotationOverride = null;
        // A new path must not inherit derivative kick / integral from the old
        // path's heading history. Without this, the first calculate() after a
        // reset (or the first call in a unit test sharing the singleton
        // controller) can saturate omega from stale PID state.
        headingController.reset();
    }

    /**
     * Sets an explicit, pre-planned sequence of waypoints (e.g. Tunnel Route or Fuel Tour).
     * Disables automatic re-planning from overwriting these waypoints.
     */
    public void setExplicitWaypoints(List<Pose2d> path) {
        setExplicitWaypoints(path, null);
    }

    /**
     * Sets an explicit, pre-planned sequence of waypoints anchored to an initial start translation.
     */
    public void setExplicitWaypoints(List<Pose2d> path, Translation2d startTranslation) {
        waypoints.clear();
        waypoints.addAll(path);
        currentWaypointIndex = 0;
        lastPlanTimestamp = Timer.getTimestamp();
        isExplicitPath = true;
        lastPathStatus = null;
        recoveryReplanDue = false;
        if (!path.isEmpty()) {
            lastPathTarget = path.get(path.size() - 1);
            pathStartTranslation = (startTranslation != null)
                    ? startTranslation
                    : path.get(0).getTranslation();
        }
    }

    /**
     * Clears explicit waypoints and returns the controller to dynamic pathfinding mode.
     */
    public void clearExplicitPath() {
        if (isExplicitPath) {
            isExplicitPath = false;
            waypoints.clear();
            currentWaypointIndex = 0;
            lastPlanTimestamp = -1.0;
        }
    }

    public boolean isExplicitPath() {
        return isExplicitPath;
    }

    /**
     * Overrides the rotational heading target while still following the
     * translational path.
     * Useful for pointing at the Hub for Shooting-On-The-Fly.
     */
    public void setRotationOverride(Supplier<Rotation2d> override) {
        this.rotationOverride = override;
    }

    /**
     * Computes field-oriented ChassisSpeeds to follow the trajectory.
     *
     * @param currentPose           Current robot pose
     * @param currentSpeeds         Current robot velocity
     * @param targetPose            Desired final pose
     * @param maxSpeed              Maximum allowable translational velocity (m/s)
     * @param isStalled             True if physical drivetrain stall/collision is
     *                              detected
     * @param allowDynamicAvoidance True if local reactive obstacle avoidance is
     *                              permitted
     * @return Field-oriented ChassisSpeeds
     */
    public ChassisSpeeds calculate(
            Pose2d currentPose,
            ChassisSpeeds currentSpeeds,
            Pose2d targetPose,
            double maxSpeed,
            boolean isStalled,
            boolean allowDynamicAvoidance) {

        double now = Timer.getTimestamp();
        double dt = lastCalculationTime > 0 ? Math.max(0.001, now - lastCalculationTime) : 0.02;
        lastCalculationTime = now;

        // Ensure destination is outside static/dynamic barriers
        targetPose = StaticPathfinder.ensurePoseOutsideObstacles(targetPose, currentPose.getTranslation());

        // ── 1. Stall escape is owned by ContactWatchdog (see arbitrate()); this
        // controller
        // tracks the path only. The isStalled flag is retained for API compatibility.
        // ── 2. Automatic Path Generation & Re-planning ──────────────────────
        double distTargetMoved = targetPose.getTranslation().getDistance(lastPathTarget.getTranslation());
        boolean needReplan = !isExplicitPath && (waypoints.isEmpty()
                || recoveryReplanDue
                || (distTargetMoved > 0.85)
                || (now - lastPlanTimestamp > 0.50 && distTargetMoved > 0.30)
                // Stuck off-segment (e.g. APF shove past the 0.45 m cross-track
                // gate in a trench): refresh from the measured pose so the new
                // segment starts near the robot instead of holding an angled
                // lookahead forever. Throttled to 0.5 s to avoid replan thrash.
                || (isStalled && !waypoints.isEmpty() && now - lastPlanTimestamp > 0.50));

        if (needReplan) {
            waypoints.clear();
            StaticPathfinder.PathResult plan =
                    StaticPathfinder.findPathWithStatus(currentPose, targetPose);
            waypoints.addAll(plan.waypoints());
            lastPathStatus = plan.status();
            recoveryReplanDue = false;
            currentWaypointIndex = 0;
            lastPathTarget = targetPose;
            lastPlanTimestamp = now;
            pathStartTranslation = currentPose.getTranslation();
            Logger.recordOutput("Trajectory/PathStatus",
                    lastPathStatus == null ? "NONE" : lastPathStatus.name());
            if (waypoints.isEmpty()) {
                // No valid route. Sep 26 made this a deliberate stop rather than a
                // straight-line command through an obstacle, which is still the right
                // call. But lastPathTimestamp was just stamped, so the next replan
                // would wait for the target to move 0.85 m -- for a static target, that
                // is an indefinite hold. Retry every REPLAN_RETRY_SEC instead, and let
                // the caller-visible flag plus the stall detectors handle a genuinely
                // unreachable target. Logged because a silent stop is indistinguishable
                // from a hang in a replay.
                lastPlanTimestamp = now - REPLAN_RETRY_SEC;
                Logger.recordOutput("Trajectory/NoRoute", true);
            } else {
                Logger.recordOutput("Trajectory/NoRoute", false);
            }
        } else if (!isExplicitPath && distTargetMoved > 0.01 && !waypoints.isEmpty()) {
            waypoints.set(waypoints.size() - 1, targetPose);
            lastPathTarget = targetPose;
        }

        if (waypoints.isEmpty()) {
            currentCommandedSpeed = 0.0;
            return new ChassisSpeeds();
        }

        boolean evacuatingStaticObstacle = currentWaypointIndex == 0
                && StaticPathfinder.isPointInStaticObstacle(currentPose.getTranslation());

        // A trench corridor is far narrower than open field, so the cross-track gate
        // has to shrink with it (see TRENCH_CROSS_TRACK_GATE_M). Computed once here
        // because waypoint progression and the lookahead clamp both depend on it.
        boolean inTrench = FieldMap.Trenches.isLowClearance(currentPose.getTranslation());
        double crossTrackGate = inTrench ? TRENCH_CROSS_TRACK_GATE_M : CROSS_TRACK_GATE_M;

        // ── 3. Waypoint Progression (Cross-Plane Projection) ────────────────
        while (!evacuatingStaticObstacle && currentWaypointIndex < waypoints.size() - 1) {
            Pose2d wp = waypoints.get(currentWaypointIndex);
            Translation2d prev = (currentWaypointIndex > 0)
                    ? waypoints.get(currentWaypointIndex - 1).getTranslation()
                    : pathStartTranslation;
            Translation2d seg = wp.getTranslation().minus(prev);
            double segLen = seg.getNorm();

            Translation2d toBot = currentPose.getTranslation().minus(wp.getTranslation());
            boolean passedPlane = false;
            if (segLen > 0.05) {
                double unitX = seg.getX() / segLen;
                double unitY = seg.getY() / segLen;
                double alongTrack = toBot.getX() * unitX + toBot.getY() * unitY;
                double crossTrack = Math.abs(toBot.getX() * unitY - toBot.getY() * unitX);
                // Crossing the waypoint plane is only progress if we passed near
                // the segment. Otherwise a shove/avoidance detour can skip a
                // tunnel corner and command a path that clips the obstacle. In a
                // trench the gate is tightened so a lateral shove cannot advance the
                // path from inside the inflated footprint.
                passedPlane = alongTrack >= 0.0 && crossTrack < crossTrackGate;
            }

            if (toBot.getNorm() < crossTrackGate || passedPlane) {
                currentWaypointIndex++;
            } else {
                break;
            }
        }

        // ── 4. Remaining Distance Along Path Polyline ────────────────────────
        double distToGoal = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double remainingDist = distToGoal;

        if (!waypoints.isEmpty()) {
            remainingDist = currentPose.getTranslation()
                    .getDistance(waypoints.get(currentWaypointIndex).getTranslation());
            for (int i = currentWaypointIndex; i < waypoints.size() - 1; i++) {
                remainingDist += waypoints.get(i).getTranslation().getDistance(waypoints.get(i + 1).getTranslation());
            }
            if (StaticPathfinder.isLineOfSightClear(currentPose.getTranslation(), targetPose.getTranslation())) {
                remainingDist = Math.min(remainingDist, distToGoal);
            }
        }

        // A consumed LOCAL_RECOVERY plan is a shake-loose, not arrival: the
        // waypoint list holds only the recovery step, so reaching the end of
        // it with the goal still far means the shake is done and the route is
        // still open, not that the robot is done. Holding here parks with a
        // static target (no replan trigger, NoRoute false, stall detectors
        // blind below their command floor) — measured as a 62 s zero-command
        // sit in the seed-2026 headless match. Replan from the new vantage on
        // the next tick instead: it either routes, or steps again. Either way
        // the command stays nonzero, so the progress watchdog (3 s give-up)
        // owns genuinely unreachable goals instead of silence.
        if (!isExplicitPath
                && !recoveryReplanDue
                && lastPathStatus == StaticPathfinder.PathStatus.LOCAL_RECOVERY
                && remainingDist < RECOVERY_CONSUMED_M
                && distToGoal > RECOVERY_REPLAN_GOAL_MIN_M) {
            recoveryReplanDue = true;
        }

        // ── 5. Kinematic Speed Profiling with Acceleration Ramping ──────────
        double speedLimit = Math.sqrt(2.0 * MAX_DECELERATION_MPS2 * Math.max(0.0, remainingDist));
        double targetSpeed = Math.min(maxSpeed, speedLimit);

        if (remainingDist < 0.04) {
            targetSpeed = 0.0;
        } else if (targetSpeed < 0.25 && remainingDist > 0.04) {
            targetSpeed = 0.25; // Carpet friction breakout floor
        }

        currentCommandedSpeed = targetSpeed;

        // ── 6. Lookahead Vector & Translation ───────────────────────────────
        // inTrench was resolved above alongside the cross-track gate.
        // In tight corridors, clamp lookahead to 0.35m to prevent cutting corners into
        // the truss
        double lookaheadDist = inTrench
                ? 0.35
                : Math.max(0.40, Math.min(0.85, 0.35 + 0.12 * currentCommandedSpeed));
        Translation2d lookaheadPoint = evacuatingStaticObstacle
                ? waypoints.get(0).getTranslation()
                : computeLookahead(currentPose, lookaheadDist, targetPose);
        Translation2d driveDir = lookaheadPoint.minus(currentPose.getTranslation());
        double norm = driveDir.getNorm();
        Translation2d unitDrive = norm > 1e-4 ? driveDir.div(norm) : new Translation2d();

        double vx = unitDrive.getX() * currentCommandedSpeed;
        double vy = unitDrive.getY() * currentCommandedSpeed;

        // ── 7. Heading Determination (SOTF vs Travel vs Target) ─────────────
        Rotation2d desiredHeading;
        if (rotationOverride != null) {
            desiredHeading = rotationOverride.get();
        } else if (currentWaypointIndex >= waypoints.size() - 1 || norm <= 0.15) {
            desiredHeading = targetPose.getRotation();
        } else {
            desiredHeading = driveDir.getAngle();
        }

        // ── 8. Low-Clearance Heading Alignment (No forced Virtual Rail) ─────
        // In low-clearance trench zones, align heading to 0°/180° if unconstrained to
        // avoid clipping truss posts.
        // Holonomic translation (vx, vy) remains natural and unconstrained, guided by
        // StaticPathfinder.
        if (inTrench && rotationOverride == null) {
            double deg = currentPose.getRotation().getDegrees();
            desiredHeading = Math.abs(deg) <= 90.0 ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
        }

        double omega = headingController.calculate(currentPose.getRotation().getRadians(), desiredHeading.getRadians());
        omega = Math.max(-4.5, Math.min(4.5, omega));

        // ── 9. Final Turn-in-Place Stance ───────────────────────────────────
        if (remainingDist < 0.05) {
            vx = 0.0;
            vy = 0.0;
        }

        ChassisSpeeds nominalSpeeds = new ChassisSpeeds(vx, vy, omega);

        // ── 10. Dynamic Reactive Avoidance Layer ────────────────────────────
        if (allowDynamicAvoidance && currentCommandedSpeed > 0.05) {
            nominalSpeeds = DynamicRouter.computeAvoidanceSpeeds(currentPose, nominalSpeeds, lookaheadPoint);
        }

        // Telemetry
        Logger.recordOutput("Trajectory/TargetSpeed", currentCommandedSpeed);
        Logger.recordOutput("Trajectory/RemainingDistance", remainingDist);
        Logger.recordOutput("Trajectory/WaypointIndex", currentWaypointIndex);
        Logger.recordOutput("Trajectory/LookaheadPoint", new Pose2d(lookaheadPoint, desiredHeading));

        return nominalSpeeds;
    }

    private Translation2d computeLookahead(Pose2d currentPose, double lookaheadDist, Pose2d targetPose) {
        if (waypoints.isEmpty() || currentWaypointIndex >= waypoints.size() - 1) {
            return targetPose.getTranslation();
        }
        double remainingNeeded = lookaheadDist;
        Translation2d pCurrent = currentPose.getTranslation();

        for (int i = currentWaypointIndex; i < waypoints.size(); i++) {
            Translation2d pNext = waypoints.get(i).getTranslation();
            double dSeg = pCurrent.getDistance(pNext);
            if (dSeg >= remainingNeeded) {
                if (dSeg > 1e-4) {
                    double frac = remainingNeeded / dSeg;
                    return new Translation2d(
                            pCurrent.getX() + frac * (pNext.getX() - pCurrent.getX()),
                            pCurrent.getY() + frac * (pNext.getY() - pCurrent.getY()));
                }
                return pNext;
            }
            remainingNeeded -= dSeg;
            pCurrent = pNext;
        }
        return targetPose.getTranslation();
    }

    /**
     * Checks whether the robot has arrived within tolerance of the final target.
     */
    public boolean isFinished(Pose2d currentPose, Pose2d targetPose, double transTol, double rotTolDeg) {
        double d = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double degErr = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
        return d < transTol && degErr < rotTolDeg;
    }

    public List<Pose2d> getWaypoints() {
        return Collections.unmodifiableList(waypoints);
    }

    public int getCurrentWaypointIndex() {
        return currentWaypointIndex;
    }

    public boolean isStalledActive() {
        return ContactWatchdog.getInstance().isPirouetteActive();
    }
}
