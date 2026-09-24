package frc.robot.Auto;

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
import frc.robot.Data.Constants;
import frc.robot.Data.FieldMap;
import frc.robot.Subsystems.Intake;
import org.littletonrobotics.junction.Logger;

/**
 * High-performance holonomic trajectory controller for swerve drive.
 * 
 * Features:
 * - Dynamic velocity-dependent lookahead vectoring
 * - Kinematic deceleration curve: v = min(v_max, sqrt(2 * a * d))
 * - Traction-preserving acceleration ramping
 * - Cross-plane monotonic waypoint progression
 * - Trench virtual rail damper for constrained corridors
 * - Independent rotation override for Shooting-On-The-Fly (SOTF)
 * - Automatic stall / pin recovery pirouette reflex
 */
public class TrajectoryController {

    private final PIDController headingController;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;
    private Pose2d lastPathTarget = new Pose2d(-999, -999, new Rotation2d());
    private double lastPlanTimestamp = -1.0;
    private Translation2d pathStartTranslation = new Translation2d();
    private boolean isExplicitPath = false;

    // Kinematics & Speed profile
    private double currentCommandedSpeed = 0.0;
    private double lastCalculationTime = -1.0;
    private static final double MAX_ACCELERATION_MPS2 = 4.25; // Traction-limited acceleration
    private static final double MAX_DECELERATION_MPS2 = 3.50; // Smooth deceleration approaching target

    // Rotation override (e.g. SOTF auto-aiming at Hub while moving)
    private Supplier<Rotation2d> rotationOverride = null;

    // Anti-stall / unstick reflex state
    private double unstickEndTime = -1.0;
    private Translation2d unstickVector = new Translation2d();

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
        unstickEndTime = -1.0;
        isExplicitPath = false;
        rotationOverride = null;
    }

    /**
     * Sets an explicit, pre-planned sequence of waypoints (e.g. Tunnel Route).
     * Disables automatic re-planning from overwriting these waypoints.
     */
    public void setExplicitWaypoints(List<Pose2d> path) {
        waypoints.clear();
        waypoints.addAll(path);
        currentWaypointIndex = 0;
        lastPlanTimestamp = Timer.getFPGATimestamp();
        isExplicitPath = true;
        if (!path.isEmpty()) {
            lastPathTarget = path.get(path.size() - 1);
            pathStartTranslation = path.get(0).getTranslation();
        }
    }

    /**
     * Overrides the rotational heading target while still following the translational path.
     * Useful for pointing at the Hub for Shooting-On-The-Fly.
     */
    public void setRotationOverride(Supplier<Rotation2d> override) {
        this.rotationOverride = override;
    }

    /**
     * Computes field-oriented ChassisSpeeds to follow the trajectory.
     *
     * @param currentPose Current robot pose
     * @param currentSpeeds Current robot velocity
     * @param targetPose Desired final pose
     * @param maxSpeed Maximum allowable translational velocity (m/s)
     * @param isStalled True if physical drivetrain stall/collision is detected
     * @param allowDynamicAvoidance True if local reactive obstacle avoidance is permitted
     * @return Field-oriented ChassisSpeeds
     */
    public ChassisSpeeds calculate(
            Pose2d currentPose,
            ChassisSpeeds currentSpeeds,
            Pose2d targetPose,
            double maxSpeed,
            boolean isStalled,
            boolean allowDynamicAvoidance) {

        double now = Timer.getFPGATimestamp();
        double dt = lastCalculationTime > 0 ? Math.max(0.001, now - lastCalculationTime) : 0.02;
        lastCalculationTime = now;

        // Ensure destination is outside static/dynamic barriers
        targetPose = StaticPathfinder.ensurePoseOutsideObstacles(targetPose, currentPose.getTranslation());

        // ── 1. Anti-Stall / Unstick Pirouette Reflex ────────────────────────
        if (isStalled && now > unstickEndTime) {
            unstickEndTime = now + 0.40;
            Translation2d vel = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
            if (vel.getNorm() < 0.10) {
                vel = targetPose.getTranslation().minus(currentPose.getTranslation());
            }
            unstickVector = vel.getNorm() > 0.10 ? vel.div(vel.getNorm()).times(-1.8) : new Translation2d(-1.5, 0.0);
            waypoints.clear();
            isExplicitPath = false;
        }

        if (now < unstickEndTime) {
            return new ChassisSpeeds(unstickVector.getX(), unstickVector.getY(), 6.0); // Spin at 6 rad/s
        }

        // ── 2. Automatic Path Generation & Re-planning ──────────────────────
        double distTargetMoved = targetPose.getTranslation().getDistance(lastPathTarget.getTranslation());
        boolean needReplan = !isExplicitPath && (waypoints.isEmpty()
                || (distTargetMoved > 0.85)
                || (now - lastPlanTimestamp > 0.50 && distTargetMoved > 0.30));

        if (needReplan) {
            waypoints.clear();
            waypoints.addAll(StaticPathfinder.findPath(currentPose, targetPose));
            currentWaypointIndex = 0;
            lastPathTarget = targetPose;
            lastPlanTimestamp = now;
            pathStartTranslation = currentPose.getTranslation();
        } else if (!isExplicitPath && distTargetMoved > 0.01 && !waypoints.isEmpty()) {
            waypoints.set(waypoints.size() - 1, targetPose);
            lastPathTarget = targetPose;
        }

        if (waypoints.isEmpty()) {
            currentCommandedSpeed = 0.0;
            return new ChassisSpeeds();
        }

        // ── 3. Waypoint Progression (Cross-Plane Projection) ────────────────
        while (currentWaypointIndex < waypoints.size() - 1) {
            Pose2d wp = waypoints.get(currentWaypointIndex);
            Translation2d prev = (currentWaypointIndex > 0)
                    ? waypoints.get(currentWaypointIndex - 1).getTranslation()
                    : pathStartTranslation;
            Translation2d seg = wp.getTranslation().minus(prev);
            double segLen = seg.getNorm();

            Translation2d toBot = currentPose.getTranslation().minus(wp.getTranslation());
            boolean passedPlane = false;
            if (segLen > 0.05) {
                double dot = toBot.getX() * (seg.getX() / segLen) + toBot.getY() * (seg.getY() / segLen);
                if (dot >= 0.0) passedPlane = true;
            }

            if (toBot.getNorm() < 0.45 || passedPlane) {
                currentWaypointIndex++;
            } else {
                break;
            }
        }

        // ── 4. Remaining Distance Along Path Polyline ────────────────────────
        double distToGoal = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double remainingDist = distToGoal;

        if (!waypoints.isEmpty()) {
            remainingDist = currentPose.getTranslation().getDistance(waypoints.get(currentWaypointIndex).getTranslation());
            for (int i = currentWaypointIndex; i < waypoints.size() - 1; i++) {
                remainingDist += waypoints.get(i).getTranslation().getDistance(waypoints.get(i + 1).getTranslation());
            }
            if (StaticPathfinder.isLineOfSightClear(currentPose.getTranslation(), targetPose.getTranslation())) {
                remainingDist = Math.min(remainingDist, distToGoal);
            }
        }

        // ── 5. Kinematic Speed Profiling with Acceleration Ramping ──────────
        double speedLimit = Math.sqrt(2.0 * MAX_DECELERATION_MPS2 * Math.max(0.0, remainingDist));
        double targetSpeed = Math.min(maxSpeed, speedLimit);

        if (remainingDist < 0.04) {
            targetSpeed = 0.0;
        } else if (targetSpeed < 0.25 && remainingDist > 0.04) {
            targetSpeed = 0.25; // Carpet friction breakout floor
        }

        // Acceleration limiter (traction control)
        double maxDeltaV = MAX_ACCELERATION_MPS2 * dt;
        if (targetSpeed > currentCommandedSpeed + maxDeltaV) {
            currentCommandedSpeed += maxDeltaV;
        } else {
            currentCommandedSpeed = targetSpeed;
        }

        // ── 6. Lookahead Vector & Translation ───────────────────────────────
        double lookaheadDist = Math.max(0.40, Math.min(0.85, 0.35 + 0.12 * currentCommandedSpeed));
        Translation2d lookaheadPoint = computeLookahead(currentPose, lookaheadDist, targetPose);
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

        // ── 8. Trench Virtual Rail Damper ───────────────────────────────────
        boolean inTrench = FieldMap.Trenches.isLowClearance(currentPose.getTranslation());
        boolean targetInTrench = FieldMap.Trenches.isLowClearance(lookaheadPoint);

        // Only lock cross-track to centerline if BOTH robot and lookahead are transiting the trench!
        if (inTrench && targetInTrench) {
            double deg = currentPose.getRotation().getDegrees();
            Rotation2d trenchHeading = Math.abs(deg) <= 90.0 ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
            if (rotationOverride == null) {
                desiredHeading = trenchHeading;
            }

            double yCenterline = currentPose.getY() >= 4.0 ? FieldMap.Trenches.TOP_CORRIDOR_Y : FieldMap.Trenches.BOT_CORRIDOR_Y;
            double crossTrackError = yCenterline - currentPose.getY();
            vy = Math.max(-1.0, Math.min(1.0, crossTrackError * 3.0));

            double signX = Math.signum(unitDrive.getX());
            if (Math.abs(signX) < 0.10) signX = targetPose.getX() > currentPose.getX() ? 1.0 : -1.0;
            vx = signX * Math.sqrt(Math.max(0.0, currentCommandedSpeed * currentCommandedSpeed - vy * vy));

            Intake.getInstance().setArmPosition(Constants.INTAKE_HORIZONTAL_POSITION);
            allowDynamicAvoidance = false;
        } else if (inTrench && !targetInTrench) {
            // Robot is exiting the trench into open carpet: allow normal vy movement!
            Intake.getInstance().setArmPosition(Constants.INTAKE_HORIZONTAL_POSITION);
            allowDynamicAvoidance = false;
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
        return Timer.getFPGATimestamp() < unstickEndTime;
    }
}