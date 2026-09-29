package frc.robot.Navigation;

import java.util.ArrayList;
import java.util.Collections;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;

import org.littletonrobotics.junction.Logger;

/**
 * DynamicRouter provides real-time local obstacle avoidance around moving opponents.
 * Artificial Potential Fields (APF) only: instantaneous vector blending (\<0.5ms)
 * with proprioceptive stall scaling (2.2x), field wall cushions, and
 * {@link #isZoneBlocked} queries for trench edge-masking in {@link StaticPathfinder}.
 */
public class DynamicRouter {

    public enum AvoidanceAlgorithm {
        POTENTIAL_FIELDS
    }

    private static AvoidanceAlgorithm activeAlgorithm = AvoidanceAlgorithm.POTENTIAL_FIELDS;

    private static final List<DynamicObstacle> activeObstacles = new ArrayList<>();
    /** Radius within which a peer exerts repulsion. Public so tests can pin the curve. */
    public static final double SAFE_DISTANCE_METERS = 1.40;
    private static final double REPULSION_STRENGTH = 2.50;
    private static final double ROBOT_RADIUS_METERS = 0.45; // Half of chassis width with bumpers

    /**
     * When repulsion exceeds the nominal command, this fraction of the nominal speed
     * is still commanded forward. Guarantees the robot keeps making progress instead
     * of cancelling to a crawl inside the stall watchdogs' blind band.
     */
    public static final double MIN_FORWARD_FRACTION = 0.35;

    /**
     * The tangential slide is a fraction of the repulsion magnitude, not a flat
     * constant, so a stronger repulsion still produces a stronger escape. A flat cap
     * saturated both the normal and proprioceptive cases to the same value and erased
     * the distinction {@code testProprioceptiveBumperRepulsionBoost} asserts.
     */
    public static final double LATERAL_FRACTION_OF_REPULSION = 0.50;

    /**
     * Absolute ceiling on the tangential slide, so the escape stays a local
     * correction around the peer rather than a detour that abandons the target.
     * {@link #Constants#MAX_SPEED} still clamps the total.
     */
    public static final double MAX_LATERAL_MPS = 2.50;

    /**
     * Distance floor for the inverse-distance repulsion.
     *
     * <p>The textbook form {@code k * (1/d - 1/d0)} has a derivative that diverges as
     * {@code d -> 0}. With the previous 0.05 m guard the normal term reached
     * {@code 2.5 * (1/0.05 - 1/1.4) ~= 48 m/s} before the {@code MAX_SPEED} clamp
     * swallowed it, so the commanded direction became a near-step function of
     * position right at contact — the controller saturated and behaved
     * discontinuously when a robot was pressed against a peer. This is the blow-up
     * discussed in arXiv:2402.11601 (subharmonic potential fields).
     *
     * <p>Raising the floor to {@link #REPULSION_MIN_DISTANCE_M} bounds the term at
     * {@code 2.5 * (1/0.35 - 1/1.4) ~= 5.4 m/s} while leaving the whole
     * behaviourally-relevant band (d &gt; 0.35 m, i.e. every approach before actual
     * contact) bit-identical to the old curve. The proprioceptive 2.2x boost still
     * orders correctly because it scales the whole falloff.
     */
    public static final double REPULSION_MIN_DISTANCE_M = 0.35;

    /**
     * Inverse-distance repulsion with a bounded derivative. Zero at
     * {@link #SAFE_DISTANCE_METERS}, and saturating rather than diverging as the gap
     * closes. Monotonically increasing in {@code distanceToEdge}.
     */
    static double repulsiveFalloff(double distanceToEdge) {
        if (distanceToEdge >= SAFE_DISTANCE_METERS) {
            return 0.0;
        }
        double d = Math.max(distanceToEdge, REPULSION_MIN_DISTANCE_M);
        return 1.0 / d - 1.0 / SAFE_DISTANCE_METERS;
    }

    /**
     * Distance floor for the perimeter-wall term. Scaled to the wall margin
     * ({@code WALL_SAFETY_MARGIN_METERS} = 0.65 m) rather than reusing the peer floor,
     * which is tuned against a 1.40 m influence radius. Bounds the wall term at
     * {@code 2.0 * (1/0.20 - 1/0.65) ~= 6.9 m/s} instead of the ~47 m/s the previous
     * 0.04 m guard allowed.
     */
    public static final double WALL_MIN_DISTANCE_M = 0.20;

    /**
     * Inverse-distance wall repulsion with a bounded derivative. Zero at
     * {@code margin}, saturating as the gap closes.
     */
    static double wallFalloff(double distanceToEdge, double margin) {
        if (distanceToEdge >= margin) {
            return 0.0;
        }
        double d = Math.max(distanceToEdge, WALL_MIN_DISTANCE_M);
        return 1.0 / d - 1.0 / margin;
    }

    // Static Field Constants (Consolidated via FieldMap)
    public static final double FIELD_LENGTH_METERS = FieldMap.FIELD_LENGTH;
    public static final double FIELD_WIDTH_METERS = FieldMap.FIELD_WIDTH;
    public static final double WALL_SAFETY_MARGIN_METERS = FieldMap.WALL_SAFETY_MARGIN;

    public static final Translation2d BLUE_HUB_CENTER = FieldMap.Hubs.BLUE_HUB_2D;
    public static final Translation2d RED_HUB_CENTER = FieldMap.Hubs.RED_HUB_2D;
    public static final double HUB_RADIUS = 1.10;

    public static final Translation2d BLUE_CLIMB_POLE = FieldMap.ClimbingTowers.BLUE_TOWER_POLE;
    public static final Translation2d RED_CLIMB_POLE = FieldMap.ClimbingTowers.RED_TOWER_POLE;
    public static final double POLE_RADIUS = FieldMap.ClimbingTowers.POLE_RADIUS;

    public static synchronized void setAlgorithm(AvoidanceAlgorithm algorithm) {
        activeAlgorithm = algorithm;
        SmartDashboard.putString("DynamicAvoidance/Algorithm", algorithm.name());
        Logger.recordOutput("DynamicAvoidance/ActiveAlgorithm", algorithm.name());
    }

    public static synchronized AvoidanceAlgorithm getAlgorithm() {
        return activeAlgorithm;
    }

    public static synchronized void registerObstacle(Translation2d pos, Translation2d vel, double radius, double durationSec, boolean isProprioceptive) {
        if (pos == null) return;
        // Single clock with DynamicObstacle expiry + all query paths below.
        double now = Timer.getTimestamp();

        // Prune expired
        activeObstacles.removeIf(obs -> obs.isExpired(now));

        // Spatial debouncing: update if close to existing obstacle
        for (int i = 0; i < activeObstacles.size(); i++) {
            DynamicObstacle existing = activeObstacles.get(i);
            if (existing.position.getDistance(pos) < 0.50) {
                activeObstacles.remove(i);
                break;
            }
        }

        activeObstacles.add(new DynamicObstacle(pos, vel, radius, durationSec, isProprioceptive));
    }

    public static synchronized void registerObstacle(Translation2d pos, Translation2d vel, double radius, double durationSec) {
        registerObstacle(pos, vel, radius, durationSec, false);
    }

    public static synchronized void registerObstacle(Translation2d pos, Translation2d vel, double radius) {
        registerObstacle(pos, vel, radius, 0.40, false);
    }

    public static synchronized void clearObstacles() {
        activeObstacles.clear();
    }

    public static synchronized List<DynamicObstacle> getActiveObstacles() {
        double now = Timer.getTimestamp();
        activeObstacles.removeIf(obs -> obs.isExpired(now));
        return Collections.unmodifiableList(new ArrayList<>(activeObstacles));
    }

    /**
     * Checks if any active dynamic obstacle is currently located within a given bounding box.
     *
     * @param xMin Minimum X coordinate in meters
     * @param xMax Maximum X coordinate in meters
     * @param yMin Minimum Y coordinate in meters
     * @param yMax Maximum Y coordinate in meters
     * @return True if an unexpired obstacle intersects the bounding box
     */
    public static synchronized boolean isZoneBlocked(double xMin, double xMax, double yMin, double yMax) {
        double now = Timer.getTimestamp();
        activeObstacles.removeIf(obs -> obs.isExpired(now));
        for (DynamicObstacle obs : activeObstacles) {
            double ox = obs.position.getX();
            double oy = obs.position.getY();
            double r = obs.radius;
            if (ox + r >= xMin && ox - r <= xMax && oy + r >= yMin && oy - r <= yMax) {
                return true;
            }
        }
        return false;
    }

    /**
     * Computes collision-free avoidance speeds using the currently active algorithm.
     *
     * @param currentPose Current robot pose on the field
     * @param nominalSpeeds Desired nominal speeds from path tracker
     * @param targetWaypoint Target lookahead waypoint in field coordinates
     * @return Blended or re-routed field-oriented ChassisSpeeds
     */
    public static synchronized ChassisSpeeds computeAvoidanceSpeeds(
            Pose2d currentPose,
            ChassisSpeeds nominalSpeeds,
            Translation2d targetWaypoint) {

        double now = Timer.getTimestamp();
        activeObstacles.removeIf(obs -> obs.isExpired(now));

        // Publish active obstacles to AdvantageScope
        Pose2d[] obstaclePoses = new Pose2d[activeObstacles.size()];
        for (int i = 0; i < activeObstacles.size(); i++) {
            obstaclePoses[i] = activeObstacles.get(i).toPose2d();
        }
        Logger.recordOutput("DynamicAvoidance/Obstacles", obstaclePoses);

        if (activeObstacles.isEmpty()) {
            return nominalSpeeds;
        }

        // Phase 2: APF-only. Legacy dashboard algorithm keys resolve to POTENTIAL_FIELDS.
        String dashAlgo = SmartDashboard.getString("DynamicAvoidance/Algorithm", activeAlgorithm.name());
        try {
            activeAlgorithm = AvoidanceAlgorithm.valueOf(dashAlgo);
        } catch (IllegalArgumentException ignored) {
        }

        ChassisSpeeds resultSpeeds = computePotentialFields(currentPose, nominalSpeeds);

        Logger.recordOutput("DynamicAvoidance/AvoidanceVx", resultSpeeds.vxMetersPerSecond);
        Logger.recordOutput("DynamicAvoidance/AvoidanceVy", resultSpeeds.vyMetersPerSecond);
        return resultSpeeds;
    }

    // =========================================================================
    // APPROACH A: Artificial Potential Fields (APF)
    // =========================================================================

    private static ChassisSpeeds computePotentialFields(Pose2d currentPose, ChassisSpeeds nominalSpeeds) {
        Translation2d robotPos = currentPose.getTranslation();
        Translation2d repulsiveVector = new Translation2d();

        for (DynamicObstacle obs : activeObstacles) {
            double distanceToEdge = robotPos.getDistance(obs.position) - obs.radius;

            if (distanceToEdge < SAFE_DISTANCE_METERS && distanceToEdge > 0.05) {
                Translation2d away = robotPos.minus(obs.position);
                double norm = away.getNorm();
                Translation2d unitAway = (norm > 1e-4) ? away.div(norm) : new Translation2d(1, 0);

                // Multiply strength if proprioceptive stall
                double kRep = obs.isProprioceptive ? REPULSION_STRENGTH * 2.2 : REPULSION_STRENGTH;
                double forceMagnitude = kRep * repulsiveFalloff(distanceToEdge);
                repulsiveVector = repulsiveVector.plus(unitAway.times(forceMagnitude));
            }
        }

        // Perimeter Wall Repulsion (Bottom Y=0, Top Y=FIELD_WIDTH, Left X=0, Right X=FIELD_LENGTH)
        // Same bounded-derivative treatment as the peer terms; the old 0.04 m floor let
        // a wall term reach ~47 m/s at contact.
        double kWall = 2.0;
        double wallMargin = WALL_SAFETY_MARGIN_METERS;
        if (robotPos.getX() < wallMargin) {
            double d = robotPos.getX() - ROBOT_RADIUS_METERS;
            if (d < wallMargin) {
                repulsiveVector = repulsiveVector.plus(
                        new Translation2d(kWall * wallFalloff(d, wallMargin), 0));
            }
        } else if (robotPos.getX() > (FIELD_LENGTH_METERS - wallMargin)) {
            double d = (FIELD_LENGTH_METERS - ROBOT_RADIUS_METERS) - robotPos.getX();
            if (d < wallMargin) {
                repulsiveVector = repulsiveVector.plus(
                        new Translation2d(-kWall * wallFalloff(d, wallMargin), 0));
            }
        }
        if (robotPos.getY() < wallMargin) {
            double d = robotPos.getY() - ROBOT_RADIUS_METERS;
            if (d < wallMargin) {
                repulsiveVector = repulsiveVector.plus(
                        new Translation2d(0, kWall * wallFalloff(d, wallMargin)));
            }
        } else if (robotPos.getY() > (FIELD_WIDTH_METERS - wallMargin)) {
            double d = (FIELD_WIDTH_METERS - ROBOT_RADIUS_METERS) - robotPos.getY();
            if (d < wallMargin) {
                repulsiveVector = repulsiveVector.plus(
                        new Translation2d(0, -kWall * wallFalloff(d, wallMargin)));
            }
        }

        double nominalSpeed = Math.hypot(nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond);
        double repulsionMagnitude = repulsiveVector.getNorm();

        double vx;
        double vy;
        if (nominalSpeed > 1e-4 && repulsionMagnitude > nominalSpeed) {
            // Repulsion outweighs the nominal command. A pure sum either reverses
            // the drive (robot commanded away from its target) or cancels to a slow
            // crawl that lands in the stall watchdogs' blind band. Decompose instead:
            // project out the component fighting the drive, keep whatever forward
            // progress survives, and route the rest into a tangential slide so the
            // robot goes around the peer instead of into it.
            Translation2d unitNominal = new Translation2d(
                    nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond)
                    .div(nominalSpeed);
            double along = repulsiveVector.dot(unitNominal);
            Translation2d lateral = new Translation2d(-unitNominal.getY(), unitNominal.getX());

            // Perpendicular sign of the repulsion picks which way to slide. Both
            // robots flanking the path produce two symmetric lateral terms that
            // cancel, so fall back to the robot's own heading parity to break the tie
            // deterministically and desynchronise mirrored pairs.
            double lateralSign = Math.signum(lateral.dot(repulsiveVector));
            if (lateralSign == 0.0) {
                lateralSign = (Math.floor(Math.abs(robotPos.getY()) / 2.0) % 2 == 0) ? 1.0 : -1.0;
            }

            // Keep at least MIN_FORWARD_FRACTION of the nominal command so the robot
            // never stalls in place, and cap the lateral slide so it stays a
            // correction rather than a detour.
            double forward = Math.max(MIN_FORWARD_FRACTION * nominalSpeed, nominalSpeed + along);
            double sideways = Math.min(repulsionMagnitude * LATERAL_FRACTION_OF_REPULSION, MAX_LATERAL_MPS);

            vx = unitNominal.getX() * forward + lateral.getX() * sideways * lateralSign;
            vy = unitNominal.getY() * forward + lateral.getY() * sideways * lateralSign;

            Logger.recordOutput("DynamicAvoidance/RepulsionOverrodeNominal", true);
            Logger.recordOutput("DynamicAvoidance/ForwardRetained", forward);
        } else {
            vx = nominalSpeeds.vxMetersPerSecond + repulsiveVector.getX();
            vy = nominalSpeeds.vyMetersPerSecond + repulsiveVector.getY();
            Logger.recordOutput("DynamicAvoidance/RepulsionOverrodeNominal", false);
        }

        // Speed clamping
        double speed = Math.hypot(vx, vy);
        double maxSpeed = Constants.MAX_SPEED;
        if (speed > maxSpeed) {
            double scale = maxSpeed / speed;
            vx *= scale;
            vy *= scale;
        }

        return new ChassisSpeeds(vx, vy, nominalSpeeds.omegaRadiansPerSecond);
    }
}
