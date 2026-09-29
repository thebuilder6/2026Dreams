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
     * Asymptotic fraction of the nominal speed still commanded forward when repulsion
     * opposes the drive completely. Replaces the previous hard
     * {@code MIN_FORWARD_FRACTION} clamp, whose threshold put a kink in the command
     * as a peer crossed it. The retained forward term is now
     * {@code nominal - saturation * (1 - exp(-backPressure / saturation))}, so it
     * approaches this value smoothly and monotonically instead of hitting it at a
     * cliff. Guarantees the robot keeps making progress rather than cancelling into
     * the stall watchdogs' blind band.
     */
    public static final double MAX_BACKPRESSURE_FRACTION = 0.65;

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
        return isZoneBlocked(xMin, xMax, yMin, yMax, null, 0.0);
    }

    /**
     * Zone-occupancy test that can ignore the querying robot's own registered
     * obstacle.
     *
     * <p>Every sim robot registers its own pose as a dynamic obstacle (see
     * {@code AIRobotInstance} step 5 and {@code AIRobotSim}'s Bot 0 block), and
     * the exclusion is not free: with a plain overlap test, a robot standing
     * <i>inside</i> a corridor satisfies the test that is supposed to detect a
     * <i>peer</i> in it, and the planner then refuses the passage the robot is
     * physically sitting in. That is the trench jitter loop -- enter, lose the
     * route, reverse out, re-enter.
     *
     * @param excludePoint   a point whose nearby obstacles are ignored (the
     *                       requesting robot's own pose), or {@code null}
     * @param excludeRadius  obstacles within this distance of {@code excludePoint}
     *                       are skipped
     * @return true if a non-excluded unexpired obstacle intersects the box
     */
    public static synchronized boolean isZoneBlocked(
            double xMin, double xMax, double yMin, double yMax,
            Translation2d excludePoint, double excludeRadius) {
        double now = Timer.getTimestamp();
        activeObstacles.removeIf(obs -> obs.isExpired(now));
        for (DynamicObstacle obs : activeObstacles) {
            // Ego exclusion. Scoped to a radius rather than blanket, so a real
            // peer inside the same corridor still masks it.
            if (excludePoint != null
                    && obs.position.getDistance(excludePoint) <= excludeRadius) {
                continue;
            }
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
        double forwardRetained;
        if (nominalSpeed > 1e-4) {
            // Decompose rather than sum. A pure sum either reverses the drive (robot
            // commanded away from its target) or cancels to a slow crawl that lands in
            // the stall watchdogs' blind band. Instead: project out the component
            // fighting the drive, saturate that back-pressure instead of flooring it,
            // and route the surplus into a tangential slide so the robot goes around the
            // peer rather than into it.
            Translation2d unitNominal = new Translation2d(
                    nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond)
                    .div(nominalSpeed);
            double along = repulsiveVector.dot(unitNominal);
            Translation2d lateral = new Translation2d(-unitNominal.getY(), unitNominal.getX());

            // Perpendicular part of the repulsion. This must be carried through
            // explicitly: a peer abeam the robot produces almost no component along the
            // drive, so treating only the opposing component would leave a robot with a
            // peer to its side driving straight past it. The plain sum handled that
            // case implicitly, and dropping it was a regression caught by
            // RepulsionBoundsTest.farPeerStillProducesAvoidance.
            Translation2d perpRepulsion = repulsiveVector.minus(unitNominal.times(along));
            double perpMagnitude = perpRepulsion.getNorm();

            // Direction of the slide: follow the perpendicular repulsion when there is
            // any, so a peer to the side steers around rather than being ignored. When
            // the peer is dead ahead the perpendicular part nearly vanishes and two
            // symmetric terms can cancel, so fall back to heading parity to break the
            // tie deterministically and desynchronise mirrored pairs.
            Translation2d slideDir;
            if (perpMagnitude > 1e-4) {
                slideDir = perpRepulsion.div(perpMagnitude);
            } else {
                double sign = (Math.floor(Math.abs(robotPos.getY()) / 2.0) % 2 == 0) ? 1.0 : -1.0;
                slideDir = lateral.times(sign);
            }

            // Continuous relaxation instead of a hard forward floor. The old form was
            // max(0.35*nominal, nominal + along): a clamp, so the command had a kink at
            // the threshold and behaviour switched discontinuously as a peer crossed it.
            // Saturating the opposing component reaches the same floor as its asymptote
            // while staying smooth and monotone for every input -- the relaxed-decay
            // form from the CLF/CBF literature (arXiv:2211.11348, 2507.14700), where
            // the progress constraint degrades continuously as the safety constraint
            // tightens instead of hitting a cliff.
            double backPressure = -Math.min(0.0, along); // >= 0, grows as repulsion opposes
            double saturation = MAX_BACKPRESSURE_FRACTION * nominalSpeed;
            double forward = nominalSpeed
                    - saturation * (1.0 - Math.exp(-backPressure / saturation));
            forwardRetained = forward;

            // Sideways correction: the perpendicular repulsion itself, plus a slide
            // proportional to how hard the peer is fighting the drive, capped so it
            // stays a local correction rather than a detour.
            double sideways = Math.min(
                    perpMagnitude + backPressure * LATERAL_FRACTION_OF_REPULSION,
                    MAX_LATERAL_MPS);

            vx = unitNominal.getX() * forward + slideDir.getX() * sideways;
            vy = unitNominal.getY() * forward + slideDir.getY() * sideways;
        } else {
            vx = nominalSpeeds.vxMetersPerSecond + repulsiveVector.getX();
            vy = nominalSpeeds.vyMetersPerSecond + repulsiveVector.getY();
            forwardRetained = 0.0;
        }

        Logger.recordOutput("DynamicAvoidance/ForwardRetained", forwardRetained);
        Logger.recordOutput("DynamicAvoidance/RepulsionMagnitude", repulsionMagnitude);

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
