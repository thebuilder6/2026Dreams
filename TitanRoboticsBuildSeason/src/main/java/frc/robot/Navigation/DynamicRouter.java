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
    private static final double SAFE_DISTANCE_METERS = 1.40; // Zone of repulsive influence
    private static final double REPULSION_STRENGTH = 2.50;
    private static final double ROBOT_RADIUS_METERS = 0.45; // Half of chassis width with bumpers

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
        double now = Timer.getFPGATimestamp();

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
                double forceMagnitude = kRep * (1.0 / distanceToEdge - 1.0 / SAFE_DISTANCE_METERS);
                repulsiveVector = repulsiveVector.plus(unitAway.times(forceMagnitude));
            }
        }

        // Perimeter Wall Repulsion (Bottom Y=0, Top Y=FIELD_WIDTH, Left X=0, Right X=FIELD_LENGTH)
        double kWall = 2.0;
        if (robotPos.getX() < WALL_SAFETY_MARGIN_METERS) {
            double d = Math.max(0.04, robotPos.getX() - ROBOT_RADIUS_METERS);
            if (d < WALL_SAFETY_MARGIN_METERS) {
                repulsiveVector = repulsiveVector.plus(new Translation2d(kWall * (1.0 / d - 1.0 / WALL_SAFETY_MARGIN_METERS), 0));
            }
        } else if (robotPos.getX() > (FIELD_LENGTH_METERS - WALL_SAFETY_MARGIN_METERS)) {
            double d = Math.max(0.04, (FIELD_LENGTH_METERS - ROBOT_RADIUS_METERS) - robotPos.getX());
            if (d < WALL_SAFETY_MARGIN_METERS) {
                repulsiveVector = repulsiveVector.plus(new Translation2d(-kWall * (1.0 / d - 1.0 / WALL_SAFETY_MARGIN_METERS), 0));
            }
        }
        if (robotPos.getY() < WALL_SAFETY_MARGIN_METERS) {
            double d = Math.max(0.04, robotPos.getY() - ROBOT_RADIUS_METERS);
            if (d < WALL_SAFETY_MARGIN_METERS) {
                repulsiveVector = repulsiveVector.plus(new Translation2d(0, kWall * (1.0 / d - 1.0 / WALL_SAFETY_MARGIN_METERS)));
            }
        } else if (robotPos.getY() > (FIELD_WIDTH_METERS - WALL_SAFETY_MARGIN_METERS)) {
            double d = Math.max(0.04, (FIELD_WIDTH_METERS - ROBOT_RADIUS_METERS) - robotPos.getY());
            if (d < WALL_SAFETY_MARGIN_METERS) {
                repulsiveVector = repulsiveVector.plus(new Translation2d(0, -kWall * (1.0 / d - 1.0 / WALL_SAFETY_MARGIN_METERS)));
            }
        }

        double vx = nominalSpeeds.vxMetersPerSecond + repulsiveVector.getX();
        double vy = nominalSpeeds.vyMetersPerSecond + repulsiveVector.getY();

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
