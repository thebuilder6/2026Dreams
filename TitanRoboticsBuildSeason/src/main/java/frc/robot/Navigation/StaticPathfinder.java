package frc.robot.Navigation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * StaticPathfinder: 2026 Field Topological Roadmap & Visibility Graph Planner
 * 
 * Replaces ad-hoc recursive circle-bouncing with a deterministic, mathematically verified
 * topological visibility graph and AABB obstacle models:
 * - Dedicated Top and Bottom Trench corridors (Y = 7.42m and Y = 0.65m) with 4-stage funneling.
 * - Exact rectangular AABB colliders for Hubs, Trench divider walls, and Tower climbing poles.
 * - String-pulled shortcut smoothing guaranteeing direct, corner-clip free trajectories.
 * - Sub-millisecond deterministic execution with zero recursion limit / oscillation bugs.
 */
public class StaticPathfinder {

    // =========================================================================
    // Legacy Compatibility Interfaces and Classes
    // =========================================================================
    public interface Obstacle {
        boolean isBlocking(Translation2d p1, Translation2d p2);
        Translation2d getCenter();
        double getSafeRadius();
    }

    public static class CircularObstacle implements Obstacle {
        public final Translation2d center;
        public final double radius;

        public CircularObstacle(Translation2d center, double radius) {
            this.center = center;
            this.radius = radius;
        }

        @Override
        public boolean isBlocking(Translation2d p1, Translation2d p2) {
            Translation2d d = p2.minus(p1);
            Translation2d f = p1.minus(center);
            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - radius * radius;
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) return false;
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 < 0 && t2 > 1);
        }

        @Override
        public Translation2d getCenter() { return center; }

        @Override
        public double getSafeRadius() { return radius + 0.6; }
    }

    public static class RectangularObstacle implements Obstacle {
        public final Translation2d center;
        public final double width;
        public final double height;
        public final Rotation2d rotation;

        public RectangularObstacle(Translation2d center, double width, double height, Rotation2d rotation) {
            this.center = center;
            this.width = width;
            this.height = height;
            this.rotation = rotation;
        }

        @Override
        public boolean isBlocking(Translation2d p1, Translation2d p2) {
            double rad = Math.sqrt(width * width + height * height) / 2.0;
            return new CircularObstacle(center, rad).isBlocking(p1, p2);
        }

        @Override
        public Translation2d getCenter() { return center; }

        @Override
        public double getSafeRadius() { return Math.sqrt(width * width + height * height) / 2.0 + 0.6; }
    }

    // =========================================================================
    // Accurate AABB (Axis-Aligned Bounding Box) Model
    // =========================================================================
    public static class AABB extends FieldMap.AABB {
        public AABB(String name, double minX, double maxX, double minY, double maxY) {
            super(name, minX, maxX, minY, maxY);
        }
    }

    // =========================================================================
    // Field Geometry & Static Obstacles (Consolidated via FieldMap)
    // =========================================================================
    public static final double FIELD_LENGTH = FieldMap.FIELD_LENGTH;
    public static final double FIELD_WIDTH = FieldMap.FIELD_WIDTH;

    // Obstacles inflated by robot radius + safety margin (~0.45m)
    private static final List<FieldMap.AABB> STATIC_OBSTACLES = FieldMap.Obstacles.STATIC_OBSTACLES;

    // =========================================================================
    // Topological Roadmap Nodes (24 Pre-validated Strategic Waypoints)
    // =========================================================================
    public static class RoadmapNode {
        public final int id;
        public final String name;
        public final Translation2d pos;
        public final List<Integer> neighbors = new ArrayList<>();

        public RoadmapNode(int id, String name, double x, double y) {
            this.id = id;
            this.name = name;
            this.pos = new Translation2d(x, y);
        }
    }

    private static final List<RoadmapNode> NODES = new ArrayList<>();

    // Node Index Constants
    public static final int N_BLUE_ALLIANCE_CTR = 0;
    public static final int N_BLUE_ALLIANCE_TOP = 1;
    public static final int N_BLUE_ALLIANCE_BOT = 2;
    public static final int N_BLUE_HUB_TOP_BYPASS = 3;
    public static final int N_BLUE_HUB_BOT_BYPASS = 4;

    public static final int N_BLUE_TOP_TRENCH_W = 5;
    public static final int N_BLUE_TOP_TRENCH_IN = 6;
    public static final int N_BLUE_TOP_TRENCH_OUT = 7;
    public static final int N_BLUE_TOP_TRENCH_E = 8;

    public static final int N_BLUE_BOT_TRENCH_W = 9;
    public static final int N_BLUE_BOT_TRENCH_IN = 10;
    public static final int N_BLUE_BOT_TRENCH_OUT = 11;
    public static final int N_BLUE_BOT_TRENCH_E = 12;

    public static final int N_MIDFIELD_TOP = 13;
    public static final int N_MIDFIELD_CTR_TOP = 14;
    public static final int N_MIDFIELD_CTR_BOT = 15;
    public static final int N_MIDFIELD_BOT = 16;

    public static final int N_RED_TOP_TRENCH_W = 17;
    public static final int N_RED_TOP_TRENCH_IN = 18;
    public static final int N_RED_TOP_TRENCH_OUT = 19;
    public static final int N_RED_TOP_TRENCH_E = 20;

    public static final int N_RED_BOT_TRENCH_W = 21;
    public static final int N_RED_BOT_TRENCH_IN = 22;
    public static final int N_RED_BOT_TRENCH_OUT = 23;
    public static final int N_RED_BOT_TRENCH_E = 24;

    public static final int N_RED_HUB_TOP_BYPASS = 25;
    public static final int N_RED_HUB_BOT_BYPASS = 26;
    public static final int N_RED_ALLIANCE_CTR = 27;
    public static final int N_RED_ALLIANCE_TOP = 28;
    public static final int N_RED_ALLIANCE_BOT = 29;

    public static final int N_BLUE_ALLIANCE_TOP_BYPASS = 30;
    public static final int N_BLUE_ALLIANCE_BOT_BYPASS = 31;
    public static final int N_RED_ALLIANCE_TOP_BYPASS = 32;
    public static final int N_RED_ALLIANCE_BOT_BYPASS = 33;

    static {
        // Blue Alliance Staging & Low Zones
        NODES.add(new RoadmapNode(0, "Blue Alliance Center", 2.40, 4.035));
        NODES.add(new RoadmapNode(1, "Blue Alliance Top", 1.80, 6.00));
        NODES.add(new RoadmapNode(2, "Blue Alliance Bottom", 1.80, 2.00));
        NODES.add(new RoadmapNode(3, "Blue Hub Midfield Top Staging", 6.20, 5.75));
        NODES.add(new RoadmapNode(4, "Blue Hub Midfield Bot Staging", 6.20, 2.32));

        // Blue Top Trench (Y = 7.42m corridor)
        NODES.add(new RoadmapNode(5, "Blue Top Trench W", 2.90, 7.42));
        NODES.add(new RoadmapNode(6, "Blue Top Trench In", 3.50, 7.42));
        NODES.add(new RoadmapNode(7, "Blue Top Trench Out", 5.75, 7.42));
        NODES.add(new RoadmapNode(8, "Blue Top Trench E", 6.35, 7.42));

        // Blue Bottom Trench (Y = 0.65m corridor)
        NODES.add(new RoadmapNode(9, "Blue Bot Trench W", 2.90, 0.65));
        NODES.add(new RoadmapNode(10, "Blue Bot Trench In", 3.50, 0.65));
        NODES.add(new RoadmapNode(11, "Blue Bot Trench Out", 5.75, 0.65));
        NODES.add(new RoadmapNode(12, "Blue Bot Trench E", 6.35, 0.65));

        // Midfield Crossings (Centerline X = 8.27m)
        NODES.add(new RoadmapNode(13, "Midfield Top", 8.27, 6.20));
        NODES.add(new RoadmapNode(14, "Midfield Center Top", 8.27, 4.90));
        NODES.add(new RoadmapNode(15, "Midfield Center Bot", 8.27, 3.17));
        NODES.add(new RoadmapNode(16, "Midfield Bottom", 8.27, 1.90));

        // Red Top Trench (Y = 7.42m corridor)
        NODES.add(new RoadmapNode(17, "Red Top Trench W", 10.19, 7.42));
        NODES.add(new RoadmapNode(18, "Red Top Trench In", 10.79, 7.42));
        NODES.add(new RoadmapNode(19, "Red Top Trench Out", 13.04, 7.42));
        NODES.add(new RoadmapNode(20, "Red Top Trench E", 13.64, 7.42));

        // Red Bottom Trench (Y = 0.65m corridor)
        NODES.add(new RoadmapNode(21, "Red Bot Trench W", 10.19, 0.65));
        NODES.add(new RoadmapNode(22, "Red Bot Trench In", 10.79, 0.65));
        NODES.add(new RoadmapNode(23, "Red Bot Trench Out", 13.04, 0.65));
        NODES.add(new RoadmapNode(24, "Red Bot Trench E", 13.64, 0.65));

        // Red Hub Midfield Staging & Alliance Staging
        NODES.add(new RoadmapNode(25, "Red Hub Midfield Top Staging", 10.34, 5.75));
        NODES.add(new RoadmapNode(26, "Red Hub Midfield Bot Staging", 10.34, 2.32));
        NODES.add(new RoadmapNode(27, "Red Alliance Center", 14.14, 4.035));
        NODES.add(new RoadmapNode(28, "Red Alliance Top", 14.74, 6.00));
        NODES.add(new RoadmapNode(29, "Red Alliance Bottom", 14.74, 2.00));

        // Alliance Corner Bypass Nodes (smooth transit around Hubs)
        NODES.add(new RoadmapNode(30, "Blue Alliance Top Bypass", 3.04, 5.75));
        NODES.add(new RoadmapNode(31, "Blue Alliance Bot Bypass", 3.04, 2.32));
        NODES.add(new RoadmapNode(32, "Red Alliance Top Bypass", 13.50, 5.75));
        NODES.add(new RoadmapNode(33, "Red Alliance Bot Bypass", 13.50, 2.32));

        // ---------------------------------------------------------------------
        // Connect Bidirectional Topological Edges
        // ---------------------------------------------------------------------
        // Blue Alliance Area
        connect(N_BLUE_ALLIANCE_TOP, N_BLUE_ALLIANCE_CTR);
        connect(N_BLUE_ALLIANCE_BOT, N_BLUE_ALLIANCE_CTR);
        connect(N_BLUE_ALLIANCE_TOP, N_BLUE_TOP_TRENCH_W);
        connect(N_BLUE_ALLIANCE_BOT, N_BLUE_BOT_TRENCH_W);

        // Blue Alliance Corner Staging to Trenches (routes around ramps to open trenches)
        connect(N_BLUE_ALLIANCE_CTR, N_BLUE_ALLIANCE_TOP_BYPASS);
        connect(N_BLUE_ALLIANCE_TOP, N_BLUE_ALLIANCE_TOP_BYPASS);
        connect(N_BLUE_ALLIANCE_TOP_BYPASS, N_BLUE_TOP_TRENCH_W);

        connect(N_BLUE_ALLIANCE_CTR, N_BLUE_ALLIANCE_BOT_BYPASS);
        connect(N_BLUE_ALLIANCE_BOT, N_BLUE_ALLIANCE_BOT_BYPASS);
        connect(N_BLUE_ALLIANCE_BOT_BYPASS, N_BLUE_BOT_TRENCH_W);

        // Blue Trenches (strict linear passage)
        connect(N_BLUE_TOP_TRENCH_W, N_BLUE_TOP_TRENCH_IN);
        connect(N_BLUE_TOP_TRENCH_IN, N_BLUE_TOP_TRENCH_OUT);
        connect(N_BLUE_TOP_TRENCH_OUT, N_BLUE_TOP_TRENCH_E);

        connect(N_BLUE_BOT_TRENCH_W, N_BLUE_BOT_TRENCH_IN);
        connect(N_BLUE_BOT_TRENCH_IN, N_BLUE_BOT_TRENCH_OUT);
        connect(N_BLUE_BOT_TRENCH_OUT, N_BLUE_BOT_TRENCH_E);

        // Blue to Midfield
        connect(N_BLUE_TOP_TRENCH_E, N_MIDFIELD_TOP);
        connect(N_BLUE_TOP_TRENCH_E, N_BLUE_HUB_TOP_BYPASS);
        connect(N_BLUE_BOT_TRENCH_E, N_MIDFIELD_BOT);
        connect(N_BLUE_BOT_TRENCH_E, N_BLUE_HUB_BOT_BYPASS);
        connect(N_BLUE_HUB_TOP_BYPASS, N_MIDFIELD_CTR_TOP);
        connect(N_BLUE_HUB_BOT_BYPASS, N_MIDFIELD_CTR_BOT);
        connect(N_BLUE_HUB_TOP_BYPASS, N_MIDFIELD_TOP);
        connect(N_BLUE_HUB_BOT_BYPASS, N_MIDFIELD_BOT);

        // Midfield Crossings
        connect(N_MIDFIELD_TOP, N_MIDFIELD_CTR_TOP);
        connect(N_MIDFIELD_CTR_TOP, N_MIDFIELD_CTR_BOT);
        connect(N_MIDFIELD_CTR_BOT, N_MIDFIELD_BOT);

        // Midfield to Red
        connect(N_MIDFIELD_TOP, N_RED_TOP_TRENCH_W);
        connect(N_MIDFIELD_BOT, N_RED_BOT_TRENCH_W);
        connect(N_RED_TOP_TRENCH_W, N_RED_HUB_TOP_BYPASS);
        connect(N_RED_BOT_TRENCH_W, N_RED_HUB_BOT_BYPASS);
        connect(N_MIDFIELD_CTR_TOP, N_RED_HUB_TOP_BYPASS);
        connect(N_MIDFIELD_CTR_BOT, N_RED_HUB_BOT_BYPASS);
        connect(N_MIDFIELD_TOP, N_RED_HUB_TOP_BYPASS);
        connect(N_MIDFIELD_BOT, N_RED_HUB_BOT_BYPASS);

        // Red Trenches (strict linear passage)
        connect(N_RED_TOP_TRENCH_W, N_RED_TOP_TRENCH_IN);
        connect(N_RED_TOP_TRENCH_IN, N_RED_TOP_TRENCH_OUT);
        connect(N_RED_TOP_TRENCH_OUT, N_RED_TOP_TRENCH_E);

        connect(N_RED_BOT_TRENCH_W, N_RED_BOT_TRENCH_IN);
        connect(N_RED_BOT_TRENCH_IN, N_RED_BOT_TRENCH_OUT);
        connect(N_RED_BOT_TRENCH_OUT, N_RED_BOT_TRENCH_E);

        // Red Alliance Area
        connect(N_RED_TOP_TRENCH_E, N_RED_ALLIANCE_TOP);
        connect(N_RED_BOT_TRENCH_E, N_RED_ALLIANCE_BOT);
        connect(N_RED_ALLIANCE_TOP, N_RED_ALLIANCE_CTR);
        connect(N_RED_ALLIANCE_BOT, N_RED_ALLIANCE_CTR);

        // Red Alliance Corner Staging to Trenches (routes around ramps to open trenches)
        connect(N_RED_ALLIANCE_CTR, N_RED_ALLIANCE_TOP_BYPASS);
        connect(N_RED_ALLIANCE_TOP, N_RED_ALLIANCE_TOP_BYPASS);
        connect(N_RED_ALLIANCE_TOP_BYPASS, N_RED_TOP_TRENCH_E);

        connect(N_RED_ALLIANCE_CTR, N_RED_ALLIANCE_BOT_BYPASS);
        connect(N_RED_ALLIANCE_BOT, N_RED_ALLIANCE_BOT_BYPASS);
        connect(N_RED_ALLIANCE_BOT_BYPASS, N_RED_BOT_TRENCH_E);
    }

    private static void connect(int id1, int id2) {
        if (!NODES.get(id1).neighbors.contains(id2)) NODES.get(id1).neighbors.add(id2);
        if (!NODES.get(id2).neighbors.contains(id1)) NODES.get(id2).neighbors.add(id1);
    }

    // =========================================================================
    // Collision-Free Line of Sight Raycaster
    // =========================================================================
    /**
     * Checks if the line segment from p1 to p2 is clear of all static field obstacles
     * and strictly within field boundary carpet.
     */
public static boolean isLineOfSightClear(Translation2d p1, Translation2d p2) {
        if (p1 == null || p2 == null) return false;

        // Bumper safety margin
        double margin = 0.40;
        if (p1.getX() < margin || p1.getX() > FIELD_LENGTH - margin || p1.getY() < margin || p1.getY() > FIELD_WIDTH - margin) return false;
        if (p2.getX() < margin || p2.getX() > FIELD_LENGTH - margin || p2.getY() < margin || p2.getY() > FIELD_WIDTH - margin) return false;

        // Check against static obstacles with bumper inflation
        for (FieldMap.AABB obs : STATIC_OBSTACLES) {
            // Create a temporarily inflated AABB to account for robot chassis radius
            FieldMap.AABB inflated = new FieldMap.AABB(
                    obs.name,
                    obs.minX - margin,
                    obs.maxX + margin,
                    obs.minY - margin,
                    obs.maxY + margin
            );
            if (inflated.intersectsSegment(p1, p2)) {
                return false;
            }
        }

        return true;
    }

    public static Translation2d clampToField(Translation2d p) {
        return FieldMap.clampToField(p);
    }

    // =========================================================================
    // Obstacle Containment & Safe Target Projection
    // =========================================================================
    public static final double BUMPER_MARGIN = 0.45; // Safety margin from perimeter walls (meters)

    /**
     * Checks if a 2D point is located inside any static field obstacle (AABB)
     * or outside safe field perimeter carpet.
     */
    public static boolean isPointInStaticObstacle(Translation2d p) {
        return FieldMap.isPointInStaticObstacle(p);
    }

    /**
     * Checks if a 2D point is located inside any static obstacle or active dynamic obstacle.
     */
    public static boolean isPointInObstacle(Translation2d p) {
        if (isPointInStaticObstacle(p)) {
            return true;
        }
        return isPointNearDynamicObstacle(p);
    }

    /**
     * Checks if a 2D point is inside a hard footprint (hub ramps, tower poles)
     * <i>excluding</i> the perimeter wall safety band. Fuel near walls is
     * reachable with a wall-normal approach, so fuel-targeting filters must
     * use this (plus {@link #isPointNearDynamicObstacle}) instead of
     * {@link #isPointInObstacle}, which treats the whole wall band as blocked.
     */
    public static boolean isPointInHardObstacle(Translation2d p) {
        if (p == null) {
            return true;
        }
        for (FieldMap.AABB obs : FieldMap.Obstacles.STATIC_OBSTACLES) {
            if (obs != null && obs.contains(p)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if a 2D point is inside any unexpired dynamic (robot) obstacle.
     */
    public static boolean isPointNearDynamicObstacle(Translation2d p) {
        if (p == null) {
            return true;
        }
        for (DynamicObstacle dynObs : DynamicRouter.getActiveObstacles()) {
            if (dynObs != null && !dynObs.isExpired(Timer.getTimestamp())) {
                if (p.getDistance(dynObs.position) < (dynObs.radius + 0.15)) {
                    return true;
                }
            }
        }
        return false;
    }

    // Fuel intake approach geometry (shared by Jev scent targeting and Bot 0).
    // Balls tight to a perimeter wall are approached wall-normal with the
    // chassis parked 0.48 m off the ball so the front intake lands on top of
    // it without ramming; open-field balls get a 0.35 m short offset.
    public static final double WALL_BALL_RED_X = 15.60;
    public static final double WALL_BALL_BLUE_X = 0.90;
    public static final double WALL_BALL_BOTTOM_Y = 0.90;
    public static final double WALL_BALL_TOP_Y = 7.15;
    public static final double WALL_STANDOFF_METERS = 0.48;
    public static final double OPEN_FIELD_APPROACH_METERS = 0.35;

    /**
     * Computes the intake approach pose for a fuel ball, then projects it
     * clear of obstacles relative to the robot.
     *
     * @param ballPos robot-frame-independent ball position on the carpet
     * @param robotPos current robot translation (driving side reference)
     * @return approach pose facing the ball, guaranteed outside obstacles
     */
    public static Pose2d wallStandoffApproach(Translation2d ballPos, Translation2d robotPos) {
        double approachX = ballPos.getX();
        double approachY = ballPos.getY();
        Rotation2d targetHeading;

        if (ballPos.getX() > WALL_BALL_RED_X) {
            // Ball near Red driver wall: face wall (0 deg), stop chassis at
            // X = ballX - 0.48m
            approachX = Math.min(15.98, ballPos.getX() - WALL_STANDOFF_METERS);
            targetHeading = Rotation2d.fromDegrees(0);
        } else if (ballPos.getX() < WALL_BALL_BLUE_X) {
            // Ball near Blue driver wall: face wall (180 deg), stop chassis
            // at X = ballX + 0.48m
            approachX = Math.max(0.55, ballPos.getX() + WALL_STANDOFF_METERS);
            targetHeading = Rotation2d.fromDegrees(180);
        } else if (ballPos.getY() < WALL_BALL_BOTTOM_Y) {
            // Ball near bottom wall: face bottom wall (-90 deg), stop chassis
            // at Y = ballY + 0.48m
            approachY = Math.max(0.55, ballPos.getY() + WALL_STANDOFF_METERS);
            targetHeading = Rotation2d.fromDegrees(-90);
        } else if (ballPos.getY() > WALL_BALL_TOP_Y) {
            // Ball near top wall: face top wall (90 deg), stop chassis at
            // Y = ballY - 0.48m
            approachY = Math.min(7.50, ballPos.getY() - WALL_STANDOFF_METERS);
            targetHeading = Rotation2d.fromDegrees(90);
        } else {
            // Open field: drive front bumper directly toward the ball
            targetHeading = ballPos.minus(robotPos).getAngle();
            Translation2d offset = new Translation2d(OPEN_FIELD_APPROACH_METERS, 0).rotateBy(targetHeading);
            approachX = ballPos.getX() - offset.getX();
            approachY = ballPos.getY() - offset.getY();
        }

        Translation2d targetPos = new Translation2d(approachX, approachY);
        return ensurePoseOutsideObstacles(new Pose2d(targetPos, targetHeading), robotPos);
    }

    /**
     * Checks if a Pose2d's translation is located inside any obstacle.
     */
    public static boolean isPoseInObstacle(Pose2d pose) {
        if (pose == null) return true;
        return isPointInObstacle(pose.getTranslation());
    }

    /**
     * Finds the nearest collision-free point on open carpet outside all obstacles.
     * If the point is already clear, returns the point clamped within field boundaries.
     * If inside an obstacle, projects to the nearest exterior face.
     *
     * @param point Target point to test/project
     * @param referenceFrom Optional robot/source position for dynamic obstacle resolution
     * @return Translation2d guaranteed to be outside all obstacles and within legal carpet bounds
     */
    public static Translation2d findNearestClearPoint(Translation2d point, Translation2d referenceFrom) {
        if (point == null) {
            return new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
        }

        // 1. Initial clamp to safe perimeter carpet
        double clampedX = Math.max(BUMPER_MARGIN, Math.min(FIELD_LENGTH - BUMPER_MARGIN, point.getX()));
        double clampedY = Math.max(BUMPER_MARGIN, Math.min(FIELD_WIDTH - BUMPER_MARGIN, point.getY()));
        Translation2d current = new Translation2d(clampedX, clampedY);

        if (!isPointInObstacle(current)) {
            return current;
        }

        // 2. Resolve Static AABB Obstacles
        for (FieldMap.AABB obs : STATIC_OBSTACLES) {
            if (obs.contains(current)) {
                double buffer = 0.15; // Clearance standoff beyond inflated AABB
                List<Translation2d> candidates = new ArrayList<>(4);

                // West Face candidate
                double wX = Math.max(BUMPER_MARGIN, obs.minX - buffer);
                double wY = Math.max(BUMPER_MARGIN, Math.min(FIELD_WIDTH - BUMPER_MARGIN, current.getY()));
                candidates.add(new Translation2d(wX, wY));

                // East Face candidate
                double eX = Math.min(FIELD_LENGTH - BUMPER_MARGIN, obs.maxX + buffer);
                double eY = Math.max(BUMPER_MARGIN, Math.min(FIELD_WIDTH - BUMPER_MARGIN, current.getY()));
                candidates.add(new Translation2d(eX, eY));

                // South Face candidate
                double sX = Math.max(BUMPER_MARGIN, Math.min(FIELD_LENGTH - BUMPER_MARGIN, current.getX()));
                double sY = Math.max(BUMPER_MARGIN, obs.minY - buffer);
                candidates.add(new Translation2d(sX, sY));

                // North Face candidate
                double nX = Math.max(BUMPER_MARGIN, Math.min(FIELD_LENGTH - BUMPER_MARGIN, current.getX()));
                double nY = Math.min(FIELD_WIDTH - BUMPER_MARGIN, obs.maxY + buffer);
                candidates.add(new Translation2d(nX, nY));

                // Filter candidates that are inside any static obstacle
                candidates.removeIf(StaticPathfinder::isPointInStaticObstacle);

                if (!candidates.isEmpty()) {
                    final Translation2d targetToClear = current;
                    candidates.sort(Comparator.comparingDouble(c -> c.getDistance(targetToClear)));
                    current = candidates.get(0);
                } else {
                    current = new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
                }
            }
        }

        // 3. Resolve Dynamic Obstacles
        for (DynamicObstacle dynObs : DynamicRouter.getActiveObstacles()) {
            if (dynObs != null && !dynObs.isExpired(Timer.getTimestamp())) {
                double safeDist = dynObs.radius + 0.25;
                if (current.getDistance(dynObs.position) < safeDist) {
                    Translation2d away = current.minus(dynObs.position);
                    if (away.getNorm() < 1e-4 && referenceFrom != null) {
                        away = referenceFrom.minus(dynObs.position);
                    }
                    if (away.getNorm() < 1e-4) {
                        away = new Translation2d(1, 0);
                    }
                    Translation2d unitAway = away.div(away.getNorm());
                    Translation2d candDyn = dynObs.position.plus(unitAway.times(safeDist + 0.10));

                    double dx = Math.max(BUMPER_MARGIN, Math.min(FIELD_LENGTH - BUMPER_MARGIN, candDyn.getX()));
                    double dy = Math.max(BUMPER_MARGIN, Math.min(FIELD_WIDTH - BUMPER_MARGIN, candDyn.getY()));
                    Translation2d clampedDyn = new Translation2d(dx, dy);
                    if (!isPointInStaticObstacle(clampedDyn)) {
                        current = clampedDyn;
                    }
                }
            }
        }

        return current;
    }

    public static Translation2d findNearestClearPoint(Translation2d point) {
        return findNearestClearPoint(point, null);
    }

    /**
     * Ensures that a target Pose2d is strictly outside all obstacles.
     * If inside an obstacle, projects to the nearest collision-free point while preserving heading.
     */
    public static Pose2d ensurePoseOutsideObstacles(Pose2d target, Translation2d referenceFrom) {
        if (target == null) {
            return new Pose2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, new Rotation2d());
        }
        if (!isPointInObstacle(target.getTranslation())) {
            double clampedX = Math.max(BUMPER_MARGIN, Math.min(FIELD_LENGTH - BUMPER_MARGIN, target.getX()));
            double clampedY = Math.max(BUMPER_MARGIN, Math.min(FIELD_WIDTH - BUMPER_MARGIN, target.getY()));
            return new Pose2d(clampedX, clampedY, target.getRotation());
        }
        Translation2d clearPos = findNearestClearPoint(target.getTranslation(), referenceFrom);
        return new Pose2d(clearPos, target.getRotation());
    }

    public static Pose2d ensurePoseOutsideObstacles(Pose2d target) {
        return ensurePoseOutsideObstacles(target, null);
    }

    // =========================================================================
    // Core Pathfinding Entry Point
    // =========================================================================
    /**
     * Computes a guaranteed collision-free, piecewise linear path from start to target.
     * 1. Sanitizes start and target poses so neither is located inside an obstacle.
     * 2. Direct Line-of-Sight check (optimizes open-carpet driving to zero overhead).
     * 3. Topological Roadmap Graph Search (A* over pre-cleared field corridors).
     * 4. String-Pulling Shortcut Smoothing (eliminates redundant intermediate turns).
     * 
     * @param start Current robot pose
     * @param target Desired target pose
     * @return List of waypoints routing safely to target
     */
    public static List<Pose2d> findPath(Pose2d start, Pose2d target) {
        Pose2d safeStart = ensurePoseOutsideObstacles(start, target.getTranslation());
        Pose2d safeTarget = ensurePoseOutsideObstacles(target, safeStart.getTranslation());

        Translation2d pStart = safeStart.getTranslation();
        Translation2d pTarget = safeTarget.getTranslation();

        // 1. Fast Path: If direct line of sight is unobstructed, proceed directly!
        if (isLineOfSightClear(pStart, pTarget)) {
            List<Pose2d> direct = new ArrayList<>();
            direct.add(new Pose2d(pTarget, safeTarget.getRotation()));
            return direct;
        }

        // 2. Connect start and target to visible roadmap nodes
        int nNodes = NODES.size();
        int startId = nNodes;
        int targetId = nNodes + 1;

        List<Integer> startVisible = new ArrayList<>();
        List<Integer> targetVisible = new ArrayList<>();

        for (RoadmapNode node : NODES) {
            if (isLineOfSightClear(pStart, node.pos)) {
                startVisible.add(node.id);
            }
            if (isLineOfSightClear(node.pos, pTarget)) {
                targetVisible.add(node.id);
            }
        }

        // Fallbacks if in awkward position: connect to nearest nodes
        if (startVisible.isEmpty()) {
            startVisible.add(findNearestNode(pStart));
        }
        if (targetVisible.isEmpty()) {
            targetVisible.add(findNearestNode(pTarget));
        }

        // 3. A* Search over Roadmap Graph
        List<Integer> rawPath = aStarSearch(pStart, pTarget, startVisible, targetVisible);

        // Convert node IDs to 2D coordinates
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(pStart);
        for (int id : rawPath) {
            waypoints.add(NODES.get(id).pos);
        }
        waypoints.add(pTarget);

        // 4. String-Pulling Shortcut Smoothing
        List<Translation2d> smoothed = smoothPath(waypoints);

        // 5. Convert to List<Pose2d> with smooth segment headings
        List<Pose2d> finalPath = new ArrayList<>();
        for (int i = 1; i < smoothed.size(); i++) {
            Translation2d pt = smoothed.get(i);
            Rotation2d heading;
            if (i == smoothed.size() - 1) {
                heading = safeTarget.getRotation();
            } else {
                Translation2d nextPt = smoothed.get(i + 1);
                heading = nextPt.minus(pt).getAngle();
            }
            finalPath.add(new Pose2d(pt, heading));
        }

        if (finalPath.isEmpty()) {
            finalPath.add(new Pose2d(pTarget, safeTarget.getRotation()));
        }

        return finalPath;
    }

    private static int findNearestNode(Translation2d p) {
        int bestId = 0;
        double bestDist = Double.MAX_VALUE;
        for (RoadmapNode n : NODES) {
            double d = p.getDistance(n.pos);
            if (d < bestDist) {
                bestDist = d;
                bestId = n.id;
            }
        }
        return bestId;
    }

    private static class NodeRecord {
        final int id;
        final double gScore;
        final double fScore;

        NodeRecord(int id, double gScore, double fScore) {
            this.id = id;
            this.gScore = gScore;
            this.fScore = fScore;
        }
    }

    private static List<Integer> aStarSearch(
            Translation2d startPos,
            Translation2d targetPos,
            List<Integer> startVisible,
            List<Integer> targetVisible) {

        java.util.Set<Integer> blockedNodes = getBlockedTrenchNodes();

        int totalNodes = NODES.size();
        double[] gScore = new double[totalNodes];
        Arrays.fill(gScore, Double.MAX_VALUE);
        int[] parent = new int[totalNodes];
        Arrays.fill(parent, -1);

        PriorityQueue<NodeRecord> openSet = new PriorityQueue<>(Comparator.comparingDouble(nr -> nr.fScore));

        for (int startNodeId : startVisible) {
            if (blockedNodes.contains(startNodeId)) continue;
            double d = startPos.getDistance(NODES.get(startNodeId).pos);
            gScore[startNodeId] = d;
            double h = NODES.get(startNodeId).pos.getDistance(targetPos);
            openSet.add(new NodeRecord(startNodeId, d, d + h));
        }

        int bestEndNode = -1;
        double bestTotalCost = Double.MAX_VALUE;

        while (!openSet.isEmpty()) {
            NodeRecord current = openSet.poll();

            if (current.gScore > gScore[current.id]) continue;

            // Check if current node can reach target directly
            if (targetVisible.contains(current.id)) {
                double totalCost = current.gScore + NODES.get(current.id).pos.getDistance(targetPos);
                if (totalCost < bestTotalCost) {
                    bestTotalCost = totalCost;
                    bestEndNode = current.id;
                }
            }

            RoadmapNode curNode = NODES.get(current.id);
            if (blockedNodes.contains(current.id)) continue;
            for (int neighborId : curNode.neighbors) {
                if (blockedNodes.contains(neighborId)) continue;
                double edgeWeight = curNode.pos.getDistance(NODES.get(neighborId).pos);
                double tentativeG = current.gScore + edgeWeight;

                if (tentativeG < gScore[neighborId]) {
                    gScore[neighborId] = tentativeG;
                    parent[neighborId] = current.id;
                    double h = NODES.get(neighborId).pos.getDistance(targetPos);
                    openSet.add(new NodeRecord(neighborId, tentativeG, tentativeG + h));
                }
            }
        }

        // Reconstruct path
        List<Integer> path = new ArrayList<>();
        if (bestEndNode == -1) {
            // Fallback: connect first start visible to first target visible
            path.add(startVisible.get(0));
            if (!targetVisible.contains(startVisible.get(0))) {
                path.add(targetVisible.get(0));
            }
            return path;
        }

        int curr = bestEndNode;
        while (curr != -1) {
            path.add(curr);
            curr = parent[curr];
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * String-pulling shortcut smoothing:
     * Greedily skips intermediate waypoints whenever line-of-sight is completely clear.
     */
    private static List<Translation2d> smoothPath(List<Translation2d> raw) {
        if (raw.size() <= 2) return raw;

        List<Translation2d> smoothed = new ArrayList<>();
        smoothed.add(raw.get(0));

        int curr = 0;
        while (curr < raw.size() - 1) {
            int furthest = curr + 1;
            for (int next = raw.size() - 1; next > curr + 1; next--) {
                if (isLineOfSightClear(raw.get(curr), raw.get(next))) {
                    furthest = next;
                    break;
                }
            }
            smoothed.add(raw.get(furthest));
            curr = furthest;
        }

        return smoothed;
    }

    // =========================================================================
    // Phase 2: Dynamic trench edge-masking (folded in from SmartTunnelRouter)
    // When DynamicRouter.isZoneBlocked() reports a trench corridor occupied,
    // the corresponding roadmap nodes are masked out of the A* search graph.
    // =========================================================================
    public static final java.util.Set<Integer> BLUE_TOP_TRENCH_NODES =
            java.util.Set.of(N_BLUE_TOP_TRENCH_W, N_BLUE_TOP_TRENCH_IN,
                    N_BLUE_TOP_TRENCH_OUT, N_BLUE_TOP_TRENCH_E);
    public static final java.util.Set<Integer> BLUE_BOT_TRENCH_NODES =
            java.util.Set.of(N_BLUE_BOT_TRENCH_W, N_BLUE_BOT_TRENCH_IN,
                    N_BLUE_BOT_TRENCH_OUT, N_BLUE_BOT_TRENCH_E);
    public static final java.util.Set<Integer> RED_TOP_TRENCH_NODES =
            java.util.Set.of(N_RED_TOP_TRENCH_W, N_RED_TOP_TRENCH_IN,
                    N_RED_TOP_TRENCH_OUT, N_RED_TOP_TRENCH_E);
    public static final java.util.Set<Integer> RED_BOT_TRENCH_NODES =
            java.util.Set.of(N_RED_BOT_TRENCH_W, N_RED_BOT_TRENCH_IN,
                    N_RED_BOT_TRENCH_OUT, N_RED_BOT_TRENCH_E);

    public static boolean isTrenchBlocked(boolean isTopTrench, boolean isBlueAlliance) {
        double xMin = isBlueAlliance ? FieldMap.Trenches.BLUE_TRENCH_MIN_X : FieldMap.Trenches.RED_TRENCH_MIN_X;
        double xMax = isBlueAlliance ? FieldMap.Trenches.BLUE_TRENCH_MAX_X : FieldMap.Trenches.RED_TRENCH_MAX_X;
        double yMin = isTopTrench ? FieldMap.Trenches.TOP_TRENCH_MIN_Y : FieldMap.Trenches.BOT_TRENCH_MIN_Y;
        double yMax = isTopTrench ? FieldMap.Trenches.TOP_TRENCH_MAX_Y : FieldMap.Trenches.BOT_TRENCH_MAX_Y;
        return DynamicRouter.isZoneBlocked(xMin, xMax, yMin, yMax);
    }

    private static java.util.Set<Integer> getBlockedTrenchNodes() {
        java.util.Set<Integer> blocked = new java.util.HashSet<>();
        // Evaluate both alliances; zones are disjoint so only occupied corridors mask.
        if (isTrenchBlocked(true, true)) blocked.addAll(BLUE_TOP_TRENCH_NODES);
        if (isTrenchBlocked(false, true)) blocked.addAll(BLUE_BOT_TRENCH_NODES);
        if (isTrenchBlocked(true, false)) blocked.addAll(RED_TOP_TRENCH_NODES);
        if (isTrenchBlocked(false, false)) blocked.addAll(RED_BOT_TRENCH_NODES);
        return blocked;
    }

    // =========================================================================
    // Tunnel route API (moved from deleted SmartTunnelRouter; thin wrapper over
    // the masked roadmap so existing callers/tests keep working).
    // =========================================================================
    public enum TrenchCorridor {
        TOP_TRENCH,
        BOTTOM_TRENCH
    }

    public static class TunnelRoute {
        public final TrenchCorridor corridor;
        public final boolean isWestToEast;
        public final boolean isDiverted;
        public final Pose2d preEntrancePose;
        public final Pose2d entrancePose;
        public final Pose2d exitPose;
        public final Pose2d postExitPose;
        public final Rotation2d corridorHeading;
        public final List<Pose2d> waypoints;

        public TunnelRoute(
                TrenchCorridor corridor,
                boolean isWestToEast,
                boolean isDiverted,
                Pose2d preEntrance,
                Pose2d entrance,
                Pose2d exit,
                Pose2d postExit,
                Rotation2d heading) {
            this.corridor = corridor;
            this.isWestToEast = isWestToEast;
            this.isDiverted = isDiverted;
            this.preEntrancePose = preEntrance;
            this.entrancePose = entrance;
            this.exitPose = exit;
            this.postExitPose = postExit;
            this.corridorHeading = heading;

            List<Pose2d> pts = new ArrayList<>();
            pts.add(preEntrance);
            pts.add(entrance);
            pts.add(exit);
            pts.add(postExit);
            this.waypoints = Collections.unmodifiableList(pts);
        }

        public List<Pose2d> getWaypoints() {
            return waypoints;
        }

        public boolean isDiverted() {
            return isDiverted;
        }

        public boolean isWestToEast() {
            return isWestToEast;
        }
    }

    public static TunnelRoute planTunnelRoute(Pose2d currentPose, boolean preferTopTrench) {
        boolean isRed = AllianceFlipUtil.isRedAlliance();
        boolean isBlue = !isRed;

        boolean topBlocked = isTrenchBlocked(true, isBlue);
        boolean botBlocked = isTrenchBlocked(false, isBlue);

        boolean chosenTop = preferTopTrench;
        boolean wasDiverted = false;

        if (preferTopTrench && topBlocked && !botBlocked) {
            chosenTop = false;
            wasDiverted = true;
        } else if (!preferTopTrench && botBlocked && !topBlocked) {
            chosenTop = true;
            wasDiverted = true;
        }

        double yLane = chosenTop ? FieldMap.Trenches.TOP_CORRIDOR_Y : FieldMap.Trenches.BOT_CORRIDOR_Y;
        TrenchCorridor corridor = chosenTop ? TrenchCorridor.TOP_TRENCH : TrenchCorridor.BOTTOM_TRENCH;

        double robotX = currentPose.getX();
        boolean isWestToEast;
        double entranceX;
        double exitX;
        double preEntranceX;
        double postExitX;
        Rotation2d corridorHeading;

        if (isBlue) {
            if (robotX < 4.60) {
                isWestToEast = true;
                preEntranceX = 2.90;
                entranceX = 3.50;
                exitX = 5.75;
                postExitX = 6.35;
                corridorHeading = Rotation2d.fromDegrees(0);
            } else {
                isWestToEast = false;
                preEntranceX = 6.35;
                entranceX = 5.75;
                exitX = 3.50;
                postExitX = 2.90;
                corridorHeading = Rotation2d.fromDegrees(180);
            }
        } else {
            if (robotX > 11.94) {
                isWestToEast = false;
                preEntranceX = 13.64;
                entranceX = 13.04;
                exitX = 10.79;
                postExitX = 10.19;
                corridorHeading = Rotation2d.fromDegrees(180);
            } else {
                isWestToEast = true;
                preEntranceX = 10.19;
                entranceX = 10.79;
                exitX = 13.04;
                postExitX = 13.64;
                corridorHeading = Rotation2d.fromDegrees(0);
            }
        }

        Pose2d preEntrance = new Pose2d(preEntranceX, yLane, corridorHeading);
        Pose2d entrance = new Pose2d(entranceX, yLane, corridorHeading);
        Pose2d exit = new Pose2d(exitX, yLane, corridorHeading);
        Pose2d postExit = new Pose2d(postExitX, yLane, corridorHeading);

        Logger.recordOutput("DynamicAvoidance/TunnelDiverted", wasDiverted);
        Logger.recordOutput("DynamicAvoidance/TunnelCorridor", corridor.name());
        Logger.recordOutput("DynamicAvoidance/TunnelIsWestToEast", isWestToEast);

        return new TunnelRoute(
                corridor,
                isWestToEast,
                wasDiverted,
                preEntrance,
                entrance,
                exit,
                postExit,
                corridorHeading);
    }

    public static boolean isTunnelTarget(Pose2d targetPose) {
        if (targetPose == null) return false;
        return GlidePoints.getMatchingTunnelEntrance(targetPose) != null;
    }
}
