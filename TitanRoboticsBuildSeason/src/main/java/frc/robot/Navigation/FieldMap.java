package frc.robot.Navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Navigation.FieldMap.Obstacles;

/**
 * Unified Field Map for the 2026 Rebuilt FIRST Robotics Competition field.
 * 
 * Single authoritative source of truth for:
 * 1. Field Perimeter & Physical Dimensions (Length, Width, Carpet boundaries)
 * 2. Hub & Goal Locations (3D & 2D centers, funnel aperture, standoff
 * distances)
 * 3. Hub Ramps & Dyn4j Colliders (dimensions, bounds, obstacle inflations)
 * 4. Trench Corridors & Low-Clearance Truss Geofences
 * 5. Alliance Scoring Zones & Midfield Boundaries
 * 6. Human Player Depots & Ball Loading Bays
 * 7. Tower Climbing Poles
 * 8. Static Obstacles (AABB bounding boxes for pathfinding and collision
 * avoidance)
 */
public final class FieldMap {

    private FieldMap() {
    } // Utility class; prevent instantiation

    /**
     * Legacy dashboard/API labels. All physical Hub, ramp, and trench-wall
     * pieces remain impassable in every mode.
     */
    public enum ObstacleHandling {
        IMPASSABLE, PHYSICS, ABSTRACT
    }

    // Prefer a conservative route until the planner models ramp slope and traction.
    private static volatile ObstacleHandling activeHandling = ObstacleHandling.IMPASSABLE;
    private static final String OBSTACLE_HANDLING_KEY = "FieldMap/ObstacleHandling";

    static {
        NetworkTableInstance.getDefault().addListener(
                SmartDashboard.getEntry(OBSTACLE_HANDLING_KEY),
                EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
                event -> {
                    NetworkTableValue value = event.valueData.value;
                    if (value == null || !value.isString())
                        return;
                    try {
                        activeHandling = ObstacleHandling.valueOf(value.getString());
                    } catch (IllegalArgumentException ignored) {
                        // Ignore invalid dashboard input and keep the last valid mode.
                    }
                });
        SmartDashboard.putString(OBSTACLE_HANDLING_KEY, activeHandling.name());
    }

    public static ObstacleHandling getObstacleHandling() {
        return activeHandling;
    }

    public static void setObstacleHandling(ObstacleHandling handling) {
        if (handling != null) {
            activeHandling = handling;
            SmartDashboard.putString(OBSTACLE_HANDLING_KEY, handling.name());
        }
    }

    // =========================================================================
    // 1. Field Perimeter & Carpet Dimensions
    // =========================================================================
    public static final double FIELD_LENGTH = 16.541;
    public static final double FIELD_WIDTH = 8.069;
    public static final double CENTERLINE_X = FIELD_LENGTH / 2.0;
    public static final double ROBOT_RADIUS = 0.45; // Half chassis width including bumpers (~35 in)
    public static final double WALL_SAFETY_MARGIN = 0.65; // Repulsion & clamping buffer

    // =========================================================================
    // 2. Hub & Goal Landmarks
    // =========================================================================
    public static final class Hubs {
        public static final double HUB_WIDTH = 1.1938; // 47.0 inches
        public static final double HUB_RADIUS = HUB_WIDTH / 2.0; // 0.5969m
        public static final double GOAL_HEIGHT = 1.575; // Upper rim height (meters)
        public static final double FUNNEL_TARGET_Z = 1.48; // Detection target inside funnel

        public static final double HUB_Y = 4.0346;
        public static final double BLUE_HUB_X = 4.6256;
        public static final double RED_HUB_X = FIELD_LENGTH - BLUE_HUB_X;
        public static final Translation3d BLUE_HUB_3D = new Translation3d(BLUE_HUB_X, HUB_Y, GOAL_HEIGHT);
        public static final Translation3d RED_HUB_3D = new Translation3d(RED_HUB_X, HUB_Y, GOAL_HEIGHT);

        public static final Translation2d BLUE_HUB_2D = new Translation2d(BLUE_HUB_3D.getX(), BLUE_HUB_3D.getY());
        public static final Translation2d RED_HUB_2D = new Translation2d(RED_HUB_3D.getX(), RED_HUB_3D.getY());

        // Tactical shooting boundaries
        public static final double SHOOTING_MIN_DISTANCE = 1.60; // Clears Hub base frame
        public static final double SHOOTING_MAX_DISTANCE = 4.20; // Maximum reliable flywheel ballistic distance
        public static final double OPTIMAL_STANDOFF_DISTANCE = 2.40; // Sweet spot for accuracy and turnover speed

        public static Translation3d getHubLocation3d(boolean isRed) {
            return isRed ? RED_HUB_3D : BLUE_HUB_3D;
        }

        public static Translation2d getHubLocation2d(boolean isRed) {
            return isRed ? RED_HUB_2D : BLUE_HUB_2D;
        }
    }

    // =========================================================================
    // 3. Hub Ramps
    // =========================================================================
    public static final class Ramps {
        public static final double RAMP_LENGTH_Y = 1.8542;
        public static final double RAMP_WIDTH_X = 1.1938;

        public static final double BLUE_NORTH_RAMP_MIN_X = Hubs.BLUE_HUB_X - RAMP_WIDTH_X / 2.0;
        public static final double BLUE_NORTH_RAMP_MAX_X = Hubs.BLUE_HUB_X + RAMP_WIDTH_X / 2.0;
        public static final double BLUE_NORTH_RAMP_MIN_Y = Hubs.HUB_Y + Hubs.HUB_WIDTH / 2.0;
        public static final double BLUE_NORTH_RAMP_MAX_Y = BLUE_NORTH_RAMP_MIN_Y + RAMP_LENGTH_Y;
        public static final double BLUE_SOUTH_RAMP_MIN_X = BLUE_NORTH_RAMP_MIN_X;
        public static final double BLUE_SOUTH_RAMP_MAX_X = BLUE_NORTH_RAMP_MAX_X;
        public static final double BLUE_SOUTH_RAMP_MAX_Y = Hubs.HUB_Y - Hubs.HUB_WIDTH / 2.0;
        public static final double BLUE_SOUTH_RAMP_MIN_Y = BLUE_SOUTH_RAMP_MAX_Y - RAMP_LENGTH_Y;

        public static final double RED_NORTH_RAMP_MIN_X = Hubs.RED_HUB_X - RAMP_WIDTH_X / 2.0;
        public static final double RED_NORTH_RAMP_MAX_X = Hubs.RED_HUB_X + RAMP_WIDTH_X / 2.0;
        public static final double RED_NORTH_RAMP_MIN_Y = BLUE_NORTH_RAMP_MIN_Y;
        public static final double RED_NORTH_RAMP_MAX_Y = BLUE_NORTH_RAMP_MAX_Y;
        public static final double RED_SOUTH_RAMP_MIN_X = RED_NORTH_RAMP_MIN_X;
        public static final double RED_SOUTH_RAMP_MAX_X = RED_NORTH_RAMP_MAX_X;
        public static final double RED_SOUTH_RAMP_MIN_Y = BLUE_SOUTH_RAMP_MIN_Y;
        public static final double RED_SOUTH_RAMP_MAX_Y = BLUE_SOUTH_RAMP_MAX_Y;

        /**
         * Legacy union bounds across the Hub and its two ramp pieces. Prefer the
         * per-piece bounds above.
         */
        @Deprecated
        public static final double BLUE_RAMP_MIN_X = BLUE_NORTH_RAMP_MIN_X;
        @Deprecated
        public static final double BLUE_RAMP_MAX_X = BLUE_NORTH_RAMP_MAX_X;
        @Deprecated
        public static final double BLUE_RAMP_MIN_Y = BLUE_SOUTH_RAMP_MIN_Y;
        @Deprecated
        public static final double BLUE_RAMP_MAX_Y = BLUE_NORTH_RAMP_MAX_Y;
        @Deprecated
        public static final double RED_RAMP_MIN_X = RED_NORTH_RAMP_MIN_X;
        @Deprecated
        public static final double RED_RAMP_MAX_X = RED_NORTH_RAMP_MAX_X;
        @Deprecated
        public static final double RED_RAMP_MIN_Y = RED_SOUTH_RAMP_MIN_Y;
        @Deprecated
        public static final double RED_RAMP_MAX_Y = RED_NORTH_RAMP_MAX_Y;

        public static boolean isPoseOnRamp(Translation2d point) {
            if (point == null)
                return false;
            double x = point.getX();
            double y = point.getY();
            return inRamp(x, y, BLUE_NORTH_RAMP_MIN_X, BLUE_NORTH_RAMP_MAX_X, BLUE_NORTH_RAMP_MIN_Y,
                    BLUE_NORTH_RAMP_MAX_Y)
                    || inRamp(x, y, BLUE_SOUTH_RAMP_MIN_X, BLUE_SOUTH_RAMP_MAX_X, BLUE_SOUTH_RAMP_MIN_Y,
                            BLUE_SOUTH_RAMP_MAX_Y)
                    || inRamp(x, y, RED_NORTH_RAMP_MIN_X, RED_NORTH_RAMP_MAX_X, RED_NORTH_RAMP_MIN_Y,
                            RED_NORTH_RAMP_MAX_Y)
                    || inRamp(x, y, RED_SOUTH_RAMP_MIN_X, RED_SOUTH_RAMP_MAX_X, RED_SOUTH_RAMP_MIN_Y,
                            RED_SOUTH_RAMP_MAX_Y);
        }

        private static boolean inRamp(double x, double y, double minX, double maxX, double minY, double maxY) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    public static final class TrenchWalls {
        public static final double WALL_X_LEN = 1.1938;
        public static final double WALL_Y_LEN = 0.3048;
        public static final double BLUE_CENTER_X = 4.61769;
        public static final double RED_CENTER_X = FIELD_LENGTH - BLUE_CENTER_X;
        public static final double SOUTH_CENTER_Y = 1.43113;
        public static final double NORTH_CENTER_Y = FIELD_WIDTH - SOUTH_CENTER_Y;
        public static final AABB BLUE_NORTH_WALL = wall("Blue North Trench Wall", BLUE_CENTER_X, NORTH_CENTER_Y);
        public static final AABB BLUE_SOUTH_WALL = wall("Blue South Trench Wall", BLUE_CENTER_X, SOUTH_CENTER_Y);
        public static final AABB RED_NORTH_WALL = wall("Red North Trench Wall", RED_CENTER_X, NORTH_CENTER_Y);
        public static final AABB RED_SOUTH_WALL = wall("Red South Trench Wall", RED_CENTER_X, SOUTH_CENTER_Y);

        private static AABB wall(String name, double centerX, double centerY) {
            return new AABB(name, centerX - WALL_X_LEN / 2.0, centerX + WALL_X_LEN / 2.0,
                    centerY - WALL_Y_LEN / 2.0, centerY + WALL_Y_LEN / 2.0);
        }
    }

    // =========================================================================
    // 4. Trench Corridors & Low-Clearance Truss Zones
    // =========================================================================
    public static final class Trenches {
        // Horizontal spans for the Trench low-overhead steel frame
        public static final double BLUE_TRENCH_MIN_X = 3.20;
        public static final double BLUE_TRENCH_MAX_X = 6.10;
        public static final double RED_TRENCH_MIN_X = 10.44;
        public static final double RED_TRENCH_MAX_X = 13.34;

        // Trench corridor Y lanes (centerlines where robot drives)
        public static final double TOP_CORRIDOR_Y = 7.42;
        public static final double BOT_CORRIDOR_Y = 0.65;

        // Trench physical corridor bounds
        public static final double TOP_TRENCH_MIN_Y = TrenchWalls.NORTH_CENTER_Y + TrenchWalls.WALL_Y_LEN / 2.0;
        public static final double TOP_TRENCH_MAX_Y = FIELD_WIDTH;
        public static final double BOT_TRENCH_MIN_Y = 0.00;
        public static final double BOT_TRENCH_MAX_Y = TrenchWalls.SOUTH_CENTER_Y - TrenchWalls.WALL_Y_LEN / 2.0;

        /**
         * Checks if a pose is located inside a low-overhead Trench ceiling corridor.
         * The intake arm must be stowed low here, and high-arc shooting is blocked.
         */
        public static boolean isLowClearance(Pose2d pose) {
            if (pose == null)
                return false;
            return isLowClearance(pose.getTranslation());
        }

        public static boolean isLowClearance(Translation2d translation) {
            if (translation == null)
                return false;
            double x = translation.getX();
            double y = translation.getY();
            boolean inTrenchX = (x >= BLUE_TRENCH_MIN_X && x <= BLUE_TRENCH_MAX_X)
                    || (x >= RED_TRENCH_MIN_X && x <= RED_TRENCH_MAX_X);
            boolean inTrenchY = (y >= TOP_TRENCH_MIN_Y) || (y <= BOT_TRENCH_MAX_Y);
            return inTrenchX && inTrenchY;
        }
    }

    // =========================================================================
    // 5. Alliance Scoring Zones & Midfield Boundaries
    // =========================================================================
    public static final class AllianceZones {
        // Blue Alliance Zone: strictly between Blue driver wall (X=0) and Blue Hub
        // (X=4.626)
        public static final double BLUE_ZONE_MAX_X = Hubs.BLUE_HUB_X;
        // Red Alliance Zone: strictly between Red Hub (X=11.915) and Red driver wall
        // (X=16.541)
        public static final double RED_ZONE_MIN_X = Hubs.RED_HUB_X;

        // Neutral Midfield carpet: between Blue Hub and Red Hub
        public static final double MIDFIELD_MIN_X = BLUE_ZONE_MAX_X;
        public static final double MIDFIELD_MAX_X = RED_ZONE_MIN_X;

        public static boolean isInAllianceZone(Translation2d translation, boolean isRedAlliance) {
            if (translation == null)
                return false;
            double x = translation.getX();
            if (isRedAlliance) {
                return x >= RED_ZONE_MIN_X && x <= FIELD_LENGTH;
            } else {
                return x >= 0.0 && x <= BLUE_ZONE_MAX_X;
            }
        }

        public static boolean isInAllianceZone(Pose2d pose, boolean isRedAlliance) {
            if (pose == null)
                return false;
            return isInAllianceZone(pose.getTranslation(), isRedAlliance);
        }

        public static boolean isInMidfield(Translation2d translation) {
            if (translation == null)
                return false;
            double x = translation.getX();
            return x >= MIDFIELD_MIN_X && x <= MIDFIELD_MAX_X;
        }
    }

    // =========================================================================
    // 6. Human Player Depots & Ball Loading Bays
    // =========================================================================
    public static final class Depots {
        // Blue Depot (Top-Left inside Blue driver station wall X ~ 0m, Y ~ 5.53m -
        // 6.44m)
        public static final Translation2d BLUE_DEPOT_LOAD_POINT = new Translation2d(0.09, 5.72);
        public static final Translation2d BLUE_DEPOT_APPROACH = new Translation2d(1.50, 6.00);
        public static final Translation2d BLUE_DEPOT_CONTEST = new Translation2d(1.50, 6.50);

        // Red Depot (Bottom-Right inside Red driver station wall X ~ 16.54m, Y ~ 1.65m
        // - 2.56m)
        public static final Translation2d RED_DEPOT_LOAD_POINT = new Translation2d(16.00, 1.88);
        public static final Translation2d RED_DEPOT_APPROACH = new Translation2d(15.04, 2.05);
        public static final Translation2d RED_DEPOT_CONTEST = new Translation2d(15.04, 1.55);

        public static Translation2d getDepotLoadPoint(boolean isRed) {
            return isRed ? RED_DEPOT_LOAD_POINT : BLUE_DEPOT_LOAD_POINT;
        }

        public static Translation2d getDepotApproach(boolean isRed) {
            return isRed ? RED_DEPOT_APPROACH : BLUE_DEPOT_APPROACH;
        }

        public static Translation2d getDepotContest(boolean isRed) {
            return isRed ? RED_DEPOT_CONTEST : BLUE_DEPOT_CONTEST;
        }
    }

    // =========================================================================
    // 7. Climbing Towers & Poles
    // =========================================================================
    public static final class ClimbingTowers {
        public static final Translation2d BLUE_TOWER_POLE = new Translation2d(1.07, 4.04);
        public static final Translation2d RED_TOWER_POLE = new Translation2d(15.47, 4.04);
        public static final double POLE_RADIUS = 0.40;

        public static Translation2d getTowerPole(boolean isRed) {
            return isRed ? RED_TOWER_POLE : BLUE_TOWER_POLE;
        }
    }

    // =========================================================================
    // 8. Axis-Aligned Bounding Box (AABB) & Static Obstacles
    // =========================================================================
    public static class AABB {
        public final String name;
        public final double minX;
        public final double maxX;
        public final double minY;
        public final double maxY;

        public AABB(String name, double minX, double maxX, double minY, double maxY) {
            this.name = name;
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        public boolean contains(Translation2d p) {
            if (p == null)
                return false;
            return contains(p.getX(), p.getY());
        }

        public boolean contains(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }

        public Translation2d getCenter() {
            return new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
        }

        public double getWidth() {
            return maxX - minX;
        }

        public double getHeight() {
            return maxY - minY;
        }

        /**
         * Ultra-fast O(1) Liang-Barsky / Slab ray-box intersection.
         * Returns true if line segment p1 -> p2 intersects this bounding box.
         */
        public boolean intersectsSegment(Translation2d p1, Translation2d p2) {
            double dx = p2.getX() - p1.getX();
            double dy = p2.getY() - p1.getY();

            double tMin = 0.0;
            double tMax = 1.0;

            // X slab
            if (Math.abs(dx) < 1e-9) {
                if (p1.getX() < minX || p1.getX() > maxX)
                    return false;
            } else {
                double t1 = (minX - p1.getX()) / dx;
                double t2 = (maxX - p1.getX()) / dx;
                if (t1 > t2) {
                    double tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }
                tMin = Math.max(tMin, t1);
                tMax = Math.min(tMax, t2);
                if (tMin > tMax)
                    return false;
            }

            // Y slab
            if (Math.abs(dy) < 1e-9) {
                if (p1.getY() < minY || p1.getY() > maxY)
                    return false;
            } else {
                double t1 = (minY - p1.getY()) / dy;
                double t2 = (maxY - p1.getY()) / dy;
                if (t1 > t2) {
                    double tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }
                tMin = Math.max(tMin, t1);
                tMax = Math.min(tMax, t2);
                if (tMin > tMax)
                    return false;
            }

            return true;
        }
    }

    // Physical obstacle bounds. Pathfinder applies its robot clearance once.
    public static final class Obstacles {
        // Exact Hub Core (1.1938m x 1.1938m centered at 4.6256, 4.0346)
        public static final AABB BLUE_HUB_CORE = new AABB("Blue Hub Core",
                Hubs.BLUE_HUB_X - Hubs.HUB_WIDTH / 2.0, Hubs.BLUE_HUB_X + Hubs.HUB_WIDTH / 2.0,
                Hubs.HUB_Y - Hubs.HUB_WIDTH / 2.0, Hubs.HUB_Y + Hubs.HUB_WIDTH / 2.0);
        public static final AABB RED_HUB_CORE = new AABB("Red Hub Core",
                Hubs.RED_HUB_X - Hubs.HUB_WIDTH / 2.0, Hubs.RED_HUB_X + Hubs.HUB_WIDTH / 2.0,
                Hubs.HUB_Y - Hubs.HUB_WIDTH / 2.0, Hubs.HUB_Y + Hubs.HUB_WIDTH / 2.0);

        // Exact Trench Walls (47" x 12")
        public static final AABB BLUE_NORTH_TRENCH_WALL = TrenchWalls.BLUE_NORTH_WALL;
        public static final AABB BLUE_SOUTH_TRENCH_WALL = TrenchWalls.BLUE_SOUTH_WALL;
        public static final AABB RED_NORTH_TRENCH_WALL = TrenchWalls.RED_NORTH_WALL;
        public static final AABB RED_SOUTH_TRENCH_WALL = TrenchWalls.RED_SOUTH_WALL;

        // Exact Ramps (73" length)
        public static final AABB BLUE_NORTH_RAMP = new AABB("Blue North Ramp",
                Ramps.BLUE_NORTH_RAMP_MIN_X, Ramps.BLUE_NORTH_RAMP_MAX_X,
                Ramps.BLUE_NORTH_RAMP_MIN_Y, Ramps.BLUE_NORTH_RAMP_MAX_Y);
        public static final AABB BLUE_SOUTH_RAMP = new AABB("Blue South Ramp",
                Ramps.BLUE_SOUTH_RAMP_MIN_X, Ramps.BLUE_SOUTH_RAMP_MAX_X,
                Ramps.BLUE_SOUTH_RAMP_MIN_Y, Ramps.BLUE_SOUTH_RAMP_MAX_Y);
        public static final AABB RED_NORTH_RAMP = new AABB("Red North Ramp",
                Ramps.RED_NORTH_RAMP_MIN_X, Ramps.RED_NORTH_RAMP_MAX_X,
                Ramps.RED_NORTH_RAMP_MIN_Y, Ramps.RED_NORTH_RAMP_MAX_Y);
        public static final AABB RED_SOUTH_RAMP = new AABB("Red South Ramp",
                Ramps.RED_SOUTH_RAMP_MIN_X, Ramps.RED_SOUTH_RAMP_MAX_X,
                Ramps.RED_SOUTH_RAMP_MIN_Y, Ramps.RED_SOUTH_RAMP_MAX_Y);

        // MATCH MAPLESIM: 2 discrete upright posts per tower (3.5" x 1.5" each)
        // Blue Posts: X=1.062, Y=3.315 and Y=4.172
        public static final AABB BLUE_TOWER_POST_SOUTH = new AABB("Blue Tower South Post",
                1.062 - 0.045, 1.062 + 0.045, 3.315 - 0.02, 3.315 + 0.02);
        public static final AABB BLUE_TOWER_POST_NORTH = new AABB("Blue Tower North Post",
                1.062 - 0.045, 1.062 + 0.045, 4.172 - 0.02, 4.172 + 0.02);

        // Red Posts: X=15.479, Y=3.897 and Y=4.754
        public static final AABB RED_TOWER_POST_SOUTH = new AABB("Red Tower South Post",
                15.479 - 0.045, 15.479 + 0.045, 3.897 - 0.02, 3.897 + 0.02);
        public static final AABB RED_TOWER_POST_NORTH = new AABB("Red Tower North Post",
                15.479 - 0.045, 15.479 + 0.045, 4.754 - 0.02, 4.754 + 0.02);

        public static final List<AABB> ALL_OBSTACLES = List.of(
                BLUE_HUB_CORE, RED_HUB_CORE,
                BLUE_NORTH_TRENCH_WALL, BLUE_SOUTH_TRENCH_WALL,
                RED_NORTH_TRENCH_WALL, RED_SOUTH_TRENCH_WALL,
                BLUE_NORTH_RAMP, BLUE_SOUTH_RAMP,
                RED_NORTH_RAMP, RED_SOUTH_RAMP,
                BLUE_TOWER_POST_SOUTH, BLUE_TOWER_POST_NORTH,
                RED_TOWER_POST_SOUTH, RED_TOWER_POST_NORTH);

        public static List<AABB> getActiveObstacles() {
            return ALL_OBSTACLES;
        }

        public static final List<AABB> STATIC_OBSTACLES = ALL_OBSTACLES;
        // Keep single-box references for backward-compatibility if tests reference
        // them:
        public static final AABB BLUE_TOWER_POLE = BLUE_TOWER_POST_SOUTH;
        public static final AABB RED_TOWER_POLE = RED_TOWER_POST_SOUTH;
    }

    // =========================================================================
    // 9. Boundary Helpers & Spatial Clamping
    // =========================================================================
    public static Translation2d clampToField(Translation2d p) {
        if (p == null)
            return new Translation2d(CENTERLINE_X, FIELD_WIDTH / 2.0);
        double clampedX = Math.max(WALL_SAFETY_MARGIN, Math.min(FIELD_LENGTH - WALL_SAFETY_MARGIN, p.getX()));
        double clampedY = Math.max(WALL_SAFETY_MARGIN, Math.min(FIELD_WIDTH - WALL_SAFETY_MARGIN, p.getY()));
        return new Translation2d(clampedX, clampedY);
    }

    public static boolean isWithinField(Translation2d p, double margin) {
        if (p == null)
            return false;
        return p.getX() >= margin && p.getX() <= (FIELD_LENGTH - margin)
                && p.getY() >= margin && p.getY() <= (FIELD_WIDTH - margin);
    }

    public static boolean isPointInStaticObstacle(Translation2d p) {
        if (p == null)
            return true;
        if (!isWithinField(p, ROBOT_RADIUS))
            return true;
        for (AABB obs : Obstacles.getActiveObstacles()) {
            if (obs.contains(p))
                return true;
        }
        return false;
    }
}
