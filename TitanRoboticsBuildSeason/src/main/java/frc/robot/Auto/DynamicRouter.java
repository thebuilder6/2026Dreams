package frc.robot.Auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * DynamicRouter provides real-time local obstacle avoidance around moving opponents and field blocks.
 * Supports three switchable algorithms:
 * 1. POTENTIAL_FIELDS (APF): Instantaneous vector blending (<0.5ms).
 * 2. DYNAMIC_WINDOW (DWA / VO): Velocity Obstacle sampling evaluating dynamic momentum (~1.5ms).
 * 3. DYNAMIC_GRID_ASTAR: 2D Occupancy Costmap + A* grid corridor navigation (~3ms).
 */
public class DynamicRouter {

    public enum AvoidanceAlgorithm {
        POTENTIAL_FIELDS,
        DYNAMIC_WINDOW,
        DYNAMIC_GRID_ASTAR
    }

    private static AvoidanceAlgorithm activeAlgorithm = AvoidanceAlgorithm.POTENTIAL_FIELDS;

    private static final List<DynamicObstacle> activeObstacles = new ArrayList<>();
    private static final double SAFE_DISTANCE_METERS = 1.40; // Zone of repulsive influence
    private static final double REPULSION_STRENGTH = 2.50;
    private static final double ROBOT_RADIUS_METERS = 0.45; // Half of chassis width with bumpers

    // Dynamic A* Grid parameters (16.5m x 8.25m field at 0.25m resolution)
    private static final double GRID_RESOLUTION = 0.25; // 25cm cells
    private static final int GRID_COLS = 66; // 16.5m / 0.25m
    private static final int GRID_ROWS = 33; // 8.25m / 0.25m
    private static final boolean[][] STATIC_BLOCKED_GRID = new boolean[GRID_COLS][GRID_ROWS];

    // Static Field Constants (Field boundaries, Hubs, Climbing Poles, Trench dividers)
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;
    public static final double WALL_SAFETY_MARGIN_METERS = 0.65; // Repulsion active within 65cm of field border

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.60, 4.035);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.94, 4.035);
    public static final double HUB_RADIUS = 1.10;

    public static final Translation2d BLUE_CLIMB_POLE = new Translation2d(1.07, 4.04);
    public static final Translation2d RED_CLIMB_POLE = new Translation2d(15.47, 4.04);
    public static final double POLE_RADIUS = 0.40;

    static {
        // Initialize static obstacle mask (Field boundaries, Hubs, Climb poles)
        for (int c = 0; c < GRID_COLS; c++) {
            for (int r = 0; r < GRID_ROWS; r++) {
                double x = c * GRID_RESOLUTION;
                double y = r * GRID_RESOLUTION;

                // Field perimeter borders with robot bumper margin
                if (x < ROBOT_RADIUS_METERS || x > (FIELD_LENGTH_METERS - ROBOT_RADIUS_METERS)
                        || y < ROBOT_RADIUS_METERS || y > (FIELD_WIDTH_METERS - ROBOT_RADIUS_METERS)) {
                    STATIC_BLOCKED_GRID[c][r] = true;
                    continue;
                }

                // Blue Hub (4.60, 4.035, radius 1.10m)
                if (Math.hypot(x - BLUE_HUB_CENTER.getX(), y - BLUE_HUB_CENTER.getY()) < (HUB_RADIUS + 0.35)) {
                    STATIC_BLOCKED_GRID[c][r] = true;
                    continue;
                }

                // Red Hub (11.94, 4.035, radius 1.10m)
                if (Math.hypot(x - RED_HUB_CENTER.getX(), y - RED_HUB_CENTER.getY()) < (HUB_RADIUS + 0.35)) {
                    STATIC_BLOCKED_GRID[c][r] = true;
                    continue;
                }

                // Blue Climb Pole (1.07, 4.04)
                if (Math.hypot(x - BLUE_CLIMB_POLE.getX(), y - BLUE_CLIMB_POLE.getY()) < (POLE_RADIUS + 0.35)) {
                    STATIC_BLOCKED_GRID[c][r] = true;
                    continue;
                }

                // Red Climb Pole (15.47, 4.04)
                if (Math.hypot(x - RED_CLIMB_POLE.getX(), y - RED_CLIMB_POLE.getY()) < (POLE_RADIUS + 0.35)) {
                    STATIC_BLOCKED_GRID[c][r] = true;
                    continue;
                }
            }
        }
    }

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
        double now = Timer.getFPGATimestamp();
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
        double now = Timer.getFPGATimestamp();
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

        double now = Timer.getFPGATimestamp();
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

        // Check for dashboard algorithm override
        String dashAlgo = SmartDashboard.getString("DynamicAvoidance/Algorithm", activeAlgorithm.name());
        try {
            activeAlgorithm = AvoidanceAlgorithm.valueOf(dashAlgo);
        } catch (IllegalArgumentException ignored) {
        }

        ChassisSpeeds resultSpeeds;
        switch (activeAlgorithm) {
            case DYNAMIC_WINDOW:
                resultSpeeds = computeDynamicWindow(currentPose, nominalSpeeds, targetWaypoint);
                break;
            case DYNAMIC_GRID_ASTAR:
                resultSpeeds = computeGridAStar(currentPose, nominalSpeeds, targetWaypoint);
                break;
            case POTENTIAL_FIELDS:
            default:
                resultSpeeds = computePotentialFields(currentPose, nominalSpeeds);
                break;
        }

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

    // =========================================================================
    // APPROACH B: Dynamic Window Approach (DWA) / Velocity Obstacles (VO)
    // =========================================================================

    private static ChassisSpeeds computeDynamicWindow(Pose2d currentPose, ChassisSpeeds nominalSpeeds, Translation2d targetWaypoint) {
        Translation2d robotPos = currentPose.getTranslation();
        double nominalSpeed = Math.hypot(nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond);
        if (nominalSpeed < 0.1) {
            return nominalSpeeds;
        }

        Translation2d toTarget = targetWaypoint.minus(robotPos);
        double targetAngleRad = Math.atan2(toTarget.getY(), toTarget.getX());

        // Sample velocity space: 16 directions x 3 speeds = 48 samples
        double bestScore = -1e9;
        Translation2d bestVel = new Translation2d(nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond);

        double[] speedFractions = new double[] { 1.0, 0.75, 0.50 };
        int angleSlices = 16;

        for (double fraction : speedFractions) {
            double testSpeed = Math.min(Constants.MAX_SPEED, nominalSpeed * fraction);

            for (int i = 0; i < angleSlices; i++) {
                // Sweep +/- 90 degrees around target heading
                double offsetAngle = ((i - angleSlices / 2.0) / (angleSlices / 2.0)) * (Math.PI / 2.0);
                double candidateHeading = targetAngleRad + offsetAngle;

                double testVx = testSpeed * Math.cos(candidateHeading);
                double testVy = testSpeed * Math.sin(candidateHeading);
                Translation2d candidateVel = new Translation2d(testVx, testVy);

                // Forward trajectory collision check (1.2 second lookahead)
                boolean collides = false;
                double minDistanceToObstacle = Double.MAX_VALUE;

                for (double t = 0.2; t <= 1.2; t += 0.2) {
                    Translation2d projectedRobot = robotPos.plus(candidateVel.times(t));

                    // Field perimeter boundary check
                    if (projectedRobot.getX() < ROBOT_RADIUS_METERS || projectedRobot.getX() > (FIELD_LENGTH_METERS - ROBOT_RADIUS_METERS)
                            || projectedRobot.getY() < ROBOT_RADIUS_METERS || projectedRobot.getY() > (FIELD_WIDTH_METERS - ROBOT_RADIUS_METERS)) {
                        collides = true;
                        break;
                    }

                    for (DynamicObstacle obs : activeObstacles) {
                        Translation2d projectedObs = obs.getPredictedPosition(t);
                        double dist = projectedRobot.getDistance(projectedObs);
                        double clearance = dist - (ROBOT_RADIUS_METERS + obs.radius);

                        if (clearance < 0.08) {
                            collides = true;
                            break;
                        }
                        if (clearance < minDistanceToObstacle) {
                            minDistanceToObstacle = clearance;
                        }
                    }
                    if (collides) break;
                }

                if (collides) continue;

                // Score candidate: Heading alignment + Clearance + Speed
                double headingDiff = Math.abs(Math.IEEEremainder(candidateHeading - targetAngleRad, 2 * Math.PI));
                double headingScore = 1.0 - (headingDiff / Math.PI); // [0, 1]
                double clearanceScore = Math.min(minDistanceToObstacle / SAFE_DISTANCE_METERS, 1.0); // [0, 1]
                double speedScore = testSpeed / Constants.MAX_SPEED; // [0, 1]

                double totalScore = (headingScore * 2.0) + (clearanceScore * 3.5) + (speedScore * 1.0);
                if (totalScore > bestScore) {
                    bestScore = totalScore;
                    bestVel = candidateVel;
                }
            }
        }

        // If all candidate trajectories collide, fall back to APF deflection
        if (bestScore <= -1e8) {
            return computePotentialFields(currentPose, nominalSpeeds);
        }

        return new ChassisSpeeds(bestVel.getX(), bestVel.getY(), nominalSpeeds.omegaRadiansPerSecond);
    }

    // =========================================================================
    // APPROACH C: Dynamic Occupancy Costmap + A* Grid
    // =========================================================================

    private static class GridNode implements Comparable<GridNode> {
        public final int col;
        public final int row;
        public double gCost;
        public double hCost;
        public GridNode parent;

        public GridNode(int col, int row, double gCost, double hCost, GridNode parent) {
            this.col = col;
            this.row = row;
            this.gCost = gCost;
            this.hCost = hCost;
            this.parent = parent;
        }

        public double fCost() {
            return gCost + hCost;
        }

        @Override
        public int compareTo(GridNode other) {
            return Double.compare(this.fCost(), other.fCost());
        }
    }

    private static ChassisSpeeds computeGridAStar(Pose2d currentPose, ChassisSpeeds nominalSpeeds, Translation2d targetWaypoint) {
        Translation2d robotPos = currentPose.getTranslation();
        double nominalSpeed = Math.hypot(nominalSpeeds.vxMetersPerSecond, nominalSpeeds.vyMetersPerSecond);
        if (nominalSpeed < 0.1) {
            return nominalSpeeds;
        }

        int startCol = (int) Math.round(robotPos.getX() / GRID_RESOLUTION);
        int startRow = (int) Math.round(robotPos.getY() / GRID_RESOLUTION);
        int goalCol = (int) Math.round(targetWaypoint.getX() / GRID_RESOLUTION);
        int goalRow = (int) Math.round(targetWaypoint.getY() / GRID_RESOLUTION);

        // Clamp to grid
        startCol = Math.max(0, Math.min(GRID_COLS - 1, startCol));
        startRow = Math.max(0, Math.min(GRID_ROWS - 1, startRow));
        goalCol = Math.max(0, Math.min(GRID_COLS - 1, goalCol));
        goalRow = Math.max(0, Math.min(GRID_ROWS - 1, goalRow));

        // Create local dynamic costmask
        boolean[][] blocked = new boolean[GRID_COLS][GRID_ROWS];
        for (int c = 0; c < GRID_COLS; c++) {
            System.arraycopy(STATIC_BLOCKED_GRID[c], 0, blocked[c], 0, GRID_ROWS);
        }

        // Inflate dynamic obstacles
        for (DynamicObstacle obs : activeObstacles) {
            double inflateRadius = obs.radius + ROBOT_RADIUS_METERS + 0.15;
            int cellRadius = (int) Math.ceil(inflateRadius / GRID_RESOLUTION);
            int obsCol = (int) Math.round(obs.position.getX() / GRID_RESOLUTION);
            int obsRow = (int) Math.round(obs.position.getY() / GRID_RESOLUTION);

            for (int dc = -cellRadius; dc <= cellRadius; dc++) {
                for (int dr = -cellRadius; dr <= cellRadius; dr++) {
                    int c = obsCol + dc;
                    int r = obsRow + dr;
                    if (c >= 0 && c < GRID_COLS && r >= 0 && r < GRID_ROWS) {
                        if (Math.hypot(dc * GRID_RESOLUTION, dr * GRID_RESOLUTION) <= inflateRadius) {
                            blocked[c][r] = true;
                        }
                    }
                }
            }
        }

        // Make start node traversable even if near an obstacle
        blocked[startCol][startRow] = false;

        // A* Search
        PriorityQueue<GridNode> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[GRID_COLS][GRID_ROWS];
        double[][] gScores = new double[GRID_COLS][GRID_ROWS];
        for (int c = 0; c < GRID_COLS; c++) {
            java.util.Arrays.fill(gScores[c], Double.MAX_VALUE);
        }

        double startH = Math.hypot(goalCol - startCol, goalRow - startRow);
        openSet.add(new GridNode(startCol, startRow, 0.0, startH, null));
        gScores[startCol][startRow] = 0.0;

        GridNode targetNode = null;
        int iterations = 0;
        int maxIterations = 350; // Guard against search explosion (<3ms guarantee)

        int[][] directions = new int[][] {
            { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 },
            { 1, 1 }, { -1, -1 }, { 1, -1 }, { -1, 1 }
        };

        while (!openSet.isEmpty() && iterations++ < maxIterations) {
            GridNode current = openSet.poll();
            if (closedSet[current.col][current.row]) continue;
            closedSet[current.col][current.row] = true;

            if (current.col == goalCol && current.row == goalRow) {
                targetNode = current;
                break;
            }

            // If we are within 2 cells of goal, consider reached
            if (Math.hypot(current.col - goalCol, current.row - goalRow) <= 2.0) {
                targetNode = current;
                break;
            }

            for (int[] dir : directions) {
                int nc = current.col + dir[0];
                int nr = current.row + dir[1];

                if (nc < 0 || nc >= GRID_COLS || nr < 0 || nr >= GRID_ROWS) continue;
                if (blocked[nc][nr] || closedSet[nc][nr]) continue;

                double stepCost = (dir[0] != 0 && dir[1] != 0) ? 1.414 : 1.0;
                double tentativeG = current.gCost + stepCost;

                if (tentativeG < gScores[nc][nr]) {
                    gScores[nc][nr] = tentativeG;
                    double h = Math.hypot(goalCol - nc, goalRow - nr);
                    openSet.add(new GridNode(nc, nr, tentativeG, h, current));
                }
            }
        }

        if (targetNode == null) {
            // A* could not find corridor; fallback to APF
            return computePotentialFields(currentPose, nominalSpeeds);
        }

        // Trace back path from targetNode to find immediate lookahead cell (3 steps ahead)
        List<GridNode> path = new ArrayList<>();
        GridNode curr = targetNode;
        while (curr != null) {
            path.add(curr);
            curr = curr.parent;
        }
        Collections.reverse(path);

        // Pick lookahead point on grid path (e.g. index 3 or end)
        int lookaheadIdx = Math.min(3, path.size() - 1);
        GridNode lookNode = path.get(lookaheadIdx);
        Translation2d nextGridPos = new Translation2d(lookNode.col * GRID_RESOLUTION, lookNode.row * GRID_RESOLUTION);

        Translation2d corridorDir = nextGridPos.minus(robotPos);
        if (corridorDir.getNorm() > 1e-4) {
            corridorDir = corridorDir.div(corridorDir.getNorm());
        } else {
            return computePotentialFields(currentPose, nominalSpeeds);
        }

        double vx = corridorDir.getX() * nominalSpeed;
        double vy = corridorDir.getY() * nominalSpeed;

        return new ChassisSpeeds(vx, vy, nominalSpeeds.omegaRadiansPerSecond);
    }
}
