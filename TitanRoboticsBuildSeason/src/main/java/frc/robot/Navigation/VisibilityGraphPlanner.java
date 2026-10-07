package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.StaticPathfinder.PathResult;
import frc.robot.Navigation.StaticPathfinder.PathStatus;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Pure, game-agnostic topological graph planner and path smoother.
 * Executes A* search over an injected {@link FieldRoadmap} with edge clearance cost
 * scaling and string-pulling shortcut smoothing.
 */
public class VisibilityGraphPlanner {

    public static final double LOCAL_RECOVERY_MAX_STEP_M = 0.50;
    public static final double LOCAL_RECOVERY_MIN_STEP_M = 0.30;
    public static final double CLEARANCE_COST_MULTIPLIER_MAX = 1.60;
    public static final double CLEARANCE_TIGHT_M = 0.30;
    public static final double OCCUPIED_TRENCH_PENALTY_M = 20.0;

    private final FieldRoadmap roadmap;
    private final ObstacleModel obstacles;
    private final double bumperMargin;

    public VisibilityGraphPlanner(FieldRoadmap roadmap, ObstacleModel obstacles, double bumperMargin) {
        this.roadmap = roadmap;
        this.obstacles = obstacles;
        this.bumperMargin = bumperMargin;
    }

    public VisibilityGraphPlanner(FieldRoadmap roadmap, ObstacleModel obstacles) {
        this(roadmap, obstacles, FieldMap.ROBOT_RADIUS);
    }

    public VisibilityGraphPlanner(FieldRoadmap roadmap) {
        this(roadmap, new FieldMapObstacleModel(), FieldMap.ROBOT_RADIUS);
    }

    public FieldRoadmap getRoadmap() {
        return roadmap;
    }

    public ObstacleModel getObstacles() {
        return obstacles;
    }

    public double getBumperMargin() {
        return bumperMargin;
    }

    public PathResult planPathWithStatus(Pose2d start, Pose2d target) {
        Pose2d safeStart = obstacles.ensurePoseOutsideObstacles(start, target.getTranslation());
        Pose2d safeTarget = obstacles.ensurePoseOutsideObstacles(target, safeStart.getTranslation());

        Translation2d pStart = safeStart.getTranslation();
        Translation2d pTarget = safeTarget.getTranslation();

        // 1. Fast Path: If direct line of sight is unobstructed, proceed directly!
        if (obstacles.isLineOfSightClear(pStart, pTarget)) {
            List<Pose2d> direct = new ArrayList<>();
            if (safeStart.getTranslation().getDistance(start.getTranslation()) > 1e-6) {
                Translation2d escapeDirection = safeStart.getTranslation().minus(start.getTranslation());
                Rotation2d escapeHeading = escapeDirection.getNorm() > 1e-6
                        ? escapeDirection.getAngle()
                        : safeStart.getRotation();
                direct.add(new Pose2d(safeStart.getTranslation(), escapeHeading));
            }
            direct.add(new Pose2d(pTarget, safeTarget.getRotation()));
            return new PathResult(direct, PathStatus.DIRECT);
        }

        // 2. Connect start and target to visible roadmap nodes
        List<Integer> startVisible = new ArrayList<>();
        List<Integer> targetVisible = new ArrayList<>();

        int nodeCount = roadmap.getNodeCount();
        for (int i = 0; i < nodeCount; i++) {
            FieldRoadmap.Node node = roadmap.getNode(i);
            if (obstacles.isLineOfSightClear(pStart, node.pos())) {
                startVisible.add(node.id());
            }
            if (obstacles.isLineOfSightClear(node.pos(), pTarget)) {
                targetVisible.add(node.id());
            }
        }

        if (startVisible.isEmpty() || targetVisible.isEmpty()) {
            return localRecoveryResult(safeStart, pStart, pTarget);
        }

        // 3. A* Search over Roadmap Graph
        List<Integer> rawPath = aStarSearch(pStart, pTarget, startVisible, targetVisible);
        if (rawPath.isEmpty()) {
            return localRecoveryResult(safeStart, pStart, pTarget);
        }

        // Convert node IDs to 2D coordinates
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(pStart);
        for (int id : rawPath) {
            waypoints.add(roadmap.getNodePosition(id));
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

        if (safeStart.getTranslation().getDistance(start.getTranslation()) > 1e-6) {
            Translation2d escapeDirection = safeStart.getTranslation().minus(start.getTranslation());
            Rotation2d escapeHeading = escapeDirection.getNorm() > 1e-6
                    ? escapeDirection.getAngle()
                    : safeStart.getRotation();
            finalPath.add(0, new Pose2d(safeStart.getTranslation(), escapeHeading));
        }

        if (finalPath.isEmpty()) {
            finalPath.add(new Pose2d(pTarget, safeTarget.getRotation()));
        }

        return new PathResult(finalPath, PathStatus.ROADMAP);
    }

    public List<Pose2d> planPath(Pose2d start, Pose2d target) {
        return planPathWithStatus(start, target).waypoints();
    }

    public PathResult localRecoveryResult(Pose2d safeStart, Translation2d pStart, Translation2d pTarget) {
        Translation2d towardTarget = pTarget.minus(pStart);
        if (towardTarget.getNorm() < 1e-6) {
            return new PathResult(List.of(), PathStatus.UNREACHABLE);
        }
        double baseAngle = towardTarget.getAngle().getDegrees();

        for (int i = 0; i < 12; i++) {
            double angleDeg = baseAngle + (i * 30.0);
            Translation2d candidate = pStart.plus(new Translation2d(
                    LOCAL_RECOVERY_MAX_STEP_M, 0.0).rotateBy(Rotation2d.fromDegrees(angleDeg)));
            if (obstacles.isLineOfSightClear(pStart, candidate)
                    && !obstacles.isPointInStaticObstacle(candidate)
                    && FieldMap.isWithinField(candidate, bumperMargin)) {
                Pose2d step = new Pose2d(candidate, Rotation2d.fromDegrees(baseAngle + (i * 30.0)));
                return new PathResult(List.of(step), PathStatus.LOCAL_RECOVERY);
            }
        }

        return new PathResult(List.of(), PathStatus.UNREACHABLE);
    }

    public double clearancePenalisedCost(Translation2d a, Translation2d b, double length) {
        double clearance = obstacles.segmentMinClearance(a, b);
        if (!(clearance < CLEARANCE_TIGHT_M)) {
            return length;
        }
        double tightness = 1.0 - (clearance / CLEARANCE_TIGHT_M);
        return length * (1.0 + (CLEARANCE_COST_MULTIPLIER_MAX - 1.0) * tightness);
    }

    public List<Integer> aStarSearch(
            Translation2d startPos,
            Translation2d targetPos,
            List<Integer> startVisible,
            List<Integer> targetVisible) {

        Set<Integer> blockedNodes = obstacles.getBlockedCorridorNodes(startPos);

        int totalNodes = roadmap.getNodeCount();
        double[] gScore = new double[totalNodes];
        Arrays.fill(gScore, Double.MAX_VALUE);
        int[] parent = new int[totalNodes];
        Arrays.fill(parent, -1);

        PriorityQueue<NodeRecord> openSet = new PriorityQueue<>(Comparator.comparingDouble(nr -> nr.fScore));

        for (int startNodeId : startVisible) {
            Translation2d startNodePos = roadmap.getNodePosition(startNodeId);
            double d = clearancePenalisedCost(startPos, startNodePos, startPos.getDistance(startNodePos))
                    + (blockedNodes.contains(startNodeId) ? OCCUPIED_TRENCH_PENALTY_M : 0.0);
            gScore[startNodeId] = d;
            double h = startNodePos.getDistance(targetPos);
            openSet.add(new NodeRecord(startNodeId, d, d + h));
        }

        int bestEndNode = -1;
        double bestTotalCost = Double.MAX_VALUE;

        while (!openSet.isEmpty()) {
            NodeRecord current = openSet.poll();

            if (current.gScore > gScore[current.id])
                continue;

            if (targetVisible.contains(current.id)) {
                Translation2d endPos = roadmap.getNodePosition(current.id);
                double totalCost = current.gScore
                        + clearancePenalisedCost(endPos, targetPos, endPos.getDistance(targetPos));
                if (totalCost < bestTotalCost) {
                    bestTotalCost = totalCost;
                    bestEndNode = current.id;
                }
            }

            FieldRoadmap.Node curNode = roadmap.getNode(current.id);
            for (int neighborId : curNode.neighbors()) {
                Translation2d neighbourPos = roadmap.getNodePosition(neighborId);
                if (!obstacles.isLineOfSightClear(curNode.pos(), neighbourPos))
                    continue;

                double edgeWeight = curNode.pos().getDistance(neighbourPos);
                double tentativeG = current.gScore + clearancePenalisedCost(
                        curNode.pos(), neighbourPos, edgeWeight)
                        + (blockedNodes.contains(neighborId) ? OCCUPIED_TRENCH_PENALTY_M : 0.0);

                if (tentativeG < gScore[neighborId]) {
                    gScore[neighborId] = tentativeG;
                    parent[neighborId] = current.id;
                    double h = neighbourPos.getDistance(targetPos);
                    openSet.add(new NodeRecord(neighborId, tentativeG, tentativeG + h));
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        if (bestEndNode == -1) {
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

    public List<Translation2d> smoothPath(List<Translation2d> raw) {
        if (raw.size() <= 2)
            return raw;

        List<Translation2d> smoothed = new ArrayList<>();
        smoothed.add(raw.get(0));

        int curr = 0;
        while (curr < raw.size() - 1) {
            int furthest = curr + 1;
            for (int next = raw.size() - 1; next > curr + 1; next--) {
                if (isTrenchCorridorTransition(raw.get(curr), raw.get(next))) {
                    continue;
                }
                if (obstacles.isLineOfSightClear(raw.get(curr), raw.get(next))) {
                    furthest = next;
                    break;
                }
            }
            smoothed.add(raw.get(furthest));
            curr = furthest;
        }

        return smoothed;
    }

    private boolean isTrenchCorridorTransition(Translation2d p1, Translation2d p2) {
        boolean p1InTopTrench = p1.getY() > 7.0;
        boolean p2InTopTrench = p2.getY() > 7.0;
        if (p1InTopTrench ^ p2InTopTrench) {
            return true;
        }

        boolean p1InBotTrench = p1.getY() < 1.0;
        boolean p2InBotTrench = p2.getY() < 1.0;
        if (p1InBotTrench ^ p2InBotTrench) {
            return true;
        }

        return false;
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
}
