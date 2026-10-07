package frc.robot.Intelligence;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Navigation.FieldMap;
import frc.robot.Navigation.StaticPathfinder;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * FuelTourOptimizer: Sequential Traveling Salesperson Problem (TSP) solver for
 * fuel harvesting tours.
 *
 * <p>Inspired by Team 6328's SalesmanSolver, this component replaces single-piece
 * greedy selection with an optimal multi-piece collection sequence. Rather than
 * stopping and re-aiming after each collected ball, the robot follows a smooth,
 * curvature-penalized tour that minimizes total travel distance and avoids abrupt
 * directional reversals.
 *
 * <p>Properties:
 * <ul>
 *   <li><b>Kinematic Cost Function:</b> Evaluates Euclidean distance plus turn penalties
 *       based on vector alignment, heavily penalizing sharp reversals (>90°) and favoring
 *       sweeping arcs.</li>
 *   <li><b>Guaranteed Global Minimum for Small Clusters:</b> For clusters of up to 5 pieces,
 *       an exhaustive permutation search (O(K!)) guarantees the exact optimal sequence.</li>
 *   <li><b>2-Opt Local Search for Larger Sets:</b> For 6–10 pieces, uses Nearest-Neighbor
 *       construction followed by iterative 2-opt edge-exchange improvements.</li>
 *   <li><b>Deterministic & Non-Blocking:</b> Runs in sub-millisecond execution time,
 *       requiring no external non-linear solver threads.</li>
 * </ul>
 */
public final class FuelTourOptimizer {

    /** Default cruise speed while harvesting fuel pieces, in m/s. */
    public static final double DEFAULT_HARVEST_SPEED_MPS = 2.50;

    /** Penalty weight for heading changes (in equivalent meters of travel). */
    public static final double TURN_PENALTY_WEIGHT_METERS = 0.75;

    /** Weight for progress towards the final exit destination (e.g. Hub). */
    public static final double EXIT_PROGRESS_WEIGHT = 0.40;

    /** Maximum candidates evaluated in a single tour optimization pass. */
    public static final int MAX_CANDIDATE_POOL_SIZE = 8;

    /**
     * Structured result of a tour optimization pass.
     */
    public record TourResult(
            boolean isValid,
            List<Translation2d> waypoints,
            Pose2d immediateTargetPose,
            double totalDistanceMeters,
            double estimatedDurationSeconds,
            Pose2d finalExitPose,
            int pieceCount
    ) {
        public static final TourResult EMPTY = new TourResult(
                false,
                Collections.emptyList(),
                new Pose2d(),
                0.0,
                0.0,
                new Pose2d(),
                0
        );
    }

    private FuelTourOptimizer() {
        // Utility class with stateless solver functions
    }

    /**
     * Plans an optimal fuel collection tour from candidate translations.
     *
     * @param robotPose        Current pose of the robot
     * @param candidates       Available candidate fuel piece locations
     * @param maxPieces        Maximum number of pieces to include in the tour
     * @param exitDestination  Desired destination after completing the tour (e.g. Hub location), or null
     * @return Optimized TourResult
     */
    public static TourResult optimizeTour(
            Pose2d robotPose,
            List<Translation2d> candidates,
            int maxPieces,
            Translation2d exitDestination) {

        if (robotPose == null || candidates == null || candidates.isEmpty() || maxPieces <= 0) {
            return TourResult.EMPTY;
        }

        // Filter valid candidates
        List<Translation2d> validCandidates = new ArrayList<>();
        for (Translation2d pt : candidates) {
            if (pt == null) continue;
            if (pt.getX() < 0.05 || pt.getX() > 16.48 || pt.getY() < 0.05 || pt.getY() > 8.00) continue;
            if (StaticPathfinder.isPointInHardObstacle(pt) || StaticPathfinder.isPointNearDynamicObstacle(pt)) continue;
            validCandidates.add(pt);
        }

        if (validCandidates.isEmpty()) {
            return TourResult.EMPTY;
        }

        // Limit candidates pool to most promising pieces near the robot
        Translation2d robotPos = robotPose.getTranslation();
        if (validCandidates.size() > MAX_CANDIDATE_POOL_SIZE) {
            validCandidates.sort(Comparator.comparingDouble(p -> p.getDistance(robotPos)));
            validCandidates = new ArrayList<>(validCandidates.subList(0, MAX_CANDIDATE_POOL_SIZE));
        }

        int targetCount = Math.min(maxPieces, validCandidates.size());

        List<Translation2d> bestOrder;
        if (validCandidates.size() <= 5) {
            bestOrder = solveExactOptimal(robotPose, validCandidates, targetCount, exitDestination);
        } else {
            bestOrder = solveNearestNeighborWith2Opt(robotPose, validCandidates, targetCount, exitDestination);
        }

        if (bestOrder.isEmpty()) {
            return TourResult.EMPTY;
        }

        // Calculate tour metrics
        double totalDist = calculatePathDistance(robotPos, bestOrder);
        double estDuration = totalDist / DEFAULT_HARVEST_SPEED_MPS;

        // Immediate target with safe wall approach
        Translation2d firstPiece = bestOrder.get(0);
        Pose2d immediateTarget = StaticPathfinder.wallStandoffApproach(firstPiece, robotPos);

        // Final exit pose facing the destination
        Translation2d lastPiece = bestOrder.get(bestOrder.size() - 1);
        Rotation2d exitHeading;
        if (exitDestination != null) {
            exitHeading = exitDestination.minus(lastPiece).getAngle();
        } else if (bestOrder.size() > 1) {
            exitHeading = lastPiece.minus(bestOrder.get(bestOrder.size() - 2)).getAngle();
        } else {
            exitHeading = lastPiece.minus(robotPos).getAngle();
        }
        Pose2d finalExitPose = StaticPathfinder.ensurePoseOutsideObstacles(
                new Pose2d(lastPiece, exitHeading), robotPos);

        return new TourResult(
                true,
                Collections.unmodifiableList(bestOrder),
                immediateTarget,
                totalDist,
                estDuration,
                finalExitPose,
                bestOrder.size()
        );
    }

    /**
     * Finds the globally optimal tour using exhaustive permutation evaluation.
     */
    private static List<Translation2d> solveExactOptimal(
            Pose2d robotPose,
            List<Translation2d> pool,
            int k,
            Translation2d exitDestination) {

        List<List<Translation2d>> combinations = new ArrayList<>();
        generateCombinations(pool, k, 0, new ArrayList<>(), combinations);

        double bestCost = Double.MAX_VALUE;
        List<Translation2d> bestTour = Collections.emptyList();

        for (List<Translation2d> subset : combinations) {
            List<List<Translation2d>> perms = new ArrayList<>();
            generatePermutations(subset, 0, perms);

            for (List<Translation2d> candidateTour : perms) {
                double cost = evaluateTourCost(robotPose, candidateTour, exitDestination);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestTour = candidateTour;
                }
            }
        }

        return bestTour;
    }

    /**
     * Solves using Nearest-Neighbor construction followed by 2-opt edge-exchange improvements.
     */
    private static List<Translation2d> solveNearestNeighborWith2Opt(
            Pose2d robotPose,
            List<Translation2d> pool,
            int k,
            Translation2d exitDestination) {

        List<Translation2d> remaining = new ArrayList<>(pool);
        List<Translation2d> tour = new ArrayList<>();
        Translation2d currentPos = robotPose.getTranslation();

        // 1. Greedy Nearest-Neighbor build
        while (tour.size() < k && !remaining.isEmpty()) {
            final Translation2d pos = currentPos;
            remaining.sort(Comparator.comparingDouble(p -> p.getDistance(pos)));
            Translation2d next = remaining.remove(0);
            tour.add(next);
            currentPos = next;
        }

        if (tour.size() <= 2) {
            return tour;
        }

        // 2. 2-opt local search passes
        boolean improved = true;
        int maxIterations = 20;
        int iter = 0;

        while (improved && iter < maxIterations) {
            improved = false;
            iter++;
            double currentCost = evaluateTourCost(robotPose, tour, exitDestination);

            for (int i = 0; i < tour.size() - 1; i++) {
                for (int j = i + 1; j < tour.size(); j++) {
                    List<Translation2d> swapped = twoOptSwap(tour, i, j);
                    double newCost = evaluateTourCost(robotPose, swapped, exitDestination);
                    if (newCost < currentCost - 1e-4) {
                        tour = swapped;
                        currentCost = newCost;
                        improved = true;
                        break;
                    }
                }
                if (improved) break;
            }
        }

        return tour;
    }

    private static List<Translation2d> twoOptSwap(List<Translation2d> tour, int i, int k) {
        List<Translation2d> result = new ArrayList<>(tour.size());
        for (int c = 0; c < i; c++) {
            result.add(tour.get(c));
        }
        for (int c = k; c >= i; c--) {
            result.add(tour.get(c));
        }
        for (int c = k + 1; c < tour.size(); c++) {
            result.add(tour.get(c));
        }
        return result;
    }

    /**
     * Evaluates total tour cost including Euclidean distance and turn angle penalties.
     */
    public static double evaluateTourCost(
            Pose2d robotPose,
            List<Translation2d> tour,
            Translation2d exitDestination) {

        if (tour.isEmpty()) return 0.0;

        double cost = 0.0;
        Translation2d prevPoint = robotPose.getTranslation();
        Translation2d prevVector = new Translation2d(
                Math.cos(robotPose.getRotation().getRadians()),
                Math.sin(robotPose.getRotation().getRadians())
        );

        for (int i = 0; i < tour.size(); i++) {
            Translation2d point = tour.get(i);
            double dist = point.getDistance(prevPoint);
            cost += dist;

            if (dist > 1e-4) {
                Translation2d travelVector = point.minus(prevPoint).div(dist);
                double dot = prevVector.getX() * travelVector.getX() + prevVector.getY() * travelVector.getY();
                dot = Math.max(-1.0, Math.min(1.0, dot));
                // Penalize sharp turns (dot = -1 means complete reversal -> penalty = 2 * weight)
                cost += TURN_PENALTY_WEIGHT_METERS * (1.0 - dot);
                prevVector = travelVector;
            }
            prevPoint = point;
        }

        // Final exit destination cost
        if (exitDestination != null) {
            double exitDist = prevPoint.getDistance(exitDestination);
            cost += exitDist * EXIT_PROGRESS_WEIGHT;
        }

        return cost;
    }

    private static double calculatePathDistance(Translation2d start, List<Translation2d> waypoints) {
        double dist = 0.0;
        Translation2d current = start;
        for (Translation2d pt : waypoints) {
            dist += current.getDistance(pt);
            current = pt;
        }
        return dist;
    }

    private static void generateCombinations(
            List<Translation2d> pool,
            int k,
            int start,
            List<Translation2d> current,
            List<List<Translation2d>> result) {
        if (current.size() == k) {
            result.add(new ArrayList<>(current));
            return;
        }
        for (int i = start; i < pool.size(); i++) {
            current.add(pool.get(i));
            generateCombinations(pool, k, i + 1, current, result);
            current.remove(current.size() - 1);
        }
    }

    private static void generatePermutations(
            List<Translation2d> list,
            int index,
            List<List<Translation2d>> result) {
        if (index == list.size() - 1) {
            result.add(new ArrayList<>(list));
            return;
        }
        for (int i = index; i < list.size(); i++) {
            Collections.swap(list, index, i);
            generatePermutations(list, index + 1, result);
            Collections.swap(list, index, i);
        }
    }

    /**
     * Gathers eligible fuel piece locations from SimulatedArena, respecting boundaries and blockers.
     *
     * @param robotPose     Current robot pose
     * @param isRedAlliance True if on Red Alliance
     * @param isAutonomous  True during autonomous (strictly enforces centerline G201)
     * @param blockedFuel   Points abandoned by watchdogs
     * @return List of valid Translation2d candidate coordinates
     */
    public static List<Translation2d> findFieldFuelCandidates(
            Pose2d robotPose,
            boolean isRedAlliance,
            boolean isAutonomous,
            Set<Translation2d> blockedFuel) {

        List<Translation2d> candidates = new ArrayList<>();
        SimulatedArena arena = SimulatedArena.getInstance();
        if (arena == null) return candidates;

        try {
            var pieces = frc.robot.Sim.MatchDeterminism.fuelOnFieldSorted();
            if (pieces == null || pieces.isEmpty()) return candidates;

            for (var piece : pieces) {
                if (piece == null || !"Fuel".equals(piece.getType())) continue;
                Translation2d pos = piece.getPoseOnField().getTranslation();

                // Field boundaries
                if (pos.getX() < 0.05 || pos.getX() > 16.48 || pos.getY() < 0.05 || pos.getY() > 8.00) continue;
                if (StaticPathfinder.isPointInHardObstacle(pos) || StaticPathfinder.isPointNearDynamicObstacle(pos)) continue;
                if (blockedFuel != null && isBlocked(blockedFuel, pos)) continue;

                // Autonomous centerline boundary rule (FRC G201)
                if (isAutonomous) {
                    if (isRedAlliance && pos.getX() < FieldMap.CENTERLINE_X + 0.15) continue;
                    if (!isRedAlliance && pos.getX() > FieldMap.CENTERLINE_X - 0.15) continue;
                } else {
                    if (isRedAlliance && pos.getX() < 3.5) continue;
                    if (!isRedAlliance && pos.getX() > 13.0) continue;
                }

                candidates.add(pos);
            }
        } catch (Exception ignored) {
        }

        return candidates;
    }

    private static boolean isBlocked(Set<Translation2d> blockedFuel, Translation2d pos) {
        if (blockedFuel == null || blockedFuel.isEmpty()) return false;
        for (Translation2d blocked : blockedFuel) {
            if (blocked != null && blocked.getDistance(pos) < 1.0) {
                return true;
            }
        }
        return false;
    }
}
