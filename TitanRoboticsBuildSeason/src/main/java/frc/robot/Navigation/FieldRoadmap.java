package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Interface representing a topological roadmap graph for field navigation.
 * Decouples graph topology and field waypoints from path planning search algorithms.
 */
public interface FieldRoadmap {

    /**
     * Immutable data record representing a single topological waypoint in the roadmap.
     *
     * @param id Unique 0-indexed node identifier.
     * @param name Descriptive name of the topological location.
     * @param pos Field-relative coordinates in meters (Blue-origin convention).
     * @param neighbors Unmodifiable list of adjacent node IDs.
     */
    record Node(int id, String name, Translation2d pos, List<Integer> neighbors) {
        public Node {
            neighbors = List.copyOf(neighbors);
        }

        public double getX() {
            return pos.getX();
        }

        public double getY() {
            return pos.getY();
        }

        public boolean hasNeighbor(int neighborId) {
            return neighbors.contains(neighborId);
        }
    }

    /** Returns total number of nodes in the roadmap graph. */
    int getNodeCount();

    /** Returns the node corresponding to the given ID. */
    Node getNode(int id);

    /** Returns an unmodifiable list of all nodes in the roadmap. */
    List<Node> getAllNodes();

    /** Returns the neighbor node IDs for the specified node ID. */
    List<Integer> getNeighbors(int id);

    /** Returns the field position of the specified node ID. */
    default Translation2d getNodePosition(int id) {
        return getNode(id).pos();
    }

    /** Returns corridor bounding regions that require single-file or constrained traversal. */
    default List<FieldMap.AABB> getConstrainedCorridors() {
        return List.of();
    }
}
