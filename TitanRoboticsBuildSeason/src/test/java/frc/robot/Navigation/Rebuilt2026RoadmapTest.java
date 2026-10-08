package frc.robot.Navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests verifying topological invariants, edge reciprocity, and coordinate consistency
 * of {@link Rebuilt2026Roadmap}.
 */
class Rebuilt2026RoadmapTest {

    private Rebuilt2026Roadmap roadmap;

    @BeforeEach
    void setUp() {
        roadmap = Rebuilt2026Roadmap.getInstance();
    }

    @Test
    void testNodeCountAndIndexing() {
        assertEquals(38, roadmap.getNodeCount(), "2026 REBUILT roadmap must have exactly 38 nodes");
        List<FieldRoadmap.Node> allNodes = roadmap.getAllNodes();
        assertEquals(38, allNodes.size());

        for (int i = 0; i < 38; i++) {
            FieldRoadmap.Node node = roadmap.getNode(i);
            assertNotNull(node, "Node " + i + " must not be null");
            assertEquals(i, node.id(), "Node id must match its index");
            assertSame(node, allNodes.get(i), "Node lookup must match getAllNodes index");
            assertNotNull(node.name());
            assertFalse(node.name().isBlank());
        }
    }

    @Test
    void testAllNodesWithinFieldBoundaries() {
        for (FieldRoadmap.Node node : roadmap.getAllNodes()) {
            Translation2d pos = node.pos();
            assertTrue(pos.getX() >= 0.0 && pos.getX() <= FieldMap.FIELD_LENGTH,
                    "Node " + node.id() + " (" + node.name() + ") X out of bounds: " + pos.getX());
            assertTrue(pos.getY() >= 0.0 && pos.getY() <= FieldMap.FIELD_WIDTH,
                    "Node " + node.id() + " (" + node.name() + ") Y out of bounds: " + pos.getY());
        }
    }

    @Test
    void testEdgeReciprocityAndNoSelfLoops() {
        int totalDirectedEdges = 0;
        Set<String> uniqueUndirectedEdges = new HashSet<>();

        for (FieldRoadmap.Node node : roadmap.getAllNodes()) {
            int u = node.id();
            List<Integer> neighbors = node.neighbors();

            assertFalse(neighbors.isEmpty(), "Node " + u + " (" + node.name() + ") must have at least one neighbor");
            assertFalse(node.hasNeighbor(u), "Node " + u + " must not have a self-loop");

            // Check no duplicate neighbor IDs
            Set<Integer> neighborSet = new HashSet<>(neighbors);
            assertEquals(neighbors.size(), neighborSet.size(), "Node " + u + " has duplicate neighbor entries");

            for (int v : neighbors) {
                totalDirectedEdges++;
                int minId = Math.min(u, v);
                int maxId = Math.max(u, v);
                uniqueUndirectedEdges.add(minId + "-" + maxId);

                FieldRoadmap.Node neighborNode = roadmap.getNode(v);
                assertNotNull(neighborNode, "Neighbor node " + v + " must exist");
                assertTrue(neighborNode.hasNeighbor(u),
                        "Edge reciprocity violation: node " + u + " has neighbor " + v
                                + ", but node " + v + " does not list " + u);
            }
        }

        // 59 unique undirected edges = 118 directed edges
        assertEquals(59, uniqueUndirectedEdges.size(), "Roadmap must contain exactly 59 unique undirected edges");
        assertEquals(118, totalDirectedEdges, "Roadmap must contain exactly 118 directed edges (59 reciprocal pairs)");
    }

    @Test
    void testMatchesStaticPathfinderNodePositions() {
        for (int i = 0; i < 38; i++) {
            Translation2d roadmapPos = roadmap.getNodePosition(i);
            Translation2d pathfinderPos = StaticPathfinder.getNodePosition(i);
            assertEquals(pathfinderPos.getX(), roadmapPos.getX(), 1e-9,
                    "Node " + i + " X pos differs between Rebuilt2026Roadmap and StaticPathfinder");
            assertEquals(pathfinderPos.getY(), roadmapPos.getY(), 1e-9,
                    "Node " + i + " Y pos differs between Rebuilt2026Roadmap and StaticPathfinder");
        }
    }

    @Test
    void testMirrorSymmetryOfStrategicWaypoints() {
        // Tower nodes
        double blueTowerTopX = roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_TOWER_WEST_TOP).getX();
        double redTowerTopX = roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_TOWER_EAST_TOP).getX();
        assertEquals(FieldMap.FIELD_LENGTH - blueTowerTopX, redTowerTopX, 1e-9, "Tower Top X must be mirrored");

        double blueTowerBotX = roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_TOWER_WEST_BOT).getX();
        double redTowerBotX = roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_TOWER_EAST_BOT).getX();
        assertEquals(FieldMap.FIELD_LENGTH - blueTowerBotX, redTowerBotX, 1e-9, "Tower Bot X must be mirrored");

        assertEquals(
                roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_TOWER_WEST_TOP).getY(),
                roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_TOWER_EAST_TOP).getY(),
                1e-9, "Tower Top Y lanes must be equal");
        assertEquals(
                roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_TOWER_WEST_BOT).getY(),
                roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_TOWER_EAST_BOT).getY(),
                1e-9, "Tower Bot Y lanes must be equal");

        // Top Trench corridor Y = 7.42m
        assertEquals(7.42, roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_TOP_TRENCH_IN).getY(), 1e-9);
        assertEquals(7.42, roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_TOP_TRENCH_IN).getY(), 1e-9);

        // Bottom Trench corridor Y = 0.65m
        assertEquals(0.65, roadmap.getNodePosition(Rebuilt2026Roadmap.N_BLUE_BOT_TRENCH_IN).getY(), 1e-9);
        assertEquals(0.65, roadmap.getNodePosition(Rebuilt2026Roadmap.N_RED_BOT_TRENCH_IN).getY(), 1e-9);
    }

    @Test
    void testConstrainedCorridors() {
        List<FieldMap.AABB> corridors = roadmap.getConstrainedCorridors();
        assertEquals(4, corridors.size(), "Must specify the 4 trench corridors (Blue Top/Bot, Red Top/Bot)");
        for (FieldMap.AABB corridor : corridors) {
            assertTrue(corridor.minX < corridor.maxX);
            assertTrue(corridor.minY < corridor.maxY);
        }
    }
}
