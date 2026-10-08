package frc.robot.Navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Concrete {@link FieldRoadmap} implementation for the 2026 REBUILT game field.
 * Contains the 38 pre-validated strategic waypoints and 59 bidirectional edges.
 */
public final class Rebuilt2026Roadmap implements FieldRoadmap {

    // Node Index Constants (Matching StaticPathfinder)
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

    public static final int N_BLUE_TOWER_WEST_TOP = 34;
    public static final int N_BLUE_TOWER_WEST_BOT = 35;
    public static final int N_RED_TOWER_EAST_TOP = 36;
    public static final int N_RED_TOWER_EAST_BOT = 37;

    public static final int TOTAL_NODES = 38;

    /** Wall-band X for the tower corridor nodes, mirrored about the field centre. */
    public static final double TOWER_WALL_X = 0.75;

    /** Y lanes for the tower corridor nodes. */
    public static final double TOWER_WALL_TOP_Y = 5.50;
    public static final double TOWER_WALL_BOT_Y = 2.30;

    private static final Rebuilt2026Roadmap INSTANCE = new Rebuilt2026Roadmap();

    public static Rebuilt2026Roadmap getInstance() {
        return INSTANCE;
    }

    private final List<Node> nodes;
    private final List<FieldMap.AABB> constrainedCorridors;

    public Rebuilt2026Roadmap() {
        List<RawNode> rawNodes = new ArrayList<>();
        // Blue Alliance Staging & Low Zones
        rawNodes.add(new RawNode(0, "Blue Alliance Center", 2.40, 4.035));
        rawNodes.add(new RawNode(1, "Blue Alliance Top", 1.80, 6.00));
        rawNodes.add(new RawNode(2, "Blue Alliance Bottom", 1.80, 2.00));
        rawNodes.add(new RawNode(3, "Blue Hub Midfield Top Staging", 6.20, 5.75));
        rawNodes.add(new RawNode(4, "Blue Hub Midfield Bot Staging", 6.20, 2.32));

        // Blue Top Trench (Y = 7.42m corridor)
        rawNodes.add(new RawNode(5, "Blue Top Trench W", 2.60, 7.42));
        rawNodes.add(new RawNode(6, "Blue Top Trench In", 3.40, 7.42));
        rawNodes.add(new RawNode(7, "Blue Top Trench Out", 5.85, 7.42));
        rawNodes.add(new RawNode(8, "Blue Top Trench E", 6.65, 7.42));

        // Blue Bottom Trench (Y = 0.65m corridor)
        rawNodes.add(new RawNode(9, "Blue Bot Trench W", 2.60, 0.65));
        rawNodes.add(new RawNode(10, "Blue Bot Trench In", 3.40, 0.65));
        rawNodes.add(new RawNode(11, "Blue Bot Trench Out", 5.85, 0.65));
        rawNodes.add(new RawNode(12, "Blue Bot Trench E", 6.65, 0.65));

        // Midfield Crossings (Centerline X = 8.27m)
        rawNodes.add(new RawNode(13, "Midfield Top", 8.27, 6.20));
        rawNodes.add(new RawNode(14, "Midfield Center Top", 8.27, 4.90));
        rawNodes.add(new RawNode(15, "Midfield Center Bot", 8.27, 3.17));
        rawNodes.add(new RawNode(16, "Midfield Bottom", 8.27, 1.90));

        // Red Top Trench (Y = 7.42m corridor)
        rawNodes.add(new RawNode(17, "Red Top Trench W", 9.89, 7.42));
        rawNodes.add(new RawNode(18, "Red Top Trench In", 10.69, 7.42));
        rawNodes.add(new RawNode(19, "Red Top Trench Out", 13.14, 7.42));
        rawNodes.add(new RawNode(20, "Red Top Trench E", 13.94, 7.42));

        // Red Bottom Trench (Y = 0.65m corridor)
        rawNodes.add(new RawNode(21, "Red Bot Trench W", 9.89, 0.65));
        rawNodes.add(new RawNode(22, "Red Bot Trench In", 10.69, 0.65));
        rawNodes.add(new RawNode(23, "Red Bot Trench Out", 13.14, 0.65));
        rawNodes.add(new RawNode(24, "Red Bot Trench E", 13.94, 0.65));

        // Red Hub Midfield Staging & Alliance Staging
        rawNodes.add(new RawNode(25, "Red Hub Midfield Top Staging", 10.34, 5.75));
        rawNodes.add(new RawNode(26, "Red Hub Midfield Bot Staging", 10.34, 2.32));
        rawNodes.add(new RawNode(27, "Red Alliance Center", 14.14, 4.035));
        rawNodes.add(new RawNode(28, "Red Alliance Top", 14.74, 6.00));
        rawNodes.add(new RawNode(29, "Red Alliance Bottom", 14.74, 2.00));

        // Alliance Corner Bypass Nodes (smooth transit around Hubs)
        rawNodes.add(new RawNode(30, "Blue Alliance Top Bypass", 3.04, 5.75));
        rawNodes.add(new RawNode(31, "Blue Alliance Bot Bypass", 3.04, 2.32));
        rawNodes.add(new RawNode(32, "Red Alliance Top Bypass", 13.50, 5.75));
        rawNodes.add(new RawNode(33, "Red Alliance Bot Bypass", 13.50, 2.32));

        // Outer driver-wall corridors behind the climbing towers
        rawNodes.add(new RawNode(34, "Blue Tower West Top", TOWER_WALL_X, TOWER_WALL_TOP_Y));
        rawNodes.add(new RawNode(35, "Blue Tower West Bot", TOWER_WALL_X, TOWER_WALL_BOT_Y));
        rawNodes.add(new RawNode(36, "Red Tower East Top", FieldMap.FIELD_LENGTH - TOWER_WALL_X, TOWER_WALL_TOP_Y));
        rawNodes.add(new RawNode(37, "Red Tower East Bot", FieldMap.FIELD_LENGTH - TOWER_WALL_X, TOWER_WALL_BOT_Y));

        // Connect Bidirectional Edges
        connect(rawNodes, N_BLUE_ALLIANCE_TOP, N_BLUE_ALLIANCE_CTR);
        connect(rawNodes, N_BLUE_ALLIANCE_BOT, N_BLUE_ALLIANCE_CTR);
        connect(rawNodes, N_BLUE_ALLIANCE_TOP, N_BLUE_TOP_TRENCH_W);
        connect(rawNodes, N_BLUE_ALLIANCE_BOT, N_BLUE_BOT_TRENCH_W);

        connect(rawNodes, N_BLUE_ALLIANCE_CTR, N_BLUE_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_BLUE_ALLIANCE_TOP, N_BLUE_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_BLUE_ALLIANCE_TOP_BYPASS, N_BLUE_TOP_TRENCH_W);

        connect(rawNodes, N_BLUE_ALLIANCE_CTR, N_BLUE_ALLIANCE_BOT_BYPASS);
        connect(rawNodes, N_BLUE_ALLIANCE_BOT, N_BLUE_ALLIANCE_BOT_BYPASS);
        connect(rawNodes, N_BLUE_ALLIANCE_BOT_BYPASS, N_BLUE_BOT_TRENCH_W);

        connect(rawNodes, N_BLUE_TOP_TRENCH_W, N_BLUE_TOP_TRENCH_IN);
        connect(rawNodes, N_BLUE_TOP_TRENCH_IN, N_BLUE_TOP_TRENCH_OUT);
        connect(rawNodes, N_BLUE_TOP_TRENCH_OUT, N_BLUE_TOP_TRENCH_E);

        connect(rawNodes, N_BLUE_BOT_TRENCH_W, N_BLUE_BOT_TRENCH_IN);
        connect(rawNodes, N_BLUE_BOT_TRENCH_IN, N_BLUE_BOT_TRENCH_OUT);
        connect(rawNodes, N_BLUE_BOT_TRENCH_OUT, N_BLUE_BOT_TRENCH_E);

        connect(rawNodes, N_BLUE_TOP_TRENCH_E, N_MIDFIELD_TOP);
        connect(rawNodes, N_BLUE_TOP_TRENCH_E, N_BLUE_HUB_TOP_BYPASS);
        connect(rawNodes, N_BLUE_BOT_TRENCH_E, N_MIDFIELD_BOT);
        connect(rawNodes, N_BLUE_BOT_TRENCH_E, N_BLUE_HUB_BOT_BYPASS);
        connect(rawNodes, N_BLUE_HUB_TOP_BYPASS, N_MIDFIELD_CTR_TOP);
        connect(rawNodes, N_BLUE_HUB_BOT_BYPASS, N_MIDFIELD_CTR_BOT);
        connect(rawNodes, N_BLUE_HUB_TOP_BYPASS, N_MIDFIELD_TOP);
        connect(rawNodes, N_BLUE_HUB_BOT_BYPASS, N_MIDFIELD_BOT);

        connect(rawNodes, N_MIDFIELD_TOP, N_MIDFIELD_CTR_TOP);
        connect(rawNodes, N_MIDFIELD_CTR_TOP, N_MIDFIELD_CTR_BOT);
        connect(rawNodes, N_MIDFIELD_CTR_BOT, N_MIDFIELD_BOT);

        connect(rawNodes, N_MIDFIELD_TOP, N_RED_TOP_TRENCH_W);
        connect(rawNodes, N_MIDFIELD_BOT, N_RED_BOT_TRENCH_W);
        connect(rawNodes, N_RED_TOP_TRENCH_W, N_RED_HUB_TOP_BYPASS);
        connect(rawNodes, N_RED_BOT_TRENCH_W, N_RED_HUB_BOT_BYPASS);
        connect(rawNodes, N_MIDFIELD_CTR_TOP, N_RED_HUB_TOP_BYPASS);
        connect(rawNodes, N_MIDFIELD_CTR_BOT, N_RED_HUB_BOT_BYPASS);
        connect(rawNodes, N_MIDFIELD_TOP, N_RED_HUB_TOP_BYPASS);
        connect(rawNodes, N_MIDFIELD_BOT, N_RED_HUB_BOT_BYPASS);

        connect(rawNodes, N_RED_TOP_TRENCH_W, N_RED_TOP_TRENCH_IN);
        connect(rawNodes, N_RED_TOP_TRENCH_IN, N_RED_TOP_TRENCH_OUT);
        connect(rawNodes, N_RED_TOP_TRENCH_OUT, N_RED_TOP_TRENCH_E);

        connect(rawNodes, N_RED_BOT_TRENCH_W, N_RED_BOT_TRENCH_IN);
        connect(rawNodes, N_RED_BOT_TRENCH_IN, N_RED_BOT_TRENCH_OUT);
        connect(rawNodes, N_RED_BOT_TRENCH_OUT, N_RED_BOT_TRENCH_E);

        connect(rawNodes, N_RED_TOP_TRENCH_E, N_RED_ALLIANCE_TOP);
        connect(rawNodes, N_RED_BOT_TRENCH_E, N_RED_ALLIANCE_BOT);
        connect(rawNodes, N_RED_ALLIANCE_TOP, N_RED_ALLIANCE_CTR);
        connect(rawNodes, N_RED_ALLIANCE_BOT, N_RED_ALLIANCE_CTR);

        connect(rawNodes, N_RED_ALLIANCE_CTR, N_RED_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_RED_ALLIANCE_TOP, N_RED_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_RED_ALLIANCE_TOP_BYPASS, N_RED_TOP_TRENCH_E);

        connect(rawNodes, N_RED_ALLIANCE_CTR, N_RED_ALLIANCE_BOT_BYPASS);
        connect(rawNodes, N_RED_ALLIANCE_BOT, N_RED_ALLIANCE_BOT_BYPASS);
        connect(rawNodes, N_RED_ALLIANCE_BOT_BYPASS, N_RED_BOT_TRENCH_E);

        connect(rawNodes, N_BLUE_TOWER_WEST_TOP, N_BLUE_ALLIANCE_TOP);
        connect(rawNodes, N_BLUE_TOWER_WEST_TOP, N_BLUE_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_BLUE_TOWER_WEST_BOT, N_BLUE_ALLIANCE_BOT);
        connect(rawNodes, N_BLUE_TOWER_WEST_BOT, N_BLUE_ALLIANCE_BOT_BYPASS);

        connect(rawNodes, N_RED_TOWER_EAST_TOP, N_RED_ALLIANCE_TOP);
        connect(rawNodes, N_RED_TOWER_EAST_TOP, N_RED_ALLIANCE_TOP_BYPASS);
        connect(rawNodes, N_RED_TOWER_EAST_BOT, N_RED_ALLIANCE_BOT);
        connect(rawNodes, N_RED_TOWER_EAST_BOT, N_RED_ALLIANCE_BOT_BYPASS);

        // Convert to immutable Node records
        List<Node> built = new ArrayList<>(rawNodes.size());
        for (RawNode rn : rawNodes) {
            built.add(new Node(rn.id, rn.name, rn.pos, Collections.unmodifiableList(rn.neighbors)));
        }
        this.nodes = Collections.unmodifiableList(built);

        // Constrained corridors (Trenches)
        this.constrainedCorridors = List.of(
            new FieldMap.AABB("Blue Top Trench Corridor",
                FieldMap.Trenches.BLUE_TRENCH_MIN_X, FieldMap.Trenches.BLUE_TRENCH_MAX_X,
                FieldMap.Trenches.TOP_TRENCH_MIN_Y, FieldMap.Trenches.TOP_TRENCH_MAX_Y),
            new FieldMap.AABB("Blue Bottom Trench Corridor",
                FieldMap.Trenches.BLUE_TRENCH_MIN_X, FieldMap.Trenches.BLUE_TRENCH_MAX_X,
                FieldMap.Trenches.BOT_TRENCH_MIN_Y, FieldMap.Trenches.BOT_TRENCH_MAX_Y),
            new FieldMap.AABB("Red Top Trench Corridor",
                FieldMap.Trenches.RED_TRENCH_MIN_X, FieldMap.Trenches.RED_TRENCH_MAX_X,
                FieldMap.Trenches.TOP_TRENCH_MIN_Y, FieldMap.Trenches.TOP_TRENCH_MAX_Y),
            new FieldMap.AABB("Red Bottom Trench Corridor",
                FieldMap.Trenches.RED_TRENCH_MIN_X, FieldMap.Trenches.RED_TRENCH_MAX_X,
                FieldMap.Trenches.BOT_TRENCH_MIN_Y, FieldMap.Trenches.BOT_TRENCH_MAX_Y)
        );
    }

    private static void connect(List<RawNode> rawNodes, int id1, int id2) {
        RawNode n1 = rawNodes.get(id1);
        RawNode n2 = rawNodes.get(id2);
        if (!n1.neighbors.contains(id2)) {
            n1.neighbors.add(id2);
        }
        if (!n2.neighbors.contains(id1)) {
            n2.neighbors.add(id1);
        }
    }

    @Override
    public int getNodeCount() {
        return nodes.size();
    }

    @Override
    public Node getNode(int id) {
        return nodes.get(id);
    }

    @Override
    public List<Node> getAllNodes() {
        return nodes;
    }

    @Override
    public List<Integer> getNeighbors(int id) {
        return nodes.get(id).neighbors();
    }

    @Override
    public List<FieldMap.AABB> getConstrainedCorridors() {
        return constrainedCorridors;
    }

    private static class RawNode {
        final int id;
        final String name;
        final Translation2d pos;
        final List<Integer> neighbors = new ArrayList<>();

        RawNode(int id, String name, double x, double y) {
            this.id = id;
            this.name = name;
            this.pos = new Translation2d(x, y);
        }
    }
}
