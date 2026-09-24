package frc.robot.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Utils.AllianceFlipUtil;

/**
 * Tactical navigation waypoints (Glide Points) and Auto-Tunneling routes.
 * Defined strictly in canonical Blue-origin coordinates and dynamically mirrored for Red Alliance.
 */
public class GlideConstants {

    public static final double FIELD_LENGTH = AllianceFlipUtil.FIELD_LENGTH;
    public static final double FIELD_WIDTH = AllianceFlipUtil.FIELD_WIDTH;

    // Y-Coordinates for Lanes
    public static final double Y_BOT_LANE = 0.65;
    public static final double Y_TOP_LANE = FIELD_WIDTH - 0.65;

    public static class GlidePoint {
        public final String name;
        public final Pose2d pose;
        public final boolean isTunnelEntrance;
        public final Pose2d tunnelExitPose;

        public GlidePoint(String name, Pose2d pose) {
            this(name, pose, false, null);
        }

        public GlidePoint(String name, Pose2d pose, boolean isTunnel, Pose2d exit) {
            this.name = name;
            this.pose = pose;
            this.isTunnelEntrance = isTunnel;
            this.tunnelExitPose = exit;
        }

        public Pose2d pose() {
            return pose;
        }

        public String name() {
            return name;
        }
    }

    // ----------------------------------------------------------------
    // CANONICAL BLUE ALLIANCE WAYPOINTS
    // ----------------------------------------------------------------
    public static final List<GlidePoint> BLUE_GLIDE_POINTS = List.of(
            new GlidePoint("Blue Feeder Top", new Pose2d(1.50, 6.00, Rotation2d.fromDegrees(-35))),
            new GlidePoint("Blue Feeder Bottom", new Pose2d(1.50, 2.20, Rotation2d.fromDegrees(35))),
            new GlidePoint("Blue Right Side Climb", new Pose2d(1.05, 2.88, Rotation2d.fromDegrees(180))),
            new GlidePoint("Blue Hub Front", new Pose2d(5.60, 4.035, Rotation2d.fromDegrees(180))),
            new GlidePoint("Blue Hub Back", new Pose2d(2.60, 4.035, Rotation2d.fromDegrees(0))),
            new GlidePoint("Blue Top Trench",
                    new Pose2d(3.50, Y_TOP_LANE, Rotation2d.fromDegrees(0)), true,
                    new Pose2d(5.75, Y_TOP_LANE, Rotation2d.fromDegrees(0))),
            new GlidePoint("Blue Bottom Trench",
                    new Pose2d(3.50, Y_BOT_LANE, Rotation2d.fromDegrees(0)), true,
                    new Pose2d(5.75, Y_BOT_LANE, Rotation2d.fromDegrees(0))),
            new GlidePoint("Midfield Top", new Pose2d(8.27, 6.10, Rotation2d.fromDegrees(-90))),
            new GlidePoint("Midfield Bottom", new Pose2d(8.27, 2.00, Rotation2d.fromDegrees(90)))
    );

    // ----------------------------------------------------------------
    // RED ALLIANCE WAYPOINTS (Dynamically Mirrored via AllianceFlipUtil)
    // ----------------------------------------------------------------
    public static final List<GlidePoint> RED_GLIDE_POINTS;

    static {
        List<GlidePoint> redPoints = new ArrayList<>();
        for (GlidePoint p : BLUE_GLIDE_POINTS) {
            String redName = p.name.replace("Blue", "Red");
            Pose2d redPose = AllianceFlipUtil.apply(p.pose, true);
            Pose2d redExit = p.isTunnelEntrance && p.tunnelExitPose != null
                    ? AllianceFlipUtil.apply(p.tunnelExitPose, true)
                    : null;
            redPoints.add(new GlidePoint(redName, redPose, p.isTunnelEntrance, redExit));
        }
        RED_GLIDE_POINTS = Collections.unmodifiableList(redPoints);
    }

    public static final Map<String, GlidePoint> GLIDE_POINTS;

    static {
        Map<String, GlidePoint> map = new HashMap<>();
        for (GlidePoint p : BLUE_GLIDE_POINTS) {
            map.put(p.name, p);
        }
        for (GlidePoint p : RED_GLIDE_POINTS) {
            map.put(p.name, p);
        }
        GLIDE_POINTS = Collections.unmodifiableMap(map);
    }

    /**
     * Finds a matching tunnel entrance waypoint for the given target pose if within tolerance.
     *
     * @param targetPose The target pose to check
     * @return The matching GlidePoint if it is a tunnel entrance, or null if not found
     */
    public static GlidePoint getMatchingTunnelEntrance(Pose2d targetPose) {
        return getMatchingTunnelEntrance(targetPose, 0.40);
    }

    /**
     * Finds a matching tunnel entrance waypoint for the given target pose if within specified tolerance.
     *
     * @param targetPose The target pose to check
     * @param toleranceMeters Distance tolerance in meters
     * @return The matching GlidePoint if it is a tunnel entrance, or null if not found
     */
    public static GlidePoint getMatchingTunnelEntrance(Pose2d targetPose, double toleranceMeters) {
        if (targetPose == null) {
            return null;
        }
        for (GlidePoint p : GLIDE_POINTS.values()) {
            if (p.isTunnelEntrance && p.tunnelExitPose != null) {
                if (p.pose.getTranslation().getDistance(targetPose.getTranslation()) <= toleranceMeters) {
                    return p;
                }
            }
        }
        return null;
    }
}
