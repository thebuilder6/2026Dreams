
package frc.robot.Data;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GlideConstants {

    // Field Dimensions (approximate for FRC 2026 Rebuilt)
    // X: 0.0 to ~16.54m, Y: 0.0 to ~8.21m

    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.05;

    // Y-Coordinates for Lanes
    private static final double Y_BOT_LANE = 0.65;
    private static final double Y_TOP_LANE = FIELD_WIDTH - 0.65;

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
    }

    public static final Map<String, GlidePoint> GLIDE_POINTS = Map.ofEntries(
            // ----------------------------------------------------------------
            // BLUE ALLIANCE POINTS
            // ----------------------------------------------------------------

            // Feeders / Loading Zones (Corners)
            Map.entry("Blue Feeder Top",
                    new GlidePoint("Blue Feeder Top", new Pose2d(1.50, 6.00, Rotation2d.fromDegrees(-35)))),
            Map.entry("Blue Feeder Bottom",
                    new GlidePoint("Blue Feeder Bottom", new Pose2d(1.50, 2.20, Rotation2d.fromDegrees(35)))),

            Map.entry("Blue Right Side Climb",
                    new GlidePoint("Blue Right Side Climb", new Pose2d(1.05, 2.88, Rotation2d.fromDegrees(180)))), // Facing
                                                                                                                   // Hub
                                                                                                                   // to
                                                                                                                   // score

            // Hub (Scoring Structure)
            Map.entry("Blue Hub Front",
                    new GlidePoint("Blue Hub Front", new Pose2d(5.60, 4.10, Rotation2d.fromDegrees(180)))), // Facing
                                                                                                            // Hub to
                                                                                                            // score
            Map.entry("Blue Hub Back",
                    new GlidePoint("Blue Hub Back", new Pose2d(2.60, 4.10, Rotation2d.fromDegrees(0)))), // Protected
                                                                                                         // zone

            // Trenches (Tunnels) - "Outbound" (From Blue Zone -> Midfield)
            // Includes Entry point and Exit point for automatic tunneling
            Map.entry("Blue Top Trench",
                    new GlidePoint("Blue Top Trench",
                            new Pose2d(3.5, Y_TOP_LANE, Rotation2d.fromDegrees(0)), true, // Entry
                            new Pose2d(5.75, Y_TOP_LANE, Rotation2d.fromDegrees(0)))), // Exit

            Map.entry("Blue Bottom Trench",
                    new GlidePoint("Blue Bottom Trench",
                            new Pose2d(3.5, Y_BOT_LANE, Rotation2d.fromDegrees(0)), true, // Entry
                            new Pose2d(5.75, Y_BOT_LANE, Rotation2d.fromDegrees(0)))), // Exit

            // Midfield / Crossing
            Map.entry("Midfield Top",
                    new GlidePoint("Midfield Top", new Pose2d(8.27, 6.10, Rotation2d.fromDegrees(-90)))),
            Map.entry("Midfield Bottom",
                    new GlidePoint("Midfield Bottom", new Pose2d(8.27, 2.00, Rotation2d.fromDegrees(90)))),

            // ----------------------------------------------------------------
            // RED ALLIANCE POINTS (Mirrored)
            // ----------------------------------------------------------------

            // Feeders / Loading Zones (Corners)
            Map.entry("Red Feeder Top",
                    new GlidePoint("Red Feeder Top",
                            new Pose2d(FIELD_LENGTH - 1.50, 6.00, Rotation2d.fromDegrees(-145)))),
            Map.entry("Red Feeder Bottom",
                    new GlidePoint("Red Feeder Bottom",
                            new Pose2d(FIELD_LENGTH - 1.50, 2.20, Rotation2d.fromDegrees(145)))),

            Map.entry("Red Left Side Climb",
                    new GlidePoint("Red Left Side Climb",
                            new Pose2d(FIELD_LENGTH - 1.05, 2.88, Rotation2d.fromDegrees(0)))),

            // Hub (Scoring Structure)
            Map.entry("Red Hub Front",
                    new GlidePoint("Red Hub Front", new Pose2d(FIELD_LENGTH - 5.60, 4.10, Rotation2d.fromDegrees(0)))), // Facing
                                                                                                                        // Hub
                                                                                                                        // to
                                                                                                                        // score
            Map.entry("Red Hub Back",
                    new GlidePoint("Red Hub Back", new Pose2d(13.94, 4.10, Rotation2d.fromDegrees(180)))), // Protected
                                                                                                           // zone

            // Trenches (Tunnels) - "Outbound" (From Red Zone -> Midfield)
            // Note: Heading is 180 to drive "forward" towards Blue side, or 0 to reverse.
            // Assuming 180 (Forward) for Red->Blue travel.

            // Tunnels (Driving towards Blue side, Heading 180)
            Map.entry("Red Top Trench",
                    new GlidePoint("Red Top Trench",
                            new Pose2d(FIELD_LENGTH - 3.5, Y_TOP_LANE, Rotation2d.fromDegrees(180)), true, // Entry
                            new Pose2d(FIELD_LENGTH - 5.75, Y_TOP_LANE, Rotation2d.fromDegrees(180)))), // Exit

            Map.entry("Red Bottom Trench",
                    new GlidePoint("Red Bottom Trench",
                            new Pose2d(FIELD_LENGTH - 3.5, Y_BOT_LANE, Rotation2d.fromDegrees(180)), true, // Entry
                            new Pose2d(FIELD_LENGTH - 5.75, Y_BOT_LANE, Rotation2d.fromDegrees(180)))) // Exit
    );

    // ----------------------------------------------------------------
    // LISTS
    // ----------------------------------------------------------------

    public static final List<GlidePoint> BLUE_GLIDE_POINTS = List.of(
            GLIDE_POINTS.get("Blue Feeder Top"),
            GLIDE_POINTS.get("Blue Feeder Bottom"),
            GLIDE_POINTS.get("Blue Hub Front"),
            GLIDE_POINTS.get("Blue Hub Back"),
            GLIDE_POINTS.get("Blue Top Trench"),
            GLIDE_POINTS.get("Blue Bottom Trench"),
            GLIDE_POINTS.get("Midfield Top"),
            GLIDE_POINTS.get("Midfield Bottom"),
            GLIDE_POINTS.get("Red Top Trench"),
            GLIDE_POINTS.get("Red Bottom Trench"));

    public static final List<GlidePoint> RED_GLIDE_POINTS = List.of(
            GLIDE_POINTS.get("Red Feeder Top"),
            GLIDE_POINTS.get("Red Feeder Bottom"),
            GLIDE_POINTS.get("Red Hub Front"),
            GLIDE_POINTS.get("Red Hub Back"),
            GLIDE_POINTS.get("Red Top Trench"),
            GLIDE_POINTS.get("Red Bottom Trench"),
            GLIDE_POINTS.get("Midfield Top"), // Shared points can be in both if needed
            GLIDE_POINTS.get("Midfield Bottom"),
            GLIDE_POINTS.get("Blue Top Trench"),
            GLIDE_POINTS.get("Blue Bottom Trench"));
}
