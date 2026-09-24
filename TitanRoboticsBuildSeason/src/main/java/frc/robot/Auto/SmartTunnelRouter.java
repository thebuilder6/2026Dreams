package frc.robot.Auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Data.FieldMap;
import frc.robot.Data.GlideConstants;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * SmartTunnelRouter provides intelligent, collision-free trench navigation:
 * 1. Bi-Directional Entry/Exit: Resolves whether approaching from Alliance Zone or Midfield.
 * 2. Opponent Blockage Detection & Auto-Diversion: Senses active obstacles in trench corridors and reroutes to the clear trench.
 * 3. Pre-Entry Funneling & Exit Transition: Generates approach and launch waypoints to prevent clipping the 53-inch steel frame.
 */
public class SmartTunnelRouter {

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

    // Trench corridor boundaries (Consolidated via FieldMap)
    public static final double BLUE_TRENCH_X_MIN = FieldMap.Trenches.BLUE_TRENCH_MIN_X;
    public static final double BLUE_TRENCH_X_MAX = FieldMap.Trenches.BLUE_TRENCH_MAX_X;
    public static final double RED_TRENCH_X_MIN = FieldMap.Trenches.RED_TRENCH_MIN_X;
    public static final double RED_TRENCH_X_MAX = FieldMap.Trenches.RED_TRENCH_MAX_X;

    public static final double TOP_TRENCH_Y_MIN = FieldMap.Trenches.TOP_TRENCH_MIN_Y;
    public static final double TOP_TRENCH_Y_MAX = FieldMap.FIELD_WIDTH;
    public static final double BOTTOM_TRENCH_Y_MIN = FieldMap.Trenches.BOT_TRENCH_MIN_Y;
    public static final double BOTTOM_TRENCH_Y_MAX = FieldMap.Trenches.BOT_TRENCH_MAX_Y;

    public static final double Y_TOP_CENTERLINE = FieldMap.Trenches.TOP_CORRIDOR_Y;
    public static final double Y_BOT_CENTERLINE = FieldMap.Trenches.BOT_CORRIDOR_Y;

    /**
     * Checks if dynamic obstacles are currently blocking the specified trench corridor.
     */
    public static boolean isTrenchBlocked(boolean isTopTrench, boolean isBlueAlliance) {
        double xMin = isBlueAlliance ? BLUE_TRENCH_X_MIN : RED_TRENCH_X_MIN;
        double xMax = isBlueAlliance ? BLUE_TRENCH_X_MAX : RED_TRENCH_X_MAX;
        double yMin = isTopTrench ? TOP_TRENCH_Y_MIN : BOTTOM_TRENCH_Y_MIN;
        double yMax = isTopTrench ? TOP_TRENCH_Y_MAX : BOTTOM_TRENCH_Y_MAX;

        return DynamicRouter.isZoneBlocked(xMin, xMax, yMin, yMax);
    }

    /**
     * Intelligently plans an unblocked, bi-directional tunnel route based on the robot's current pose.
     *
     * @param currentPose Current robot pose on the field
     * @param preferTopTrench Preferred trench corridor (top or bottom)
     * @return Planned TunnelRoute with funneled approach, corridor transit, and exit points
     */
    public static TunnelRoute planTunnelRoute(Pose2d currentPose, boolean preferTopTrench) {
        boolean isRed = AllianceFlipUtil.isRedAlliance();
        boolean isBlue = !isRed;

        // 1. Evaluate Obstacle Congestion in both trenches
        boolean topBlocked = isTrenchBlocked(true, isBlue);
        boolean botBlocked = isTrenchBlocked(false, isBlue);

        boolean chosenTop = preferTopTrench;
        boolean wasDiverted = false;

        if (preferTopTrench && topBlocked && !botBlocked) {
            chosenTop = false; // Divert to bottom trench
            wasDiverted = true;
        } else if (!preferTopTrench && botBlocked && !topBlocked) {
            chosenTop = true; // Divert to top trench
            wasDiverted = true;
        }

        double yLane = chosenTop ? Y_TOP_CENTERLINE : Y_BOT_CENTERLINE;
        TrenchCorridor corridor = chosenTop ? TrenchCorridor.TOP_TRENCH : TrenchCorridor.BOTTOM_TRENCH;

        // 2. Determine Bi-Directional Entry and Exit based on current robot X
        double robotX = currentPose.getX();
        boolean isWestToEast;
        double entranceX;
        double exitX;
        double preEntranceX;
        double postExitX;
        Rotation2d corridorHeading;

        if (isBlue) {
            // Blue Alliance: Alliance zone is X < 4.60, Midfield is X >= 4.60
            if (robotX < 4.60) {
                // West -> East (Alliance Zone to Midfield)
                isWestToEast = true;
                preEntranceX = 2.90;
                entranceX = 3.50;
                exitX = 5.75;
                postExitX = 6.35;
                corridorHeading = Rotation2d.fromDegrees(0);
            } else {
                // East -> West (Midfield returning to Alliance Zone)
                isWestToEast = false;
                preEntranceX = 6.35;
                entranceX = 5.75;
                exitX = 3.50;
                postExitX = 2.90;
                corridorHeading = Rotation2d.fromDegrees(180);
            }
        } else {
            // Red Alliance: Alliance zone is X > 11.94, Midfield is X <= 11.94
            if (robotX > 11.94) {
                // East -> West (Red Alliance Zone to Midfield)
                isWestToEast = false;
                preEntranceX = 13.64;
                entranceX = 13.04;
                exitX = 10.79;
                postExitX = 10.19;
                corridorHeading = Rotation2d.fromDegrees(180);
            } else {
                // West -> East (Midfield returning to Red Alliance Zone)
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

    /**
     * Determines whether a given target pose represents an intent to tunnel through a trench.
     */
    public static boolean isTunnelTarget(Pose2d targetPose) {
        if (targetPose == null) return false;
        return GlideConstants.getMatchingTunnelEntrance(targetPose) != null;
    }
}
