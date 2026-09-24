package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Data.Constants;
import frc.robot.Data.FieldMap;

/**
 * Standardizes coordinate geometry to Blue-origin coordinates across the entire codebase.
 * Automatically mirrors translations, rotations, and poses when on the Red Alliance.
 */
public class AllianceFlipUtil {

    public static final double FIELD_LENGTH = FieldMap.FIELD_LENGTH;
    public static final double FIELD_WIDTH = FieldMap.FIELD_WIDTH;

    /**
     * @return true if currently on Red alliance.
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /**
     * Flips a Translation2d based on an explicit alliance flag.
     */
    public static Translation2d apply(Translation2d translation, boolean isRed) {
        if (isRed) {
            return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
        }
        return translation;
    }

    /**
     * Flips a Translation2d if on Red Alliance, leaving Blue unchanged.
     */
    public static Translation2d apply(Translation2d translation) {
        return apply(translation, isRedAlliance());
    }

    /**
     * Flips a Translation3d based on an explicit alliance flag.
     */
    public static Translation3d apply(Translation3d translation, boolean isRed) {
        if (isRed) {
            return new Translation3d(FIELD_LENGTH - translation.getX(), translation.getY(), translation.getZ());
        }
        return translation;
    }

    /**
     * Flips a Translation3d if on Red Alliance, leaving Blue unchanged.
     */
    public static Translation3d apply(Translation3d translation) {
        return apply(translation, isRedAlliance());
    }

    /**
     * Flips a Rotation2d based on an explicit alliance flag (mirrored across field X axis).
     */
    public static Rotation2d apply(Rotation2d rotation, boolean isRed) {
        if (isRed) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        }
        return rotation;
    }

    /**
     * Flips a Rotation2d if on Red Alliance.
     */
    public static Rotation2d apply(Rotation2d rotation) {
        return apply(rotation, isRedAlliance());
    }

    /**
     * Flips a Pose2d based on an explicit alliance flag.
     */
    public static Pose2d apply(Pose2d pose, boolean isRed) {
        if (isRed) {
            return new Pose2d(apply(pose.getTranslation(), isRed), apply(pose.getRotation(), isRed));
        }
        return pose;
    }

    /**
     * Flips a Pose2d if on Red Alliance.
     */
    public static Pose2d apply(Pose2d pose) {
        return apply(pose, isRedAlliance());
    }

    /**
     * Converts a driver-relative heading intent into an absolute global field-relative Rotation2d.
     * On Blue Alliance: Heading 0° is facing away from Blue DS (+X).
     * On Red Alliance: Heading 0° relative to driver is facing away from Red DS (-X, or 180° in field coordinates).
     * 
     * @param driverHeading Relative heading angle from driver's perspective (0° = away, 90° = left, 180° = toward, -90° = right)
     * @return Canonical field-relative Rotation2d in standard Blue-origin coordinates
     */
    public static Rotation2d getDriverRelativeHeading(Rotation2d driverHeading) {
        if (isRedAlliance()) {
            return driverHeading.plus(Rotation2d.fromDegrees(180));
        }
        return driverHeading;
    }

    /**
     * Checks if a given field translation is inside the specified alliance's scoring zone.
     * Blue Alliance Zone: X <= 4.597m (Blue Alliance Wall to Blue Hub)
     * Red Alliance Zone: X >= 11.938m (Red Hub to Red Alliance Wall)
     * 
     * @param translation Field translation in standard Blue-origin coordinates
     * @param isRedAlliance True if checking Red Alliance zone, false for Blue Alliance zone
     * @return True if translation is within the alliance zone
     */
    public static boolean isTranslationInAllianceZone(Translation2d translation, boolean isRedAlliance) {
        return FieldMap.AllianceZones.isInAllianceZone(translation, isRedAlliance);
    }

    /**
     * Checks if a given field pose is inside the specified alliance's scoring zone.
     * 
     * @param pose Field pose in standard Blue-origin coordinates
     * @param isRedAlliance True if checking Red Alliance zone, false for Blue Alliance zone
     * @return True if pose is within the alliance zone
     */
    public static boolean isPoseInAllianceZone(Pose2d pose, boolean isRedAlliance) {
        if (pose == null) return false;
        return isTranslationInAllianceZone(pose.getTranslation(), isRedAlliance);
    }

    /**
     * Checks if a given field pose is inside the current robot alliance's scoring zone.
     * 
     * @param pose Field pose in standard Blue-origin coordinates
     * @return True if pose is within the current alliance zone
     */
    public static boolean isPoseInAllianceZone(Pose2d pose) {
        return isPoseInAllianceZone(pose, isRedAlliance());
    }
}
