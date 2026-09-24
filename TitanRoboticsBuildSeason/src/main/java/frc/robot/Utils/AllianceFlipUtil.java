package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Data.Constants;

/**
 * Standardizes coordinate geometry to Blue-origin coordinates across the entire codebase.
 * Automatically mirrors translations, rotations, and poses when on the Red Alliance.
 */
public class AllianceFlipUtil {

    public static final double FIELD_LENGTH = 16.535; // 4.597 + 11.938
    public static final double FIELD_WIDTH = 8.070;

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
}
