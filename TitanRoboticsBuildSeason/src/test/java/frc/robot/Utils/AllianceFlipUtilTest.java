package frc.robot.Utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

public class AllianceFlipUtilTest {

    @Test
    public void testBlueAlliancePreservesCoordinates() {
        Translation2d bluePoint = new Translation2d(4.597, 4.035);
        Translation2d result = AllianceFlipUtil.apply(bluePoint, false);
        assertEquals(4.597, result.getX(), 0.001);
        assertEquals(4.035, result.getY(), 0.001);

        Rotation2d rot = Rotation2d.fromDegrees(45);
        assertEquals(45.0, AllianceFlipUtil.apply(rot, false).getDegrees(), 0.001);
    }

    @Test
    public void testRedAllianceMirrorsCoordinates() {
        Translation2d blueHub = new Translation2d(4.597, 4.035);
        Translation2d redHub = AllianceFlipUtil.apply(blueHub, true);

        // 16.535 - 4.597 = 11.938
        assertEquals(11.938, redHub.getX(), 0.005);
        assertEquals(4.035, redHub.getY(), 0.001);

        Translation3d blueHub3d = new Translation3d(4.597, 4.035, 1.829);
        Translation3d redHub3d = AllianceFlipUtil.apply(blueHub3d, true);
        assertEquals(11.938, redHub3d.getX(), 0.005);
        assertEquals(4.035, redHub3d.getY(), 0.001);
        assertEquals(1.829, redHub3d.getZ(), 0.001);

        // Heading mirror: facing 0 deg (facing red side) becomes 180 deg
        Rotation2d rot0 = Rotation2d.fromDegrees(0);
        Rotation2d flipped0 = AllianceFlipUtil.apply(rot0, true);
        assertEquals(180.0, Math.abs(flipped0.getDegrees()), 0.001);

        // Pose flip
        Pose2d pose = new Pose2d(blueHub, rot0);
        Pose2d flippedPose = AllianceFlipUtil.apply(pose, true);
        assertEquals(11.938, flippedPose.getX(), 0.005);
        assertEquals(4.035, flippedPose.getY(), 0.001);
        assertEquals(180.0, Math.abs(flippedPose.getRotation().getDegrees()), 0.001);
    }
}
