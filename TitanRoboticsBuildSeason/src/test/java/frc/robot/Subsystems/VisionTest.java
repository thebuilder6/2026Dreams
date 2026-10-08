package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Subsystems.Vision.RejectionReason;
import frc.robot.Subsystems.vision.CameraConfig;
import frc.robot.Subsystems.vision.CameraConfig.CameraRole;
import frc.robot.Subsystems.vision.CameraConfig.CameraType;
import frc.robot.Subsystems.vision.VisionConfig;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.Subsystems.vision.VisionIOSim;

public class VisionTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testDeclarativeCameraConfiguration() {
        CameraConfig frontLL = CameraConfig.limelight("limelight-front")
                .withTransform(new Transform3d(new Translation3d(0.30, 0.05, 0.50), new Rotation3d(0.0, Math.toRadians(20.0), 0.0)))
                .withRole(CameraRole.APRILTAG)
                .withStdDevMultiplier(0.9)
                .withMegaTag2(true);

        assertEquals("limelight-front", frontLL.getName());
        assertEquals(CameraType.LIMELIGHT, frontLL.getType());
        assertEquals(CameraRole.APRILTAG, frontLL.getRole());
        assertEquals(0.30, frontLL.getCameraForwardOffsetMeters(), 1e-4);
        assertEquals(0.50, frontLL.getCameraHeightMeters(), 1e-4);
        assertEquals(20.0, frontLL.getCameraPitchDegrees(), 1e-2);
        assertEquals(0.9, frontLL.getStdDevMultiplier(), 1e-4);
        assertTrue(frontLL.isMegaTag2());
        assertTrue(frontLL.isEnabled());

        CameraConfig coproc = CameraConfig.photonVision("orange-pi-intake")
                .withRole(CameraRole.OBJECT_DETECTION)
                .withStdDevMultiplier(1.5)
                .withEnabled(false);

        assertEquals("orange-pi-intake", coproc.getName());
        assertEquals(CameraType.PHOTONVISION, coproc.getType());
        assertEquals(CameraRole.OBJECT_DETECTION, coproc.getRole());
        assertEquals(1.5, coproc.getStdDevMultiplier(), 1e-4);
        assertFalse(coproc.isEnabled());
    }

    @Test
    public void testVisionConfigPresets() {
        List<CameraConfig> dual = VisionConfig.Presets.dualDefault();
        assertEquals(2, dual.size());
        assertEquals("limelight-front", dual.get(0).getName());
        assertEquals("rubik-pi-coprocessor", dual.get(1).getName());

        List<CameraConfig> singleLL = VisionConfig.Presets.singleLimelight();
        assertEquals(1, singleLL.size());
        assertEquals(CameraType.LIMELIGHT, singleLL.get(0).getType());

        List<CameraConfig> singlePV = VisionConfig.Presets.singlePhotonVision();
        assertEquals(1, singlePV.size());
        assertEquals(CameraType.PHOTONVISION, singlePV.get(0).getType());

        List<CameraConfig> dualPV = VisionConfig.Presets.dualPhotonVision();
        assertEquals(2, dualPV.size());
        assertEquals("photon-front", dualPV.get(0).getName());
        assertEquals("photon-back", dualPV.get(1).getName());
    }

    @Test
    public void testMultiCameraRegistrationAndAccessors() {
        CameraConfig c1 = CameraConfig.limelight("ll-left");
        CameraConfig c2 = CameraConfig.limelight("ll-right");
        CameraConfig c3 = CameraConfig.photonVision("pv-back");

        VisionIOSim s1 = new VisionIOSim(c1);
        VisionIOSim s2 = new VisionIOSim(c2);
        VisionIOSim s3 = new VisionIOSim(c3);

        Vision vision = new Vision(Arrays.asList(c1, c2, c3), Arrays.asList(s1, s2, s3));

        assertEquals(3, vision.getCameras().size());
        assertNotNull(vision.getCamera("ll-left"));
        assertNotNull(vision.getCamera("ll-right"));
        assertNotNull(vision.getCamera("pv-back"));
        assertNull(vision.getCamera("non-existent"));

        assertEquals("ll-left", vision.getPrimaryCamera().getName());
        assertEquals("ll-right", vision.getSecondaryCamera().getName());
        assertTrue(vision.isAllCamerasConnected());
    }

    @Test
    public void testRejectionGatesMatrix() {
        CameraConfig cfg = CameraConfig.limelight("ll-test");
        VisionIOSim sim = new VisionIOSim(cfg);
        Vision vision = new Vision(Arrays.asList(cfg), Arrays.asList(sim));

        // 1. State: No target
        VisionIO mockNoTargetIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = false;
                in.tagCount = 0;
            }
        };
        Vision visionNoTarget = new Vision(Arrays.asList(cfg), Arrays.asList(mockNoTargetIO));
        visionNoTarget.update();
        assertEquals(RejectionReason.NO_TARGET, visionNoTarget.getRejectionReason("ll-test"));
        assertFalse(visionNoTarget.isAccepted());

        // 2. High latency rejection (> 150ms)
        VisionIO mockLatencyIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.0;
                in.latencyMs = 200.0; // Exceeds 150ms limit
                in.estimatedPose = new Pose2d(5.0, 5.0, new Rotation2d());
            }
        };
        Vision visionLatency = new Vision(Arrays.asList(cfg), Arrays.asList(mockLatencyIO));
        visionLatency.update();
        assertEquals(RejectionReason.HIGH_LATENCY, visionLatency.getRejectionReason("ll-test"));
        assertFalse(visionLatency.isAccepted());

        // 3. High tag distance rejection (> 4.0m)
        VisionIO mockDistanceIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 6.5; // Exceeds 4.0m limit
                in.latencyMs = 20.0;
                in.estimatedPose = new Pose2d(5.0, 5.0, new Rotation2d());
            }
        };
        Vision visionDist = new Vision(Arrays.asList(cfg), Arrays.asList(mockDistanceIO));
        visionDist.update();
        assertEquals(RejectionReason.HIGH_DISTANCE, visionDist.getRejectionReason("ll-test"));

        // 4. Single-tag high ambiguity rejection (> 0.40)
        VisionIO mockAmbiguityIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 1;
                in.avgTagDist = 2.0;
                in.latencyMs = 20.0;
                in.ambiguity = 0.65; // High ambiguity
                in.estimatedPose = new Pose2d(5.0, 5.0, new Rotation2d());
            }
        };
        Vision visionAmbiguity = new Vision(Arrays.asList(cfg), Arrays.asList(mockAmbiguityIO));
        visionAmbiguity.update();
        assertEquals(RejectionReason.HIGH_AMBIGUITY, visionAmbiguity.getRejectionReason("ll-test"));

        // 5. Outside field boundary rejection
        VisionIO mockOutsideIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.0;
                in.latencyMs = 20.0;
                in.estimatedPose = new Pose2d(-2.5, 4.0, new Rotation2d()); // Negative X outside field
            }
        };
        Vision visionOutside = new Vision(Arrays.asList(cfg), Arrays.asList(mockOutsideIO));
        visionOutside.update();
        assertEquals(RejectionReason.OUTSIDE_FIELD, visionOutside.getRejectionReason("ll-test"));

        // 6. Valid measurement inside field -> ACCEPTED
        VisionIO mockValidIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.5;
                in.latencyMs = 25.0;
                in.estimatedPose = new Pose2d(4.0, 4.0, new Rotation2d());
            }
        };
        Vision visionValid = new Vision(Arrays.asList(cfg), Arrays.asList(mockValidIO));
        visionValid.update();
        assertEquals(RejectionReason.ACCEPTED, visionValid.getRejectionReason("ll-test"));
        assertTrue(visionValid.isAccepted());
        assertTrue(visionValid.getCamera("ll-test").getStdDev() > 0.0);
    }

    @Test
    public void testDynamicStandardDeviationScaling() {
        CameraConfig cfgBase = CameraConfig.limelight("base-cam").withStdDevMultiplier(1.0);
        CameraConfig cfgScaled = CameraConfig.photonVision("scaled-cam").withStdDevMultiplier(1.5);

        VisionIO mockBaseIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.0;
                in.latencyMs = 20.0;
                in.estimatedPose = new Pose2d(4.0, 4.0, new Rotation2d());
            }
        };
        VisionIO mockScaledIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.0;
                in.latencyMs = 20.0;
                in.estimatedPose = new Pose2d(4.0, 4.0, new Rotation2d());
            }
        };

        Vision vision = new Vision(Arrays.asList(cfgBase, cfgScaled), Arrays.asList(mockBaseIO, mockScaledIO));
        vision.update();

        double stdDevBase = vision.getCamera("base-cam").getStdDev();
        double stdDevScaled = vision.getCamera("scaled-cam").getStdDev();

        assertTrue(stdDevBase > 0.0);
        assertEquals(stdDevBase * 1.5, stdDevScaled, 1e-4, "StdDev must scale proportionally with camera multiplier");
    }

    @Test
    public void testNeuralObjectDetectionAndMountGeometry() {
        CameraConfig customMountCfg = CameraConfig.photonVision("rubik-mount")
                .withTransform(new Transform3d(
                        new Translation3d(0.35, 0.0, 0.55), // Height = 0.55m, Forward = 0.35m
                        new Rotation3d(0.0, Math.toRadians(-20.0), 0.0))) // -20 deg down-tilt
                .withRole(CameraRole.OBJECT_DETECTION);

        VisionIOSim sim = new VisionIOSim(customMountCfg);
        Vision vision = new Vision(Arrays.asList(customMountCfg), Arrays.asList(sim));

        // Inject simulated game piece
        sim.setGamePieceDetected(true, 0.0, 0.0, 5.0);
        vision.update();

        assertTrue(vision.hasGamePiece());
        double dist = vision.getGamePieceDistanceMeters();
        assertTrue(dist > 0.5, "Distance should be positive and non-zero: " + dist);

        Translation2d rel = vision.getGamePieceRobotRelativeTranslation();
        assertTrue(rel.getX() > 0.5, "Game piece translation should be in front of robot");
    }

    @Test
    public void testPerCameraDashboardEnableToggle() {
        CameraConfig cfg = CameraConfig.limelight("cam-toggle-test");
        VisionIO mockIO = new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs in) {
                in.isConnected = true;
                in.hasTarget = true;
                in.tagCount = 2;
                in.avgTagDist = 2.0;
                in.latencyMs = 20.0;
                in.estimatedPose = new Pose2d(4.0, 4.0, new Rotation2d());
            }
        };

        Vision vision = new Vision(Arrays.asList(cfg), Arrays.asList(mockIO));
        vision.update();
        assertEquals(RejectionReason.ACCEPTED, vision.getRejectionReason("cam-toggle-test"));
        assertTrue(vision.isAccepted());
        assertTrue(vision.isCameraEnabled("cam-toggle-test"));

        // Disable camera via API
        vision.setCameraEnabled("cam-toggle-test", false);
        assertFalse(vision.isCameraEnabled("cam-toggle-test"));
        vision.update();

        assertEquals(RejectionReason.VISION_DISABLED, vision.getRejectionReason("cam-toggle-test"));
        assertFalse(vision.isAccepted());

        // Re-enable camera
        vision.setCameraEnabled("cam-toggle-test", true);
        assertTrue(vision.isCameraEnabled("cam-toggle-test"));
        vision.update();

        assertEquals(RejectionReason.ACCEPTED, vision.getRejectionReason("cam-toggle-test"));
        assertTrue(vision.isAccepted());
    }
}
