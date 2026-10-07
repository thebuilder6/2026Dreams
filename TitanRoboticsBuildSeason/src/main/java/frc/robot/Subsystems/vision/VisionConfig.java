package frc.robot.Subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Data.Constants.DrivebaseConstants;
import frc.robot.Telemetry.TunableNumber;

/**
 * Central configuration registry and runtime tuning parameters for the Vision subsystem.
 * Allows quick camera setup via presets, custom builders, and live dashboard tuning.
 */
public final class VisionConfig {

    // ── Live Rejection & Gating Tunables ─────────────────────────────────────
    public static final TunableNumber MAX_TAG_DIST = new TunableNumber(
            "Vision/Config/MaxTagDistMeters", DrivebaseConstants.VISION_MAX_TAG_DIST);
    public static final TunableNumber SINGLE_TAG_MAX_DIST = new TunableNumber(
            "Vision/Config/SingleTagMaxDistMeters", DrivebaseConstants.VISION_SINGLE_TAG_MAX_DIST);
    public static final TunableNumber MAX_AMBIGUITY = new TunableNumber(
            "Vision/Config/MaxAmbiguity", DrivebaseConstants.VISION_MAX_AMBIGUITY);
    public static final TunableNumber MAX_YAW_RATE = new TunableNumber(
            "Vision/Config/MaxYawRateDegPerSec", DrivebaseConstants.VISION_MAX_YAW_RATE);
    public static final TunableNumber MAX_LATENCY_MS = new TunableNumber(
            "Vision/Config/MaxLatencyMs", 150.0);

    // ── Live Standard Deviation Weighting Tunables ───────────────────────────
    public static final TunableNumber BASE_STD_DEV = new TunableNumber(
            "Vision/Config/BaseStdDev", DrivebaseConstants.VISION_BASE_STD_DEV);
    public static final TunableNumber SINGLE_TAG_PENALTY = new TunableNumber(
            "Vision/Config/SingleTagPenalty", DrivebaseConstants.VISION_SINGLE_TAG_PENALTY);
    public static final TunableNumber DIST_PENALTY_DIVISOR = new TunableNumber(
            "Vision/Config/DistPenaltyDivisor", DrivebaseConstants.VISION_DIST_PENALTY_DIVISOR);
    public static final TunableNumber MASTER_ENABLED = new TunableNumber(
            "Vision/Config/MasterEnabled", 1.0);

    // ── Active Camera Registry ───────────────────────────────────────────────
    private static List<CameraConfig> activeCameras = new ArrayList<>(Presets.dualDefault());

    private VisionConfig() {}

    /**
     * Gets the list of currently configured cameras.
     */
    public static synchronized List<CameraConfig> getCameras() {
        return Collections.unmodifiableList(activeCameras);
    }

    /**
     * Replaces the active camera configurations.
     */
    public static synchronized void setCameras(CameraConfig... configs) {
        activeCameras = new ArrayList<>(Arrays.asList(configs));
    }

    /**
     * Replaces the active camera configurations with a list.
     */
    public static synchronized void setCameras(List<CameraConfig> configs) {
        activeCameras = new ArrayList<>(configs);
    }

    /**
     * Adds an additional camera configuration to the active registry.
     */
    public static synchronized void addCamera(CameraConfig config) {
        activeCameras.add(config);
    }

    /**
     * Updates all TunableNumbers from NetworkTables.
     */
    public static void updateTunables() {
        MAX_TAG_DIST.update();
        SINGLE_TAG_MAX_DIST.update();
        MAX_AMBIGUITY.update();
        MAX_YAW_RATE.update();
        MAX_LATENCY_MS.update();
        BASE_STD_DEV.update();
        SINGLE_TAG_PENALTY.update();
        DIST_PENALTY_DIVISOR.update();
        MASTER_ENABLED.update();
    }

    public static boolean isMasterEnabled() {
        return MASTER_ENABLED.get() > 0.5;
    }

    // ── Presets for Rapid Setup ──────────────────────────────────────────────
    public static final class Presets {

        /**
         * Default Dual-Vision configuration:
         * - Limelight 3/3G on front (X = +0.25m, Z = 0.45m, +15° pitch) with MegaTag2.
         * - Orange Pi 5 PhotonVision for ground Fuel tracking and auxiliary AprilTags (X = +0.25m, Z = 0.45m, -15° pitch).
         */
        public static List<CameraConfig> dualDefault() {
            return Arrays.asList(
                    CameraConfig.limelight("limelight-front")
                            .withTransform(new Transform3d(
                                    new Translation3d(0.25, 0.0, 0.45),
                                    new Rotation3d(0.0, Math.toRadians(15.0), 0.0)))
                            .withRole(CameraConfig.CameraRole.HYBRID)
                            .withStdDevMultiplier(1.0)
                            .withMegaTag2(true),
                    CameraConfig.photonVision("rubik-pi-coprocessor")
                            .withTransform(new Transform3d(
                                    new Translation3d(
                                            DrivebaseConstants.RUBIK_PI_CAMERA_FORWARD_OFFSET_METERS,
                                            0.0,
                                            DrivebaseConstants.RUBIK_PI_CAMERA_HEIGHT_METERS),
                                    new Rotation3d(0.0, Math.toRadians(DrivebaseConstants.RUBIK_PI_CAMERA_PITCH_DEG), 0.0)))
                            .withRole(CameraConfig.CameraRole.HYBRID)
                            .withStdDevMultiplier(1.2));
        }

        /**
         * Single front Limelight configuration.
         */
        public static List<CameraConfig> singleLimelight() {
            return Collections.singletonList(
                    CameraConfig.limelight("limelight-front")
                            .withTransform(new Transform3d(
                                    new Translation3d(0.25, 0.0, 0.45),
                                    new Rotation3d(0.0, Math.toRadians(15.0), 0.0)))
                            .withRole(CameraConfig.CameraRole.HYBRID)
                            .withStdDevMultiplier(1.0)
                            .withMegaTag2(true));
        }

        /**
         * Single PhotonVision coprocessor configuration.
         */
        public static List<CameraConfig> singlePhotonVision() {
            return Collections.singletonList(
                    CameraConfig.photonVision("rubik-pi-coprocessor")
                            .withTransform(new Transform3d(
                                    new Translation3d(0.25, 0.0, 0.45),
                                    new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)))
                            .withRole(CameraConfig.CameraRole.HYBRID)
                            .withStdDevMultiplier(1.0));
        }

        /**
         * Dual PhotonVision coprocessor configuration (front + back).
         */
        public static List<CameraConfig> dualPhotonVision() {
            return Arrays.asList(
                    CameraConfig.photonVision("photon-front")
                            .withTransform(new Transform3d(
                                    new Translation3d(0.25, 0.0, 0.45),
                                    new Rotation3d(0.0, Math.toRadians(15.0), 0.0)))
                            .withRole(CameraConfig.CameraRole.APRILTAG)
                            .withStdDevMultiplier(1.0),
                    CameraConfig.photonVision("photon-back")
                            .withTransform(new Transform3d(
                                    new Translation3d(-0.25, 0.0, 0.45),
                                    new Rotation3d(0.0, Math.toRadians(15.0), Math.PI)))
                            .withRole(CameraConfig.CameraRole.APRILTAG)
                            .withStdDevMultiplier(1.0));
        }
    }
}
