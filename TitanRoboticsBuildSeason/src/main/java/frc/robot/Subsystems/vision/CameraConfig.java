package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Declarative configuration for an individual vision camera.
 * Specifies hardware platform, role, 3D mounting transform relative to
 * robot center, trust multiplier, and feature flags.
 */
public class CameraConfig {

    public enum CameraType {
        LIMELIGHT,
        PHOTONVISION
    }

    public enum CameraRole {
        APRILTAG,
        OBJECT_DETECTION,
        HYBRID
    }

    private final String name;
    private final CameraType type;
    private CameraRole role;
    private Transform3d robotToCamera;
    private double stdDevMultiplier;
    private boolean useMegaTag2;
    private boolean enabled;

    public CameraConfig(
            String name,
            CameraType type,
            CameraRole role,
            Transform3d robotToCamera,
            double stdDevMultiplier,
            boolean useMegaTag2,
            boolean enabled) {
        this.name = name;
        this.type = type;
        this.role = role != null ? role : CameraRole.APRILTAG;
        this.robotToCamera = robotToCamera != null ? robotToCamera : new Transform3d();
        this.stdDevMultiplier = stdDevMultiplier > 0.0 ? stdDevMultiplier : 1.0;
        this.useMegaTag2 = useMegaTag2;
        this.enabled = enabled;
    }

    /**
     * Creates a Limelight camera configuration with default settings.
     *
     * @param name NetworkTables table name (e.g., "limelight-front")
     * @return Configured CameraConfig instance
     */
    public static CameraConfig limelight(String name) {
        return new CameraConfig(
                name,
                CameraType.LIMELIGHT,
                CameraRole.HYBRID,
                new Transform3d(new Translation3d(0.25, 0.0, 0.45), new Rotation3d(0.0, Math.toRadians(15.0), 0.0)),
                1.0,
                true,
                true);
    }

    /**
     * Creates a Limelight camera configuration with custom mount transform.
     */
    public static CameraConfig limelight(String name, Transform3d robotToCamera) {
        return limelight(name).withTransform(robotToCamera);
    }

    /**
     * Creates a PhotonVision camera configuration with default settings.
     *
     * @param name NetworkTables camera name (e.g., "rubik-pi-coprocessor")
     * @return Configured CameraConfig instance
     */
    public static CameraConfig photonVision(String name) {
        return new CameraConfig(
                name,
                CameraType.PHOTONVISION,
                CameraRole.HYBRID,
                new Transform3d(new Translation3d(0.25, 0.0, 0.45), new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)),
                1.2,
                false,
                true);
    }

    /**
     * Creates a PhotonVision camera configuration with custom mount transform.
     */
    public static CameraConfig photonVision(String name, Transform3d robotToCamera) {
        return photonVision(name).withTransform(robotToCamera);
    }

    public CameraConfig withTransform(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera != null ? robotToCamera : new Transform3d();
        return this;
    }

    public CameraConfig withRole(CameraRole role) {
        this.role = role != null ? role : CameraRole.APRILTAG;
        return this;
    }

    public CameraConfig withStdDevMultiplier(double multiplier) {
        this.stdDevMultiplier = multiplier > 0.0 ? multiplier : 1.0;
        return this;
    }

    public CameraConfig withMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
        return this;
    }

    public CameraConfig withEnabled(boolean enabled) {
        this.enabled = enabled;
        return this;
    }

    public String getName() {
        return name;
    }

    public CameraType getType() {
        return type;
    }

    public CameraRole getRole() {
        return role;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    public double getStdDevMultiplier() {
        return stdDevMultiplier;
    }

    public boolean isMegaTag2() {
        return useMegaTag2;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Returns camera mounting height from carpet in meters.
     */
    public double getCameraHeightMeters() {
        return robotToCamera.getZ();
    }

    /**
     * Returns camera forward offset from robot center in meters.
     */
    public double getCameraForwardOffsetMeters() {
        return robotToCamera.getX();
    }

    /**
     * Returns camera pitch angle in degrees (+up, -down).
     */
    public double getCameraPitchDegrees() {
        return Math.toDegrees(robotToCamera.getRotation().getY());
    }

    @Override
    public String toString() {
        return String.format(
                "CameraConfig[name=%s, type=%s, role=%s, x=%.3f, y=%.3f, z=%.3f, pitch=%.1f°, mult=%.2f, enabled=%b]",
                name,
                type,
                role,
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                getCameraPitchDegrees(),
                stdDevMultiplier,
                enabled);
    }
}
