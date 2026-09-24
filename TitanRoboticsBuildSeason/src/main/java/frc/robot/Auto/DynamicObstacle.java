package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Represents a dynamic moving obstacle on the field (e.g. opponent robot).
 */
public class DynamicObstacle {

    public final Translation2d position;
    public final Translation2d velocity;
    public final double radius;
    public final double expiryTimestamp;
    public final boolean isProprioceptive;

    public DynamicObstacle(Translation2d position, Translation2d velocity, double radius, double durationSec, boolean isProprioceptive) {
        this.position = position;
        this.velocity = velocity;
        this.radius = radius;
        this.expiryTimestamp = Timer.getFPGATimestamp() + durationSec;
        this.isProprioceptive = isProprioceptive;
    }

    public DynamicObstacle(Translation2d position, Translation2d velocity, double radius, double durationSec) {
        this(position, velocity, radius, durationSec, false);
    }

    public DynamicObstacle(Translation2d position, Translation2d velocity) {
        this(position, velocity, 0.55, 0.40, false); // Default 0.55m radius (FRC robot with bumpers) & 400ms TTL
    }

    /**
     * Checks if this obstacle has exceeded its time-to-live.
     */
    public boolean isExpired(double now) {
        return now > expiryTimestamp;
    }

    /**
     * Predicts the obstacle's future position assuming constant velocity.
     */
    public Translation2d getPredictedPosition(double dt) {
        return position.plus(velocity.times(dt));
    }

    /**
     * Converts to Pose2d for 2D/3D visualization in AdvantageScope.
     */
    public Pose2d toPose2d() {
        double headingRad = Math.atan2(velocity.getY(), velocity.getX());
        return new Pose2d(position, new Rotation2d(headingRad));
    }
}
