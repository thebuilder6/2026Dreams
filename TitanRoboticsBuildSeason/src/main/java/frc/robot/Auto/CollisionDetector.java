package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CollisionDetector {

    private double filteredAccelX = 0.0;
    private double filteredAccelY = 0.0;
    private double prevFilteredAccelX = 0.0;
    private double prevFilteredAccelY = 0.0;
    private double lastTimestamp = -1.0;
    private double lastCollisionTimestamp = -1.0;
    private double lastJerkMagnitude = 0.0;
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();

    private double stallStartTime = -1.0;
    private double stallDuration = 0.0;

    private static final double COLLISION_JERK_THRESHOLD = 120.0;
    private static final double COLLISION_DECEL_THRESHOLD = 10.0;
    private static final double COLLISION_DEBOUNCE_SEC = 0.35;

    private static final double STALL_CMD_SPEED_MIN = 1.00;
    private static final double STALL_ACTUAL_SPEED_MAX = 0.20;
    private static final double STALL_CURRENT_AMPS = 28.0;

    public void update(
            Pose2d currentPose,
            ChassisSpeeds actualSpeeds,
            ChassisSpeeds commandedSpeeds,
            double accelXG,
            double accelYG,
            double driveCurrentAmps) {

        double now = Timer.getFPGATimestamp();
        double dt = lastTimestamp > 0.0 ? (now - lastTimestamp) : 0.02;
        if (dt < 1e-4) dt = 0.02;

        double rawAx = accelXG * 9.80665;
        double rawAy = accelYG * 9.80665;
        if (Math.hypot(accelXG, accelYG) < 1e-3) {
            rawAx = (actualSpeeds.vxMetersPerSecond - prevSpeeds.vxMetersPerSecond) / dt;
            rawAy = (actualSpeeds.vyMetersPerSecond - prevSpeeds.vyMetersPerSecond) / dt;
        }

        double alpha = 0.35;
        filteredAccelX = alpha * rawAx + (1.0 - alpha) * filteredAccelX;
        filteredAccelY = alpha * rawAy + (1.0 - alpha) * filteredAccelY;

        double jerkX = (filteredAccelX - prevFilteredAccelX) / dt;
        double jerkY = (filteredAccelY - prevFilteredAccelY) / dt;
        lastJerkMagnitude = Math.hypot(jerkX, jerkY);

        double prevSpeed = Math.hypot(prevSpeeds.vxMetersPerSecond, prevSpeeds.vyMetersPerSecond);
        boolean isImpact = false;

        if (prevSpeed > 0.40) {
            double uVx = prevSpeeds.vxMetersPerSecond / prevSpeed;
            double uVy = prevSpeeds.vyMetersPerSecond / prevSpeed;
            double decelOpposing = -(filteredAccelX * uVx + filteredAccelY * uVy);
            double jerkOpposing = -(jerkX * uVx + jerkY * uVy);
            if (decelOpposing > COLLISION_DECEL_THRESHOLD && jerkOpposing > COLLISION_JERK_THRESHOLD) {
                isImpact = true;
            }
        } else if (Math.hypot(filteredAccelX, filteredAccelY) > 15.0 && lastJerkMagnitude > (COLLISION_JERK_THRESHOLD * 1.5)) {
            isImpact = true;
        }

        if (isImpact && (now - lastCollisionTimestamp > COLLISION_DEBOUNCE_SEC)) {
            lastCollisionTimestamp = now;
            Translation2d dir = prevSpeed > 0.40
                    ? new Translation2d(prevSpeeds.vxMetersPerSecond, prevSpeeds.vyMetersPerSecond).div(prevSpeed)
                    : new Translation2d(-filteredAccelX, -filteredAccelY);
            if (dir.getNorm() > 1e-3) dir = dir.div(dir.getNorm());
            Translation2d obsPos = currentPose.getTranslation().plus(dir.rotateBy(currentPose.getRotation()).times(0.65));
            DynamicRouter.registerObstacle(obsPos, new Translation2d(), 0.55, 0.65, true);
        }

        double cmdSpeed = Math.hypot(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
        double actSpeed = Math.hypot(actualSpeeds.vxMetersPerSecond, actualSpeeds.vyMetersPerSecond);
        boolean stallCondition = (cmdSpeed > STALL_CMD_SPEED_MIN)
                && (actSpeed < STALL_ACTUAL_SPEED_MAX)
                && (driveCurrentAmps > STALL_CURRENT_AMPS);

        if (stallCondition) {
            if (stallStartTime < 0) stallStartTime = now;
            stallDuration = now - stallStartTime;
        } else {
            stallStartTime = -1.0;
            stallDuration = Math.max(0.0, stallDuration - dt * 2.0);
        }

        prevFilteredAccelX = filteredAccelX;
        prevFilteredAccelY = filteredAccelY;
        prevSpeeds = actualSpeeds;
        lastTimestamp = now;

        Logger.recordOutput("CollisionDetector/JerkMagnitude", lastJerkMagnitude);
        Logger.recordOutput("CollisionDetector/ImpactDetected", isImpactDetected());
        Logger.recordOutput("CollisionDetector/StallDuration", stallDuration);
    }

    public boolean isImpactDetected() {
        return (Timer.getFPGATimestamp() - lastCollisionTimestamp) < 0.20;
    }

    public boolean isStalled() {
        return stallDuration > 0.25;
    }

    public double getStallDuration() {
        return stallDuration;
    }

    public void reset() {
        filteredAccelX = 0.0;
        filteredAccelY = 0.0;
        prevFilteredAccelX = 0.0;
        prevFilteredAccelY = 0.0;
        stallStartTime = -1.0;
        stallDuration = 0.0;
        lastCollisionTimestamp = -1.0;
        lastTimestamp = -1.0;
        lastJerkMagnitude = 0.0;
        prevSpeeds = new ChassisSpeeds();
    }

    public double getLastJerkMagnitude() {
        return lastJerkMagnitude;
    }
}