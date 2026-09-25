package frc.robot.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * True 2D Vector Slew Rate Limiter.
 * 
 * Unlike independent 1D slew rate limiters applied separately to X and Y axes,
 * this limiter constrains the Euclidean rate of change of the 2D velocity vector:
 *   ||v_target - v_prev|| <= maxRate * dt
 * 
 * This preserves directional heading during sudden translation angle shifts (e.g.
 * transitioning from full forward to full strafe), eliminating elliptical trajectory
 * distortion and corner clipping.
 */
public class Vector2dSlewRateLimiter {

    private double rateLimit;
    private Translation2d prevVal;
    private double prevTime;

    /**
     * Creates a 2D vector slew rate limiter with the specified rate limit and initial (0, 0).
     * 
     * @param rateLimit The maximum rate of change of the 2D vector magnitude per second (e.g. m/s^2).
     */
    public Vector2dSlewRateLimiter(double rateLimit) {
        this(rateLimit, new Translation2d(0.0, 0.0));
    }

    /**
     * Creates a 2D vector slew rate limiter with the specified rate limit and initial translation.
     * 
     * @param rateLimit The maximum rate of change of the 2D vector magnitude per second (e.g. m/s^2).
     * @param initialValue Initial vector state.
     */
    public Vector2dSlewRateLimiter(double rateLimit, Translation2d initialValue) {
        this.rateLimit = rateLimit;
        this.prevVal = (initialValue != null) ? initialValue : new Translation2d(0.0, 0.0);
        this.prevTime = Timer.getFPGATimestamp();
    }

    /**
     * Filters the target 2D translation vector to enforce the maximum acceleration limit.
     * 
     * @param target The commanded target translation vector.
     * @return The rate-limited translation vector.
     */
    public Translation2d calculate(Translation2d target) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - prevTime;
        prevTime = currentTime;

        // Guard against negative, zero, or excessively large dt (e.g. paused simulation or loop overrun)
        if (dt <= 0.0) {
            return prevVal;
        }
        if (dt > 0.1) {
            dt = 0.02; // Clamp dt to nominal loop duration on large delays
        }

        double maxChange = rateLimit * dt;
        Translation2d delta = target.minus(prevVal);
        double deltaNorm = delta.getNorm();

        if (deltaNorm <= maxChange) {
            prevVal = target;
        } else {
            // Step along the delta vector by the maximum allowed change magnitude
            Translation2d step = delta.times(maxChange / deltaNorm);
            prevVal = prevVal.plus(step);
        }

        return prevVal;
    }

    /**
     * Filters target Cartesian coordinates (X, Y) to enforce the 2D vector slew rate limit.
     * 
     * @param targetX The commanded target X component.
     * @param targetY The commanded target Y component.
     * @return The rate-limited translation vector.
     */
    public Translation2d calculate(double targetX, double targetY) {
        return calculate(new Translation2d(targetX, targetY));
    }

    /**
     * Resets the limiter state to the specified translation vector.
     * 
     * @param value The value to reset to.
     */
    public void reset(Translation2d value) {
        this.prevVal = (value != null) ? value : new Translation2d(0.0, 0.0);
        this.prevTime = Timer.getFPGATimestamp();
    }

    /**
     * Resets the limiter state to Cartesian (x, y).
     * 
     * @param x X component.
     * @param y Y component.
     */
    public void reset(double x, double y) {
        reset(new Translation2d(x, y));
    }

    /**
     * Sets a new rate limit.
     * 
     * @param rateLimit The maximum rate of change per second.
     */
    public void setRateLimit(double rateLimit) {
        this.rateLimit = Math.max(0.0, rateLimit);
    }

    /**
     * Gets the current rate limit.
     */
    public double getRateLimit() {
        return rateLimit;
    }

    /**
     * Gets the latest calculated translation vector.
     */
    public Translation2d getLastValue() {
        return prevVal;
    }

    /**
     * Alias for {@link #getLastValue()}.
     */
    public Translation2d getTranslation() {
        return prevVal;
    }
}
