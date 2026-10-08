package frc.robot.Hardware;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * BreakerModel: First-principles thermal simulation of the FRC 120A Main Circuit Breaker.
 *
 * <p>Inspired by Team 6328's BreakerModel, this class implements Miner's rule for
 * cumulative thermal fatigue / damage accumulation on the bimetallic trip mechanism.
 *
 * <p>Physics:
 * <ul>
 *   <li>When current exceeds the rated 120A threshold, thermal damage accumulates at rate
 *       {@code dD/dt = 1.0 / T_trip(I)}, where {@code T_trip(I)} is interpolated from the
 *       official manufacturer datasheet time-current curve.</li>
 *   <li>When current is at or below 120A, the bimetallic element cools down exponentially
 *       according to time constant {@code TAU_COOL = 60.0 s}:
 *       {@code D(t + dt) = D(t) * exp(-dt / TAU_COOL)}.</li>
 *   <li>When damage reaches 1.0, the breaker physically trips (open circuit).</li>
 * </ul>
 */
public class BreakerModel {

    /** Rated continuous current of the standard FRC main breaker, in amperes. */
    public static final double RATED_CURRENT_AMPS = 120.0;

    /** Exponential thermal cooldown time constant of the bimetallic element, in seconds. */
    public static final double TAU_COOL_SECONDS = 60.0;

    /** Sentinel trip time for currents below the rated trip threshold (effectively infinite). */
    public static final double SENTINEL_TRIP_TIME_SECONDS = 1.0e6;

    // Datasheet normalized current multipliers (I / I_rated)
    private static final double[] I_NORM_POINTS = {1.35, 2.0, 2.25, 2.5, 3.0, 4.0, 5.0};

    // Maximum trip time ratings from manufacturer datasheet, in seconds
    private static final double[] TRIP_TIME_POINTS = {
            30.0 * 60.0, // 1.35x (162A) -> 30 minutes
            70.0,        // 2.00x (240A) -> 70 seconds
            38.0,        // 2.25x (270A) -> 38 seconds
            25.0,        // 2.50x (300A) -> 25 seconds
            15.0,        // 3.00x (360A) -> 15 seconds
            10.0,        // 4.00x (480A) -> 10 seconds
            7.0          // 5.00x (600A) -> 7 seconds
    };

    private static final double MIN_TRIP_TIME_SECONDS = TRIP_TIME_POINTS[TRIP_TIME_POINTS.length - 1];

    private static final InterpolatingDoubleTreeMap tripTimeMap = new InterpolatingDoubleTreeMap();

    static {
        for (int i = 0; i < I_NORM_POINTS.length; i++) {
            tripTimeMap.put(I_NORM_POINTS[i], TRIP_TIME_POINTS[i]);
        }
    }

    private double damageFraction = 0.0;

    public BreakerModel() {
        this.damageFraction = 0.0;
    }

    /**
     * Resets accumulated thermal damage to zero (e.g. at match start or between tests).
     */
    public void reset() {
        damageFraction = 0.0;
    }

    /**
     * Advances the thermal breaker simulation by {@code dtSeconds} given bus current.
     *
     * @param totalCurrentAmps Instantaneous bus current drawn by all robot circuits
     * @param dtSeconds        Elapsed time since last update (e.g. 0.020 for 50 Hz)
     * @return Current damage fraction in range [0.0, 1.0]
     */
    public double update(double totalCurrentAmps, double dtSeconds) {
        if (dtSeconds <= 0.0) {
            return damageFraction;
        }

        double absCurrent = Math.abs(totalCurrentAmps);

        if (absCurrent > RATED_CURRENT_AMPS) {
            double tripTime = getTripTimeSeconds(absCurrent);
            if (tripTime > 0.0) {
                damageFraction += dtSeconds / tripTime;
            }
        } else {
            // Exponential cooling down toward ambient
            damageFraction *= Math.exp(-dtSeconds / TAU_COOL_SECONDS);
        }

        // Clamp between 0.0 and 1.0
        damageFraction = Math.max(0.0, Math.min(1.0, damageFraction));
        return damageFraction;
    }

    /**
     * Returns expected trip time in seconds for a constant continuous current.
     *
     * @param currentAmps Current in amperes
     * @return Expected seconds until trip, or SENTINEL_TRIP_TIME_SECONDS if <= 120A
     */
    public static double getTripTimeSeconds(double currentAmps) {
        double absCurrent = Math.abs(currentAmps);
        double normalized = absCurrent / RATED_CURRENT_AMPS;

        if (normalized <= 1.0) {
            return SENTINEL_TRIP_TIME_SECONDS;
        }
        if (normalized <= I_NORM_POINTS[0]) {
            // Linear interpolate between rated threshold and first datasheet point (1.35x)
            double t = (normalized - 1.0) / (I_NORM_POINTS[0] - 1.0);
            return SENTINEL_TRIP_TIME_SECONDS * (1.0 - t) + TRIP_TIME_POINTS[0] * t;
        }
        if (normalized >= I_NORM_POINTS[I_NORM_POINTS.length - 1]) {
            return MIN_TRIP_TIME_SECONDS;
        }

        return tripTimeMap.get(normalized);
    }

    /**
     * Current cumulative thermal damage fraction in range [0.0, 1.0].
     * 1.0 corresponds to an active trip condition.
     */
    public double getDamageFraction() {
        return damageFraction;
    }

    /**
     * Sets the damage fraction directly (useful for testing or state restoration).
     */
    public void setDamageFraction(double fraction) {
        this.damageFraction = Math.max(0.0, Math.min(1.0, fraction));
    }

    /**
     * True if accumulated thermal damage reached 1.0.
     */
    public boolean isTripped() {
        return damageFraction >= 1.0;
    }
}
