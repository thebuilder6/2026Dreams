package frc.robot.Sim;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
/**
 * Minimal, deterministic G418 pin referee for the standalone runner.
 *
 * <p>The full sim's {@link RefereeSim} is a subsystem that measures the parked
 * player against Bot 0 through {@code SwerveBase} and {@code AIRobotSim}
 * singletons — neither exists in a MapleSim-free run. This class re-implements
 * only the pin rule on plain bot state so the standalone can attribute penalty
 * points per bot, which is the input the individual-EPA evaluator needs.
 *
 * <p>Semantics mirror {@code RefereeSim.evaluatePinningRule}: an aggressor
 * driving into an opponent within {@link #CONTACT_DIST_M}; the first 3 s is a
 * MINOR FOUL (5 pts), every further {@link #PIN_ESCALATION_SEC} uncorrected is a
 * MAJOR FOUL (15 pts); separation of {@link #PIN_SEPARATION_M} ends the count.
 *
 * <p><b>Honest expectation:</b> the kinematic runner has no rigid-body contact,
 * only a positional push-out, so sustained overlap is rare and this referee
 * mostly reads zero — the same way the full-sim headless 3v3 cannot reach G418
 * (see {@code KNOWN_ISSUES.md} §E). It exists so the penalty channel is real and
 * correct, not fabricated.
 */
final class StandaloneReferee {
    /**
     * Center distance below which two robots are in contact, m: exactly the
     * push-out equilibrium, so pairs resting at the separation floor never
     * accrue — only real overlap plus active closing counts. (A wider gate
     * kept chasing defenders perpetually "in contact"; running after
     * separation instead would zero the channel entirely, including true pins.)
     */
    static final double CONTACT_DIST_M = StandaloneMatchRunner.PEER_MIN_DISTANCE_M;
    /** Pin duration that draws the first (minor) foul, s. */
    static final double PIN_MAX_SEC = 3.0;
    /** Additional uncorrected duration per escalating major foul, s. */
    static final double PIN_ESCALATION_SEC = 3.0;
    /** Separation that ends a pin count, m. */
    static final double PIN_SEPARATION_M = 1.83;
    /** Speed projection toward the opponent above which a bot is actively pinning, m/s. */
    static final double AGGRESSOR_DOT_MPS = 0.15;
    /** Victim speed below which it counts as held rather than contesting free, m/s. */
    static final double VICTIM_HELD_MPS = 0.30;
    /** Maximum escalations per contact engagement; separation is required to reset. */
    static final int MAX_PIN_VIOLATIONS = 3;

    private final Map<String, Double> pinTimes = new HashMap<>();
    private final Map<String, Integer> pinViolations = new HashMap<>();

    void reset() {
        pinTimes.clear();
        pinViolations.clear();
    }

    /** One fixed-dt tick of pin evaluation over the current roster. */
    void update(List<StandaloneBot> bots, double dtSec) {
        for (int i = 0; i < bots.size(); i++) {
            for (int j = i + 1; j < bots.size(); j++) {
                StandaloneBot a = bots.get(i);
                StandaloneBot b = bots.get(j);
                if (a.isRed() == b.isRed()) {
                    continue;
                }
                String key = i + "-" + j;
                double dist = a.getPose().getTranslation().getDistance(b.getPose().getTranslation());
                if (dist >= PIN_SEPARATION_M) {
                    // Clean separation ends the count and resets escalation.
                    pinTimes.remove(key);
                    pinViolations.remove(key);
                    continue;
                }
                double aInto = projectionToward(a, b);
                double bInto = projectionToward(b, a);
                StandaloneBot aggressor = aInto >= bInto ? a : b;
                StandaloneBot victim = aggressor == a ? b : a;
                double closing = Math.max(aInto, bInto);
                // A pin traps someone: the aggressor drives in AND the victim
                // cannot get away. A fleeing victim (full speed, glued chaser)
                // is a chase, not a pin — the old max-only gate counted those.
                if (dist >= CONTACT_DIST_M || closing <= AGGRESSOR_DOT_MPS
                        || victimSpeed(victim) >= VICTIM_HELD_MPS) {
                    // Touching without driving in, a free victim, or between
                    // contact and separation: hold the count, no accrual.
                    // Passive proximity is not a PIN; only trapping contact is.
                    decay(key, dtSec);
                    continue;
                }

                double t = pinTimes.getOrDefault(key, 0.0) + dtSec;
                pinTimes.put(key, t);
                if (t < PIN_MAX_SEC) {
                    continue;
                }
                int deserved = Math.min(MAX_PIN_VIOLATIONS,
                        1 + (int) ((t - PIN_MAX_SEC) / PIN_ESCALATION_SEC));
                int already = pinViolations.getOrDefault(key, 0);
                if (deserved <= already) {
                    continue;
                }
                for (int v = already; v < deserved; v++) {
                    if (v == 0) {
                        aggressor.recordMinorFoul();
                    } else {
                        aggressor.recordMajorFoul();
                    }
                }
                pinViolations.put(key, deserved);
            }
        }
    }

    private void decay(String key, double dtSec) {
        Double held = pinTimes.get(key);
        if (held == null) {
            return;
        }
        double decayed = held - dtSec;
        if (decayed <= 0.0) {
            pinTimes.remove(key);
            pinViolations.remove(key);
        } else {
            pinTimes.put(key, decayed);
        }
    }

    /** Speed norm of a bot, m/s. */
    private static double victimSpeed(StandaloneBot bot) {
        ChassisSpeeds v = bot.getVelocity();
        return Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
    }

    /** Speed of {@code from} along the unit vector toward {@code to}, m/s. */
    private static double projectionToward(StandaloneBot from, StandaloneBot to) {
        Translation2d axis = to.getPose().getTranslation().minus(from.getPose().getTranslation());
        double norm = axis.getNorm();
        if (norm < 1e-6) {
            return 0.0;
        }
        Translation2d unit = axis.div(norm);
        return from.getVelocity().vxMetersPerSecond * unit.getX()
                + from.getVelocity().vyMetersPerSecond * unit.getY();
    }
}
