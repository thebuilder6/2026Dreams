package frc.robot.Intelligence.utility.ast;

/**
 * The closed set of sensor terminals an evolved expression may read.
 *
 * <p>Each terminal carries a {@link Tier} describing <b>which knowledge kind can
 * honestly supply it</b> (see {@code docs/KNOWLEDGE_MODEL.md}). A genome that
 * reads a {@link Tier#CLAIRVOYANT} terminal is fine to evolve against sparring
 * bots in sim, but under {@link Tier#OBSERVED} that terminal evaluates to
 * {@code 0.0} — the same zero a real robot would genuinely measure — so the
 * policy cannot silently act on data the hardware does not have.
 *
 * <p>All terminal values are normalized to {@code [0.0, 1.0]} by the caller when
 * it builds an {@link EvalContext}; the normalization is documented per
 * constant so a builder and a consumer agree.
 */
public enum Terminal {
    /** Hopper fill: {@code heldFuel / ballCapacity}. */
    HELD_RATIO(Tier.OBSERVED),
    /** Match clock: {@code secondsRemaining / 150}. */
    REMAINING_TIME(Tier.OBSERVED),
    /** Seconds until our hub flips, squashed to {@code [0,1]} over 20 s; 0 when no flip. */
    TIME_UNTIL_SHIFT(Tier.OBSERVED),
    /** 1 when our hub is live now. */
    MY_HUB_ACTIVE(Tier.OBSERVED),
    /** 1 when the opponent hub is live now. */
    OPP_HUB_ACTIVE(Tier.OBSERVED),
    /** 1 when our hub will be live after the next shift. */
    MY_HUB_NEXT(Tier.OBSERVED),
    /** 1 when the opponent hub will be live after the next shift. */
    OPP_HUB_NEXT(Tier.OBSERVED),
    /** 1 when our bumpers are inside our alliance zone (odometry). */
    IN_ALLIANCE_ZONE(Tier.OBSERVED),
    /** Distance to our hub, squashed to {@code [0,1]} over 0-8 m (near = 1). */
    DIST_TO_HUB(Tier.OBSERVED),
    /** Distance from the nearest opponent to ITS hub, squashed over 0-8 m. */
    OPP_DIST_TO_HUB(Tier.OBSERVED),
    /** Score differential from our point of view, mapped {@code [-1,1] -> [0,1]}. */
    SCORE_DIFF(Tier.OBSERVED),
    /** Loose fuel in midfield, squashed over 0-30. */
    MIDFIELD_FUEL(Tier.CLAIRVOYANT),
    /** Loose fuel in our alliance zone, squashed over 0-30. */
    ALLIANCE_FUEL(Tier.CLAIRVOYANT),
    /** Loose fuel in the opponent zone, squashed over 0-30. */
    OPPONENT_FUEL(Tier.CLAIRVOYANT),
    /** 1 when an ally is within 3 m (needs ally poses). */
    ALLY_NEAR_ME(Tier.CLAIRVOYANT),
    /** 1 when the opponent is in a low-clearance trench (needs opponent pose). */
    OPP_IN_TRENCH(Tier.CLAIRVOYANT);

    /** Which knowledge kind can honestly produce this terminal. */
    public enum Tier {
        /** Available from on-board sensing + the FMS. */
        OBSERVED,
        /** Requires a full field picture; zero under the observed tier. */
        CLAIRVOYANT
    }

    private final Tier tier;

    Terminal(Tier tier) {
        this.tier = tier;
    }

    public Tier tier() {
        return tier;
    }

    /** Case-insensitive lookup by {@link #name()}; null when unknown. */
    public static Terminal fromName(String name) {
        if (name == null) {
            return null;
        }
        String upper = name.trim().toUpperCase(java.util.Locale.ROOT);
        for (Terminal t : values()) {
            if (t.name().equals(upper)) {
                return t;
            }
        }
        return null;
    }
}
