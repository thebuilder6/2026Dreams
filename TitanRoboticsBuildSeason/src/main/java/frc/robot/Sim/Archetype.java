package frc.robot.Sim;

/**
 * AI Competitor Archetypes governing System 2 macro utility weights and tactical behaviors.
 */
public enum Archetype {
    AUTONOMOUS_CYCLER("Autonomous Fuel Cycler"),
    DEFENSE_BULLY("Aggressive Defense Bully"),
    ADAPTIVE_COMPETITOR("Adaptive Match Competitor"),
    TACTICAL_DEFENDER("Tactical Defender"),
    LEAD_PURSUIT_INTERCEPTOR("Lead Pursuit Interceptor"),
    CO_PILOT("Autonomous Teleop Co-Pilot");

    public final String displayName;

    Archetype(String displayName) {
        this.displayName = displayName;
    }

    public static final Archetype PINNING_BULLY = DEFENSE_BULLY;

    public static Archetype fromString(String name) {
        if (name == null) return AUTONOMOUS_CYCLER;
        for (Archetype a : values()) {
            if (a.name().equalsIgnoreCase(name) || a.displayName.equalsIgnoreCase(name)) {
                return a;
            }
        }
        if ("PINNING_BULLY".equalsIgnoreCase(name)) return DEFENSE_BULLY;
        return AUTONOMOUS_CYCLER;
    }
}
