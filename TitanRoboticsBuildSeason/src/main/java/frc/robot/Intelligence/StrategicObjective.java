package frc.robot.Intelligence;

/**
 * Universal FRC Strategic Objectives arbitrated by System 2 Executive Strategist.
 * Represents macro intentions across games (goals, feeders, loose pieces, defense, endgame),
 * with 2026 Rebuilt and game-agnostic aliases.
 */
public enum StrategicObjective {
    STOCKPILE_DEPOT,      // Hub is inactive; fill hopper at alliance depot
    VACUUM_MIDFIELD,       // Harvest dense clusters along centerline
    CYCLE_SCORE_HUB,      // Hub is active; transit to standoff arc and score
    STAGE_STANDOFF,       // Hopper is full but Hub is inactive; wait at standoff arc
    DENY_SHOOTING_LANE,   // Defend; block opponent's line-of-sight to their hub
    SHADOW_MIDLINE,       // Defend; mirror opponent across field centerline
    LEAD_INTERCEPT,       // Defend; lead-pursuit interception of opponent
    SWEEP_ALLIANCE_ZONE,   // Harvest loose fuel in the home scoring zone
    POACH_OPPONENT_ZONE,   // Harvest loose fuel in opponent territory during teleop
    SHUTTLE_PASS,          // Lob held fuel home from midfield while the Hub is inactive
    LONG_RANGE_SNIPE,      // Shoot from the outer edge of the alliance zone
    CHOKE_TRENCH,           // Contest an occupied trench approach
    SCREEN_FOR_ALLY,        // Interpose between a loaded ally and a defender
    BAIT_PIN_FOUL,          // Hold position during an observed opponent pin
    RUSH_CLIMB,           // Endgame (t <= 20s); navigate to climbing tower
    IDLE;                 // Standby / no action

    // Backward-compatible and game-agnostic aliases:
    public static final StrategicObjective HARVEST_FEEDER = STOCKPILE_DEPOT;
    public static final StrategicObjective HARVEST_FIELD_PIECES = VACUUM_MIDFIELD;
    public static final StrategicObjective HARVEST_FIELD_FUEL = VACUUM_MIDFIELD;
    public static final StrategicObjective SCORE_HUB = CYCLE_SCORE_HUB;
    public static final StrategicObjective SCORE_GOAL = CYCLE_SCORE_HUB;
    public static final StrategicObjective STAGE_SCORING_WINDOW = STAGE_STANDOFF;
    public static final StrategicObjective DENY_SCORING_LANE = DENY_SHOOTING_LANE;
    public static final StrategicObjective RUSH_ENDGAME = RUSH_CLIMB;

    public boolean isOffensive() {
        return this == STOCKPILE_DEPOT || this == VACUUM_MIDFIELD || this == CYCLE_SCORE_HUB
                || this == STAGE_STANDOFF || this == SWEEP_ALLIANCE_ZONE || this == POACH_OPPONENT_ZONE
                || this == SHUTTLE_PASS || this == LONG_RANGE_SNIPE || this == RUSH_CLIMB;
    }

    public boolean isDefensive() {
        return this == DENY_SHOOTING_LANE || this == SHADOW_MIDLINE || this == LEAD_INTERCEPT
                || this == CHOKE_TRENCH || this == BAIT_PIN_FOUL;
    }

    public boolean isTeamwork() {
        return this == SCREEN_FOR_ALLY || this == SHUTTLE_PASS;
    }
}
