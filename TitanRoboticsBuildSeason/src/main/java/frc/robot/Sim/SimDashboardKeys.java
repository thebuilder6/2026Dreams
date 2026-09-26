package frc.robot.Sim;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * SimDashboardKeys: single source of truth for every {@code Simulation/*}
 * NetworkTables key backing the Elastic Simulation tabs.
 *
 * <p>Past UI breakage came from a code/Elastic topic mismatch
 * ({@code AllyBot0/...} vs {@code Ally1/...}), so all producers must use
 * these constants (or the per-bot prefix helpers) instead of string
 * literals. {@code SimDashboardKeysTest} asserts every {@code Simulation/*}
 * topic in {@code elastic-layout.json} is a member of {@link #allKeys()}.
 */
public final class SimDashboardKeys {

    private SimDashboardKeys() {
    }

    /**
     * Topic prefix for an opponent bot, e.g. {@code "Simulation/Bot1"}.
     *
     * @param botId opponent bot id (0-2)
     */
    public static String botPrefix(int botId) {
        return "Simulation/Bot" + botId;
    }

    /**
     * Topic prefix for an ally bot, e.g. {@code "Simulation/Ally0"}.
     *
     * @param allyIndex ally slot index (0-2; slot 0 is training-only)
     */
    public static String allyPrefix(int allyIndex) {
        return "Simulation/Ally" + allyIndex;
    }

    /** Suffix appended to a per-bot prefix for the archetype string. */
    public static final String SUFFIX_ARCHETYPE = "/Archetype";

    // --- Bot 0 (lead sparring bot) ---
    public static final String BOT0_ARCHETYPE = "Simulation/Bot0/Archetype";
    public static final String BOT0_ARCHETYPE_CHOOSER = "Simulation/Bot0/ArchetypeChooser";
    public static final String BOT0_FUEL = "Simulation/Bot0/Fuel";
    public static final String BOT0_MARK = "Simulation/Bot0/Mark";
    public static final String BOT0_OBJECTIVE = "Simulation/Bot0/Objective";
    public static final String BOT0_POSE = "Simulation/Bot0/Pose";
    public static final String BOT0_SCORE = "Simulation/Bot0/Score";
    public static final String BOT0_STALLED = "Simulation/Bot0/Stalled";
    public static final String BOT0_STATE_DETAIL = "Simulation/Bot0/StateDetail";
    public static final String BOT0_TARGET_POSE = "Simulation/Bot0/TargetPose";

    // --- Bot 1 / Bot 2 (additional opponents) ---
    public static final String BOT1_ARCHETYPE = "Simulation/Bot1/Archetype";
    public static final String BOT1_ARCHETYPE_CHOOSER = "Simulation/Bot1/ArchetypeChooser";
    public static final String BOT2_ARCHETYPE = "Simulation/Bot2/Archetype";
    public static final String BOT2_ARCHETYPE_CHOOSER = "Simulation/Bot2/ArchetypeChooser";

    // --- Ally bots ---
    public static final String ALLY1_ARCHETYPE = "Simulation/Ally1/Archetype";
    public static final String ALLY1_ARCHETYPE_CHOOSER = "Simulation/Ally1/ArchetypeChooser";
    public static final String ALLY2_ARCHETYPE = "Simulation/Ally2/Archetype";
    public static final String ALLY2_ARCHETYPE_CHOOSER = "Simulation/Ally2/ArchetypeChooser";

    // --- Legacy / global AI mode ---
    public static final String AI_MODE = "Simulation/AIMode";
    public static final String AI_MODE_CHOOSER = "Simulation/AIModeChooser";

    // --- Counts & choosers ---
    public static final String ALLY_ACTIVE_COUNT = "Simulation/AllyActiveCount";
    public static final String ALLY_COUNT = "Simulation/AllyCount";
    public static final String ALLY_COUNT_CHOOSER = "Simulation/AllyCountChooser";
    public static final String ALLY_COUNT_CHOOSER_SELECTED = "Simulation/AllyCountChooser/selected";
    public static final String OPPONENT_COUNT = "Simulation/OpponentCount";
    public static final String OPPONENT_COUNT_CHOOSER = "Simulation/OpponentCountChooser";
    public static final String OPPONENT_COUNT_CHOOSER_SELECTED = "Simulation/OpponentCountChooser/selected";
    public static final String MULTI_BOT_ACTIVE_COUNT = "Simulation/MultiBotActiveCount";

    // --- Speed scales ---
    public static final String ALLY_SPEED_PERCENT = "Simulation/AllySpeedPercent";
    public static final String OPPONENT_SPEED_PERCENT = "Simulation/OpponentSpeedPercent";

    // --- Legacy single-opponent telemetry ---
    public static final String OPPONENT_ACTIVE = "Simulation/OpponentActive";
    public static final String OPPONENT_AI_STATE = "Simulation/OpponentAIState";
    public static final String OPPONENT_FUEL_COUNT = "Simulation/OpponentFuelCount";
    public static final String OPPONENT_POSE = "Simulation/OpponentPose";
    public static final String OPPONENT_SCORE_COUNT = "Simulation/OpponentScoreCount";
    public static final String OPPONENT_STALLED = "Simulation/OpponentStalled";
    public static final String OPPONENT_TARGET_POSE = "Simulation/OpponentTargetPose";

    // --- Aggregate scores ---
    public static final String TOTAL_OPPONENT_SCORE = "Simulation/TotalOpponentScore";
    public static final String TOTAL_OPPONENT_FUEL = "Simulation/TotalOpponentFuel";
    public static final String TOTAL_ALLY_SCORE = "Simulation/TotalAllyScore";
    public static final String TOTAL_ALLY_FUEL = "Simulation/TotalAllyFuel";

    // --- Match control & player game state ---
    public static final String RESET = "Simulation/Reset";
    public static final String RESPAWN_BALLS = "Simulation/RespawnBalls";
    public static final String RUNNING = "Simulation/Running";
    public static final String SCORE = "Simulation/Score";
    public static final String HELD_BALLS = "Simulation/HeldBalls";
    public static final String LAST_SHOT_SCORED = "Simulation/LastShotScored";
    public static final String TIME_REMAINING_SEC = "Simulation/TimeRemainingSec";
    public static final String TIME_REMAINING_VALID = "Simulation/TimeRemainingValid";
    public static final String HUB_ACTIVE_BLUE = "Simulation/HubActive/Blue";
    public static final String HUB_ACTIVE_RED = "Simulation/HubActive/Red";
    public static final String GAME_PIECES = "Simulation/GamePieces";
    public static final String FULL_MATCH_BALL_DENSITY = "Simulation/FullMatchBallDensity";
    public static final String DEBUG_AI = "Simulation/DebugAI";

    /** Per-bot suffixes addressed through {@link #botPrefix} / {@link #allyPrefix}. */
    private static final String[] PER_BOT_SUFFIXES = {
            "/Archetype", "/ArchetypeChooser", "/Pose", "/TargetPose", "/Objective",
            "/StateDetail", "/Fuel", "/Score", "/Stalled", "/Mark"
    };

    private static final Set<String> ALL_KEYS = buildAllKeys();

    private static Set<String> buildAllKeys() {
        Set<String> keys = new HashSet<>(Arrays.asList(
                BOT0_ARCHETYPE, BOT0_ARCHETYPE_CHOOSER, BOT0_FUEL, BOT0_MARK, BOT0_OBJECTIVE,
                BOT0_POSE, BOT0_SCORE, BOT0_STALLED, BOT0_STATE_DETAIL, BOT0_TARGET_POSE,
                BOT1_ARCHETYPE, BOT1_ARCHETYPE_CHOOSER,
                BOT2_ARCHETYPE, BOT2_ARCHETYPE_CHOOSER,
                ALLY1_ARCHETYPE, ALLY1_ARCHETYPE_CHOOSER,
                ALLY2_ARCHETYPE, ALLY2_ARCHETYPE_CHOOSER,
                AI_MODE, AI_MODE_CHOOSER,
                ALLY_ACTIVE_COUNT, ALLY_COUNT, ALLY_COUNT_CHOOSER, ALLY_COUNT_CHOOSER_SELECTED,
                OPPONENT_COUNT, OPPONENT_COUNT_CHOOSER, OPPONENT_COUNT_CHOOSER_SELECTED,
                MULTI_BOT_ACTIVE_COUNT,
                ALLY_SPEED_PERCENT, OPPONENT_SPEED_PERCENT,
                OPPONENT_ACTIVE, OPPONENT_AI_STATE, OPPONENT_FUEL_COUNT, OPPONENT_POSE,
                OPPONENT_SCORE_COUNT, OPPONENT_STALLED, OPPONENT_TARGET_POSE,
                TOTAL_OPPONENT_SCORE, TOTAL_OPPONENT_FUEL, TOTAL_ALLY_SCORE, TOTAL_ALLY_FUEL,
                RESET, RESPAWN_BALLS, RUNNING, SCORE, HELD_BALLS, LAST_SHOT_SCORED,
                TIME_REMAINING_SEC, TIME_REMAINING_VALID, HUB_ACTIVE_BLUE, HUB_ACTIVE_RED,
                GAME_PIECES, FULL_MATCH_BALL_DENSITY, DEBUG_AI));
        for (int botId = 0; botId <= 2; botId++) {
            for (String suffix : PER_BOT_SUFFIXES) {
                keys.add(botPrefix(botId) + suffix);
            }
        }
        for (int allyIndex = 1; allyIndex <= 2; allyIndex++) {
            for (String suffix : PER_BOT_SUFFIXES) {
                keys.add(allyPrefix(allyIndex) + suffix);
            }
        }
        return Collections.unmodifiableSet(keys);
    }

    /**
     * Every {@code Simulation/*} topic the code can publish or subscribe,
     * including per-bot keys addressed via the prefix helpers. The Elastic
     * layout contract test asserts dashboard topics are members of this set.
     */
    public static Set<String> allKeys() {
        return ALL_KEYS;
    }
}
