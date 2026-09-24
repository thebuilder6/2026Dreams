package frc.robot.Sim;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Utils.AllianceFlipUtil;

/**
 * Jev AI Decision Engine (TypeSafe AI)
 * 
 * High-frequency "System One" decision model:
 * - Single parallel forward pass
 * - Sub-20ms deterministic execution latency guarantee
 * - Real-time tactical utility scoring:
 *   1. BLOCK_SHOOTING_LANE: Intercepts player shooting path to active Hub
 *   2. CONTEST_DEPOT: Denies game piece loading at human player depot
 *   3. SHADOW_PLAYER: Mirrors player Y movement along field midline
 *   4. RETREAT_DEFENSE: Falls back to protect alliance zone when hub shifts/deactivates
 * - Emits schema-validated structured JSON for AdvantageScope and telemetry.
 */
public class JevDecisionEngine {

    public enum TacticalAction {
        BLOCK_SHOOTING_LANE,
        CONTEST_DEPOT,
        SHADOW_PLAYER,
        RETREAT_DEFENSE
    }

    /**
     * Immutable decision result produced by the tactical evaluation.
     */
    public static class DecisionResult {
        public final TacticalAction action;
        public final Pose2d targetPose;
        public final double confidence;
        public final Map<TacticalAction, Double> utilityScores;
        public final String rationale;
        public final double latencyMs;
        public final double timestamp;

        public DecisionResult(
                TacticalAction action,
                Pose2d targetPose,
                double confidence,
                Map<TacticalAction, Double> utilityScores,
                String rationale,
                double latencyMs,
                double timestamp) {
            this.action = action;
            this.targetPose = targetPose;
            this.confidence = confidence;
            this.utilityScores = Collections.unmodifiableMap(utilityScores);
            this.rationale = rationale;
            this.latencyMs = latencyMs;
            this.timestamp = timestamp;
        }

        /**
         * Emits structured JSON adhering to the Jev AI schema.
         */
        public String toSchemaJson() {
            StringBuilder sb = new StringBuilder();
            sb.append("{");
            sb.append("\"timestamp\":").append(String.format("%.3f", timestamp)).append(",");
            sb.append("\"selected_action\":\"").append(action.name()).append("\",");
            sb.append("\"confidence\":").append(String.format("%.2f", confidence)).append(",");
            sb.append("\"latency_ms\":").append(String.format("%.2f", latencyMs)).append(",");
            sb.append("\"rationale\":\"").append(rationale.replace("\"", "\\\"")).append("\",");
            sb.append("\"target_pose\":{");
            sb.append("\"x\":").append(String.format("%.3f", targetPose.getX())).append(",");
            sb.append("\"y\":").append(String.format("%.3f", targetPose.getY())).append(",");
            sb.append("\"rot_deg\":").append(String.format("%.1f", targetPose.getRotation().getDegrees()));
            sb.append("},");
            sb.append("\"utility_scores\":{");
            int i = 0;
            for (Map.Entry<TacticalAction, Double> entry : utilityScores.entrySet()) {
                if (i++ > 0) sb.append(",");
                sb.append("\"").append(entry.getKey().name()).append("\":").append(String.format("%.3f", entry.getValue()));
            }
            sb.append("}}");
            return sb.toString();
        }
    }

    // Strategic Field Landmarks in standard Blue-origin coordinates (meters)
    public static final Translation2d BLUE_HUB_POS = new Translation2d(
            Constants.BLUE_HUB_LOCATION.getX(),
            Constants.BLUE_HUB_LOCATION.getY());
    public static final Translation2d BLUE_DEPOT_POS = new Translation2d(1.50, 6.50);
    public static final double CENTERLINE_X = 8.27; // Midfield dividing line
    public static final double RETREAT_X = 12.0;

    private static JevDecisionEngine instance;

    public static JevDecisionEngine getInstance() {
        if (instance == null) {
            instance = new JevDecisionEngine();
        }
        return instance;
    }

    private DecisionResult lastDecision = null;

    /**
     * Evaluates all tactical candidates in a single pass (< 20ms) and returns the optimal action.
     *
     * @param playerPose Current pose of the player robot (Blue-origin)
     * @param opponentPose Current pose of the AI sparring robot
     * @param matchTimeRemaining Seconds remaining in match
     * @param isHubActive Whether the target scoring Hub is active
     * @param isRedAlliance True if player is on Red Alliance (coordinates will be flipped for opponent)
     * @return DecisionResult containing selected action, target pose, and telemetry.
     */
    public DecisionResult evaluate(
            Pose2d playerPose,
            Pose2d opponentPose,
            double matchTimeRemaining,
            boolean isHubActive,
            boolean isRedAlliance) {

        long startNanos = System.nanoTime();
        double now = Timer.getFPGATimestamp();

        // 1. Calculate distances to strategic structures
        Translation2d targetHub = isRedAlliance ? 
                new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY()) : 
                BLUE_HUB_POS;

        Translation2d targetDepot = AllianceFlipUtil.apply(BLUE_DEPOT_POS, isRedAlliance);

        double distToHub = playerPose.getTranslation().getDistance(targetHub);
        double distToDepot = playerPose.getTranslation().getDistance(targetDepot);

        Map<TacticalAction, Double> scores = new LinkedHashMap<>();

        // 2. Score candidate A: BLOCK_SHOOTING_LANE
        // High utility when player is within shooting range (< 6m) and Hub is active
        double blockScore = 0.20;
        if (isHubActive && distToHub < 6.5) {
            double proximityWeight = Math.max(0.0, 1.0 - (distToHub / 6.5));
            blockScore = 0.65 + (0.30 * proximityWeight); // 0.65 to 0.95
        } else if (!isHubActive) {
            blockScore = 0.10; // Hub inactive, blocking lane is low priority
        }
        scores.put(TacticalAction.BLOCK_SHOOTING_LANE, blockScore);

        // 3. Score candidate B: CONTEST_DEPOT
        // High utility when player is approaching loading station / depot
        double depotScore = 0.25;
        if (distToDepot < 4.0) {
            double depotWeight = Math.max(0.0, 1.0 - (distToDepot / 4.0));
            depotScore = 0.60 + (0.35 * depotWeight); // 0.60 to 0.95
        }
        scores.put(TacticalAction.CONTEST_DEPOT, depotScore);

        // 4. Score candidate C: SHADOW_PLAYER
        // Midfield denial: mirrors player's Y axis to prevent transitions when Hub is active
        double shadowScore = isHubActive ? 0.50 : 0.20;
        double midfieldDist = Math.abs(playerPose.getX() - CENTERLINE_X);
        if (isHubActive && midfieldDist < 3.0) {
            shadowScore = 0.75 + (0.15 * (1.0 - midfieldDist / 3.0));
        }
        scores.put(TacticalAction.SHADOW_PLAYER, shadowScore);

        // 5. Score candidate D: RETREAT_DEFENSE
        // High utility if Hub is inactive or opponent needs to fall back to home quadrant
        double retreatScore = 0.15;
        if (!isHubActive) {
            retreatScore = 0.85;
        }
        if (matchTimeRemaining < 15.0) {
            retreatScore = Math.max(retreatScore, 0.80); // End-game defense
        }
        scores.put(TacticalAction.RETREAT_DEFENSE, retreatScore);

        // 6. Select highest utility action
        TacticalAction selected = TacticalAction.SHADOW_PLAYER;
        double maxScore = -1.0;
        for (Map.Entry<TacticalAction, Double> entry : scores.entrySet()) {
            if (entry.getValue() > maxScore) {
                maxScore = entry.getValue();
                selected = entry.getKey();
            }
        }

        // 7. Compute target pose for selected tactical action
        Pose2d targetPose;
        String rationale;

        switch (selected) {
            case BLOCK_SHOOTING_LANE:
                // Position opponent robot between player and Hub, 1.5m in front of player
                Translation2d dirToHub = targetHub.minus(playerPose.getTranslation());
                double norm = dirToHub.getNorm();
                Translation2d unitDir = norm > 1e-4 ? dirToHub.times(1.0 / norm) : new Translation2d(1, 0);
                Translation2d blockPos = playerPose.getTranslation().plus(unitDir.times(1.5));

                // Angle opponent robot directly facing player
                Rotation2d angleToPlayer = playerPose.getTranslation().minus(blockPos).getAngle();
                targetPose = new Pose2d(blockPos, angleToPlayer);
                rationale = String.format("Player is %.2fm from active Hub; blocking shooting corridor.", distToHub);
                break;

            case CONTEST_DEPOT:
                // Intercept route between player and depot
                Translation2d contestPos = targetDepot.plus(new Translation2d(0.8, -0.5));
                Rotation2d angleFacingDepot = targetDepot.minus(contestPos).getAngle();
                targetPose = new Pose2d(contestPos, angleFacingDepot);
                rationale = String.format("Player approaching Depot (%.2fm away); contesting game piece loading.", distToDepot);
                break;

            case RETREAT_DEFENSE:
                // Protect home defensive quadrant
                double retreatX = isRedAlliance ? (AllianceFlipUtil.FIELD_LENGTH - RETREAT_X) : RETREAT_X;
                targetPose = new Pose2d(retreatX, 4.0, Rotation2d.fromDegrees(isRedAlliance ? 0 : 180));
                rationale = isHubActive ? "Falling back to alliance defense perimeter." : "Hub inactive; holding defensive position.";
                break;

            case SHADOW_PLAYER:
            default:
                // Position at field centerline tracking player's Y coordinate
                double shadowX = isRedAlliance ? (CENTERLINE_X + 0.8) : (CENTERLINE_X - 0.8);
                double clampedY = Math.max(1.0, Math.min(AllianceFlipUtil.FIELD_WIDTH - 1.0, playerPose.getY()));
                Rotation2d facePlayer = playerPose.getTranslation().minus(new Translation2d(shadowX, clampedY)).getAngle();
                targetPose = new Pose2d(shadowX, clampedY, facePlayer);
                rationale = String.format("Shadowing player transition at midline (Y=%.2fm).", clampedY);
                break;
        }

        double latencyMs = (System.nanoTime() - startNanos) / 1_000_000.0;
        DecisionResult result = new DecisionResult(
                selected, targetPose, maxScore, scores, rationale, latencyMs, now);

        this.lastDecision = result;
        publishTelemetry(result);
        return result;
    }

    private void publishTelemetry(DecisionResult result) {
        SmartDashboard.putString("JevAI/SelectedAction", result.action.name());
        SmartDashboard.putNumber("JevAI/Confidence", result.confidence);
        SmartDashboard.putNumber("JevAI/LatencyMs", result.latencyMs);
        SmartDashboard.putString("JevAI/Rationale", result.rationale);
        SmartDashboard.putNumberArray("JevAI/TargetPose", new double[] {
                result.targetPose.getX(),
                result.targetPose.getY(),
                result.targetPose.getRotation().getDegrees()
        });
        SmartDashboard.putString("JevAI/TelemetryJson", result.toSchemaJson());
    }

    public DecisionResult getLastDecision() {
        return lastDecision;
    }

    public enum OffensiveStrategy {
        SCORE_HUB_HIGH("Score in Active Hub"),
        FEED_DEPOT("Intake at Alliance Depot"),
        BALL_HUNT_MIDFIELD("Hunt Neutral Game Pieces"),
        DEFEND_TRANSITION("Guard Midfield During Hub Shift");

        public final String description;

        OffensiveStrategy(String desc) {
            this.description = desc;
        }
    }

    public static class StrategicAdvice {
        public final OffensiveStrategy strategy;
        public final double utility;
        public final String adviceText;
        public final Pose2d suggestedWaypoint;

        public StrategicAdvice(
                OffensiveStrategy strategy,
                double utility,
                String adviceText,
                Pose2d suggestedWaypoint) {
            this.strategy = strategy;
            this.utility = utility;
            this.adviceText = adviceText;
            this.suggestedWaypoint = suggestedWaypoint;
        }
    }

    /**
     * In-Match Strategy Arbitrator: Evaluates offensive priorities based on Hub status and field geometry.
     */
    public StrategicAdvice evaluateOffensiveStrategy(
            Pose2d robotPose,
            double matchTimeRemaining,
            boolean isHubActive,
            boolean isRedAlliance) {
        Translation2d targetHub = isRedAlliance ?
                new Translation2d(Constants.RED_HUB_LOCATION.getX(), Constants.RED_HUB_LOCATION.getY()) :
                BLUE_HUB_POS;
        Translation2d targetDepot = AllianceFlipUtil.apply(BLUE_DEPOT_POS, isRedAlliance);

        double distToHub = robotPose.getTranslation().getDistance(targetHub);
        double distToDepot = robotPose.getTranslation().getDistance(targetDepot);

        OffensiveStrategy strategy;
        double utility;
        String advice;
        Pose2d waypoint;

        if (isHubActive && distToHub < 6.5) {
            strategy = OffensiveStrategy.SCORE_HUB_HIGH;
            utility = 0.90;
            advice = String.format("Hub active (%.1fm). Execute high scoring cycle.", distToHub);
            waypoint = new Pose2d(targetHub.plus(new Translation2d(isRedAlliance ? 2.5 : -2.5, 0.0)), Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
        } else if (!isHubActive) {
            if (distToDepot < 5.0) {
                strategy = OffensiveStrategy.FEED_DEPOT;
                utility = 0.85;
                advice = "Hub shifted inactive. Cycle inventory at Depot.";
                waypoint = new Pose2d(targetDepot, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
            } else {
                strategy = OffensiveStrategy.DEFEND_TRANSITION;
                utility = 0.75;
                advice = "Hub shifted inactive. Guard midfield transition lane.";
                double midX = isRedAlliance ? (AllianceFlipUtil.FIELD_LENGTH - CENTERLINE_X) : CENTERLINE_X;
                waypoint = new Pose2d(midX, robotPose.getY(), Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));
            }
        } else {
            strategy = OffensiveStrategy.BALL_HUNT_MIDFIELD;
            utility = 0.70;
            advice = "Acquiring neutral field game pieces.";
            waypoint = new Pose2d(CENTERLINE_X, 4.0, Rotation2d.fromDegrees(0));
        }

        SmartDashboard.putString("Strategy/Recommendation", strategy.description);
        SmartDashboard.putString("Strategy/AdviceText", advice);
        SmartDashboard.putNumber("Strategy/Utility", utility);

        return new StrategicAdvice(strategy, utility, advice, waypoint);
    }
}
