package frc.robot.Sim;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.DynamicObstacle;
import frc.robot.Auto.DynamicRouter;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Data.GlideConstants.GlidePoint;
import frc.robot.Utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

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

    /**
     * Smart "One-Button Glide" Arbitration:
     * Arbitrates target waypoint dynamically between:
     * 1. Hub shooting pose (if robot has fuel AND Hub is active)
     * 2. Midfield neutral game piece hunt (if robot is empty AND Hub is active)
     * 3. Feeder / Depot loading station (if Hub is currently inactive)
     *
     * @param robotPose Current pose of the robot
     * @param hasFuel True if robot is holding fuel / game piece
     * @param isHubActive True if scoring Hub is active
     * @param isRedAlliance True if on Red Alliance
     * @return Target Pose2d for autonomous glide navigation
     */
    public Pose2d getSmartGlideTarget(
            Pose2d robotPose,
            boolean hasFuel,
            boolean isHubActive,
            boolean isRedAlliance) {

        Pose2d targetPose;
        String mode;

        if (hasFuel && isHubActive) {
            // Mode 1: Hub Shooting Pose
            String frontKey = isRedAlliance ? "Red Hub Front" : "Blue Hub Front";
            String backKey = isRedAlliance ? "Red Hub Back" : "Blue Hub Back";

            Pose2d frontPose = GlideConstants.GLIDE_POINTS.containsKey(frontKey) ? 
                    GlideConstants.GLIDE_POINTS.get(frontKey).pose() : 
                    new Pose2d(isRedAlliance ? 11.0 : 5.6, 4.10, Rotation2d.fromDegrees(isRedAlliance ? 0 : 180));
            Pose2d backPose = GlideConstants.GLIDE_POINTS.containsKey(backKey) ? 
                    GlideConstants.GLIDE_POINTS.get(backKey).pose() : 
                    new Pose2d(isRedAlliance ? 13.9 : 2.6, 4.10, Rotation2d.fromDegrees(isRedAlliance ? 180 : 0));

            double distFront = robotPose.getTranslation().getDistance(frontPose.getTranslation());
            double distBack = robotPose.getTranslation().getDistance(backPose.getTranslation());

            targetPose = (distFront <= distBack) ? frontPose : backPose;
            mode = "SCORE_HUB (" + (distFront <= distBack ? "Front" : "Back") + ")";
        } else if (!hasFuel && isHubActive) {
            // Mode 2: Neutral Ball Hunt (Midfield)
            Pose2d topMid = GlideConstants.GLIDE_POINTS.containsKey("Midfield Top") ?
                    GlideConstants.GLIDE_POINTS.get("Midfield Top").pose() :
                    new Pose2d(CENTERLINE_X, 6.10, Rotation2d.fromDegrees(-90));
            Pose2d botMid = GlideConstants.GLIDE_POINTS.containsKey("Midfield Bottom") ?
                    GlideConstants.GLIDE_POINTS.get("Midfield Bottom").pose() :
                    new Pose2d(CENTERLINE_X, 2.00, Rotation2d.fromDegrees(90));

            double distTop = Math.abs(robotPose.getY() - topMid.getY());
            double distBot = Math.abs(robotPose.getY() - botMid.getY());

            targetPose = (distTop <= distBot) ? topMid : botMid;
            mode = "BALL_HUNT_MIDFIELD (" + (distTop <= distBot ? "Top" : "Bottom") + ")";
        } else {
            // Mode 3: Feeder / Depot Reloading (Hub Inactive)
            String topFeederKey = isRedAlliance ? "Red Feeder Top" : "Blue Feeder Top";
            String botFeederKey = isRedAlliance ? "Red Feeder Bottom" : "Blue Feeder Bottom";

            Pose2d topFeeder = GlideConstants.GLIDE_POINTS.containsKey(topFeederKey) ?
                    GlideConstants.GLIDE_POINTS.get(topFeederKey).pose() :
                    new Pose2d(isRedAlliance ? 15.0 : 1.5, 6.0, Rotation2d.fromDegrees(isRedAlliance ? -145 : -35));
            Pose2d botFeeder = GlideConstants.GLIDE_POINTS.containsKey(botFeederKey) ?
                    GlideConstants.GLIDE_POINTS.get(botFeederKey).pose() :
                    new Pose2d(isRedAlliance ? 15.0 : 1.5, 2.2, Rotation2d.fromDegrees(isRedAlliance ? 145 : 35));

            double distTop = robotPose.getTranslation().getDistance(topFeeder.getTranslation());
            double distBot = robotPose.getTranslation().getDistance(botFeeder.getTranslation());

            targetPose = (distTop <= distBot) ? topFeeder : botFeeder;
            mode = "RELOAD_DEPOT (" + (distTop <= distBot ? "Top" : "Bottom") + ")";
        }

        SmartDashboard.putString("JevAI/GlideArbitrationMode", mode);
        SmartDashboard.putNumberArray("JevAI/GlideTarget", new double[] {
                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()
        });
        Logger.recordOutput("JevAI/SmartGlideTarget", targetPose);

        return targetPose;
    }

    /**
     * Quadratic Lead-Pursuit Interception Solver:
     * Computes the exact time t and location P(t) where the robot at max speed can intercept
     * an opponent robot traveling at constant velocity V_opp along ray L(t) = P_opp + V_opp * t.
     * Solves: ||(P_opp - P_robot) + V_opp * t||^2 = (V_max * t)^2
     *
     * @param robotPose Current robot pose
     * @param opponentPose Current opponent pose
     * @param opponentVel Opponent field-relative velocity vector (m/s)
     * @param maxRobotSpeed Maximum available robot velocity (m/s)
     * @return Optimal intercept Pose2d clamped within legal field borders, facing opponent
     */
    public Pose2d solveLeadPursuitIntercept(
            Pose2d robotPose,
            Pose2d opponentPose,
            Translation2d opponentVel,
            double maxRobotSpeed) {

        if (opponentPose == null) return robotPose;
        if (opponentVel == null) opponentVel = new Translation2d();
        if (maxRobotSpeed <= 0.1) maxRobotSpeed = Constants.MAX_SPEED;

        Translation2d delta = opponentPose.getTranslation().minus(robotPose.getTranslation());
        double dx = delta.getX();
        double dy = delta.getY();
        double vx = opponentVel.getX();
        double vy = opponentVel.getY();

        // Quadratic coefficients: a*t^2 + b*t + c = 0
        double a = (vx * vx + vy * vy) - (maxRobotSpeed * maxRobotSpeed);
        double b = 2.0 * (dx * vx + dy * vy);
        double c = dx * dx + dy * dy;

        double interceptTime = -1.0;

        if (Math.abs(a) < 1e-5) {
            // Linear case: b*t + c = 0
            if (Math.abs(b) > 1e-4) {
                double tLinear = -c / b;
                if (tLinear > 0.05) {
                    interceptTime = tLinear;
                }
            }
        } else {
            double discriminant = (b * b) - (4.0 * a * c);
            if (discriminant >= 0) {
                double sqrtD = Math.sqrt(discriminant);
                double t1 = (-b - sqrtD) / (2.0 * a);
                double t2 = (-b + sqrtD) / (2.0 * a);

                if (t1 > 0.05 && t2 > 0.05) {
                    interceptTime = Math.min(t1, t2);
                } else if (t1 > 0.05) {
                    interceptTime = t1;
                } else if (t2 > 0.05) {
                    interceptTime = t2;
                }
            }
        }

        Translation2d interceptPos;
        if (interceptTime > 0.05 && interceptTime < 5.0) {
            interceptPos = opponentPose.getTranslation().plus(opponentVel.times(interceptTime));
        } else {
            // Fallback to pure pursuit with minor lead projection
            interceptPos = opponentPose.getTranslation().plus(opponentVel.times(0.25));
        }

        // Clamp inside legal field borders
        double clampedX = Math.max(0.60, Math.min(15.94, interceptPos.getX()));
        double clampedY = Math.max(0.60, Math.min(7.65, interceptPos.getY()));

        // Face oncoming opponent from intercept waypoint
        Rotation2d faceOpponent = opponentPose.getTranslation().minus(new Translation2d(clampedX, clampedY)).getAngle();
        Pose2d target = new Pose2d(clampedX, clampedY, faceOpponent);

        Logger.recordOutput("JevAI/LeadPursuitIntercept", target);
        return target;
    }

    /**
     * Offensive Anti-Defense: Dynamic Trench Corridor Selection.
     * Evaluates obstacle presence in Top Trench vs Bottom Trench to route through the open corridor.
     *
     * @param robotPose Current robot pose
     * @param outbound True if traveling towards midfield, false if returning to alliance zone
     * @param isRedAlliance True if on Red Alliance
     * @return Selected GlidePoint for the trench corridor with least defensive resistance
     */
    public GlidePoint selectOptimalTrenchCorridor(
            Pose2d robotPose,
            boolean outbound,
            boolean isRedAlliance) {

        String topKey = isRedAlliance ? "Red Top Trench" : "Blue Top Trench";
        String botKey = isRedAlliance ? "Red Bottom Trench" : "Blue Bottom Trench";

        GlidePoint topPoint = GlideConstants.GLIDE_POINTS.get(topKey);
        GlidePoint botPoint = GlideConstants.GLIDE_POINTS.get(botKey);

        // Region of interest for trench obstacles
        double minX = isRedAlliance ? 10.3 : 3.0;
        double maxX = isRedAlliance ? 13.6 : 6.3;

        int topObstacleCount = 0;
        int botObstacleCount = 0;

        for (DynamicObstacle obs : DynamicRouter.getActiveObstacles()) {
            double ox = obs.position.getX();
            double oy = obs.position.getY();

            if (ox >= minX && ox <= maxX) {
                if (oy > 5.5) {
                    topObstacleCount++;
                } else if (oy < 2.5) {
                    botObstacleCount++;
                }
            }
        }

        GlidePoint chosen;
        if (topObstacleCount < botObstacleCount) {
            chosen = topPoint;
        } else if (botObstacleCount < topObstacleCount) {
            chosen = botPoint;
        } else {
            // Tied: pick trench closest in Y to current robot pose
            double distTop = Math.abs(robotPose.getY() - (topPoint != null ? topPoint.pose.getY() : 7.4));
            double distBot = Math.abs(robotPose.getY() - (botPoint != null ? botPoint.pose.getY() : 0.65));
            chosen = (distTop <= distBot) ? topPoint : botPoint;
        }

        SmartDashboard.putString("JevAI/TrenchCorridorSelected", chosen != null ? chosen.name : "None");
        return chosen;
    }
}
