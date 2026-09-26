package frc.robot.Intelligence;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.ConcurrentHashMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Non-blocking client for TypeSafe System One. All request construction, HTTP
 * work, and response parsing run off the robot control thread.
 */
public final class TypeSafeJevClient {
    public static final URI ENDPOINT = URI.create("https://api.typesafe.ai/v1/systemone");
    public static final long MIN_REQUEST_INTERVAL_NANOS = 150_000_000L;
    public static final long GLOBAL_DISPATCH_INTERVAL_MILLIS = 150L;
    private static final long SIM_REQUEST_INTERVAL_NANOS = 1_000_000_000L;
    public static final long REQUEST_TIMEOUT_MILLIS = 1_000L;

    public record JevDecision(
            StrategicObjective objective,
            double confidence,
            Map<String, Double> probabilityDistribution,
            double opponentThreatProbability,
            double latencyMs,
            double timestamp) {
        public JevDecision {
            probabilityDistribution = Collections.unmodifiableMap(
                    new LinkedHashMap<>(probabilityDistribution));
        }
    }

    private static final TypeSafeJevClient INSTANCE = new TypeSafeJevClient();
    private static final ObjectMapper MAPPER = new ObjectMapper();
    private static final Map<String, String> OBJECTIVE_CRITERIA = Map.ofEntries(
            Map.entry("CYCLE_SCORE_HUB", "The alliance Hub is active and the robot has fuel to score."),
            Map.entry("STAGE_STANDOFF", "The alliance Hub is inactive, the robot holds fuel, and should wait at the scoring standoff."),
            Map.entry("VACUUM_MIDFIELD", "The hopper has capacity and harvesting midfield fuel is locally allowed."),
            Map.entry("STOCKPILE_DEPOT", "The alliance Hub is inactive and the robot should reload at its own depot."),
            Map.entry("SWEEP_ALLIANCE_ZONE", "Loose fuel is present in the alliance zone and sweeping is locally allowed."),
            Map.entry("DENY_SHOOTING_LANE", "A defensive role should block a tracked opponent's line to its active Hub."),
            Map.entry("LEAD_INTERCEPT", "A defensive role should intercept a tracked opponent."),
            Map.entry("RUSH_CLIMB", "Teleop endgame is active and the player robot should navigate to its climbing tower."));

    private final ExecutorService worker = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "TypeSafeJevClient");
        thread.setDaemon(true);
        return thread;
    });
    private final HttpClient httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofMillis(600))
            .executor(worker)
            .build();
    private final ConcurrentHashMap<String, Session> sessions = new ConcurrentHashMap<>();
    private final ConcurrentLinkedQueue<Session> dispatchQueue = new ConcurrentLinkedQueue<>();
    private final ScheduledExecutorService dispatcher = Executors.newSingleThreadScheduledExecutor(runnable -> {
        Thread thread = new Thread(runnable, "TypeSafeJevDispatcher");
        thread.setDaemon(true);
        return thread;
    });
    private volatile String apiKey = discoverApiKey();

    private record PendingRequest(
            String context,
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String apiKey) {}

    private static final class Session {
        private final AtomicBoolean requestInFlight = new AtomicBoolean(false);
        private final AtomicLong lastDispatchNanos = new AtomicLong(0L);
        private final AtomicReference<JevDecision> latestDecision = new AtomicReference<>();
        private final AtomicReference<String> lastError = new AtomicReference<>("");
        private final AtomicReference<PendingRequest> pendingRequest = new AtomicReference<>();
        private final AtomicBoolean requestQueued = new AtomicBoolean(false);
    }

    private TypeSafeJevClient() {
        dispatcher.scheduleAtFixedRate(this::dispatchNext, 0L,
                GLOBAL_DISPATCH_INTERVAL_MILLIS, TimeUnit.MILLISECONDS);
    }

    public static TypeSafeJevClient getInstance() {
        return INSTANCE;
    }

    private static String discoverApiKey() {
        String environmentKey = System.getenv("TYPESAFE_API_KEY");
        if (environmentKey != null && !environmentKey.isBlank()) {
            return environmentKey.trim();
        }
        String propertyKey = System.getProperty("typesafe.api.key", "");
        return propertyKey == null ? "" : propertyKey.trim();
    }

    public boolean hasValidKey() {
        return apiKey != null && !apiKey.isBlank();
    }

    /** Test/configuration hook. API keys are never logged or published. */
    public void setApiKey(String apiKey) {
        this.apiKey = apiKey == null ? "" : apiKey.trim();
    }

    public JevDecision getLatestDecision() {
        return getLatestDecision("CoPilot");
    }

    public JevDecision getLatestDecision(String context) {
        return session(context).latestDecision.get();
    }

    public String getLastError() {
        return getLastError("CoPilot");
    }

    public String getLastError(String context) {
        return session(context).lastError.get();
    }

    public double getOpponentThreatProbability() {
        JevDecision decision = getLatestDecision();
        return decision == null ? 0.0 : decision.opponentThreatProbability();
    }

    public boolean isRequestInFlight() {
        return isRequestInFlight("CoPilot");
    }

    public boolean isRequestInFlight(String context) {
        return session(context).requestInFlight.get();
    }

    /**
     * Queues a request only when the prior request has completed and the
     * per-caller dispatch interval has elapsed. A single fair dispatcher spaces
     * all callers' requests globally. Returns immediately.
     */
    public boolean evaluateAsync(WorldState world, MatchKnowledge knowledge, Archetype archetype) {
        return evaluateAsync("CoPilot", world, knowledge, archetype);
    }

    /** Independent async request state for each logical Jev caller. */
    public boolean evaluateAsync(String context, WorldState world, MatchKnowledge knowledge, Archetype archetype) {
        String requestContext = normalizeContext(context);
        Session session = session(requestContext);
        String requestKey = apiKey;
        if (world == null || requestKey == null || requestKey.isBlank()) {
            return false;
        }

        long dispatchNanos = System.nanoTime();
        long minimumInterval = requestContext.startsWith("Sim/")
                ? SIM_REQUEST_INTERVAL_NANOS
                : MIN_REQUEST_INTERVAL_NANOS;
        MatchKnowledge requestKnowledge = knowledge == null ? MatchKnowledge.unknown() : knowledge;
        PendingRequest request = new PendingRequest(
                requestContext, world, requestKnowledge, archetype, requestKey);

        synchronized (session) {
            // Coalesce repeated updates while queued so the service receives the
            // newest world snapshot when this caller reaches its fair turn.
            if (session.requestQueued.get()) {
                session.pendingRequest.set(request);
                return true;
            }
            if (session.requestInFlight.get()) {
                return false;
            }
            long previous = session.lastDispatchNanos.get();
            if (previous != 0L && dispatchNanos - previous < minimumInterval) {
                return false;
            }
            session.pendingRequest.set(request);
            session.requestInFlight.set(true);
            session.requestQueued.set(true);
            dispatchQueue.offer(session);
            return true;
        }
    }

    private void dispatchNext() {
        Session session = dispatchQueue.poll();
        if (session == null) return;

        PendingRequest request;
        long startNanos = System.nanoTime();
        synchronized (session) {
            if (!session.requestQueued.get()) return;
            session.requestQueued.set(false);
            request = session.pendingRequest.getAndSet(null);
            if (request == null) {
                session.requestInFlight.set(false);
                return;
            }
            session.lastDispatchNanos.set(startNanos);
        }

        try {
            worker.execute(() -> sendRequest(session, request.context(), request.world(),
                    request.knowledge(), request.archetype(), request.apiKey(), startNanos));
        } catch (RuntimeException exception) {
            session.lastError.set("worker_unavailable");
            session.requestInFlight.set(false);
        }
    }

    private void sendRequest(
            Session session,
            String context,
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String requestKey,
            long startNanos) {
        try {
            String json = MAPPER.writeValueAsString(buildPayload(world, knowledge, archetype, context));
            HttpRequest request = HttpRequest.newBuilder(ENDPOINT)
                    .timeout(Duration.ofMillis(REQUEST_TIMEOUT_MILLIS))
                    .header("Authorization", "Bearer " + requestKey)
                    .header("Content-Type", "application/json")
                    .POST(HttpRequest.BodyPublishers.ofString(json))
                    .build();

            CompletableFuture<HttpResponse<String>> responseFuture = httpClient.sendAsync(
                    request, HttpResponse.BodyHandlers.ofString());
            responseFuture.thenApplyAsync(response -> {
                if (response.statusCode() != 200) {
                    throw new IllegalStateException("http_status_" + response.statusCode());
                }
                try {
                    return parseDecision(response.body(), startNanos);
                } catch (Exception exception) {
                    throw new CompletionException(exception);
                }
            }, worker).whenComplete((decision, failure) -> {
                try {
                    if (failure == null && decision != null) {
                        session.latestDecision.set(decision);
                        session.lastError.set("");
                    } else {
                        session.lastError.set(safeErrorCode(failure));
                    }
                } finally {
                    session.requestInFlight.set(false);
                }
            });
        } catch (Exception exception) {
            session.lastError.set(safeErrorCode(exception));
            session.requestInFlight.set(false);
        }
    }

    private static String safeErrorCode(Throwable throwable) {
        if (throwable == null) return "request_failed";
        Throwable cause = throwable;
        while (cause.getCause() != null) cause = cause.getCause();
        if (cause instanceof IllegalStateException && cause.getMessage() != null
                && cause.getMessage().startsWith("http_status_")) {
            return cause.getMessage();
        }
        return cause.getClass().getSimpleName().isBlank()
                ? "request_failed"
                : cause.getClass().getSimpleName();
    }

    static JevDecision parseDecision(String responseBody, long startNanos) throws Exception {
        JsonNode root = MAPPER.readTree(responseBody);
        JsonNode answers = root.path("answers");
        JsonNode objectiveAnswer = answers.path("strategic_objective");
        if (!"choice".equals(objectiveAnswer.path("type").asText())) {
            throw new IllegalArgumentException("missing_choice_answer");
        }

        StrategicObjective objective = StrategicObjective.valueOf(objectiveAnswer.path("choice").asText());
        double confidence = objectiveAnswer.path("confidence").asDouble(Double.NaN);
        if (!Double.isFinite(confidence) || confidence < 0.0 || confidence > 1.0) {
            throw new IllegalArgumentException("invalid_confidence");
        }

        Map<String, Double> probabilities = new LinkedHashMap<>();
        JsonNode probabilityNode = objectiveAnswer.path("probabilities");
        probabilityNode.properties().forEach(entry -> {
            if (entry.getValue().isNumber()) {
                double probability = entry.getValue().asDouble();
                if (!Double.isFinite(probability) || probability < 0.0 || probability > 1.0) {
                    throw new IllegalArgumentException("invalid_probability");
                }
                probabilities.put(entry.getKey(), probability);
            }
        });
        if (probabilities.isEmpty() || !probabilities.containsKey(objective.name())) {
            throw new IllegalArgumentException("missing_probabilities");
        }

        JsonNode threatAnswer = answers.path("opponent_threat_high");
        double threatProbability = threatAnswer.path("noul").asDouble(0.0);
        if (!Double.isFinite(threatProbability) || threatProbability < 0.0 || threatProbability > 1.0) {
            throw new IllegalArgumentException("invalid_noul_probability");
        }
        long completionNanos = System.nanoTime();
        return new JevDecision(objective, confidence, probabilities, threatProbability,
                (completionNanos - startNanos) / 1_000_000.0,
                completionNanos / 1_000_000_000.0);
    }

    private Session session(String context) {
        return sessions.computeIfAbsent(normalizeContext(context), ignored -> new Session());
    }

    private static String normalizeContext(String context) {
        return context == null || context.isBlank() ? "CoPilot" : context;
    }

    private static Map<String, Object> buildPayload(
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype) {
        return buildPayload(world, knowledge, archetype, "CoPilot");
    }

    private static Map<String, Object> buildPayload(
            WorldState world,
            MatchKnowledge knowledge,
            Archetype archetype,
            String context) {
        Map<String, Object> state = new LinkedHashMap<>();
        state.put("agent_id", context);
        state.put("robot_pose", poseState(world.selfPose()));
        state.put("robot_velocity_mps", velocityState(world.selfVelocity()));
        state.put("match_time_remaining_s", world.matchTimeRemaining());
        state.put("alliance_hub_active", world.isAllianceHubActive());
        state.put("opponent_hub_active", world.isOpponentHubActive());
        state.put("seconds_until_hub_shift", world.timeUntilHubShift());
        state.put("held_fuel", world.heldFuelCount());
        state.put("hopper_capacity", WorldState.DEFAULT_MAX_CAPACITY);
        state.put("alliance", world.isRedAlliance() ? "red" : "blue");
        state.put("autonomous", world.isAutonomous());
        state.put("archetype", archetype.name());
        state.put("score_differential", knowledge.scoreDifferential());
        state.put("allies_held_fuel", knowledge.alliesHeldFuel());
        state.put("opponents_held_fuel", knowledge.opponentsHeldFuel());
        state.put("opponent_observed", knowledge.opponentObserved());
        state.put("ally_poses", poseListState(knowledge.allyPoses()));
        state.put("opponent_poses", poseListState(knowledge.opponentPoses()));

        Map<String, Object> questions = new LinkedHashMap<>();
        questions.put("strategic_objective", Map.of(
                "type", "choice",
                "instructions", "Choose a currently useful robot macro objective. The local robot software enforces field and mechanism safety.",
                "criteria", OBJECTIVE_CRITERIA));
        questions.put("opponent_threat_high", Map.of(
                "type", "noul",
                "instructions", "Is a tracked opponent currently threatening our scoring zone or holding significant fuel?",
                "criteria", Map.of(
                        "true", "A tracked opponent is near our scoring zone or carries significant fuel.",
                        "false", "No tracked opponent currently presents that threat.")));

        return Map.of("model", "jev-latest", "state", state, "questions", questions);
    }

    private static Map<String, Double> poseState(Pose2d pose) {
        return Map.of("x_m", pose.getX(), "y_m", pose.getY(), "heading_deg", pose.getRotation().getDegrees());
    }

    private static List<Map<String, Double>> poseListState(List<Pose2d> poses) {
        return poses.stream().map(TypeSafeJevClient::poseState).toList();
    }

    private static Map<String, Double> velocityState(ChassisSpeeds speeds) {
        return Map.of("vx_mps", speeds.vxMetersPerSecond,
                "vy_mps", speeds.vyMetersPerSecond,
                "omega_rad_s", speeds.omegaRadiansPerSecond);
    }
}
