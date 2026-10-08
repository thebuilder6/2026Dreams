package frc.robot.Sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.StrategicObjective;
import frc.robot.Intelligence.WorldState;

/**
 * Samples live sim-match states into decision-card TSV rows.
 *
 * <p>Why this exists. Hand-authored cards probe what a human imagines; this
 * probes what actually happened. A headless 3v3 already builds a
 * {@link WorldState} plus clairvoyant {@link MatchKnowledge} per bot per tick
 * (see {@code AIRobotInstance.update}), so the exact card inputs are sitting
 * there for free — no wpilog replay parsing, no coordinate conversion, no
 * second zoning implementation.
 *
 * <p>Sampling policy (hybrid, per bot): emit when the situated objective
 * changes, on a stall rising edge, on a hub-schedule phase change (first bot
 * processed in that tick wins the row), or every {@link #PERIODIC_SEC} of
 * match time as a floor. Consecutive samples with an identical
 * (objective, held-fuel bucket, phase) key are dropped unless the stall edge
 * fired. Transitions and stall edges bypass the {@link #MAX_SHIFT_PERIODIC}
 * budget (hard safety {@link #MAX_ROWS_TOTAL}); shift and periodic rows stop
 * once that budget is spent. Queuing-lane poses (Y &le; 0) never sample.
 *
 * <p>Opt-in and default off: {@link #arm} with a null/blank path leaves the
 * sampler disarmed, in which case {@link #maybeSample} is one volatile read
 * and a return. Enable with
 * {@code -PsnapshotCards=path/to/sampled.tsv} (forwarded to
 * {@code frc.headless.snapshotCards} by {@code build.gradle}, exactly like
 * {@code resultJsonl}). Rows append; merging them into the reviewed
 * {@code decision_cards.tsv} stays manual so a human eyeballs sampled rows
 * before they join the judged set.
 *
 * <p>Translation notes. The tracked opponent is the bot's mark
 * ({@code WorldState.opponentPose}); extras are every known opponent pose
 * farther than {@link #TRACKED_MATCH_M} from it, so the report does not list
 * the same body twice. {@code oppZonePieces} is written 0 because the
 * explicit list takes over. {@code expected} stays blank: the sampled
 * situated objective rode the commitment latch and {@code blockedFuel}, while
 * the card evaluates the pure utility race, so autofilling it would assert an
 * equality that does not hold. The provenance lives in {@code situation}
 * and the {@code S<seed>r<replica>-<label>-<t>s} id.
 */
public final class CardSnapshotSampler {

    /** Shift + periodic rows stop here per match; transitions/stalls bypass. */
    static final int MAX_SHIFT_PERIODIC = 12;
    /** Absolute safety ceiling so a thrashing match cannot flood the file. */
    static final int MAX_ROWS_TOTAL = 48;
    /** Periodic floor, seconds of match time. */
    static final double PERIODIC_SEC = 30.0;
    /** Poses this close to the tracked mark are the mark, not extras. */
    static final double TRACKED_MATCH_M = 0.05;

    private static volatile String outPath;
    private static volatile long seed;
    private static volatile int replica;
    private static volatile String variant = "baseline";

    private static final Map<String, BotState> bots = new ConcurrentHashMap<>();
    private static final List<String> rows = new ArrayList<>();
    private static volatile HubSchedule.Phase lastPhase;
    private static volatile int shiftPeriodicCount;
    private static volatile int totalCount;

    private CardSnapshotSampler() {}

    private static final class BotState {
        String lastObjective;
        int lastBucket = -1;
        String lastEmittedKey;
        double lastPeriodicT = Double.NaN;
        boolean lastStalled;
    }

    /**
     * Arms the sampler for one match. A null/blank path disarms (the default).
     * Clears all per-match state so re-arming is safe.
     */
    public static synchronized void arm(String tsvPath, long seedIn, int replicaIn, String variantIn) {
        bots.clear();
        synchronized (rows) {
            rows.clear();
        }
        lastPhase = null;
        shiftPeriodicCount = 0;
        totalCount = 0;
        seed = seedIn;
        replica = replicaIn;
        variant = (variantIn == null || variantIn.isBlank()) ? "baseline" : variantIn;
        outPath = (tsvPath == null || tsvPath.isBlank()) ? null : tsvPath.trim();
    }

    /** Test-only hook: drops all armed state. */
    static synchronized void disarmForTests() {
        arm(null, 0, 0, "baseline");
    }

    static boolean isArmed() {
        return outPath != null;
    }

    /** Held-fuel bucket for the dedup key. Boundaries mirror the card sweep. */
    static int bucket(int heldFuel) {
        if (heldFuel <= 0) return 0;
        if (heldFuel < 8) return 1;
        if (heldFuel < 18) return 2;
        if (heldFuel < 30) return 3;
        return 4;
    }

    /**
     * Samples one bot's tick. Called from {@code AIRobotInstance.update} right
     * after policy evaluation; every argument is already in scope there.
     */
    public static void maybeSample(String botLabel, Archetype archetype, WorldState world,
            MatchKnowledge knowledge, StrategicObjective objective, boolean stalled, int heldPieces) {
        if (outPath == null || botLabel == null || world == null || knowledge == null || objective == null) {
            return;
        }
        if (world.selfPose() == null || world.selfPose().getY() <= 0.0) {
            return; // queuing lane, not the field
        }
        double t = world.matchTimeRemaining();
        if (!Double.isFinite(t) || t < 0.0) {
            return;
        }
        HubSchedule.Phase phase;
        try {
            phase = HubSchedule.currentPhase();
        } catch (Exception e) {
            phase = HubSchedule.phaseFor(t, world.isAutonomous());
        }

        BotState st = bots.computeIfAbsent(botLabel, k -> new BotState());
        String obj = objective.name();
        int buck = bucket(heldPieces);
        boolean transition = st.lastObjective != null && !st.lastObjective.equals(obj);
        boolean stallEdge = stalled && !st.lastStalled;
        boolean shiftEdge = lastPhase != null && lastPhase != phase;
        boolean periodic = Double.isNaN(st.lastPeriodicT) || (st.lastPeriodicT - t) >= PERIODIC_SEC;
        st.lastObjective = obj;
        st.lastBucket = buck;
        st.lastStalled = stalled;
        lastPhase = phase;
        if (Double.isNaN(st.lastPeriodicT)) {
            st.lastPeriodicT = t; // floor starts counting from first sighting
            periodic = false;
        }

        String key = obj + "|" + buck + "|" + phase.name();
        if (key.equals(st.lastEmittedKey) && !stallEdge) {
            return; // same situation as last emitted row; stall edge still counts
        }
        boolean priority = transition || stallEdge;
        boolean budgeted = shiftEdge || periodic || firstEmission(st);
        if (!priority && !budgeted) {
            return;
        }
        synchronized (CardSnapshotSampler.class) {
            if (totalCount >= MAX_ROWS_TOTAL) {
                return;
            }
            if (!priority) {
                if (shiftPeriodicCount >= MAX_SHIFT_PERIODIC) {
                    return;
                }
                shiftPeriodicCount++;
            }
            totalCount++;
        }
        if (periodic) {
            st.lastPeriodicT = t;
        }
        st.lastEmittedKey = key;
        String row = toRow(botLabel, archetype, world, knowledge, obj, stalled, heldPieces);
        synchronized (rows) {
            rows.add(row);
        }
    }

    private static boolean firstEmission(BotState st) {
        return st.lastEmittedKey == null;
    }

    /**
     * Pure translator: one bot snapshot to one 27-column TSV row. No statics,
     * no HubSchedule read beyond what the caller passes — directly unit
     * testable.
     */
    static String toRow(String botLabel, Archetype archetype, WorldState world,
            MatchKnowledge knowledge, String situatedObjective, boolean stalled, int heldPieces) {
        long t = Math.round(world.matchTimeRemaining());
        String id = "S" + seed + "r" + replica + "-" + botLabel + "-" + t + "s";
        Pose2d self = world.selfPose();
        Pose2d mark = world.opponentPose() != null ? world.opponentPose() : new Pose2d();
        String safeVariant = variant.replaceAll("\\s+", " ");
        String situation = "SIM seed " + seed + " r" + replica + " " + safeVariant + ", " + botLabel
                + " (" + archetype.name() + ") at " + t + "s remaining, situated " + situatedObjective
                + (stalled ? " (STALLED)" : "") + ", held " + heldPieces;
        // The situated objective is recorded by the caller in the id-adjacent
        // flow; here the row carries the state, and `expected` stays blank for
        // human review (see class javadoc).
        String[] f = new String[27];
        f[0] = id;
        f[1] = archetype.name();
        f[2] = fmt2(self.getX());
        f[3] = fmt2(self.getY());
        f[4] = Integer.toString(world.heldFuelCount());
        f[5] = fmt1(world.matchTimeRemaining());
        f[6] = Boolean.toString(world.isAllianceHubActive());
        f[7] = Boolean.toString(world.isOpponentHubActive());
        f[8] = fmt1(world.timeUntilHubShift());
        f[9] = fmt2(mark.getX());
        f[10] = fmt2(mark.getY());
        f[11] = Boolean.toString(world.isAutonomous());
        f[12] = Boolean.toString(knowledge.opponentObserved());
        f[13] = Integer.toString(knowledge.scoreDifferential());
        f[14] = "0"; // explicit oppExtras takes over; see below
        f[15] = Integer.toString(knowledge.alliesHeldFuel());
        f[16] = situation;
        f[17] = "";
        f[18] = "Auto-sampled snapshot; human review needed before trusting.";
        f[19] = Integer.toString(knowledge.allianceZoneFuel());
        f[20] = Integer.toString(knowledge.midfieldFuel());
        f[21] = Integer.toString(knowledge.opponentZoneFuel());
        f[22] = Boolean.toString(world.hasShooter());
        f[23] = Integer.toString(world.ballCapacity());
        f[24] = Boolean.toString(world.hasClimber());
        f[25] = formatPoseList(knowledge.allyPoses());
        f[26] = formatPoseList(extrasExcludingTracked(knowledge.opponentPoses(), mark));
        return String.join("\t", f);
    }

    /** Opponent poses farther than {@link #TRACKED_MATCH_M} from the mark. */
    static List<Pose2d> extrasExcludingTracked(List<Pose2d> poses, Pose2d mark) {
        List<Pose2d> out = new ArrayList<>();
        if (poses == null) {
            return out;
        }
        for (Pose2d p : poses) {
            if (p == null) {
                continue;
            }
            if (mark != null
                    && p.getTranslation().getDistance(mark.getTranslation()) <= TRACKED_MATCH_M) {
                continue;
            }
            out.add(p);
        }
        return out;
    }

    static String formatPoseList(List<Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            return "";
        }
        StringBuilder sb = new StringBuilder();
        for (Pose2d p : poses) {
            if (p == null) {
                continue;
            }
            if (sb.length() > 0) {
                sb.append(';');
            }
            sb.append(fmt2(p.getX())).append(',').append(fmt2(p.getY()));
        }
        return sb.toString();
    }

    private static String fmt2(double v) {
        return String.format(java.util.Locale.ROOT, "%.2f", v);
    }

    private static String fmt1(double v) {
        return String.format(java.util.Locale.ROOT, "%.1f", v);
    }

    /** Appends sampled rows to the armed path; no-op when disarmed or empty. */
    public static synchronized void flush() {
        String path = outPath;
        List<String> copy;
        synchronized (rows) {
            if (path == null || rows.isEmpty()) {
                return;
            }
            copy = new ArrayList<>(rows);
            rows.clear();
        }
        try {
            Path out = Paths.get(path);
            if (out.getParent() != null) {
                Files.createDirectories(out.getParent());
            }
            StringBuilder sb = new StringBuilder();
            for (String r : copy) {
                sb.append(r).append(System.lineSeparator());
            }
            Files.writeString(out, sb.toString(), StandardCharsets.UTF_8,
                    StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            System.out.println("[CardSampler] appended " + copy.size() + " snapshot row(s) -> " + out);
        } catch (IOException e) {
            System.err.println("[CardSampler] failed to append snapshot rows: " + e.getMessage());
        }
    }
}
