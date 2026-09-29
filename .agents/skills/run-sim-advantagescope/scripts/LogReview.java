import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Reviews an AdvantageKit .wpilog using only wpiutil (offline-safe).
 * Reports metadata, topic inventory, per-bot stuck windows (identical
 * ActualPose bytes), IsStuck timelines, StateDetail transitions, last
 * Score/Fuel, and string records mentioning exceptions.
 */
public class LogReview {
    private static final double STILL_SEC = 1.0;
    private static final Pattern BOT_PAT = Pattern.compile("/(Bot\\d|Ally\\d)/");
    private static final Pattern META_PAIR =
            Pattern.compile("\"([^\"]+)\"\\s*:\\s*\"([^\"]*)\"");
    private static final String[] DETAIL_ROOTS =
            {"Simulation", "AI_Telemetry", "RealOutputs", "SmartDashboard", "FieldSimulation"};

    private static String norm(String name) {
        String n = name;
        for (;;) {
            if (n.startsWith("/")) {
                n = n.substring(1);
                continue;
            }
            if (n.startsWith("NT:")) {
                n = n.substring(3);
                continue;
            }
            break;
        }
        if (n.startsWith("AdvantageKit/")) n = n.substring("AdvantageKit/".length());
        return n;
    }

    private static String botOf(String name) {
        String n = "/" + norm(name) + "/";
        Matcher bm = BOT_PAT.matcher(n);
        if (bm.find()) return bm.group(1);
        if (n.contains("/AI_Telemetry/")) return "Bot0-legacy";
        return null;
    }

    private static boolean overlapsEnabled(
            List<long[]> spans, Long openStart, long sUs, long eUs) {
        for (long[] sp : spans) {
            if (sp[0] <= eUs && sp[1] >= sUs) return true;
        }
        return openStart != null && openStart <= eUs;
    }

    private static boolean parkedAt(
            Map<String, List<Long>> times, Map<String, List<String>> vals,
            String bot, long tsUs) {
        List<Long> t = times.get(bot);
        List<String> v = vals.get(bot);
        if (t == null || v == null) return false;
        String cur = null;
        for (int i = 0; i < t.size(); i++) {
            if (t.get(i) <= tsUs) cur = v.get(i);
            else break;
        }
        return cur != null && (cur.equals("DISABLED") || cur.equals("IDLE"));
    }

    private static double[] poseXY(byte[] raw) {
        try {
            if (raw == null || raw.length < 16) return null;
            java.nio.ByteBuffer bb =
                    java.nio.ByteBuffer.wrap(raw).order(java.nio.ByteOrder.LITTLE_ENDIAN);
            return new double[]{bb.getDouble(0), bb.getDouble(8)};
        } catch (Exception e) {
            return null;
        }
    }

    private static String fmtXY(double[] xy) {
        return xy == null ? "?" : String.format("(%.2f, %.2f)", xy[0], xy[1]);
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.out.println("usage: LogReview <file.wpilog>");
            return;
        }
        DataLogReader reader = new DataLogReader(args[0]);
        if (!reader.isValid()) {
            System.out.println("INVALID LOG: " + args[0]);
            return;
        }

        Map<Integer, String> names = new HashMap<>();
        Map<Integer, String> types = new HashMap<>();
        List<String> rawMeta = new ArrayList<>();
        Map<String, Long> prefixCounts = new TreeMap<>();
        Map<String, byte[]> lastPose = new HashMap<>();
        Map<String, Long> poseSince = new HashMap<>();
        Map<String, Long> longestStill = new HashMap<>();
        Map<String, Integer> stillWindows = new HashMap<>();
        Map<String, List<String>> stillStarts = new HashMap<>();
        Map<String, Long> stuckStart = new HashMap<>();
        Map<String, Long> longestStuck = new HashMap<>();
        Map<String, Integer> stuckRuns = new HashMap<>();
        Map<String, String> lastDetail = new HashMap<>();
        Map<String, List<String>> detailChanges = new HashMap<>();
        Map<String, List<Long>> detailTimes = new HashMap<>();
        Map<String, List<String>> detailVals = new HashMap<>();
        Map<String, double[]> stillStartXY = new HashMap<>();
        Map<String, String> lastScalar = new HashMap<>();
        Map<String, Long> detailTopics = new TreeMap<>();
        Map<String, Boolean> dsState = new HashMap<>();
        List<String> dsChanges = new ArrayList<>();
        List<long[]> enabledSpans = new ArrayList<>();
        Long enabledOpen = null;
        Map<String, String> fileMeta = new LinkedHashMap<>();
        List<String> errorHits = new ArrayList<>();

        long tMin = Long.MAX_VALUE, tMax = Long.MIN_VALUE, records = 0;

        for (DataLogRecord r : reader) {
            records++;
            long ts = r.getTimestamp();
            if (ts < tMin) tMin = ts;
            if (ts > tMax) tMax = ts;
            try {
                if (r.isStart()) {
                    var d = r.getStartData();
                    names.put(d.entry, d.name);
                    types.put(d.entry, d.type);
                    continue;
                }
                if (r.isSetMetadata()) {
                    rawMeta.add(r.getSetMetadataData().metadata);
                    continue;
                }
                if (r.isFinish() || r.isControl()) continue;
                String name = names.get(r.getEntry());
                if (name == null) continue;
                String type = types.getOrDefault(r.getEntry(), "?");
                String n = norm(name);
                int slash = n.indexOf('/');
                String prefix = slash > 0 ? n.substring(0, slash) : n;
                prefixCounts.merge(prefix, 1L, Long::sum);
                for (String root : DETAIL_ROOTS) {
                    if (n.equals(root) || n.startsWith(root + "/")) {
                        detailTopics.merge(n + " [" + type + "]", 1L, Long::sum);
                        break;
                    }
                }
                boolean isDsFlag = (n.equals("DS:enabled") || n.equals("DS:autonomous")
                                || n.equals("DS:test") || n.endsWith("/DriverStation/Enabled")
                                || n.endsWith("/DriverStation/Autonomous")
                                || n.endsWith("/DriverStation/Test"))
                        && type.equals("boolean");
                if (isDsFlag) {
                    boolean v = r.getBoolean();
                    Boolean prev = dsState.get(n);
                    if ((prev == null || prev != v) && dsChanges.size() < 60) {
                        dsChanges.add(String.format(
                                "%.1fs %s=%s", ts / 1e6, n, v));
                    }
                    boolean isEnabledFlag =
                            n.equals("DS:enabled") || n.endsWith("/DriverStation/Enabled");
                    if (isEnabledFlag) {
                        if (v && enabledOpen == null) {
                            enabledOpen = ts;
                        } else if (!v && enabledOpen != null) {
                            enabledSpans.add(new long[]{enabledOpen, ts});
                            enabledOpen = null;
                        }
                    }
                    dsState.put(n, v);
                    continue;
                }
                if (n.startsWith("RealMetadata/") && type.equals("string")) {
                    fileMeta.putIfAbsent(n.substring("RealMetadata/".length()), r.getString());
                    continue;
                }

                String bot = botOf(name);

                if (name.endsWith("/ActualPose") && bot != null) {
                    byte[] raw = r.getRaw();
                    byte[] prev = lastPose.get(bot);
                    if (prev != null && java.util.Arrays.equals(prev, raw)) {
                        // still moving the window; finalized on change/end
                    } else {
                        if (prev != null) {
                            long sUs = poseSince.get(bot);
                            double stillSec = (ts - sUs) / 1e6;
                            boolean parked = parkedAt(detailTimes, detailVals, bot, sUs);
                            if (stillSec >= STILL_SEC && !parked
                                    && overlapsEnabled(enabledSpans, enabledOpen, sUs, ts)) {
                                stillWindows.merge(bot, 1, Integer::sum);
                                longestStill.merge(bot, ts - sUs, Math::max);
                                stillStarts.computeIfAbsent(bot, k -> new ArrayList<>());
                                List<String> starts = stillStarts.get(bot);
                                if (starts.size() < 10) {
                                    starts.add(String.format("%.1fs-%.1fs at %s",
                                            sUs / 1e6, ts / 1e6,
                                            fmtXY(stillStartXY.get(bot))));
                                }
                            }
                        }
                        lastPose.put(bot, raw.clone());
                        poseSince.put(bot, ts);
                        stillStartXY.put(bot, poseXY(raw));
                    }
                    continue;
                }
                if (name.endsWith("IsStuck") && type.equals("boolean") && bot != null) {
                    boolean stuck = r.getBoolean();
                    Long start = stuckStart.get(bot);
                    if (stuck && start == null) {
                        stuckStart.put(bot, ts);
                    } else if (!stuck && start != null) {
                        stuckRuns.merge(bot, 1, Integer::sum);
                        longestStuck.merge(bot, ts - start, Math::max);
                        stuckStart.remove(bot);
                    }
                    continue;
                }
                if (name.endsWith("StateDetail") && type.equals("string") && bot != null) {
                    String v = r.getString();
                    String prev = lastDetail.get(bot);
                    if (prev != null && !prev.equals(v)) {
                        detailChanges.computeIfAbsent(bot, k -> new ArrayList<>());
                        List<String> ch = detailChanges.get(bot);
                        if (ch.size() < 30) {
                            ch.add(String.format("%.1fs %s -> %s", ts / 1e6, prev, v));
                        }
                    }
                    if (prev == null || !prev.equals(v)) {
                        detailTimes.computeIfAbsent(bot, k -> new ArrayList<>()).add(ts);
                        detailVals.computeIfAbsent(bot, k -> new ArrayList<>()).add(v);
                    }
                    lastDetail.put(bot, v);
                    continue;
                }
                if (bot != null
                        && (name.endsWith("/Score") || name.endsWith("ScoreCount")
                                || name.endsWith("/HeldFuel")
                                || name.endsWith("/Fuel") || name.endsWith("/Archetype")
                                || name.endsWith("/Objective"))) {
                    lastScalar.put(name, scalarToString(r, type));
                    continue;
                }
                if (type.equals("string")) {
                    String v = r.getString();
                    if (v.toLowerCase(java.util.Locale.ROOT).contains("exception")
                            && errorHits.size() < 40) {
                        errorHits.add(String.format(
                                "%.1fs %s: %s", ts / 1e6, name,
                                v.length() > 140 ? v.substring(0, 140) : v));
                    }
                }
            } catch (Exception e) {
                // One bad record must not kill the review; keep scanning.
            }
        }
        // Finalize open still windows and stuck runs at end of log.
        for (String bot : lastPose.keySet()) {
            long sUs = poseSince.get(bot);
            double stillSec = (tMax - sUs) / 1e6;
            boolean parked = parkedAt(detailTimes, detailVals, bot, sUs);
            if (stillSec >= STILL_SEC && !parked
                    && overlapsEnabled(enabledSpans, enabledOpen, sUs, tMax)) {
                stillWindows.merge(bot, 1, Integer::sum);
                longestStill.merge(bot, tMax - sUs, Math::max);
                stillStarts.computeIfAbsent(bot, k -> new ArrayList<>());
                List<String> starts = stillStarts.get(bot);
                if (starts.size() < 10) {
                    starts.add(String.format("%.1fs-%.1fs (log end) at %s",
                            sUs / 1e6, tMax / 1e6, fmtXY(stillStartXY.get(bot))));
                }
            }
        }
        for (Map.Entry<String, Long> e : stuckStart.entrySet()) {
            stuckRuns.merge(e.getKey(), 1, Integer::sum);
            longestStuck.merge(e.getKey(), tMax - e.getValue(), Math::max);
        }

        Map<String, String> meta = new LinkedHashMap<>();
        for (String m : rawMeta) {
            Matcher mm = META_PAIR.matcher(m);
            while (mm.find()) meta.put(mm.group(1), mm.group(2));
        }

        System.out.println("== " + args[0]);
        System.out.println("records=" + records
                + " span_s=" + String.format("%.1f", (tMax - tMin) / 1e6));
        System.out.println("-- metadata --");
        if (meta.isEmpty() && fileMeta.isEmpty()) {
            for (String m : rawMeta.subList(0, Math.min(10, rawMeta.size()))) {
                System.out.println("  raw: " + (m.length() > 160 ? m.substring(0, 160) : m));
            }
        } else {
            meta.forEach((k, v) -> System.out.println("  " + k + "=" + v));
            fileMeta.forEach((k, v) -> System.out.println("  " + k + "=" + v));
        }
        System.out.println("-- topic prefixes --");
        prefixCounts.forEach((k, v) -> System.out.println("  " + k + " n=" + v));
        System.out.println("-- Simulation/AI_Telemetry/RealOutputs/SmartDashboard/FieldSimulation topics --");
        if (detailTopics.isEmpty()) {
            System.out.println("  none present in this log");
        } else {
            detailTopics.forEach((k, v) -> System.out.println("  " + k + " n=" + v));
        }
        System.out.println("-- DS enabled/auto/test transitions --");
        if (dsChanges.isEmpty()) System.out.println("  none");
        dsChanges.forEach(c -> System.out.println("  " + c));
        System.out.println("-- stuck: identical-pose windows >= " + STILL_SEC
                + "s while enabled --");
        if (stillWindows.isEmpty()) System.out.println("  none");
        stillWindows.forEach((b, n) -> System.out.println("  " + b + " windows=" + n
                + " longest_s=" + String.format("%.1f", longestStill.get(b) / 1e6)
                + " at=" + stillStarts.getOrDefault(b, List.of())));
        System.out.println("-- IsStuck true runs --");
        if (stuckRuns.isEmpty()) System.out.println("  none");
        stuckRuns.forEach((b, n) -> System.out.println("  " + b + " runs=" + n
                + " longest_s=" + String.format("%.1f", longestStuck.get(b) / 1e6)));
        System.out.println("-- StateDetail transitions --");
        if (detailChanges.isEmpty()) System.out.println("  none recorded");
        detailChanges.forEach((b, ch) -> {
            System.out.println("  " + b + " last=" + lastDetail.get(b));
            ch.forEach(c -> System.out.println("    " + c));
        });
        System.out.println("-- last Score/Fuel/Archetype --");
        if (lastScalar.isEmpty()) System.out.println("  none");
        new TreeMap<>(lastScalar).forEach((k, v) -> System.out.println("  " + k + "=" + v));
        System.out.println("-- string records mentioning exception --");
        if (errorHits.isEmpty()) System.out.println("  none");
        errorHits.forEach(e -> System.out.println("  " + e));
    }

    private static String scalarToString(DataLogRecord r, String type) {
        try {
            if (type.equals("string")) return r.getString();
            if (type.equals("boolean")) return Boolean.toString(r.getBoolean());
            if (type.startsWith("int")) return Long.toString(r.getInteger());
            if (type.startsWith("double") || type.startsWith("float")) {
                return Double.toString(r.getDouble());
            }
        } catch (Exception ignored) {
        }
        return "?";
    }
}
