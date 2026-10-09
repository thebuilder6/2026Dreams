package frc.robot.Sim;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;

/**
 * AdvantageScope replay writer for standalone matches using native WPILib DataLogWriter.
 * Writes standard .wpilog files compatible with AdvantageScope 3D field views without
 * requiring LoggedRobot or HAL initialization.
 */
public final class StandaloneReplay {
    private static DataLogWriter writer;
    private static DoubleLogEntry timeRemainingEntry;
    private static DoubleLogEntry blueScoreEntry;
    private static DoubleLogEntry redScoreEntry;
    private static StructArrayLogEntry<Pose3d> fuelEntry;
    // Flush cadence. Array appends crash the JVM with an uncaught C++ exception
    // once ~3 MB sits unflushed (bisected Oct 08: Pose3d[108] x500 ticks crashes,
    // same x500 with a flush every 50 passes; wpiutil 2026.2.1). 100 ticks at the
    // standalone 50 Hz rate keeps the buffer far below that ceiling.
    private static final int FLUSH_EVERY_TICKS = 100;
    private static long tickCount;
    // Wall-clock base (us) + elapsed sim time. Data records must share an epoch
    // with the entry start/schema records (timestamp 0 = wall-clock "now"): the
    // old synthetic 0-based clock mixed wall-clock stamps (tick 0 lands on
    // timestamp 0 = now) with 0..150M us data, so AdvantageScope showed a
    // billion-second timeline. Elapsed derives from the observed remaining-time
    // drop, so any duration works (the old code also hardcoded 150 s).
    private static long timeBaseUs;
    private static double startRemainingSec;

    private static List<StructLogEntry<Pose2d>> botPoseEntries;
    private static List<DoubleLogEntry> botHeldEntries;
    private static List<DoubleLogEntry> botScoredEntries;
    private static List<StringLogEntry> botObjectiveEntries;

    private StandaloneReplay() {}

    /** Starts recording to {@code logPath}. Returns true if started. */
    public static synchronized boolean start(String logPath) {
        if (writer != null) {
            return false;
        }
        tickCount = 0;
        timeBaseUs = System.currentTimeMillis() * 1000L;
        startRemainingSec = Double.NaN;
        try {
            File f = new File(logPath);
            if (f.getParentFile() != null) {
                f.getParentFile().mkdirs();
            }
            writer = new DataLogWriter(logPath);
            timeRemainingEntry = new DoubleLogEntry(writer, "Standalone/TimeRemaining");
            blueScoreEntry = new DoubleLogEntry(writer, "Standalone/BlueScore");
            redScoreEntry = new DoubleLogEntry(writer, "Standalone/RedScore");
            fuelEntry = StructArrayLogEntry.create(writer, "Standalone/Fuel", Pose3d.struct);

            botPoseEntries = new ArrayList<>();
            botHeldEntries = new ArrayList<>();
            botScoredEntries = new ArrayList<>();
            botObjectiveEntries = new ArrayList<>();

            return true;
        } catch (IOException e) {
            System.err.println("[StandaloneReplay] Failed to start log writer: " + e.getMessage());
            writer = null;
            return false;
        }
    }

    /** Records one tick. */
    public static synchronized void recordTick(double timeRemainingSec, List<StandaloneBot> bots,
            FuelStore fuel, int blueScored, int redScored) {
        if (writer == null) {
            return;
        }
        if (Double.isNaN(startRemainingSec)) {
            startRemainingSec = timeRemainingSec;
        }
        double elapsedSec = Math.max(0.0, startRemainingSec - timeRemainingSec);
        long nowUs = timeBaseUs + (long) (elapsedSec * 1_000_000L);

        timeRemainingEntry.append(timeRemainingSec, nowUs);
        blueScoreEntry.append(blueScored, nowUs);
        redScoreEntry.append(redScored, nowUs);

        // Lazily initialize bot entries if needed
        while (botPoseEntries.size() < bots.size()) {
            int i = botPoseEntries.size();
            StandaloneBot b = bots.get(i);
            int slot = 0;
            for (int k = 0; k < i; k++) {
                if (bots.get(k).isRed() == b.isRed()) slot++;
            }
            String prefix = "Standalone/" + (b.isRed() ? "Red" : "Blue") + slot + "/";
            botPoseEntries.add(StructLogEntry.create(writer, prefix + "Pose", Pose2d.struct));
            botHeldEntries.add(new DoubleLogEntry(writer, prefix + "Held"));
            botScoredEntries.add(new DoubleLogEntry(writer, prefix + "Scored"));
            botObjectiveEntries.add(new StringLogEntry(writer, prefix + "Objective"));
        }

        for (int i = 0; i < bots.size(); i++) {
            StandaloneBot bot = bots.get(i);
            botPoseEntries.get(i).append(bot.getPose(), nowUs);
            botHeldEntries.get(i).append(bot.getHeld(), nowUs);
            botScoredEntries.get(i).append(bot.getScored(), nowUs);
            if (bot.getObjective() != null) {
                botObjectiveEntries.get(i).append(bot.getObjective().name(), nowUs);
            }
        }

        List<Translation2d> pieces = fuel.positions();
        Pose3d[] cloud = new Pose3d[pieces.size()];
        for (int i = 0; i < pieces.size(); i++) {
            Translation2d at = pieces.get(i);
            cloud[i] = new Pose3d(at.getX(), at.getY(), 0.075, new Rotation3d());
        }
        fuelEntry.append(cloud, nowUs);

        if (++tickCount % FLUSH_EVERY_TICKS == 0) {
            writer.flush();
        }
    }

    /** Flushes and closes the log. */
    public static synchronized void end() {
        if (writer == null) {
            return;
        }
        try {
            writer.flush();
            writer.close();
        } finally {
            writer = null;
            timeRemainingEntry = null;
            blueScoreEntry = null;
            redScoreEntry = null;
            fuelEntry = null;
            botPoseEntries = null;
            botHeldEntries = null;
            botScoredEntries = null;
            botObjectiveEntries = null;
        }
    }
}
