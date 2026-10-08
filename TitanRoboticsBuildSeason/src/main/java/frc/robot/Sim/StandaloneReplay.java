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
        long nowUs = (long) ((150.0 - timeRemainingSec) * 1_000_000L);

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
