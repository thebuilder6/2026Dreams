package frc.robot.Telemetry;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

/**
 * Startup janitor for on-Rio log storage.
 *
 * <p>On 2026-10-07 a match ran with {@code /home/lvuser/logs} below 50 MB free:
 * AdvantageKit failed to open {@code /U/logs/*.wpilog} (no USB stick) and the
 * REV logger deleted its own log in a loop. This deletes oldest-first
 * {@code *.wpilog}/{@code *.revlog} files until a free-space floor is met,
 * before {@code Logger.start()} opens its output file. Never throws and never
 * touches non-log files. No-op off the Rio (dirs do not exist).
 */
public final class LogStorage {

    /** Free-space floor per log dir; matches the REV 50 MB warning with headroom. */
    public static final long MIN_FREE_BYTES = 200L * 1024L * 1024L;

    private static final String[] MANAGED_SUFFIXES = {".wpilog", ".revlog"};

    private LogStorage() {
    }

    /**
     * Frees log space in both Rio log dirs. Safe to call on any platform.
     *
     * @return total files deleted across both dirs.
     */
    public static int ensureLogSpace() {
        int deleted = 0;
        deleted += cleanDirectory(new File("/U/logs"), MIN_FREE_BYTES);
        deleted += cleanDirectory(new File("/home/lvuser/logs"), MIN_FREE_BYTES);
        return deleted;
    }

    /**
     * Deletes oldest-first managed log files in {@code dir} until {@code
     * minFreeBytes} is usable or no managed files remain.
     *
     * @return files deleted.
     */
    static int cleanDirectory(File dir, long minFreeBytes) {
        try {
            if (dir == null || !dir.isDirectory()) {
                return 0;
            }
            if (dir.getUsableSpace() >= minFreeBytes) {
                return 0;
            }
            File[] logs = dir.listFiles((d, name) -> {
                String lower = name.toLowerCase(java.util.Locale.ROOT);
                for (String suffix : MANAGED_SUFFIXES) {
                    if (lower.endsWith(suffix)) {
                        return true;
                    }
                }
                return false;
            });
            if (logs == null || logs.length == 0) {
                return 0;
            }
            Arrays.sort(logs, Comparator.comparingLong(File::lastModified));
            int deleted = deleteUntilFree(dir, java.util.Arrays.asList(logs), minFreeBytes);
            System.out.println("[LogStorage] " + dir + ": deleted " + deleted
                    + " old log(s), " + (dir.getUsableSpace() / 1024 / 1024) + " MB free");
            return deleted;
        } catch (Throwable t) {
            System.out.println("[LogStorage] Notice: cleanup of " + dir + " unavailable: " + t.getMessage());
            return 0;
        }
    }

    /**
     * Deletes oldest-first from {@code oldestFirst} until {@code dir} meets
     * {@code minFreeBytes} or the list is exhausted.
     *
     * @return files deleted.
     */
    static int deleteUntilFree(File dir, java.util.List<File> oldestFirst, long minFreeBytes) {
        int deleted = 0;
        for (File log : oldestFirst) {
            if (dir.getUsableSpace() >= minFreeBytes) {
                break;
            }
            if (log.delete()) {
                deleted++;
            }
        }
        return deleted;
    }
}
