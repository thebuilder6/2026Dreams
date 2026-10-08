package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class LogStorageTest {

    @TempDir
    Path tempDir;

    private static File touch(File dir, String name, long lastModified) throws Exception {
        File f = new File(dir, name);
        Files.write(f.toPath(), new byte[] {1, 2, 3});
        assertTrue(f.setLastModified(lastModified));
        return f;
    }

    @Test
    public void testMissingDirIsNoOp() {
        assertEquals(0, LogStorage.cleanDirectory(new File(tempDir.toFile(), "nope"), 1024));
    }

    @Test
    public void testNullDirIsNoOp() {
        assertEquals(0, LogStorage.cleanDirectory(null, 1024));
    }

    @Test
    public void testSufficientSpaceDeletesNothing() throws Exception {
        File dir = tempDir.toFile();
        File kept = touch(dir, "akit_old.wpilog", 1000L);
        assertEquals(0, LogStorage.cleanDirectory(dir, 1L));
        assertTrue(kept.exists());
    }

    @Test
    public void testFullPurgeDeletesOldestFirstAndPreservesNonLogs() throws Exception {
        File dir = tempDir.toFile();
        File oldest = touch(dir, "old.wpilog", 1000L);
        File middle = touch(dir, "mid.revlog", 2000L);
        File newest = touch(dir, "new.wpilog", 3000L);
        File notes = touch(dir, "keep.txt", 500L);

        // Impossible floor forces the loop to drain every managed file.
        int deleted = LogStorage.cleanDirectory(dir, Long.MAX_VALUE);

        assertEquals(3, deleted);
        assertFalse(oldest.exists());
        assertFalse(middle.exists());
        assertFalse(newest.exists());
        assertTrue(notes.exists(), "non-log files must never be deleted");
    }

    @Test
    public void testDeleteUntilFreeStopsAtFloor() throws Exception {
        File dir = tempDir.toFile();
        File a = touch(dir, "a.wpilog", 1000L);
        File b = touch(dir, "b.wpilog", 2000L);
        List<File> oldestFirst = Arrays.asList(a, b);

        // Floor of 0 is already met: nothing deleted.
        assertEquals(0, LogStorage.deleteUntilFree(dir, oldestFirst, 0L));
        assertTrue(a.exists());
        assertTrue(b.exists());
    }

    @Test
    public void testEnsureLogSpaceNeverThrows() {
        assertDoesNotThrow(LogStorage::ensureLogSpace);
    }
}
