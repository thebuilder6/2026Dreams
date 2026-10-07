package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Auto.Missions.MissionBase;
import org.junit.jupiter.api.Test;

public class AutoMissionExecutorTest {

    private static class TestMission extends MissionBase {
        private volatile boolean ran = false;

        @Override
        protected void routine() throws AutoMissionEndedException {
            ran = true;
            while (isActiveWithThrow()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }

    @Test
    public void testAutoMissionExecutionAndStop() throws InterruptedException {
        AutoMissionExecutor executor = new AutoMissionExecutor();
        TestMission mission = new TestMission();

        executor.setAutoMission(mission);
        assertEquals(mission, executor.getAutoMission());
        assertFalse(executor.isStarted());

        executor.start();
        Thread.sleep(50);
        assertTrue(executor.isStarted());
        assertTrue(mission.ran);

        executor.interrupt();
        assertTrue(executor.isInterrupted());

        executor.resume();
        assertFalse(executor.isInterrupted());

        executor.stop();
        Thread.sleep(50);
        assertFalse(executor.isStarted());

        executor.reset();
        assertNull(executor.getAutoMission());
    }

    @Test
    public void testNullAutoMissionOperations() {
        AutoMissionExecutor executor = new AutoMissionExecutor();
        assertDoesNotThrow(() -> {
            executor.start();
            assertFalse(executor.isStarted());
            assertFalse(executor.isInterrupted());
            executor.interrupt();
            executor.resume();
            executor.stop();
            executor.reset();
        });
    }
}
