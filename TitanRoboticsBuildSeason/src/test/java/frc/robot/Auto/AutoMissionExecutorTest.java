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

    private static class ActionMission extends MissionBase {
        private volatile boolean actionStarted = false;
        private volatile boolean actionDone = false;

        @Override
        protected void routine() throws AutoMissionEndedException {
            runAction(new frc.robot.Interfaces.Actions() {
                @Override public void start() { actionStarted = true; }
                @Override public void update() {}
                @Override public boolean isFinished() { return false; }
                @Override public void done() { actionDone = true; }
            });
        }
    }

    @Test
    public void testMissionActionTerminationOnStop() throws InterruptedException {
        AutoMissionExecutor executor = new AutoMissionExecutor();
        ActionMission mission = new ActionMission();

        executor.setAutoMission(mission);
        executor.start();
        Thread.sleep(50);
        assertTrue(mission.actionStarted);
        assertFalse(mission.actionDone);

        executor.stop();
        Thread.sleep(50);
        assertTrue(mission.actionDone, "action.done() must be invoked on stop");
        assertFalse(executor.isStarted());
    }

    private static class QuickMission extends MissionBase {
        private volatile boolean doneCalled = false;

        @Override
        protected void routine() throws AutoMissionEndedException {
            // Completes immediately
        }

        @Override
        public void done() {
            super.done();
            doneCalled = true;
        }
    }

    @Test
    public void testMissionNaturalCompletionClearsActive() throws InterruptedException {
        AutoMissionExecutor executor = new AutoMissionExecutor();
        QuickMission mission = new QuickMission();

        executor.setAutoMission(mission);
        executor.start();
        Thread.sleep(50);

        assertTrue(mission.doneCalled, "done() must be called when routine finishes naturally");
        assertFalse(mission.isActive(), "isActive() must be false after routine completes");
        assertFalse(executor.isStarted(), "executor.isStarted() must be false after routine completes");
    }
}
