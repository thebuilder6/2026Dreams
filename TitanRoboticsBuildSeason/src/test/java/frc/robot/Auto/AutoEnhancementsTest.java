package frc.robot.Auto;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.Auto.Actions.*;
import frc.robot.Interfaces.Actions;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveBase;

public class AutoEnhancementsTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    private static class DummyAction implements Actions {
        private boolean started = false;
        private boolean finished = false;
        private boolean doneCalled = false;
        private final Set<Class<?>> requirements;

        public DummyAction(Class<?>... reqs) {
            this.requirements = Set.of(reqs);
        }

        @Override
        public void start() {
            started = true;
        }

        @Override
        public void update() {
            finished = true;
        }

        @Override
        public boolean isFinished() {
            return finished;
        }

        @Override
        public void done() {
            doneCalled = true;
        }

        @Override
        public Set<Class<?>> getRequirements() {
            return requirements;
        }
    }

    @Test
    public void testBranchActionTrueBranch() {
        DummyAction trueAction = new DummyAction(Intake.class);
        DummyAction falseAction = new DummyAction(Shooter.class);

        BranchAction branch = new BranchAction(() -> true, trueAction, falseAction);
        assertEquals(Set.of(Intake.class, Shooter.class), branch.getRequirements());

        assertFalse(branch.isFinished());
        branch.start();
        assertTrue(trueAction.started);
        assertFalse(falseAction.started);

        branch.update();
        assertTrue(branch.isFinished());

        branch.done();
        assertTrue(trueAction.doneCalled);
        assertFalse(falseAction.doneCalled);
    }

    @Test
    public void testBranchActionFalseBranch() {
        DummyAction trueAction = new DummyAction(Intake.class);
        DummyAction falseAction = new DummyAction(Shooter.class);

        BranchAction branch = new BranchAction(() -> false, trueAction, falseAction);
        branch.start();
        assertFalse(trueAction.started);
        assertTrue(falseAction.started);

        branch.update();
        assertTrue(branch.isFinished());

        branch.done();
        assertFalse(trueAction.doneCalled);
        assertTrue(falseAction.doneCalled);
    }

    @Test
    public void testWaitForBallActionTimeout() {
        WaitForBallAction waitAction = new WaitForBallAction(0.05);
        assertEquals(Set.of(Intake.class), waitAction.getRequirements());

        waitAction.start();
        // Initially should not be finished unless simulated hopper already has balls
        try {
            Thread.sleep(60);
        } catch (InterruptedException ignored) {}

        // After 60ms (> 50ms timeout), it must be finished
        assertTrue(waitAction.isFinished());
        waitAction.done();
    }

    @Test
    public void testParallelActionResourceUnionAndConflict() {
        DummyAction actionA = new DummyAction(SwerveBase.class);
        DummyAction actionB = new DummyAction(Intake.class);
        ParallelAction parallelSafe = new ParallelAction(actionA, actionB);
        assertEquals(Set.of(SwerveBase.class, Intake.class), parallelSafe.getRequirements());

        // Conflict case: two concurrent actions claiming SwerveBase
        DummyAction actionC = new DummyAction(SwerveBase.class);
        assertDoesNotThrow(() -> {
            ParallelAction parallelConflict = new ParallelAction(actionA, actionC);
            assertEquals(Set.of(SwerveBase.class), parallelConflict.getRequirements());
        });
    }

    @Test
    public void testParallelRaceActionResourceUnion() {
        DummyAction actionA = new DummyAction(SwerveBase.class);
        DummyAction actionB = new DummyAction(Shooter.class);
        ParallelRaceAction race = new ParallelRaceAction(actionA, actionB);
        assertEquals(Set.of(SwerveBase.class, Shooter.class), race.getRequirements());
    }

    @Test
    public void testSeriesActionResourceUnion() {
        DummyAction actionA = new DummyAction(SwerveBase.class);
        DummyAction actionB = new DummyAction(Intake.class);
        SeriesAction series = new SeriesAction(actionA, actionB);
        assertEquals(Set.of(SwerveBase.class, Intake.class), series.getRequirements());
    }

    @Test
    public void testFollowChoreoPathCacheLifecycle() {
        FollowChoreoPath.clearCache();
        // Warming non-existent trajectory should cache an empty optional cleanly
        FollowChoreoPath.warmCache("NonExistentTraj");
        var opt = FollowChoreoPath.getTrajectory("NonExistentTraj");
        assertNotNull(opt);
        assertTrue(opt.isEmpty());

        FollowChoreoPath path = new FollowChoreoPath("NonExistentTraj", false);
        assertEquals(Set.of(SwerveBase.class), path.getRequirements());
        assertTrue(path.isFinished());
        assertTrue(path.getMarkerPose("FakeMarker").isEmpty());
        assertEquals(Double.MAX_VALUE, path.getDistanceToMarker("FakeMarker"));
        assertFalse(path.isWithinMarkerDistance("FakeMarker", 1.0));
    }

    @Test
    public void testWaitUntilMarkerActionWithCustomTolerance() {
        FollowChoreoPath path = new FollowChoreoPath("NonExistentTraj", false);
        WaitUntilMarkerAction markerAction = new WaitUntilMarkerAction(path, "TestMarker", 0.30);
        // Path is finished (empty), so WaitUntilMarkerAction must finish safely without throwing
        assertTrue(markerAction.isFinished());
    }

    @Test
    public void testWaitActionNullSafetyAndExecution() {
        WaitAction waitAction = new WaitAction(0.05);

        // Pre-start: must not throw NullPointerException
        assertDoesNotThrow(() -> assertFalse(waitAction.isFinished()));
        assertDoesNotThrow(waitAction::done);

        // Lifecycle execution
        waitAction.start();
        try {
            Thread.sleep(60);
        } catch (InterruptedException ignored) {}

        assertTrue(waitAction.isFinished());
        assertDoesNotThrow(waitAction::done);
    }

    @Test
    public void testIntakeActionNullSafetyAndExecution() {
        IntakeAction intakeAction = new IntakeAction(0.05, Intake.IntakeState.INTAKING);
        assertEquals(Set.of(Intake.class), intakeAction.getRequirements());
        assertEquals(Intake.IntakeState.INTAKING, intakeAction.intakeState);

        // Pre-start: must not throw NullPointerException
        assertDoesNotThrow(() -> assertFalse(intakeAction.isFinished()));
        assertDoesNotThrow(intakeAction::done);

        // Lifecycle execution
        intakeAction.start();
        intakeAction.update();
        try {
            Thread.sleep(60);
        } catch (InterruptedException ignored) {}

        assertTrue(intakeAction.isFinished());
        assertDoesNotThrow(intakeAction::done);
    }
}
