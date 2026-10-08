package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.AutoMissionChooser;
import frc.robot.Interfaces.Actions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/*
    Class: MissionBase
    Description: An abstract class that is the basis of the robot's autonomous routines. 
                 This is implemented in auto missions (which are routines that do actions).
    Author: Unknown
 */

public abstract class MissionBase {
    
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;

    protected abstract void routine() throws AutoMissionEndedException;

    public void setStartPose() {
        
    }

    public void run() {
        mActive = true;

        try {
            if (AutoMissionChooser.getDelay() > 0.05) {
                runAction(new frc.robot.Auto.Actions.WaitAction(AutoMissionChooser.getDelay()));
            }
            routine();
            done();
        } 
        catch (AutoMissionEndedException e) {
            DriverStation.reportError("AUTO MISSION DONE!!!! ENDED EARLY!!!!", false);
        } finally {
            mActive = false;
        }
    }

    public void done() {
        System.out.println("Auto mission done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoMissionEndedException {
        if (!isActive()) {
            throw new AutoMissionEndedException();
        }

        return isActive();
    }

    public void interrupt() {
        System.out.println("** Auto mission interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mission resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Actions action) throws AutoMissionEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);
        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } 
            catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        action.start();

        double nextLoopTimestamp = Timer.getFPGATimestamp();

        // Run action paced to exact 50.0 Hz using FPGA monotonic microsecond clock
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();
            nextLoopTimestamp += mUpdateRate;

            double now = Timer.getFPGATimestamp();
            double sleepSeconds = nextLoopTimestamp - now;
            if (sleepSeconds > 0.001) {
                try {
                    Thread.sleep((long) (sleepSeconds * 1000.0));
                } 
                catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            } else if (sleepSeconds < -0.050) {
                // If loop overran by >50ms, resync anchor to prevent burst catchup
                nextLoopTimestamp = now;
            }
        }

        action.done();
        isActiveWithThrow();
    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}