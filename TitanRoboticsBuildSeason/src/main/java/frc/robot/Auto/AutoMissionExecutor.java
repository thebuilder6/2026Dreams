package frc.robot.Auto;

import frc.robot.Auto.Missions.MissionBase;

/**
 * Class: AutoMissionExecutor
 * Description: This class selects, runs, and (if necessary) stops a specified autonomous Mission.
 * Author: Unknown
 */
public class AutoMissionExecutor {
    private MissionBase mAutoMission = null;
    private Thread mThread = null;

    public void setAutoMission(MissionBase newAutoMission) {
        mAutoMission = newAutoMission;
        mThread = new Thread(new Runnable() {
            
            @Override
            public void run() {
                if (mAutoMission != null) {
                    mAutoMission.run();
                }
            }
        });
    }

    public void start() {
        if (mThread != null) {
            mThread.start();
        }
    }

    public boolean isStarted() {
        return mAutoMission != null && mAutoMission.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        mAutoMission = null;
    }

    public void stop() {
        if (mAutoMission != null) {
            mAutoMission.stop();
        }

        mThread = null;
    }

    public MissionBase getAutoMission() {
        return mAutoMission;
    }

    public boolean isInterrupted() {
        return mAutoMission == null ? false :  mAutoMission.getIsInterrupted();
    }

    public void interrupt() {
        if (mAutoMission == null) {
            return;
        }
        mAutoMission.interrupt();
    }

    public void resume() {
        if (mAutoMission == null) {
            return;
        }
        mAutoMission.resume();
    }
}