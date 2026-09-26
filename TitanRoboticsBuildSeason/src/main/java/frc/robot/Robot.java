// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.HMI.Alerts.Alert;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.Data.Constants;
import frc.robot.Auto.AutoMissionExecutor;
import frc.robot.Auto.Missions.MissionBase;
import frc.robot.Intelligence.Sparring.AIRobotSim;
import frc.robot.Sim.GameSim;
import frc.robot.Telemetry.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.HMI.LEDs;
import frc.robot.Intelligence.MatchCoach;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Vision;
import frc.robot.Test.TestMode;
import frc.robot.HMI.Alerts.AlertManager;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private String m_autoSelected;

  Teleop teleop;
  SwerveBase swerveBase;
  TestMode testMode;
  XboxController testController;
  private AutoMissionExecutor mAutoMissionExecutor = new AutoMissionExecutor();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // AdvantageKit Logger Configuration for AdvantageScope & Deterministic Replay
    Logger.recordMetadata("ProjectName", BuildConstants.ROBOT_NAME);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitDirty", BuildConstants.DIRTY == 1 ? "true" : "false");
    switch (Constants.getMode()) {
        case REAL:
            Logger.addDataReceiver(new WPILOGWriter()); // USB stick "/U/logs" or "/home/lvuser/logs"
            Logger.addDataReceiver(new NT4Publisher());
            break;
        case SIM:
            Logger.addDataReceiver(new NT4Publisher());
            break;
        case REPLAY:
            setUseTiming(false); // Run cycles as fast as possible during replay
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
    }
    Logger.start();

    swerveBase = SwerveBase.getInstance();
    Vision.getInstance();
    Shooter.getInstance();
    Intake.getInstance();
    Dashboard.getInstance();
    if (isSimulation()) {
        GameSim.getInstance();
        AIRobotSim.getInstance();
    }
    MatchCoach.getInstance();
    LEDs.getInstance();
    teleop = new Teleop();
    testMode = TestMode.getInstance();
    SubsystemManager.initializeSubsystems();
    swerveBase.update();

    testController = new XboxController(0); // Assuming port 0 for testing

    // Silence joystick unplugged warnings to prevent console I/O stalls
    DriverStation.silenceJoystickConnectionWarning(true);

    // Disable LiveWindow to reduce NetworkTable noise
    edu.wpi.first.wpilibj.livewindow.LiveWindow.setEnabled(false);
    edu.wpi.first.wpilibj.livewindow.LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotInit() {
    // Start WPILib WebServer to serve elastic-layout.json for Elastic Dashboard remote loading (Ctrl+D)
    try {
      edu.wpi.first.net.WebServer.start(5800, edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().getPath());
    } catch (Throwable t) {
      System.out.println("[WebServer] Notice: Elastic layout WebServer on port 5800 could not be started: " + t.getMessage());
    }

    // The PortForwarder Trick: Forward coprocessor web interfaces and camera streams over USB tether (172.22.11.2)
    try {
      edu.wpi.first.net.PortForwarder.add(5801, "limelight-front.local", 5800);      // Limelight Web Dashboard
      edu.wpi.first.net.PortForwarder.add(5802, "limelight-front.local", 5802);      // Limelight Camera Stream
      edu.wpi.first.net.PortForwarder.add(5803, "rubik-pi-coprocessor.local", 5800); // PhotonVision Web Dashboard
      edu.wpi.first.net.PortForwarder.add(5804, "rubik-pi-coprocessor.local", 1181); // PhotonVision Primary Stream
      edu.wpi.first.net.PortForwarder.add(5805, "rubik-pi-coprocessor.local", 1182); // PhotonVision Secondary Stream
      System.out.println("[PortForwarder] Coprocessor ports 5801-5805 successfully forwarded.");
    } catch (Throwable t) {
      System.out.println("[PortForwarder] Notice: Port forwarding setup encountered: " + t.getMessage());
    }

    // Fast 100Hz odometry polling scheduled with a 5ms timeslot offset from the 20ms main loop
    addPeriodic(() -> {
      if (swerveBase != null) {
        swerveBase.updateOdometryFast();
      }
    }, 0.010, 0.005);
  }

  private final java.util.List<Notifier> subLoopNotifiers = new java.util.ArrayList<>();

  /**
   * Timeslot sub-loop scheduling: runs callbacks at higher rates (e.g. 100 Hz / 10ms)
   * with an initial timeslot offset (e.g. 5ms) to interleave cleanly with the main 20ms loop.
   *
   * @param callback Runnable to execute
   * @param periodSeconds Interval between calls (e.g. 0.010 for 100 Hz)
   * @param offsetSeconds Initial timeslot offset (e.g. 0.005 for 5ms offset)
   */
  public void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
    Notifier periodicNotifier = new Notifier(callback);
    periodicNotifier.setName("SubLoop-" + (int) (1.0 / periodSeconds) + "Hz");
    subLoopNotifiers.add(periodicNotifier);

    if (offsetSeconds > 0) {
      Notifier offsetNotifier = new Notifier(() -> {
        periodicNotifier.startPeriodic(periodSeconds);
      });
      offsetNotifier.startSingle(offsetSeconds);
      subLoopNotifiers.add(offsetNotifier);
    } else {
      periodicNotifier.startPeriodic(periodSeconds);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SubsystemManager.updateSubsystems();
    SubsystemManager.logSubsystems();
    AlertManager.update();
    
    // Only update test mode when in test mode or when test mode switch is explicitly active
    if (testMode != null && (DriverStation.isTest() || testMode.isEnabled())) {
      testMode.update();
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    if (isSimulation()) {
      GameSim.getInstance().resetGame();
      AIRobotSim.getInstance().reset();
    }
    m_autoSelected = Dashboard.getInstance().getAutoChooser().getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    mAutoMissionExecutor.stop();
    mAutoMissionExecutor.reset();

    MissionBase mission = Dashboard.getInstance().getAutoChooser().getAutoMissionForParams(m_autoSelected).orElse(null);

    if (mission != null) {
      mAutoMissionExecutor.setAutoMission(mission);
      mAutoMissionExecutor.start();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    teleop.init();
    if (isSimulation()) {
      // Sim acts as FMS: seed the SHIFT 1 hub order from the AUTO fuel result
      // (most AUTO fuel -> own hub inactive first; tie -> random per 6.4.1).
      char seed = frc.robot.Sim.HubSchedule.seedFromAutoResult();
      frc.robot.Sim.HubSchedule.setShiftSeed(seed);
      Dashboard.getInstance().setGameData(String.valueOf(seed));
      try {
        edu.wpi.first.wpilibj.simulation.DriverStationSim
            .setGameSpecificMessage(String.valueOf(seed));
      } catch (Exception ignored) {
      }
      System.out.println("[HubSchedule] SHIFT 1 seed from AUTO: '" + seed + "' inactive first");
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    teleop.teleopPeriodic();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mAutoMissionExecutor.stop();
    teleop.reset();
    swerveBase.stop();
    Shooter.getInstance().stop();
    Intake.getInstance().stop();
    swerveBase.setMotorBrake(true);
    if (testMode != null) {
      testMode.cleanup();
    }
    // Proactive GC flush: sweep heap while disabled to prevent mid-match GC pauses
    System.gc();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    if (testMode != null) {
      testMode.setEnabled(true);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // MapleSim's SimulatedBattery is a single STATIC battery shared by every registered
    // drivetrain (player + up to 3 opponents + 2 allies = ~48 motor sims on one 13.5V model).
    // That sags below brownout voltage and spams DriverStation.reportError every sub-tick
    // ("[MapleSim] BrownOut Detected..."), and the sagged voltage also feeds our own
    // SwerveBase brownout throttle. The library's own escape hatch locks voltage to nominal;
    // our Robot.simulationPeriodic BatterySim model remains authoritative for RoboRIO voltage.
    try {
      if (isSimulation()) {
        swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery.disableBatterySim();
      }
    } catch (Throwable t) {
      System.out.println("[SimulatedBattery] Notice: could not disable MapleSim battery sim: " + t.getMessage());
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SubsystemManager.simulationUpdateSubsystems();
    swervelib.simulation.ironmaple.simulation.SimulatedArena.getInstance().simulationPeriodic();

    // Calculate total current draw
    double totalCurrentDraw = 0.0;
    for (frc.robot.Interfaces.Subsystem subsystem : SubsystemManager.getSubsystems()) {
      totalCurrentDraw += subsystem.getSimulationCurrentDraw();
    }

    // Set the simulated battery voltage based on current draw
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw);
    RoboRioSim.setVInVoltage(loadedVoltage);
  }

  @Override
  public void close() {
    for (Notifier notifier : subLoopNotifiers) {
      try {
        notifier.stop();
        notifier.close();
      } catch (Throwable ignored) {
      }
    }
    subLoopNotifiers.clear();
    super.close();
  }
}
