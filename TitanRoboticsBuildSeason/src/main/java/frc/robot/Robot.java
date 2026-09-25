// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Sim.AIRobotSim;
import frc.robot.Sim.GameSim;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.MatchCoach;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Subsystems.Vision;
import frc.robot.Test.TestMode;
import frc.robot.Utils.AlertManager;

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
    Logger.recordMetadata("ProjectName", "TitanRobotics2026");
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
}
