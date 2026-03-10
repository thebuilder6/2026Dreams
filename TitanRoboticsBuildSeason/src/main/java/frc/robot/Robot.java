// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.AutoMissionExecutor;
import frc.robot.Auto.Missions.MissionBase;
import frc.robot.Sim.AIRobotSim;
import frc.robot.Sim.GameSim;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Dashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SubsystemManager;
import frc.robot.Subsystems.SwerveBase;
import frc.robot.Test.TestMode;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
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
    // Start data logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    swerveBase = SwerveBase.getInstance();
    Shooter.getInstance();
    Intake.getInstance();
    Climber.getInstance();
    Dashboard.getInstance();
    GameSim.getInstance();
    AIRobotSim.getInstance();
    LEDs.getInstance();
    teleop = new Teleop();
    testMode = TestMode.getInstance();
    SubsystemManager.initializeSubsystems();
    swerveBase.update();

    testController = new XboxController(0); // Assuming port 0 for testing

    // Disable LiveWindow to reduce NetworkTable noise
    edu.wpi.first.wpilibj.livewindow.LiveWindow.setEnabled(false);
    edu.wpi.first.wpilibj.livewindow.LiveWindow.disableAllTelemetry();
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
    
    // Update test mode if enabled
    if (testMode != null) {
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
    GameSim.getInstance().resetGame();
    AIRobotSim.getInstance().reset();
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
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Run the scheduler to execute any scheduled commands
    CommandScheduler.getInstance().run();
    
    // Test mode handles SysID integration internally
    if (testMode != null && testMode.isEnabled()) {
      // Test mode already handles SysID in its update loop
    }
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
