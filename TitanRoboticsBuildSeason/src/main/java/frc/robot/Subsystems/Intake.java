package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants;
import frc.robot.Data.Constants.IntakeConstants;
import frc.robot.Interfaces.Subsystem;
import frc.robot.Subsystems.intake.IntakeIO;
import frc.robot.Subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.Subsystems.intake.IntakeIOSim;
import frc.robot.Subsystems.intake.IntakeIOSparkMax;
import frc.robot.Utils.Alert;
import frc.robot.Utils.Alert.AlertType;

/*
 * Class: Intake
 * Description: Ground intake mechanism with articulated pivot arm, feedforward gravity compensation,
 *              closed-loop continuous profiled PID, jam detection, and 3D simulation.
 * Authors: Mai, InfiniteQuery
 */
public class Intake implements Subsystem {

    private static Intake instance = null;

    // IO Abstraction (AdvantageKit pattern)
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    public enum IntakeState {
        DISABLED,
        STANDBY,
        STANDBY_INTAKING,
        STANDBY_REVERSED,
        INTAKING,
        DOWN,
        REVERSED,
        MANUAL,
        IDLE,
        FEEDING,
        EJECTING,
        CHARACTERIZATION
    }

    private String stateStr = "Disabled";
    private double power = -0.5;
    private double armVoltage = 0.0;

    private final ProfiledPIDController pivotProfiledPIDController;
    private final ArmFeedforward feedforward;

    private double currentPosition = 128.0;
    private double unmodifiedAbsolutePosition = 0.0;
    private double upPosition = Constants.INTAKE_UP_POSITION;
    private double downPosition = Constants.INTAKE_DOWN_POSITION;
    private double goal = Constants.INTAKE_UP_POSITION;
    private double manualPosition;

    // Jam detection & Alerts
    private final Timer stallTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean isEjectingJam = false;
    private final Alert intakeEncoderAlert = new Alert("Intake", "Absolute Encoder Disconnected: Fallback Active", AlertType.ERROR);
    private final Alert intakeJamAlert = new Alert("Intake", "Roller Jam Detected: Auto-Clearing", AlertType.WARNING);
    private double lastValidPosition = Constants.INTAKE_UP_POSITION;
    private double lastMotorRotations = 0.0;

    // Simulation
    private frc.robot.Sim.ArmSim armSim;

    public static Intake getInstance() {
        if (instance == null) {
            IntakeIO io = RobotBase.isSimulation() ? new IntakeIOSim() : new IntakeIOSparkMax();
            instance = new Intake(io);
        }
        return instance;
    }

    public Intake(IntakeIO io) {
        this.io = io;

        feedforward = new ArmFeedforward(
                Constants.INTAKE_ARM_KS,
                Constants.INTAKE_ARM_KG,
                Constants.INTAKE_ARM_KV,
                Constants.INTAKE_ARM_KA);

        pivotProfiledPIDController = new ProfiledPIDController(
                Constants.INTAKE_ARM_KP,
                Constants.INTAKE_ARM_KI,
                Constants.INTAKE_ARM_KD,
                new TrapezoidProfile.Constraints(Constants.MAX_ARM_VELOCITY, Constants.MAX_ARM_ACCELERATION));

        // Tell the PID controller that 0 and 360 are continuous
        pivotProfiledPIDController.enableContinuousInput(0, 360);

        if (RobotBase.isSimulation() && io instanceof IntakeIOSim simIO) {
            armSim = simIO.getArmSim();
        } else if (RobotBase.isSimulation()) {
            armSim = new frc.robot.Sim.ArmSim();
        }

        SubsystemManager.registerSubsystem(this);
    }

    private void armControlFunction() {
        pivotProfiledPIDController.setGoal(goal);

        double pidOutput = pivotProfiledPIDController.calculate(currentPosition);
        double targetVelocityRadians = Math.toRadians(pivotProfiledPIDController.getSetpoint().velocity);

        double ffOutput = feedforward.calculate(
                Math.toRadians(currentPosition - Constants.INTAKE_HORIZONTAL_POSITION),
                targetVelocityRadians);

        armVoltage = pidOutput + ffOutput;
        io.setArmVoltage(armVoltage);
    }

    public void setState(String state) {
        if (this.stateStr.equals("Disabled") && !state.equals("Disabled")) {
            pivotProfiledPIDController.reset(currentPosition);
        }
        this.stateStr = state;
    }

    public void setState(IntakeState state) {
        switch (state) {
            case INTAKING:
                setState("Intaking");
                break;
            case DOWN:
                setState("Down");
                break;
            case REVERSED:
            case EJECTING:
                setState("Reversed");
                break;
            case STANDBY_INTAKING:
                setState("StandbyIntaking");
                break;
            case STANDBY_REVERSED:
                setState("StandbyReversed");
                break;
            case DISABLED:
                setState("Disabled");
                break;
            case CHARACTERIZATION:
                setState("Characterization");
                break;
            case STANDBY:
            case IDLE:
            default:
                setState("Standby");
                break;
        }
    }

    public String getStateString() {
        return stateStr;
    }

    public IntakeState getState() {
        switch (stateStr) {
            case "Intaking":
                return IntakeState.INTAKING;
            case "Down":
                return IntakeState.DOWN;
            case "Reversed":
                return IntakeState.REVERSED;
            case "StandbyIntaking":
                return IntakeState.STANDBY_INTAKING;
            case "StandbyReversed":
                return IntakeState.STANDBY_REVERSED;
            case "Disabled":
                return IntakeState.DISABLED;
            case "Manual":
                return IntakeState.MANUAL;
            case "Characterization":
                return IntakeState.CHARACTERIZATION;
            case "Standby":
            default:
                return IntakeState.STANDBY;
        }
    }

    public void manualIntakeControl(double manualInput) {
        setState("Manual");
        double normalizedInput = (manualInput + 1.0) / 2.0;
        double modifiedManualPosition = Constants.INTAKE_DOWN_POSITION
                + (normalizedInput * (Constants.INTAKE_UP_POSITION - Constants.INTAKE_DOWN_POSITION));
        this.manualPosition = MathUtil.clamp(modifiedManualPosition,
                Math.min(Constants.INTAKE_DOWN_POSITION, Constants.INTAKE_UP_POSITION),
                Math.max(Constants.INTAKE_DOWN_POSITION, Constants.INTAKE_UP_POSITION));
    }

    public void setArmPosition(double positionDeg) {
        goal = positionDeg;
        stateStr = "Manual";
        manualPosition = positionDeg;
    }

    public double getArmPosition() {
        return currentPosition;
    }

    public void setArmVoltage(double volts) {
        stateStr = "Characterization";
        // Mechanical angle safety boundaries:
        // Physical travel is roughly 245 deg (down) to 350 deg (up).
        // If arm position is at or beyond boundary, clamp voltage driving into mechanical stop.
        if (currentPosition <= 242.0 && volts < 0.0) {
            volts = 0.0;
        } else if (currentPosition >= 353.0 && volts > 0.0) {
            volts = 0.0;
        }
        armVoltage = volts;
        io.setArmVoltage(volts);
    }

    public void setCharacterizationVoltage(double armVolts) {
        setArmVoltage(armVolts);
    }

    public double getArmAppliedVoltage() {
        return armVoltage;
    }

    public double getArmPositionRads() {
        return Math.toRadians(currentPosition);
    }

    public double getArmVelocityRads() {
        return Math.toRadians(inputs.armVelocityDegPerSec);
    }

    public void runRollers(double speed) {
        io.setRollerSpeed(speed);
    }

    public void setRollerVoltage(double volts) {
        io.setRollerVoltage(volts);
    }

    public double getRollerVelocityRPM() {
        return inputs.rollerVelocityRPM;
    }

    public void runHopper(double speed) {
        io.setHopperSpeed(speed);
    }

    public void setHopperVoltage(double volts) {
        io.setHopperVoltage(volts);
    }

    public double getHopperVelocityRPM() {
        return inputs.hopperVelocityRPM;
    }

    public double getArmCurrentAmps() {
        return inputs.armCurrentAmps;
    }

    public double getRollerCurrentAmps() {
        return inputs.rollerCurrentAmps;
    }

    public void stop() {
        setState("Disabled");
        armVoltage = 0.0;
        io.stop();
    }

    public boolean isAtTargetPosition() {
        return Math.abs(currentPosition - goal) < 5.0;
    }

    public boolean isJammed() {
        return isEjectingJam;
    }

    public double getCurrentDraw() {
        return Math.abs(inputs.armCurrentAmps) + Math.abs(inputs.rollerCurrentAmps) + Math.abs(inputs.hopperCurrentAmps);
    }

    public IntakeIO getIO() {
        return io;
    }

    public IntakeIOInputs getInputs() {
        return inputs;
    }

    @Override
    public void update() {
        io.updateInputs(inputs);

        boolean encoderHealthy = RobotBase.isSimulation() || inputs.encoderConnected;
        intakeEncoderAlert.set(!encoderHealthy);

        if (encoderHealthy) {
            currentPosition = inputs.armPositionDeg;
            lastValidPosition = currentPosition;
            lastMotorRotations = inputs.armMotorRotations;
        } else {
            // Graceful degradation: Track delta rotations from SparkMax internal relative encoder
            double deltaRotations = inputs.armMotorRotations - lastMotorRotations;
            currentPosition = MathUtil.inputModulus(lastValidPosition + (deltaRotations * 3.6), 0, 360);
        }

        // Automated Jam Detection and Ejection
        if (!RobotBase.isSimulation() && Math.abs(inputs.rollerAppliedVolts) > 1.0 
                && inputs.rollerCurrentAmps > IntakeConstants.STALL_CURRENT_LIMIT) {
            stallTimer.start();
            if (stallTimer.hasElapsed(IntakeConstants.STALL_TIME)) {
                isEjectingJam = true;
                ejectTimer.restart();
                stallTimer.reset();
                stallTimer.stop();
            }
        } else {
            stallTimer.reset();
            stallTimer.stop();
        }

        if (isEjectingJam) {
            intakeJamAlert.set(true);
            if (ejectTimer.hasElapsed(IntakeConstants.EJECT_TIME)) {
                isEjectingJam = false;
                ejectTimer.stop();
                ejectTimer.reset();
                intakeJamAlert.set(false);
            }
        } else {
            intakeJamAlert.set(false);
        }

        switch (stateStr) {
            case "Standby":
                goal = upPosition;
                armControlFunction();
                io.setRollerSpeed(0);
                io.setHopperSpeed(0);
                break;

            case "StandbyIntaking":
                goal = upPosition;
                armControlFunction();
                io.setRollerSpeed(power);
                io.setHopperSpeed(Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case "StandbyReversed":
                goal = upPosition;
                armControlFunction();
                io.setRollerSpeed(-power);
                io.setHopperSpeed(-Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case "Intaking":
                goal = downPosition;
                armControlFunction();
                io.setRollerSpeed(isEjectingJam ? -power : power);
                io.setHopperSpeed(Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case "Down":
                goal = downPosition;
                armControlFunction();
                io.setRollerSpeed(0);
                io.setHopperSpeed(0);
                break;

            case "Reversed":
                goal = downPosition;
                armControlFunction();
                io.setRollerSpeed(-power);
                io.setHopperSpeed(-Constants.IntakeConstants.HOPPER_SPEED);
                break;

            case "Manual":
                goal = manualPosition;
                armControlFunction();
                break;

            case "Characterization":
                // Voltages controlled directly by characterization/diagnostic routines
                break;

            case "Disabled":
            default:
                io.stop();
                break;
        }
    }

    @Override
    public void simulationUpdate() {
    }

    @Override
    public double getSimulationCurrentDraw() {
        double current = inputs.armCurrentAmps;
        current += inputs.rollerCurrentAmps;
        current += inputs.hopperCurrentAmps;
        return current;
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation() && io instanceof IntakeIOSim simIO) {
            SwerveBase.getInstance().getMapleSimDrive().ifPresent(simIO::attachMapleSimDrivetrain);
        }
    }

    public swervelib.simulation.ironmaple.simulation.IntakeSimulation getMapleIntakeSim() {
        if (io instanceof IntakeIOSim simIO) {
            return simIO.getMapleIntakeSim();
        }
        return null;
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Intake/Pivot Position", currentPosition);
        SmartDashboard.putNumber("Intake/Pivot Goal", goal);
        SmartDashboard.putNumber("Intake/Arm Applied Output", inputs.armAppliedVolts);
        SmartDashboard.putNumber("Intake/Arm Speed", inputs.armVelocityDegPerSec);
        SmartDashboard.putNumber("Intake/Wheels Applied Output", inputs.rollerAppliedVolts);
        SmartDashboard.putString("Intake/State", stateStr);
        SmartDashboard.putNumber("Intake/Setpoint Position", pivotProfiledPIDController.getSetpoint().position);
        SmartDashboard.putNumber("Intake/Setpoint Velocity", pivotProfiledPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber("Intake/Arm Voltage", armVoltage);

        // 3D Visualizer
        Translation3d armPivot = new Translation3d(0.2, 0, 0.2);
        Rotation3d armRotation = new Rotation3d(0, -Math.toRadians(currentPosition), 0);
        Pose3d armPose = new Pose3d(armPivot, armRotation);

        SmartDashboard.putNumberArray("Subsystems/Intake/ArmPose3d", new double[] {
                armPose.getX(), armPose.getY(), armPose.getZ(),
                armPose.getRotation().getQuaternion().getW(),
                armPose.getRotation().getQuaternion().getX(),
                armPose.getRotation().getQuaternion().getY(),
                armPose.getRotation().getQuaternion().getZ()
        });
        org.littletonrobotics.junction.Logger.recordOutput("Subsystems/Intake/ArmPose3d", armPose);
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "IntakeMechanism";
    }
}
