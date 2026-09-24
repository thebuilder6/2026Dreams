package frc.robot.Subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import frc.robot.Sim.ArmSim;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * Desktop simulation implementation of IntakeIO wrapping ArmSim and MapleSim IntakeSimulation.
 */
public class IntakeIOSim implements IntakeIO {

    private final ArmSim armSim;
    private IntakeSimulation mapleIntakeSim = null;
    private double armAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double hopperAppliedVolts = 0.0;

    public IntakeIOSim() {
        armSim = new ArmSim();
    }

    /**
     * Initializes MapleSim IntakeSimulation once the drivetrain simulation is ready.
     * @param driveTrainSim AbstractDriveTrainSimulation of the robot
     */
    public void attachMapleSimDrivetrain(AbstractDriveTrainSimulation driveTrainSim) {
        if (driveTrainSim != null && mapleIntakeSim == null) {
            this.mapleIntakeSim = IntakeSimulation.OverTheBumperIntake(
                    "Fuel",
                    driveTrainSim,
                    Meters.of(0.65), // 65cm width across bumper
                    Meters.of(0.25), // 25cm extension
                    IntakeSimulation.IntakeSide.FRONT,
                    frc.robot.Data.Constants.IntakeConstants.MAX_HELD_BALLS // 30 fuel capacity
            );
            this.mapleIntakeSim.setGamePiecesCount(frc.robot.Sim.GameSim.getInstance().getHeldBalls());
            // Note: IntakeSimulation constructor already invokes register(SimulatedArena.getInstance()).
            // Do NOT call register() here, as doing so adds a second ContactListener to the physics world,
            // which causes every single ball contact event to increment the held count twice (+2).
        }
    }

    public IntakeSimulation getMapleIntakeSim() {
        return mapleIntakeSim;
    }

    public ArmSim getArmSim() {
        return armSim;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        armSim.update(armAppliedVolts);

        inputs.encoderConnected = true;
        inputs.armPositionDeg = Units.radiansToDegrees(armSim.getAngleRads());
        inputs.armVelocityDegPerSec = Units.radiansToDegrees(armSim.getVelocityRadsPerSec());
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.armMotorRotations = (inputs.armPositionDeg / 360.0) * 100.0; // 100:1 gear ratio

        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrentAmps = Math.abs(rollerAppliedVolts) > 0.1 ? 8.0 : 0.0;
        inputs.rollerVelocityRPM = (rollerAppliedVolts / 12.0) * 5676.0;

        inputs.hopperAppliedVolts = hopperAppliedVolts;
        inputs.hopperCurrentAmps = Math.abs(hopperAppliedVolts) > 0.1 ? 4.0 : 0.0;
        inputs.hopperVelocityRPM = (hopperAppliedVolts / 12.0) * 5676.0;
    }

    @Override
    public void setArmVoltage(double volts) {
        this.armAppliedVolts = volts;
    }

    @Override
    public void setRollerVoltage(double volts) {
        this.rollerAppliedVolts = volts;
        updateMapleIntakeRunning();
    }

    @Override
    public void setRollerSpeed(double speed) {
        this.rollerAppliedVolts = speed * 12.0;
        updateMapleIntakeRunning();
    }

    private void updateMapleIntakeRunning() {
        if (mapleIntakeSim != null) {
            if (Math.abs(rollerAppliedVolts) > 0.1) {
                if (!mapleIntakeSim.isRunning()) {
                    mapleIntakeSim.startIntake();
                }
            } else {
                if (mapleIntakeSim.isRunning()) {
                    mapleIntakeSim.stopIntake();
                }
            }
        }
    }

    @Override
    public void setHopperVoltage(double volts) {
        this.hopperAppliedVolts = volts;
    }

    @Override
    public void setHopperSpeed(double speed) {
        this.hopperAppliedVolts = speed * 12.0;
    }

    @Override
    public void stop() {
        setArmVoltage(0.0);
        setRollerSpeed(0.0);
        setHopperSpeed(0.0);
        if (mapleIntakeSim != null && mapleIntakeSim.isRunning()) {
            mapleIntakeSim.stopIntake();
        }
    }
}
