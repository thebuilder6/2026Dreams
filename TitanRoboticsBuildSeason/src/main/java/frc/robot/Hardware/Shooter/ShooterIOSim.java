package frc.robot.Hardware.Shooter;

import frc.robot.Subsystems.Shooter;

import frc.robot.Sim.ShooterSim;

/**
 * Desktop simulation implementation of ShooterIO wrapping ShooterSim physics.
 */
public class ShooterIOSim implements ShooterIO {

    private final ShooterSim shooterSim;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    public ShooterIOSim() {
        shooterSim = new ShooterSim();
    }

    public ShooterSim getShooterSim() {
        return shooterSim;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterSim.update(leftAppliedVolts, rightAppliedVolts);

        inputs.leftVelocityRPM = shooterSim.getLeftVelocityRPM();
        inputs.rightVelocityRPM = shooterSim.getRightVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerVelocityRPM = (kickerAppliedVolts / 12.0) * 5676.0;
        inputs.leftCurrentAmps = shooterSim.getLeftCurrentDrawAmps();
        inputs.rightCurrentAmps = shooterSim.getRightCurrentDrawAmps();
        inputs.kickerCurrentAmps = Math.abs(kickerAppliedVolts) > 0.1 ? 2.5 : 0.0;
        inputs.leftBusVolts = 12.0;
        inputs.rightBusVolts = 12.0;
    }

    @Override
    public void setFlywheelVoltages(double leftVolts, double rightVolts) {
        this.leftAppliedVolts = leftVolts;
        this.rightAppliedVolts = rightVolts;
    }

    @Override
    public void setKickerVoltage(double kickerVolts) {
        this.kickerAppliedVolts = kickerVolts;
        if (Math.abs(kickerVolts) > 1.0) {
            shooterSim.launchSimulatedFuel(shooterSim.getLeftVelocityRPM(), shooterSim.getRightVelocityRPM());
        }
    }

    @Override
    public void stop() {
        setFlywheelVoltages(0.0, 0.0);
        setKickerVoltage(0.0);
    }
}
