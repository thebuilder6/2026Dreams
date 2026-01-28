package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.ClimberConstants;
import frc.robot.Interfaces.Subsystem;

public class Climber implements Subsystem {

    private static Climber instance = null;

    private final DoubleSolenoid climberSolenoid;
    private final Compressor compressor;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private Climber() {
        climberSolenoid = new DoubleSolenoid(ClimberConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM,
                ClimberConstants.FORWARD_CHANNEL, ClimberConstants.REVERSE_CHANNEL);
        compressor = new Compressor(ClimberConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        SubsystemManager.registerSubsystem(this);
    }

    public void extend() {
        climberSolenoid.set(Value.kForward);
    }

    public void retract() {
        climberSolenoid.set(Value.kReverse);
    }

    public void off() {
        climberSolenoid.set(Value.kOff);
    }

    @Override
    public void update() {
        // Periodic updates if needed
    }

    @Override
    public void initialize() {
        retract();
    }

    @Override
    public void log() {
        SmartDashboard.putString("Climber/Solenoid State", climberSolenoid.get().toString());
        SmartDashboard.putBoolean("Climber/Pressure Switch", compressor.getPressureSwitchValue());
        SmartDashboard.putNumber("Climber/Current", compressor.getCurrent());
        SmartDashboard.putBoolean("Climber/Active", compressor.isEnabled());
    }

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public String getName() {
        return "Climber";
    }
}
