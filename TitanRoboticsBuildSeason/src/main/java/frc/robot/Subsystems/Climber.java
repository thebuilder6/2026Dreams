package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.Constants.ClimberConstants;
import frc.robot.Interfaces.Subsystem;

public class Climber implements Subsystem {

    public enum ClimberState {
        UP, DOWN, STATIONARY
    }

    private static Climber instance = null;

    private final DoubleSolenoid m_doubleSolenoidLeft;
    private final DoubleSolenoid m_doubleSolenoidRight;
    private final Compressor m_compressor;

    private ClimberState state = ClimberState.STATIONARY;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private Climber() {
        m_doubleSolenoidLeft = new DoubleSolenoid(ClimberConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM,
                ClimberConstants.LEFT_FORWARD_CHANNEL, ClimberConstants.LEFT_REVERSE_CHANNEL);
        m_doubleSolenoidRight = new DoubleSolenoid(ClimberConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM,
                ClimberConstants.RIGHT_FORWARD_CHANNEL, ClimberConstants.RIGHT_REVERSE_CHANNEL);
        m_compressor = new Compressor(ClimberConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM);
        m_compressor.enableDigital();

        SubsystemManager.registerSubsystem(this);
    }

    public void setState(ClimberState state) {
        this.state = state;
    }

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    @Override
    public void update() {
        switch (state) {
            case UP:
                m_doubleSolenoidLeft.set(Value.kForward);
                m_doubleSolenoidRight.set(Value.kForward);
                break;
            case DOWN:
                m_doubleSolenoidLeft.set(Value.kReverse);
                m_doubleSolenoidRight.set(Value.kReverse);
                break;
            case STATIONARY:
            default:
                m_doubleSolenoidLeft.set(Value.kOff);
                m_doubleSolenoidRight.set(Value.kOff);
                break;
        }
    }

    @Override
    public void initialize() {
        state = ClimberState.STATIONARY;
    }

    @Override
    public void log() {
        SmartDashboard.putString("Climber/State", state.toString());
        SmartDashboard.putString("Climber/Left Solenoid", m_doubleSolenoidLeft.get().toString());
        SmartDashboard.putString("Climber/Right Solenoid", m_doubleSolenoidRight.get().toString());
        SmartDashboard.putBoolean("Climber/Pressure Switch", m_compressor.getPressureSwitchValue());
        SmartDashboard.putNumber("Climber/Current", m_compressor.getCurrent());
        SmartDashboard.putBoolean("Climber/Compressor Active", m_compressor.isEnabled());
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
