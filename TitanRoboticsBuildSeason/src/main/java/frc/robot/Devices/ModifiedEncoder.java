package frc.robot.Devices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ModifiedEncoder {

    private DutyCycleEncoder dutyCycleEncoder;
    private Encoder encoder;

    public ModifiedEncoder(int CANID) {
        DigitalInput input = new DigitalInput(CANID);
        dutyCycleEncoder = new DutyCycleEncoder(input);
    }

    public double get() {
        return dutyCycleEncoder != null ? dutyCycleEncoder.get() : 0;
    }

    public void setDistancePerPulse(double distancePerPulse) {
        if (encoder != null) {
            encoder.setDistancePerPulse(distancePerPulse);
        }
    }

    public double getAbsolutePosition() {
        return dutyCycleEncoder != null ? (dutyCycleEncoder.get() * 360) : 0;

    }

}
