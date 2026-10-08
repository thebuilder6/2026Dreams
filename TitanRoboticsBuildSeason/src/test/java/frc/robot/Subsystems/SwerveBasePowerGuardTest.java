package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

public class SwerveBasePowerGuardTest {

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testGuardedCurrentReadNeverThrowsAndIsFinite() {
        SwerveBase swerve = SwerveBase.getInstance();
        double amps = assertDoesNotThrow(swerve::readTotalCurrentSafe);
        assertTrue(Double.isFinite(amps), "guarded read must be finite, got " + amps);
        assertTrue(amps >= 0.0, "guarded read must be non-negative, got " + amps);
    }

    @Test
    public void testGuardedReadIsRepeatable() {
        SwerveBase swerve = SwerveBase.getInstance();
        double first = swerve.readTotalCurrentSafe();
        double second = swerve.readTotalCurrentSafe();
        assertTrue(Double.isFinite(first) && Double.isFinite(second));
    }
}
