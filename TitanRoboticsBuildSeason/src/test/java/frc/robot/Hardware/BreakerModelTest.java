package frc.robot.Hardware;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BreakerModelTest {

    private BreakerModel breaker;

    @BeforeEach
    void setup() {
        breaker = new BreakerModel();
    }

    @Test
    void testNormalCurrentDoesNotAccumulateDamage() {
        // 100A is below rated 120A
        breaker.update(100.0, 10.0);
        assertEquals(0.0, breaker.getDamageFraction(), 1e-6);
        assertFalse(breaker.isTripped());
    }

    @Test
    void testTripTimeDatasheetRatings() {
        // Below rated current -> sentinel infinite trip time
        assertTrue(BreakerModel.getTripTimeSeconds(100.0) >= BreakerModel.SENTINEL_TRIP_TIME_SECONDS);
        assertTrue(BreakerModel.getTripTimeSeconds(120.0) >= BreakerModel.SENTINEL_TRIP_TIME_SECONDS);

        // 240A is exactly 2.0x -> datasheet specifies 70 seconds
        assertEquals(70.0, BreakerModel.getTripTimeSeconds(240.0), 0.1);

        // 360A is 3.0x -> datasheet specifies 15 seconds
        assertEquals(15.0, BreakerModel.getTripTimeSeconds(360.0), 0.1);

        // 600A is 5.0x -> datasheet specifies 7 seconds
        assertEquals(7.0, BreakerModel.getTripTimeSeconds(600.0), 0.1);
    }

    @Test
    void testOvercurrentAccumulatesDamageLinearlyWithTime() {
        // At 240A (2.0x rated), trip time is 70s.
        // Running for 35s should accumulate exactly 35 / 70 = 0.50 damage
        double damage = breaker.update(240.0, 35.0);

        assertEquals(0.50, damage, 0.01);
        assertEquals(0.50, breaker.getDamageFraction(), 0.01);
        assertFalse(breaker.isTripped());
    }

    @Test
    void testCoolingDownUnderRatedThreshold() {
        // Set damage to 0.50
        breaker.setDamageFraction(0.50);

        // Cool for one time constant (TAU_COOL = 60s) at 50A (< 120A)
        // Expected damage = 0.50 * exp(-1) = 0.50 * 0.36788 = ~0.1839
        breaker.update(50.0, 60.0);

        double expected = 0.50 * Math.exp(-1.0);
        assertEquals(expected, breaker.getDamageFraction(), 0.02);
    }

    @Test
    void testBreakerTripsWhenDamageReachesOne() {
        // Running at 360A (trip time 15s) for 15.5s
        breaker.update(360.0, 15.5);

        assertEquals(1.0, breaker.getDamageFraction(), 1e-6);
        assertTrue(breaker.isTripped());
    }

    @Test
    void testResetClearsDamage() {
        breaker.setDamageFraction(0.85);
        assertEquals(0.85, breaker.getDamageFraction(), 1e-6);

        breaker.reset();
        assertEquals(0.0, breaker.getDamageFraction(), 1e-6);
        assertFalse(breaker.isTripped());
    }
}
