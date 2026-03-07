package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class BallisticsTest {
    private static final double DELTA = 0.001;

    @Test
    void testParabolic() {
        Interception s = Ballistics.parabolic(10, Math.PI / 4);
        // https://www.vcalc.com/wiki/ballistic-range
        assertEquals(10.194, s.range(), DELTA);
        // https://www.vcalc.com/wiki/ballistic-travel-time
        assertEquals(1.442, s.tof(), DELTA);
    }

    @Test
    void testNewton() {
    }

}
