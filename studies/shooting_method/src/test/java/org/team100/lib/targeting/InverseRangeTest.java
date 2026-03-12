package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** See RangeTest. */
public class InverseRangeTest {
    private static final double DELTA = 0.001;

    /** Inverse of RangeTest.test0 */
    @Test
    void test0() {
        Drag drag = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        double targetHeight = 0;
        double minTargetElevation = 1;
        double muzzleSpeed = 10;
        double omega = 1;
        double range = 3;
        double minElevation = 0.75;
        double maxElevation = 2.5;
        InverseRange ir = new InverseRange(
                drag,
                minElevation,
                maxElevation,
                targetHeight,
                minTargetElevation,
                muzzleSpeed,
                omega);
        FiringParameters p = ir.apply(range).get();

        assertEquals(1, p.elevation(), DELTA);
        assertEquals(1.18, p.tof(), DELTA);

        // the target elevation is 1.3 but it's not recorded anywhere.

    }

}
