package org.team100.lib.uncertainty;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VisionNoiseTest {

    private static final double DELTA = 0.001;

    @Test
    void testFigure5() {
        assertEquals(0.03, VisionNoise.figure5(1), DELTA);
        assertEquals(0.00, VisionNoise.figure5(0), DELTA);
    }

    @Test
    void testFigure6() {
        assertEquals(Math.toRadians(0.342), VisionNoise.figure6(Math.PI / 4), DELTA);
        // The figure doesn't actually show a value for zero; it is presumably very
        // high. We cap it at 3.
        assertEquals(3, VisionNoise.figure6(0), DELTA);
    }

    @Test
    void testVisionStdDevs() {
        double targetRangeM = 1.0;
        IsotropicNoiseSE2 visionStdDev = VisionNoise.get(targetRangeM, 0.1);
        assertEquals(0.041, visionStdDev.cartesian(), DELTA);
        assertEquals(0.041, visionStdDev.rotation(), DELTA);
    }

}
