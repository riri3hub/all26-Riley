package org.team100.lib.subsystems.swerve.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VCGTest {
    private static final double DELTA = 0.001;

    @Test
    void testDown() {
        double elevatorPosition = 0;
        assertEquals(0.18, VCG.vcg(elevatorPosition), DELTA);
    }

    @Test
    void testUp() {
        double elevatorPosition = 2;
        assertEquals(0.58, VCG.vcg(elevatorPosition), DELTA);
    }

}
