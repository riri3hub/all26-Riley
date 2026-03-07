package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VariableVelocityRangeTest {
    private static final double DELTA = 0.001;

    @Test
    void testRange() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        Interception s = rangeSolver.getSolution(8, 50, Math.PI / 4);
        assertEquals(2.825, s.range(), DELTA);
        assertEquals(1.010, s.tof(), DELTA);
    }

    @Test
    void testRangeCached() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        VariableVelocityRangeCache r = new VariableVelocityRangeCache(
                rangeSolver, 0.7, 1.5, 50);
        Interception s = r.get(8, Math.PI / 4);
        assertEquals(2.825, s.range(), 0.01);
        assertEquals(1.011, s.tof(), DELTA);
    }
}
