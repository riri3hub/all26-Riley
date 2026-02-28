package org.team100.lib.util.wave;

import org.junit.jupiter.api.Test;

public class StairTest {
    private static final boolean DEBUG = false;

    @Test
    void test0() {
        Stair stair = new Stair(1.0);
        for (double t = 0; t < 5; t += 0.1) {
            if (DEBUG)
                System.out.printf("%5.1f %6.3f\n", t, stair.applyAsDouble(t));
        }
    }

}
