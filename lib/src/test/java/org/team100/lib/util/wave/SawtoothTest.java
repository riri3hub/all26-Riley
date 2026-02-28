package org.team100.lib.util.wave;

import org.junit.jupiter.api.Test;

public class SawtoothTest {

    private static final boolean DEBUG = false;

    @Test
    void test0() {
        Sawtooth fn = new Sawtooth(1.0, 1.0, 1.0);
        for (double t = 0; t < 5; t += 0.1) {
            if (DEBUG)
                System.out.printf("%5.1f %6.3f\n", t, fn.applyAsDouble(t));
        }
    }

}
