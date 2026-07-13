package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class FrictionTest {
    private static final double DELTA = 0.001;

    @Test
    void testFriction() {
        // static friction = 2, dynamic friction = 1
        Friction friction = new Friction(2, 1, 0, 1);
        // under the static friction limit, so this is static
        assertEquals(2, friction.frictionFFVolts(0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(1, friction.frictionFFVolts(2), DELTA);
        // under the static friction limit, so this is static
        assertEquals(-2, friction.frictionFFVolts(-0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(-1, friction.frictionFFVolts(-2), DELTA);
        // want to go negative, get negative
        assertEquals(-2, friction.frictionFFVolts(-0.5), DELTA);
        // moving positive, want to go negative, get negative
        assertEquals(-1, friction.frictionFFVolts(-2), DELTA);
    }

}
