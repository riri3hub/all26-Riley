package org.team100.frc2026;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class TargeterTest {
    @Test
    void testShoot() {
        // in the alliance zone, about 6m from the target
        Targeter targeter = new Targeter(() -> new Translation2d(1, 1));
        OptionalDouble angle = targeter.angle();
        assertEquals(0.441, angle.getAsDouble(), 0.001);
        OptionalDouble speed = targeter.speed();
        assertEquals(11.019, speed.getAsDouble(), 0.001);
    }

    @Test
    void testLob() {
        // in the center, so about 6m from the target
        Targeter targeter = new Targeter(() -> new Translation2d(8, 0));
        OptionalDouble angle = targeter.angle();
        assertEquals(0.3, angle.getAsDouble(), 0.001);
        OptionalDouble speed = targeter.speed();
        assertEquals(10, speed.getAsDouble(), 0.001);
    }

}
