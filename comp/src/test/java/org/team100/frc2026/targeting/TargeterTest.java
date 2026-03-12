package org.team100.frc2026.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class TargeterTest {
    @Test
    void testShoot() {
        // in the alliance zone, about 6m from the target
        Targeter targeter = new Targeter(() -> new Translation2d(1, 1));
        double angle = targeter.forRange(6).get().elevation();
        assertEquals(0.448, angle, 0.001);
        double speed = targeter.forRange(6).get().speed();
        assertEquals(11.130, speed, 0.001);
    }

    @Test
    void testLob() {
        // in the center, so about 6m from the target
        Targeter targeter = new Targeter(() -> new Translation2d(8, 0));
        double angle = targeter.forRange(6).get().elevation();
        assertEquals(0.3, angle, 0.001);
        double speed = targeter.forRange(6).get().speed();
        assertEquals(10, speed, 0.001);
    }

}
