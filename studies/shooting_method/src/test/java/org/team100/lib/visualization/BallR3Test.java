package org.team100.lib.visualization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Drag;

import edu.wpi.first.math.geometry.Rotation2d;

public class BallR3Test {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    ModelSE2 robot = new ModelSE2();
    Rotation2d azimuth = new Rotation2d();
    Rotation2d elevation = new Rotation2d();

    @Test
    void testBall() {
        LoggerFactory field = new TestLoggerFactory(new TestPrimitiveLogger());
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        BallR3 b = new BallR3(field, d, () -> robot, () -> azimuth, () -> elevation, 10, 1);
        assertNull(b.m_location);
        elevation = new Rotation2d(Math.PI / 4);
        b.launch();
        assertEquals(0, b.location().getX(), DELTA);
        assertEquals(0, b.location().getY(), DELTA);
        assertEquals(0, b.location().getZ(), DELTA);
        assertEquals(7.071, b.velocity().x(), DELTA);
        assertEquals(0, b.velocity().y(), DELTA);
        assertEquals(7.071, b.velocity().z(), DELTA);
        if (DEBUG)
            System.out.println("t, x, y, z, vx, vy, vz");
        for (double t = 0; t < 1.1; t += 0.02) {
            if (DEBUG) {
                System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                        t,
                        b.location().getX(),
                        b.location().getY(),
                        b.location().getZ(),
                        b.velocity().x(),
                        b.velocity().y(),
                        b.velocity().z());
            }
            b.fly();
        }
    }

}
