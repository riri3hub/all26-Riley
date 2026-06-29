package org.team100.lib.visualization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.five_bar.kinematics.FiveBarKinematics;
import org.team100.lib.subsystems.five_bar.kinematics.JointPositions;
import org.team100.lib.subsystems.five_bar.kinematics.Point;
import org.team100.lib.subsystems.five_bar.kinematics.Scenario;

public class FiveBarVisualizationTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private Scenario regularPentagon() {
        Scenario s = new Scenario();
        // unit side length
        // all sides the same
        s.a1 = 1.0;
        s.a2 = 1.0;
        s.a3 = 1.0;
        s.a4 = 1.0;
        s.a5 = 1.0;
        return s;
    }

    @Test
    void testPentagon() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);

        Scenario s = regularPentagon();
        double t1 = 1.25664;
        double t5 = 1.88496;
        JointPositions j = m_kinematics.forward(s, t1, t5).get();
        List<Point> p = FiveBarVisualization.links(j);
        // sides are all equal
        assertEquals(1.0, p.get(0).norm(), kDelta);
        assertEquals(1.0, p.get(1).norm(), kDelta);
        assertEquals(1.0, p.get(2).norm(), kDelta);
        assertEquals(1.0, p.get(3).norm(), kDelta);
        // parent-relative angles are all equal
        assertEquals(1.257, p.get(0).angle().orElseThrow(), kDelta);
        assertEquals(1.257, p.get(1).angle().orElseThrow(), kDelta);
        assertEquals(1.257, p.get(2).angle().orElseThrow(), kDelta);
        assertEquals(1.257, p.get(3).angle().orElseThrow(), kDelta);
    }
}
