package org.team100.lib.subsystems.five_bar.kinematics;

import static java.lang.Math.sqrt;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

class FiveBarKinematicsTest {
    private static final double DELTA = 0.001;
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

    /**
     * A regular pentagon.
     * https://en.wikipedia.org/wiki/Pentagon
     */
    @Test
    void testPentagonInverse() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = regularPentagon();
        // Endpoint x is at the center of a5.
        double x3 = -0.5;
        // Height makes a pentagon.
        double y3 = sqrt(5 + 2 * sqrt(5)) / 2;
        // Check the calculation above
        assertEquals(1.539, y3, DELTA);
        ActuatorAngles p = m_kinematics.inverse(s, x3, y3).get();
        // 360/5 = 72 degrees
        assertEquals(1.256, p.q1(), DELTA);
        // The supplementary angle.
        assertEquals(1.885, p.q5(), DELTA);
    }

    /** the inverse of the case above */
    @Test
    void testPentagonForward() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = regularPentagon();
        double t1 = 1.256;
        double t5 = 1.885;
        JointPositions j = m_kinematics.forward(s, t1, t5).get();
        assertEquals(-0.5, j.P3().x(), DELTA);
        assertEquals(1.539, j.P3().y(), DELTA);
    }

    private Scenario littleHouse() {
        Scenario s = new Scenario();
        // Unit side length.
        s.a1 = 1.0;
        s.a2 = 0.5 * sqrt(2);
        s.a3 = 0.5 * sqrt(2);
        s.a4 = 1.0;
        s.a5 = 1.0;
        return s;
    }

    /** A square with a roof. */
    @Test
    void testLittleHouseInverse() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = littleHouse();
        // Endpoint x in the center of a5.
        double x3 = -0.5;
        // height
        double y3 = 1.5;
        // check the height
        ActuatorAngles p = m_kinematics.inverse(s, x3, y3).get();
        // 90 degrees
        assertEquals(1.571, p.q1(), DELTA);
        // also 90 degrees
        assertEquals(1.571, p.q5(), DELTA);
    }

    /** Inverse of the case above */
    @Test
    void testLittleHouseForward() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = littleHouse();
        double t1 = 1.571;
        double t5 = 1.571;
        JointPositions j = m_kinematics.forward(s, t1, t5).get();
        assertEquals(-0.5, j.P3().x(), DELTA);
        assertEquals(1.5, j.P3().y(), DELTA);
    }

    /**
     * Note that the kinematics always choose the "extended" option,
     * and so can't reach the baseline.
     */
    // @Test
    void testInverseOnTheBaseline() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = regularPentagon();
        // Endpoint x is at the center of a5.
        double x3 = -0.5;

        for (double y3 = 1.6; y3 >= 0; y3 -= 0.1) {
            ActuatorAngles p = m_kinematics.inverse(s, x3, y3).get();
            System.out.printf("%f, %f, %f\n", y3, p.q1(), p.q5());
        }
    }

    @Test
    void testInfeasibleInverse() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = regularPentagon();
        double x3 = -0.5;
        double y3 = 10;
        Optional<ActuatorAngles> p = m_kinematics.inverse(s, x3, y3);
        assertTrue(p.isEmpty());
    }

    @Test
    void testInfeasibleForward() {
        final FiveBarKinematics m_kinematics = new FiveBarKinematics(logger);
        Scenario s = regularPentagon();
        double t1 = 0;
        double t5 = Math.PI;
        Optional<JointPositions> j = m_kinematics.forward(s, t1, t5);
        assertTrue(j.isEmpty());
    }
}
