package org.team100.lib.subsystems.five_bar;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.five_bar.kinematics.Scenario;

public class FiveBarMechTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TotalCurrentLog currentLog = new TotalCurrentLog(logger);

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
    void testWide() {
        assertFalse(new FiveBarMech(logger, currentLog, regularPentagon()).feasible(0, Math.PI));
    }

    @Test
    void testNarrow() {
        assertFalse(new FiveBarMech(logger, currentLog, regularPentagon()).feasible(Math.PI, 0));
    }

    @Test
    void testOK() {
        assertTrue(new FiveBarMech(logger, currentLog, regularPentagon()).feasible(Math.PI / 2, Math.PI / 2));
    }

}
