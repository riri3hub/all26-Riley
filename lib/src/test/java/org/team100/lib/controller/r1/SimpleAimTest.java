package org.team100.lib.controller.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Translation2d;

public class SimpleAimTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory field = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        Mutable.unpublishAll();
        FeedbackR1 feedback = new FullStateFeedback(log, 1, 0.01, false, 1, 1);
        SimpleAim aim = new SimpleAim(field, log, () -> 5.0, feedback);
        aim.reset();
        // on target
        assertEquals(0, aim.getOmega(new ModelSE2(), new Translation2d(1, 0)), DELTA);
        // steer left
        assertEquals(Math.PI / 2, aim.getOmega(new ModelSE2(), new Translation2d(0, 1)), DELTA);
        // steer left gently
        assertEquals(0.01, aim.getOmega(new ModelSE2(), new Translation2d(1, 0.01)), DELTA);
        // steer right
        assertEquals(-Math.PI / 2, aim.getOmega(new ModelSE2(), new Translation2d(0, -1)), DELTA);
        // target equals pose
        assertEquals(0, aim.getOmega(new ModelSE2(), new Translation2d(0, 0)), DELTA);
    }

}
