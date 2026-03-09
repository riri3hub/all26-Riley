package org.team100.lib.controller.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Solution;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LeadingAimTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        Mutable.unpublishAll();
        FeedbackR1 feedback = new FullStateFeedback(log, 1, 0.01, false, 1, 1);
        LeadingAim aim = new LeadingAim(log, () -> 5.0, feedback);
        aim.reset(new ModelSE2());
        assertEquals(0, aim.getOmega(
                new ModelSE2(),
                new Solution(new Rotation2d(), 0, null)), DELTA);
        aim.reset(new ModelSE2());
        // feedback uses previous goal
        assertEquals(0, aim.getOmega(
                new ModelSE2(),
                new Solution(new Rotation2d(Math.PI / 2), 0, null)), DELTA);
        // now feedback has it
        assertEquals(Math.PI / 2, aim.getOmega(
                new ModelSE2(),
                new Solution(new Rotation2d(Math.PI / 2), 0, null)), DELTA);
        // strafing
        aim.reset(new ModelSE2(new Pose2d(), new VelocitySE2(0, 1, 0)));
        // velocity feedforward uses current goal
        assertEquals(-1, aim.getOmega(
                new ModelSE2(new Pose2d(), new VelocitySE2(0, 1, 0)),
                new Solution(new Rotation2d(0), -1, null)), DELTA);
    }

}
