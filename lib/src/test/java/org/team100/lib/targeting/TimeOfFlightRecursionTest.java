package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TimeOfFlightRecursionTest {
    private static final double DELTA = 0.001;

    /** See ShootingMethodTest.testMotionlessParabolic() */
    @Test
    void testMotionlessParabolic() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        double v = 7;
        InverseRange ir = new InverseRange(d, 0, v, 0);
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.01);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = tofr.solve(
                new ModelSE2(), targetPosition, targetVelocity);
        Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), DELTA);
        assertEquals(0.206, x.elevation().getRadians(), DELTA);
    }

    /**
     * See ShootingMethodTest.testAwayFromTarget()
     * 
     * This takes a whole lot of iterations to match the shooting method, note the
     * tight tolerance.
     */
    @Test
    void testAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        InverseRange ir = new InverseRange(d, 0, 10, 0);
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.0001);
        // driving away from the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(-2, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        Optional<Solution> o = tofr.solve(
                new ModelSE2(
                        new Pose2d(),
                        new VelocitySE2(robotVelocity.x(), robotVelocity.y(), 0)),
                targetPosition, targetVelocity);
        Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), DELTA);
        assertEquals(0.390, x.elevation().getRadians(), DELTA);
    }

    /** See ShootingMethodTest.testStrafing() */
    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        double v = 7;
        InverseRange ir = new InverseRange(d, 0, v, 0);
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.001);
        // driving to the left
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(0, 2);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        Optional<Solution> o = tofr.solve(
                new ModelSE2(new Pose2d(),
                        new VelocitySE2(robotVelocity.x(), robotVelocity.y(), 0)),
                targetPosition, targetVelocity);
        Solution x = o.orElseThrow();
        assertEquals(-0.484, x.azimuth().getRadians(), DELTA);
        assertEquals(0.449, x.elevation().getRadians(), DELTA);
    }

}
