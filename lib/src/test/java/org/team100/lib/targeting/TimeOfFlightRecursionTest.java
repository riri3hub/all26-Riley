package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.TimeOfFlightRecursion.Looper;
import org.team100.lib.targeting.TimeOfFlightRecursion.Looper.LoopSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TimeOfFlightRecursionTest {
    private static final double DELTA = 0.001;

    /** Motionless solves in one step. */
    @Test
    void testMotionlessParabolicSteps() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        double v = 7;
        InverseRange ir = new InverseRange(d, 0.01, 1, 0, 0, v, 0);
        Translation2d T0 = new Translation2d(2, 0);
        GlobalVelocityR2 vT = GlobalVelocityR2.ZERO;
        Looper looper = new Looper(ir, T0, vT);
        double targetTOF = ir.apply(T0.getNorm()).tof();
        assertEquals(0.291, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF);
        assertEquals(0.206, soln.params().elevation(), DELTA);
        assertEquals(0.291, soln.params().tof(), DELTA);
    }

    /** See ShootingMethodTest.testMotionlessParabolic() */
    @Test
    void testMotionlessParabolic() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        double v = 7;
        // note the range here is for "direct" fire
        // TODO: use indirect fire
        InverseRange ir = new InverseRange(d, 0.01, 1, 0, 0, v, 0);
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
     * Takes quite a few iterations for these inputs, chasing
     * the target which is moving away.
     */
    @Test
    void testAwaySteps() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        InverseRange ir = new InverseRange(d, 0.01, 0.8, 0, 0, 10, 0);

        Translation2d T0 = new Translation2d(2, 0);
        GlobalVelocityR2 vT = new GlobalVelocityR2(2, 0);

        Looper looper = new Looper(ir, T0, vT);

        double targetTOF = ir.apply(T0.getNorm()).tof();
        assertEquals(0.279, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF);
        assertEquals(0.238, soln.params().elevation(), DELTA);
        assertEquals(0.401, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.284, soln.params().elevation(), DELTA);
        assertEquals(0.465, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.313, soln.params().elevation(), DELTA);
        assertEquals(0.504, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.332, soln.params().elevation(), DELTA);
        assertEquals(0.529, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.346, soln.params().elevation(), DELTA);
        assertEquals(0.546, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.355, soln.params().elevation(), DELTA);
        assertEquals(0.558, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.362, soln.params().elevation(), DELTA);
        assertEquals(0.567, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.368, soln.params().elevation(), DELTA);
        assertEquals(0.574, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.371, soln.params().elevation(), DELTA);
        assertEquals(0.579, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.375, soln.params().elevation(), DELTA);
        assertEquals(0.583, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.377, soln.params().elevation(), DELTA);
        assertEquals(0.586, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.379, soln.params().elevation(), DELTA);
        assertEquals(0.588, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.381, soln.params().elevation(), DELTA);
        assertEquals(0.590, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.382, soln.params().elevation(), DELTA);
        assertEquals(0.591, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.383, soln.params().elevation(), DELTA);
        assertEquals(0.592, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.383, soln.params().elevation(), DELTA);
        assertEquals(0.593, soln.params().tof(), DELTA);
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
        // Note direct-fire constraint
        // TODO: use indirect fire
        InverseRange ir = new InverseRange(d, 0.01, 0.8, 0, 0, 10, 0);
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
        assertEquals(0.390, x.elevation().getRadians(), 0.006);
    }

    /** More of an arc */
    @Test
    void testHighAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        // note high minimum elevation
        InverseRange ir = new InverseRange(d, 0.6, 2, 0, 0.7, 10, 0);
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
        assertEquals(0.65, x.elevation().getRadians(), 0.006);
    }

    /**
     * Converges a bit faster than the "away" case.
     */
    @Test
    void testStrafingSteps() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        double v = 7;
        InverseRange ir = new InverseRange(d, 0.01, 0.5, 0, 0, v, 0);

        Translation2d T0 = new Translation2d(2, 0);
        GlobalVelocityR2 vT = new GlobalVelocityR2(0, -2);

        Looper looper = new Looper(ir, T0, vT);

        double targetTOF = ir.apply(T0.getNorm()).tof();
        assertEquals(0.422, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF);
        assertEquals(0.407, soln.params().elevation(), DELTA);
        assertEquals(0.486, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.431, soln.params().elevation(), DELTA);
        assertEquals(0.508, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.441, soln.params().elevation(), DELTA);
        assertEquals(0.518, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.445, soln.params().elevation(), DELTA);
        assertEquals(0.522, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.446, soln.params().elevation(), DELTA);
        assertEquals(0.523, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF);
        assertEquals(0.446, soln.params().elevation(), DELTA);
        assertEquals(0.523, soln.params().tof(), DELTA);
    }

    /** See ShootingMethodTest.testStrafing() */
    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        double v = 7;
        InverseRange ir = new InverseRange(d, 0.01, 0.5, 0, 0, v, 0);
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
        assertEquals(-0.484, x.azimuth().getRadians(), 0.002);
        assertEquals(0.449, x.elevation().getRadians(), 0.003);
    }

}
