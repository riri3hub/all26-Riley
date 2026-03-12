package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.geometry.StateR2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.tuning.Mutable;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class ShootingMethodTest {
    private static final boolean DEBUG = false;

    /** Verify the Jacobian is doing what it should do. */
    @Test
    void testJacobian() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        // low min elevation so that this test works
        // TODO: make a more realistic case
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        double v = 5;
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Translation2d T0 = targetPosition.minus(robotPosition);
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);
        Vector<N2> x0 = VecBuilder.fill(0, 0.1);
        Function<Vector<N2>, Vector<N2>> fn = (x) -> {
            Rotation2d azimuth = new Rotation2d(x.get(0));
            double elevation = x.get(1);
            Interception rangeSolution = rangeSolver.getSolution(v, 0, elevation);
            Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
            Translation2d T = vT.integrate(T0, rangeSolution.tof());
            Translation2d err = b.minus(T);
            return GeometryUtil.toVec(err);
        };
        Matrix<N2, N2> j = NumericalJacobian100.numericalJacobian2(
                Nat.N2(), Nat.N2(), fn, x0);
        // jacobian is
        // [dy1/dx1 dy1/dx2]
        // [dy2/dx1 dy2/dx2]
        // x is control (azimuth, elevation)
        // y is translation error, (x,y)
        // more azimuth pushes target in y, not x
        assertEquals(0, j.get(0, 0), 0.001);
        // more azimuth pushes target in y; for low elevation radius is low
        assertEquals(0.506, j.get(1, 0), 0.002);
        // more elevation makes x error more positive
        assertEquals(4.995, j.get(0, 1), 0.08);
        // more elevation doesn't change y error
        assertEquals(0, j.get(1, 1), 0.001);

        if (DEBUG)
            System.out.println(StrUtil.matStr(j));

    }

    @Test
    void testDumpParabolic() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        double v = 7;
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        ShootingMethod m = new ShootingMethod(ir, 0.1, 1.5, 0.0001, 0.1);
        m.dump();
    }

    /**
     * For parabolic paths, we can check against the analytical solution.
     * 
     * This takes a few iterations, compared to 1 for TOF.
     * 
     * (Turn on NewtonsMethod.DEBUG to see.)
     */
    @Test
    void testMotionlessParabolic() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        double v = 7;
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        // Indirect fire, 0.75-1.5 rad
        //
        // The tolerance here must be larger than the RangeSolver step size,
        // otherwise the solution will just oscillate between rangesolver solutions,
        // neither of which is within the tolerance.
        //
        // Note the boundary is pi/4, not 0.7, which leaves a little divot
        // at the boundary that traps the solver.
        ShootingMethod m = new ShootingMethod(ir, Math.PI / 4, Math.PI / 2, 0.01, Math.PI / 4);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();

        double azimuth = 0;
        double elevation = 1.365; // Indirect fire
        double r = 2; // The range we asked for
        double tof = 1.396;
        checkX(x, azimuth, elevation, 0.01);
        checkSolution(ir, x, r, tof, 0.01);

        // check analytic solution is the same
        double g = 9.81;
        double R = v * v * Math.sin(2 * elevation) / g;
        double t = 2 * v * Math.sin(elevation) / g;
        assertEquals(r, R, 0.004);
        assertEquals(tof, t, 0.002);
    }

    /** Similar to above but with drag. */
    @Test
    void testMotionless() {
        Mutable.unpublishAll();
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        double v = 7;
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        ShootingMethod m = new ShootingMethod(ir, 0.7, 1.5, 0.0001, 0.7);
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();

        double azimuth = 0;
        double elevation = 1.067;
        double r = 2;
        // lower arc takes less time
        double tof = 0.982;

        checkX(x, azimuth, elevation, 0.01);
        checkSolution(ir, x, r, tof, 0.01);
    }

    @Test
    void testDumpWithDrag() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        double v = 7;
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.1, 1.4, 0.0001, 0.1);
        m.dump();
    }

    /** Robot is driving towards the target. */
    @Test
    void testTowardsTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(7, 0, e);
        ShootingMethod m = new ShootingMethod(ir, Math.PI / 4, Math.PI / 2, 0.001, 1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();

        double azimuth = 0;
        // higher elevation than the motionless case
        // with indirect fire, to aim closer, you aim higher.
        double elevation = 1.378;
        // approaching target means shorter range relative to a motionless gun
        double r = 0.907;
        // higher arc = more time in the air
        double tof = 1.093;

        checkX(x, azimuth, elevation, 0.01);
        checkSolution(ir, x, r, tof, 0.01);
    }

    /** Robot is driving away pretty fast. */
    @Test
    void testAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        // more v, otherwise it's impossible.
        IRange ir = (e) -> rangeSolver.getSolution(12, 0, e);
        // still indirect fire
        ShootingMethod m = new ShootingMethod(ir, 0.7, 1.5, 0.001, 0.7);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(-2, 0, 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();

        double azimuth = 0;
        double elevation = 0.721;
        // target is much further away when the ball reaches it
        double r = 4.108;
        // time of flight is not much longer because v is so high
        double tof = 1.054;
        checkX(x, azimuth, elevation, 0.01);
        checkSolution(ir, x, r, tof, 0.01);
    }

    /** Target is receding too fast to reach. */
    @Test
    void testImpossible() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        // low min elevation so that this test works
        // TODO: make a more realistic case
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(10, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.1, 1.4, 0.001, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(-10, 0, 0)),
                new StateR2(targetPosition, targetVelocity));
        assertTrue(o.isEmpty());
    }

    /**
     * Target is ahead, robot is moving to the left.
     */
    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        // low min elevation so that this test works
        // TODO: make a more realistic case
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0.01, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(7, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.1, 1.4, 0.01, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(0, 2, 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();

        // aim to the right
        double azimuth = -0.479;
        // pretty high elevation
        double elevation = 0.440;
        // target is further away when the ball reaches it
        double r = 2.245;
        // flight time is longer
        double tof = 0.518;
        checkX(x, azimuth, elevation, 0.01);
        checkSolution(ir, x, r, tof, 0.01);
    }

    /**
     * With Range caching off, using DT of 0.01 s (see RangeSolverTest), on my
     * machine this solves in about 350 us, so the RoboRIO could probably do it in
     * 1.5 ms.
     * 
     * With caching on, it takes 10 us on my machine, so maybe 50 us
     * on the roboRIO, so 30x savings.
     * 
     * Seems like using the cache is a good idea, especially once we add muzzle
     * velocity to the optimization.
     */
    // disable to speed up tests
    // @Test
    void testPerformance() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        double v = 7;
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.1, 1.4, 0.0001, 0.1);

        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        int iterations = 10000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            // this solve takes 7 iterations
            m.solve(new ModelSE2(),
                    new StateR2(targetPosition, targetVelocity));
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }

    }

    private void checkX(Solution x, double azimuth, double elevation, double delta) {
        assertEquals(azimuth, x.azimuth().getRadians(), delta);
        assertEquals(elevation, x.parameters().elevation(), delta);
    }

    private void checkSolution(IRange range, Solution x, double r, double tof, double delta) {
        Interception s = range.get(x.parameters().elevation());
        assertEquals(r, s.range(), delta);
        assertEquals(tof, s.tof(), delta);
    }

}
