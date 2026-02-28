package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.state.ModelSE2;
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
        RangeSolver rangeSolver = new RangeSolver(d, 0);
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
            FiringSolution rangeSolution = rangeSolver.getSolution(v, 0, elevation);
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
        assertEquals(0.506, j.get(1, 0), 0.001);
        // more elevation makes x error more positive
        assertEquals(4.995, j.get(0, 1), 0.001);
        // more elevation doesn't change y error
        assertEquals(0, j.get(1, 1), 0.001);

        if (DEBUG)
            System.out.println(StrUtil.matStr(j));

    }

    /** For parabolic paths, we can check against the analytical solution. */
    @Test
    void testMotionlessParabolic() {
        if (DEBUG)
            System.out.println("## ShootingMethodTest.testMotionlessParabolic");
        double g = 9.81;
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        double v = 7;
        // tight tolerance for testing
        // note this tolerance is smaller than the range accuracy
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        ShootingMethod m = new ShootingMethod(ir, 0.001, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(new ModelSE2(), targetPosition, targetVelocity);
        Solution x = o.orElseThrow();

        double azimuth = 0;
        double elevation = 0.206;
        double r = 2;
        double tof = 0.292;
        checkX(x, azimuth, elevation);
        checkSolution(ir, x, r, tof);

        // check analytic solution is the same
        double R = v * v * Math.sin(2 * elevation) / g;
        double t = 2 * v * Math.sin(elevation) / g;
        assertEquals(r, R, 0.001);
        assertEquals(tof, t, 0.001);
    }

    /** Same scenario as above but with drag. */
    @Test
    void testMotionless() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        double v = 7;
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);

        // tight tolerance for testing
        // note this tolerance is smaller than the range accuracy
        ShootingMethod m = new ShootingMethod(ir, 0.001, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(new ModelSE2(), targetPosition, targetVelocity);
        Solution x = o.orElseThrow();

        double azimuth = 0;
        // higher elevation than the zero-drag case
        double elevation = 0.346;
        double r = 2;
        // tof is higher too
        double tof = 0.423;

        checkX(x, azimuth, elevation);
        checkSolution(ir, x, r, tof);
    }

    /** Target is approaching. */
    @Test
    void testTowardsTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        IRange ir = (e) -> rangeSolver.getSolution(7, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.0001, 0.1);

        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(1, 0, 0)),
                targetPosition, targetVelocity);
        Solution x = o.orElseThrow();

        double azimuth = 0;
        // lower elevation than the motionless case.
        double elevation = 0.256;
        // approaching target means shorter range
        double r = 1.675;
        // less time in the air
        double tof = 0.324;

        checkX(x, azimuth, elevation);
        checkSolution(ir, x, r, tof);
    }

    /** Target is receding pretty fast. */
    @Test
    void testAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        IRange ir = (e) -> rangeSolver.getSolution(10, 0, e);
        // velocity is higher because the target is receding.
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.0001, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(-2, 0, 0)),
                targetPosition, targetVelocity);
        Solution x = o.orElseThrow();

        double azimuth = 0;
        // higher elevation (in addition to higher velocity)
        double elevation = 0.390;
        // target is much further away when the ball reaches it
        double r = 3.206;
        // time of flight is much longer
        double tof = 0.603;
        checkX(x, azimuth, elevation);
        checkSolution(ir, x, r, tof);
    }

    /** Target is receding too fast to reach. */
    @Test
    void testImpossible() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        IRange ir = (e) -> rangeSolver.getSolution(10, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.0001, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(-10, 0, 0)),
                targetPosition, targetVelocity);
        assertTrue(o.isEmpty());
    }

    /**
     * Target is ahead, robot is moving to the left.
     */
    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        IRange ir = (e) -> rangeSolver.getSolution(7, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.01, 0.1);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<Solution> o = m.solve(
                new ModelSE2(new Pose2d(), new VelocitySE2(0, 2, 0)),
                targetPosition, targetVelocity);
        Solution x = o.orElseThrow();

        // aim to the right
        double azimuth = -0.484;
        // pretty high elevation
        double elevation = 0.449;
        // target is further away when the ball reaches it
        double r = 2.259;
        // flight time is longer
        double tof = 0.526;
        checkX(x, azimuth, elevation);
        checkSolution(ir, x, r, tof);
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
        RangeSolver rangeSolver = new RangeSolver(d, 0);
        double v = 7;
        IRange ir = (e) -> rangeSolver.getSolution(v, 0, e);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(ir, 0.0001, 0.1);

        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        int iterations = 10000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            // this solve takes 7 iterations
            m.solve(new ModelSE2(), targetPosition, targetVelocity);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }

    }

    private void checkX(Solution x, double azimuth, double elevation) {
        assertEquals(azimuth, x.azimuth().getRadians(), 0.001);
        assertEquals(elevation, x.elevation().getRadians(), 0.001);
    }

    private void checkSolution(IRange range, Solution x, double r, double tof) {
        FiringSolution s = range.get(x.elevation().getRadians());
        assertEquals(r, s.range(), 0.001);
        assertEquals(tof, s.tof(), 0.001);
    }

}
