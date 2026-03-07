package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

public class VariableVelocityShootingMethodTest {
    private static final boolean DEBUG = false;



    @Test
    void testJacobian1() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        // this is the point used below
        Interception soln = rangeSolver.getSolution(5, 0, 1);
        assertEquals(2.318, soln.range(), 0.001);
        assertEquals(0.857, soln.tof(), 0.001);
        // without drag, for target height equal to gun height, this should be the same.
        assertEquals(1, soln.targetElevation(), 0.001);
        IVVRange ivvr = (v, e) -> rangeSolver.getSolution(v, 0, e);
        verifyJacobian(ivvr);
    }

    void verifyJacobian(IVVRange ivvr) {
        // this is a parabola
        double targetElevation = 0.5;
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Translation2d T0 = targetPosition.minus(robotPosition);
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);
        // azimuth 0, velocity 5, elevation 1
        final Vector<N3> x0 = VecBuilder.fill(0, 5, 1);
        final Function<Vector<N3>, Vector<N3>> fn = (x) -> {
            Rotation2d azimuth = new Rotation2d(x.get(0));
            double velocity = x.get(1);
            double elevation = x.get(2);
            if (DEBUG)
                System.out.printf("x az %f v %f el %f\n",
                        azimuth.getRadians(), velocity, elevation);
            Interception rangeSolution = ivvr.get(velocity, elevation);
            if (rangeSolution == null) {
                if (DEBUG)
                    System.out.println("null");
                return VecBuilder.fill(10, 10, 10);
            }
            if (DEBUG)
                System.out.printf("soln %s\n", rangeSolution);
            Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
            Translation2d T = vT.integrate(T0, rangeSolution.tof());
            Translation2d err = b.minus(T);
            if (DEBUG)
                System.out.printf("err %10.8f\n", err.getX());
            double elevationErr = rangeSolution.targetElevation() - targetElevation;
            if (DEBUG)
                System.out.printf("elevationErr %f\n", elevationErr);
            // error is x - target
            return VecBuilder.fill(err.getX(), err.getY(), elevationErr);
        };
        Matrix<N3, N3> J = NumericalJacobian100.numericalJacobian2(
                Nat.N3(), Nat.N3(), fn, x0);

        // jacobian is
        // [dy1/dx1 dy1/dx2 dy1/dx3]
        // [dy2/dx1 dy2/dx2 dy2/dx3]
        // [dy3/dx1 dy3/dx2 dy3/dx3]
        // x is control (az, v, el)
        // y is error, (xerr, yerr, te)
        // so J should be
        // dxerr/daz dxerr/dv dxerr/del
        // dyerr/daz dyerr/dv dyerr/del
        // dte/daz dte/dv dte/del

        // more azimuth pushes target in y, not x
        assertEquals(0, J.get(0, 0), 0.002);
        // more azimuth pushes target in y
        // so jacobian is the radius
        assertEquals(2.318, J.get(1, 0), 0.003);
        // azimuth doesn't change target elevation at all.
        assertEquals(0, J.get(2, 0), 0.001);

        // more velocity makes xerror more positive
        assertEquals(0.464, J.get(0, 1), 0.05);
        // more velocity doesn't change y error
        assertEquals(0, J.get(1, 1), 0.001);
        // more velocity should not change target elevation at all
        // but it kinda does
        assertEquals(0, J.get(2, 1), 0.2);

        // at 1 radian, more elevation means less range
        assertEquals(-0.914, J.get(0, 2), 0.1);
        // more elevation doesn't change y error
        assertEquals(0, J.get(1, 2), 0.001);
        // more elevation makes el error more positive (should be 1)
        assertEquals(1.476, J.get(2, 2), 0.05);

        if (DEBUG)
            System.out.println(StrUtil.matStr(J));
    }

    @Test
    void testMotionlessParabolic1() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        IVVRange ivvr = (v, e) -> rangeSolver.getSolution(v, 0, e);
        verifyMotionlessParabolic(ivvr);
    }

    @Test
    void testMotionlessParabolic2() {
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        IVVRange ivvr = new VariableVelocityRangeCache(
                rangeSolver, 0.7, 1.5, 0);
        verifyMotionlessParabolic(ivvr);
    }

    void verifyMotionlessParabolic(final IVVRange ivvr) {
        double tolerance = 0.01;
        VariableVelocityShootingMethod m = new VariableVelocityShootingMethod(
                ivvr, 0.7, 1.5, tolerance);
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        double targetElevation = 0.5;
        double initialElevation = 0.1;
        Optional<VariableVelocityShootingMethod.Solution> o = m.solve(
                robotPosition,
                robotVelocity,
                targetPosition,
                targetVelocity,
                targetElevation,
                initialElevation);
        VariableVelocityShootingMethod.Solution x = o.orElseThrow();

        double azimuth = 0;
        double elevation = 0.5;
        double R = 2;
        double g = 9.81;
        double v = Math.sqrt(R * g / Math.sin(2 * elevation));
        double tof = 2 * v * Math.sin(elevation) / g;

        assertEquals(azimuth, x.azimuth().getRadians(), 0.01);
        assertEquals(elevation, x.elevation().getRadians(), 0.01);
        assertEquals(v, x.velocity(), 0.01);

        Interception s = ivvr.get(x.velocity(), x.elevation().getRadians());
        assertEquals(R, s.range(), 0.01);
        assertEquals(tof, s.tof(), 0.01);

    }

}
