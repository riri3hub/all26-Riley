package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.frc2026.field.FieldConstants2026;

public class RangeSolverTest {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;

    /**
     * Target is level with the gun
     * min target elevation is high
     * gun elevation is low
     * so there are no solutions.
     */
    @Test
    void testTargetElevationConstraint() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        int minTargetElevation = 1;
        RangeSolver rangeSolver = new RangeSolver(d, 0, minTargetElevation, 0.001);
        Interception solution = rangeSolver.getSolution(7, 0, 0.4);
        assertNull(solution);
    }

    @Test
    void testRange() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        IRange ir = (e) -> rangeSolver.getSolution(8, 50, e);
        Interception s = ir.get(Math.PI / 4);
        assertEquals(2.825, s.range(), DELTA);
        assertEquals(1.010, s.tof(), DELTA);
    }

    /**
     * How should we choose a value for DT? We should choose a value such that the
     * expected integration error is tolerable. How do we know the integration
     * error? Try varying DT to see how it affects the outcome. If smaller DT gets
     * the same answer as larger DT, then there's no reason to use the smaller
     * one.
     * 
     * This can be viewed here:
     * 
     * https://docs.google.com/spreadsheets/d/1DTxWqwi1vgajUmj-J2YUuK79FHySJTi9KHT-RQxUAb0
     * 
     * The interpolation in the solver makes the variation of the solution with dt
     * very small.
     * 
     * Assuming the value for very small dt is "correct," the absolute error is very
     * roughly the square of dt, a little worse (dt^2.5) at larger values.
     * 
     * For 1 cm range accuracy in integration (for this middle-of-the-road
     * trajectory), a reasonable dt would be something like 0.1 sec.
     * 
     * Note 0.1 sec is really coarse. RK4 integration works!
     * 
     * But this definitely does not work for shorter trajectories.
     */
    @Test
    void testDt() {
        Drag d = new Drag(0.5, 0.1, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        double ddt = 0.0001;
        double dtmax = 0.25;
        for (double dt = ddt; dt < dtmax; dt += ddt) {
            Interception s = rangeSolver.solveWithDt(10, 1, 1, dt);
            if (DEBUG)
                System.out.printf("%20.10f %20.10f\n", dt, s.range());
        }
    }

    @Test
    void testBlarg() {
        // this case was broken before because the min target elevation
        // parameter was zero.
        RangeSolver rangeSolver = new RangeSolver(
                FieldConstants2026.FUEL_DRAG,
                FieldConstants2026.HUB.getZ(),
                FieldConstants2026.HUB_ELEVATION,
                0.001);
        Interception s = rangeSolver.solveWithDt(7, 1, 0.88, 0.001);
        assertNull(s);
    }

    /**
     * Using the coarse DT value determined in errorStudy(), what's the performance
     * of the integration?
     * 
     * On my machine, which is ~4x faster than a RoboRIO, I get something like 6 us
     * per solve, so the RoboRIO might get 25 us (!).
     */
    // disable to speed up tests
    // @Test
    void testPerformance() {
        Drag d = new Drag(0.5, 0, 0.1, 0.1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            // dt from below, 0.25 s
            rangeSolver.solveWithDt(10, 0, 1, 0.25);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }

    /**
     * Without drag, this should yield the analytic result.
     * 
     * R = v^2 sin(2 elevation) / g
     * 
     * t = 2 v sin(elevation) / g
     * 
     * https://phys.libretexts.org/Bookshelves/University_Physics/Physics_(Boundless)/3%3A_Two-Dimensional_Kinematics/3.3%3A_Projectile_Motion
     */
    @Test
    void testParabola() {
        // note min v, this doesn't work well for very low v
        for (double v = 4; v < 20; v += 1) {
            for (double elevation = 0.1; elevation < 1.4; elevation += 0.1) {
                verify(v, elevation);
            }
        }
    }

    @Test
    void testParabola2() {
        verify(5, 1);
    }

    /** Verify parabola */
    void verify(double v, double elevation) {
        double g = 9.81;
        double R = v * v * Math.sin(2 * elevation) / g;
        double t = 2 * v * Math.sin(elevation) / g;
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);
        Interception s = rangeSolver.getSolution(v, 0, elevation);
        assertNotNull(s, String.format(
                "v %6.3f elevation %6.3f R %6.3f t %6.3f\n",
                v, elevation, R, t));
        double rError = R - s.range();
        double tError = t - s.tof();
        double eError = elevation - s.targetElevation();
        if (DEBUG)
            System.out.printf("%6.3f, %6.3f, %10.7f, %10.7f, %10.7f\n",
                    v, elevation, rError, tError, eError);
        // coarse DT means expected position error is a little higher
        assertEquals(R, s.range(), 0.02, "range");
        assertEquals(t, s.tof(), DELTA, "tof");
        // 0.5 degree scatter here
        assertEquals(elevation, s.targetElevation(), 0.01, "target elevation");
    }

    /**
     * For a single range and velocity, show how dt affects error.
     * 
     * It is rare to shoot more than half of the (16m long) field.
     * In 2024 the "long" shots were 5 m, so use that.
     * 
     * Also in 2024, the muzzle velocity was 15 m/s.
     * 
     * We never studied surface-to-surface paths in 2024. Probably the "lob" shot
     * used an elevation of something like 1 rad, so use that.
     * 
     * Charts are here:
     * 
     * https://docs.google.com/document/d/1S9JrUbZsVmBq0wWBROYl0kICUvJro_HmXTB98dDyfMk
     * 
     * For range error, it would be nice to achieve 0.01 m. Is it possible?
     * 
     * For ToF, the RoboRIO can't do anything on a smaller timescale than its
     * 0.02-second clock, so that seems like a good goal.
     * 
     * The 0.02-second resolution does mean that, to achieve 0.01 m spatial
     * resolution, the relative target speed needs to be less than 0.5 m/s,
     * unrealistically slow.
     * 
     * A reasonable spatial tolerance might be 0.05 m, with 0.02 s temporal
     * tolerance.
     * 
     * The binding constraint appears to be distance, requiring dt < 0.25 s.
     */
    @Test
    void errorStudy() {
        double g = 9.81;
        double v = 15;
        double elevation = 1;
        // no air so we can compute the real answer
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);

        // System.out.println("dt, rErr, tErr");
        if (DEBUG)
            System.out.println("dt, R, range, t, tof");
        for (double dt = 0.001; dt < 0.5; dt += 0.001) {
            // true answer
            double R = v * v * Math.sin(2 * elevation) / g;
            double t = 2 * v * Math.sin(elevation) / g;
            // approximation
            Interception s = rangeSolver.solveWithDt(v, 0, elevation, dt);
            // System.out.printf("%12.9f, %12.9f, %12.9f\n", dt, R - s.range(), t -
            // s.tof());
            if (DEBUG)
                System.out.printf("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f\n",
                        dt, R, s == null ? 0 : s.range(), t, s == null ? 0 : s.tof());
        }
    }

    /**
     * Is the numerical jacobian being fooled by the error? Look at elevation.
     * 
     * This should work for the minimum (worst case) elevation
     * 
     *
     */
    @Test
    void errorStudy2() {
        double g = 9.81;
        double dt = 0.25;
        double elevation = 0.1;
        // no air so we can compute the real answer
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        // System.out.println("dt, rErr, tErr");
        if (DEBUG)
            System.out.println("v, R, range, t, tof");
        for (double v = 0.01; v < 20; v += 0.01) {
            // true answer
            double R = v * v * Math.sin(2 * elevation) / g;
            double t = 2 * v * Math.sin(elevation) / g;
            // approximation
            Interception s = rangeSolver.solveWithDt(v, 0, elevation, dt);
            if (DEBUG)
                System.out.printf("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f\n",
                        v, R, s == null ? 0 : s.range(), t, s == null ? 0 : s.tof());
        }
    }

    /** For velocity, the results are bad below about 1.5 m/s. */
    @Test
    void errorStudy3() {
        double g = 9.81;
        double v = 15;
        double dt = 0.25;
        // no air so we can compute the real answer
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        // System.out.println("dt, rErr, tErr");
        if (DEBUG)
            System.out.println("elevation, R, range, t, tof");
        for (double elevation = 0.01; elevation < 1.3; elevation += 0.01) {
            // true answer
            double R = v * v * Math.sin(2 * elevation) / g;
            double t = 2 * v * Math.sin(elevation) / g;
            // approximation
            Interception s = rangeSolver.solveWithDt(v, 0, elevation, dt);
            if (DEBUG)
                System.out.printf("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f\n",
                        elevation, R, s == null ? 0 : s.range(), t, s == null ? 0 : s.tof());
        }
    }

    /**
     * For the newton solver to work, the results need to be good within the entire
     * search space, which is a rectangle in (v, elevation) space.
     * 
     * Very low elevations should be allowed, e.g. 0.1 radians.
     * Very low velocities are not very useful: below 3 m/s the range is under 1 m.
     * An acceptable DT for those bounds would be 0.05 s.
     */
    @Test
    void testFull() {
        double g = 9.81;
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 0, 0.001);

        if (DEBUG)
            System.out.println("dt, v, elevation, R, range, t, tof");
        for (double dt = 0.005; dt < 0.15; dt += 0.005) {
            for (double v = 1; v < 20; v += 1) {
                for (double elevation = 0.1; elevation < 1.5; elevation += 0.1) {
                    double R = v * v * Math.sin(2 * elevation) / g;
                    double t = 2 * v * Math.sin(elevation) / g;
                    Interception s = rangeSolver.solveWithDt(v, 0, elevation, dt);
                    double rangeAbsoluteError = Math.abs(R - s.range());
                    double tofAbsoluteError = Math.abs(t - s.tof());
                    if (rangeAbsoluteError > 0.01) // 1 cm tolerance
                        continue;
                    if (tofAbsoluteError > 0.02) // roborio clock
                        continue;
                    if (DEBUG)
                        System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %10.7f, %10.7f\n",
                                dt, v, elevation, R, s.range(), t, s.tof());

                }
            }
        }
    }

    @Test
    void testShort() {
        double g = 9.81;
        Drag d = new Drag(0, 0, 0, 1, 0);
        RangeSolver rangeSolver = new RangeSolver(d, 0, 1, 0.001);
        double v = 1;
        double elevation = 0.1;
        double dt = 0.1;
        double R = v * v * Math.sin(2 * elevation) / g;
        double t = 2 * v * Math.sin(elevation) / g;
        Interception s = rangeSolver.solveWithDt(v, 0, elevation, dt);
        if (DEBUG)
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %10.7f, %10.7f\n",
                    dt, v, elevation, R, s == null ? 0 : s.range(), t, s == null ? 0 : s.tof());
    }

}
