package org.team100.lib.targeting;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * Solves the initial-value problem.
 * 
 * Given initial conditions of elevation and fixed muzzle velocity, integrates
 * the drag model until position is less than zero.
 * 
 * Returns a firing solution (range and time of flight).
 */
public class RangeSolver {
    private static final boolean DEBUG = false;

    /**
     * RK4 integration with this resolution.
     * 
     * See RangeSolverTest for choice of DT
     */
    private final double INTEGRATION_DT;

    private final Drag m_drag;
    private final double m_targetHeight;
    private final double m_minTargetElevation;

    /**
     * @param drag               Drag model
     * @param targetHeight       Height of the target above the firing height (not
     *                           the
     *                     floor)
     * @param minTargetElevation Minimum path elevation at arrival, from horizontal.
     *                           The HUB has a funnel that looks like about 1 radian
     *                           above horizontal.
     */
    public RangeSolver(
            Drag drag, double targetHeight, double minTargetElevation, double timeStep) {
        m_drag = drag;
        m_targetHeight = targetHeight;
        m_minTargetElevation = minTargetElevation;
        INTEGRATION_DT = timeStep;
    }

    /**
     * Solves the initial-value problem.
     * 
     * Both range and time-of-flight are always slight underestimates.
     * 
     * @param v         muzzle speed in m/s
     * @param omega     spin in rad/s, positive is backspin
     * @param elevation in rad
     */
    public Interception getSolution(
            double v, double omega, double elevation) {
        return solveWithDt(v, omega, elevation, INTEGRATION_DT);
    }

    /** Package-private for testing */
    Interception solveWithDt(
            double v, double omega, double elevation, double dt) {
        if (DEBUG)
            System.out.printf("range  solver solve for %f %f %f %f\n",
                    v, omega, elevation, dt);
        if (dt < 1e-6)
            throw new IllegalArgumentException("must use nonzero dt");
        double vx = v * Math.cos(elevation);
        double vy = v * Math.sin(elevation);
        // state is (x, y, theta, vx, vy, omega)
        Matrix<N6, N1> x = VecBuilder.fill(0, 0, 0, vx, vy, omega);
        Matrix<N6, N1> prevX = x;
        double t = 0;
        for (t = 0; t < 10; t += dt) {
            // this is the x for t+dt.
            x = NumericalIntegration.rk4(m_drag, prevX, dt);
            double range = x.get(0, 0);
            double height = x.get(1, 0);
            double prevRange = prevX.get(0, 0);
            double prevHeight = prevX.get(1, 0);

            double dy = height - prevHeight;
            if (DEBUG)
                System.out.printf("t %f prevRange %f range %f prevHeight %f height %f dy %f\n",
                        t, prevRange, range, prevHeight, height, dy);
            // on the way down, and below the target height
            if (dy < 0 && height < m_targetHeight) {
                if (DEBUG)
                    System.out.println("impact");
                // This used to interpolate to find the exact tof at the target
                // height but it was confusing so I took it out.

                double drange = range - prevRange; // a positive number
                // to compute the target elevation, look at the last two points.
                if (DEBUG)
                    System.out.printf("prevRange %f range %f prevHeight %f height %f\n",
                            prevRange, range, prevHeight, height);
                double targetElevation = Math.atan2(-1.0 * dy, drange);
                if (DEBUG)
                    System.out.printf("e %f\n", targetElevation);
                if (targetElevation < m_minTargetElevation) {
                    if (DEBUG)
                        System.out.printf("target elevation too low");
                    return null;
                }
                return new Interception(range, t, targetElevation);
            }
            prevX = x;
        }
        // if we got to the end, there's no (useful) solution.
        return null;
    }
}
