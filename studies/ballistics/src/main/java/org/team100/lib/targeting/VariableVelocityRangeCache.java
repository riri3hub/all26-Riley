package org.team100.lib.targeting;

import org.team100.lib.util.NestedInterpolatingTreeMap;

import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Provides a firing solution from elevation and muzzle velocity.
 * Uses a constant spin rate, which is probably wrong. Maybe spin rate is a
 * function of velocity?
 */
public class VariableVelocityRangeCache implements IVVRange {
    /**
     * Precomputation lower bound. Very low velocities don't work well with the
     * solvers and they're not useful anyway.
     */
    private static final double MIN_V = 3;
    /**
     * Precomputation upper bound. Very high velocity is possible but maybe not
     * useful.
     */
    private static final double MAX_V = 20;
    /** Precomputation step. */
    private static final double V_STEP = 0.5;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.05;

    /** Precomputation lower bound. */
    private final double m_minElevation;
    /** Precomputation upper bound. */
    private final double m_maxElevation;

    /**
     * Cache.
     * 
     * It won't take very much space (tens of KB), but computing all the values
     * requires running two layers of iterative solvers many times, so it's slow
     * to create.
     * 
     * The step values above came from fiddling with testJacobian().
     * 
     * 
     * key1 = velocity (m/s)
     * key2 = elevation (rad)
     * value = solution
     */
    private final NestedInterpolatingTreeMap<Double, Interception> m_map;

    public VariableVelocityRangeCache(
            RangeSolver rangeSolver, double minElevation, double maxElvation, double omega) {
        m_minElevation = minElevation;
        m_maxElevation = maxElvation;
        m_map = new NestedInterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new InterceptionInterpolator());
        for (double v = MIN_V; v < MAX_V; v += V_STEP) {
            for (double elevation = m_minElevation; elevation < m_maxElevation; elevation += ELEVATION_STEP) {
                Interception solution = rangeSolver.getSolution(v, omega, elevation);
                if (solution == null) {
                    // no solution
                    continue;
                }
                m_map.put(v, elevation, solution);
            }
        }
    }

    /**
     * @param v         velocity in m/s
     * @param elevation in radians
     */
    public Interception get(double v, double elevation) {
        return m_map.get(v, elevation);
    }
}
