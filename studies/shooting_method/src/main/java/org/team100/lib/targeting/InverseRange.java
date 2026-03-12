package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.util.InterpolatingMap100;
import org.team100.lib.util.OptUtil;

import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Lookup shooting parameters for a given target range
 * 
 * DO NOT USE THIS FOR COMP
 */
public class InverseRange implements DoubleFunction<Optional<FiringParameters>> {
    private static final boolean DEBUG = false;

    /** Precomputation lower bound. */
    private final double m_minElevation;
    /** Precomputation upper bound. */
    private final double m_maxElevation;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.01;

    /**
     * key = distance in meters
     * value = solution
     */
    private final InterpolatingMap100<Double, FiringParameters> m_map;

    public InverseRange(
            Drag d,
            double minElevation,
            double maxElevation,
            double targetHeight,
            double minTargetElevation,
            double muzzleVelocity,
            double omega) {
        m_minElevation = minElevation;
        m_maxElevation = maxElevation;
        RangeSolver rangeSolver = new RangeSolver(d, targetHeight, minTargetElevation, 0.001);
        m_map = new InterpolatingMap100<>(
                InverseInterpolator.forDouble(), FiringParameters::interpolate);
        if (DEBUG)
            System.out.println("range, elevation, tof");
        for (double elevation = m_minElevation; elevation <= m_maxElevation; elevation += ELEVATION_STEP) {
            Interception solution = rangeSolver.getSolution(muzzleVelocity, omega, elevation);
            if (solution == null) {
                if (DEBUG)
                    System.out.printf("null for v %f omega %f elevation %f\n", muzzleVelocity, omega, elevation);
                continue;
            }
            FiringParameters params = new FiringParameters(
                    solution.range(), muzzleVelocity, elevation, solution.tof());
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n",
                        solution.range(), params.elevation(), params.tof());
            m_map.put(solution.range(), params);
        }
        // Don't allow an empty map, it will fail later.
        if (m_map.size() == 0)
            throw new IllegalArgumentException("no solutions");
    }

    public Optional<FiringParameters> apply(double range) {
        // sampling off the end of the range should be an error.
        return OptUtil.emptyIfNull(m_map.get(range));
    }
}
