package org.team100.lib.targeting;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Firing Parameter function with no model.
 * 
 * To calibrate:
 * 
 * * make sure the shooter velocity is constant
 * * measure the distance to the target
 * * adjust the elevation to hit the target
 * * measure the TOF with a stopwatch
 */
public class EmpiricalInverseRange implements DoubleFunction<FiringParameters> {
    /** key = distance in meters, value = solution */
    private final InterpolatingTreeMap<Double, FiringParameters> m_map;

    public EmpiricalInverseRange() {
        m_map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringParametersInterpolator());
        // these values should be determined experimentally.
        m_map.put(0.0, new FiringParameters(0, 5, 0.00, 0.00));
        m_map.put(1.0, new FiringParameters(1, 5, 0.05, 0.16));
        m_map.put(2.0, new FiringParameters(2, 5, 0.16, 0.37));
        m_map.put(3.0, new FiringParameters(3, 5, 0.47, 0.75));
    }

    public FiringParameters apply(double range) {
        return m_map.get(range);
    }

}
