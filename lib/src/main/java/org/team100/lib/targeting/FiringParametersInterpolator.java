package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

public class FiringParametersInterpolator implements Interpolator<FiringParameters> {

    @Override
    public FiringParameters interpolate(FiringParameters a, FiringParameters b, double t) {
        return new FiringParameters(
                MathUtil.interpolate(a.range(), b.range(), t),
                MathUtil.interpolate(a.speed(), b.speed(), t),
                MathUtil.interpolate(a.elevation(), b.elevation(), t),
                MathUtil.interpolate(a.tof(), b.tof(), t));
    }

}
