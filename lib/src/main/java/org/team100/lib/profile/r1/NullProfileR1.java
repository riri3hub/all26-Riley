package org.team100.lib.profile.r1;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

/** Always returns the initial state. */
public class NullProfileR1 implements ProfileR1 {

    @Override
    public ControlR1 calculate(double dt, ControlR1 initial, ModelR1 goal) {
        return initial;
    }

    @Override
    public NullProfileR1 scale(double s) {
        return this;
    }

    public double getMaxVelocity() {
        return 0;
    }
}
