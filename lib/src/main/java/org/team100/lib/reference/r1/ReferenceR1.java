package org.team100.lib.reference.r1;

import org.team100.lib.state.ModelR1;

/**
 * Provides current and next references for servos.
 * 
 * Goal must be set prior to initialization.
 * 
 * NOTE: this class doesn't know anything about angle wrapping.
 */
public interface ReferenceR1 {

    void setGoal(ModelR1 goal);

    /** Set setpoint to measurement. */
    void init(ModelR1 measurement);

    SetpointsR1 get();

    /** The profile has reached the goal. */
    boolean profileDone();
}
