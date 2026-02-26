package org.team100.lib.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;

/**
 * Uses mechanism position control.
 * 
 * Uses a profile with velocity feedforward, also extra torque (e.g. for
 * gravity). There's no feedback at this level, and no feedforward calculation
 * either: the mechanism does that.
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder so that
 * positional commands make sense.
 */
public class OutboardAngularPositionServo extends AngularPositionServoImpl {

    private final DoubleLogger m_log_ff_torque;
    private final ControlR1Logger m_log_control;

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReferenceR1 ref) {
        super(parent, mech, ref);
        LoggerFactory log = parent.type(this);
        m_log_ff_torque = log.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_control = log.ControlR1Logger(Level.TRACE, "setpoint (rad)");
    }

    /**
     * Pass the next setpoint directly to the mechanism's position controller.
     * Ignores current setpoint. We only use the "next" setpoint.
     */
    void actuate(SetpointsR1 unwrappedSetpoint, double torqueNm) {

        ControlR1 nextUnwrappedSetpoint = unwrappedSetpoint.next();

        m_mechanism.setUnwrappedPosition(
                nextUnwrappedSetpoint.x(),
                nextUnwrappedSetpoint.v(),
                nextUnwrappedSetpoint.a(),
                torqueNm);

        m_log_control.log(() -> nextUnwrappedSetpoint);
        m_log_ff_torque.log(() -> torqueNm);
    }

}
