package org.team100.lib.servo;

import org.team100.lib.reference.r1.SetpointsR1;

/**
 * Linear position control, e.g. for elevators.
 * 
 * The "servo" layer wraps the mechanism control with a profile, if desired.
 */
public interface LinearPositionServo {
    /**
     * Zeros controller errors, sets setpoint and goal to current measurement.
     * 
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * Initializes the profile if necessary.
     * This is movement and force on the output.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * @param goalM             meters
     * @param feedForwardTorque used for gravity compensation
     */
    void setPositionProfiled(double goalM, double feedForwardTorqueNm);

    /**
     * Invalidates the current profile, sets the setpoint directly.
     * This takes both current and next setpoints so that the implementation can
     * choose the current one for feedback and the next one for feedforward.
     *
     * You need to keep calling this to keep actuating. 
     */
    void setPositionDirect(SetpointsR1 setpoint, double feedForwardTorqueNm);

    double getPosition();

    double getVelocity();

    boolean atSetpoint();

    /** Useful for sequencing, without waiting for the controller. */
    boolean profileDone();

    /** Profile is done and we're on the setpoint. */
    boolean atGoal();

    void stop();

    void close();

    void periodic();
}
