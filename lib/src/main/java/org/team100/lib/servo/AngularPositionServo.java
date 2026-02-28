package org.team100.lib.servo;

import org.team100.lib.music.Player;

/**
 * Angular position control, e.g. for swerve steering axes or arm axes.
 * 
 * An angular servo should generally get "wrapped" input. It figures out what
 * "unwrapped" commands to give the underlying mechanism.
 */
public interface AngularPositionServo extends Player {
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
     * Invalidates the current profile.
     */
    void setDutyCycle(double dutyCycle);

    void setTorqueLimit(double torqueNm);

    /**
     * Initializes the profile if necessary.
     * 
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * uses the current and next setpoints for actuation.
     * 
     * Because it uses a profile, this method is not suitable for anything that
     * needs to be coordinated, e.g. with another mechanism, or with a target. It's
     * suitable for simple, smooth control of independent mechanisms.
     * 
     * @param wrappedGoalRad The goal angle here wraps within [-pi, pi], using
     *                       output measurements, e.g. shaft radians, not motor
     *                       radians.
     * @param torqueNm       Feedforward for gravity or spring compensation.
     */
    void setPositionProfiled(double wrappedGoalRad, double torqueNm);

    /**
     * Invalidates the current profile, sets the setpoint directly, using the
     * supplied position and zero for acceleration.
     * 
     * This is really only appropriate for the Outboard control case, because the
     * feedback controller there can be much firmer than in the Outboard case, but
     * it does work with Onboard feedback.
     * 
     * This method goes the "short way" directly and the "long way" with a profile.
     * 
     * @param wrappedGoalRad Where the mechanism should be at the next time step.
     *                       The angle here wraps within [-pi, pi], using
     *                       output measurements, e.g. shaft radians, not motor
     *                       radians.
     * @param velocityRad_S  The desired velocity at the next time step, used for
     *                       feedforward.
     * @param torqueNm       Feedforward for gravity or spring compensation.
     */
    void setPositionDirect(double wrappedGoalRad, double velocityRad_S, double torqueNm);

    /**
     * For unwrapped goal
     */
    void actuateWithProfile(double unwrappedGoalX, double torqueNm);

    /**
     * This is the "wrapped" value, i.e. it is periodic within +/- pi.
     * 
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return Current position measurement, radians.
     */
    double getWrappedPositionRad();

    /** The "unwrapped" value domain is infinite. */
    double getUnwrappedPositionRad();

    /** Mechanism is following the desired setpoint. */
    boolean atSetpoint();

    boolean profileDone();

    /**
     * Profile is done, and we're on the setpoint.
     * 
     * Note this is affected by the setpoint update.
     * 
     * It really makes the most sense to call this *before* updating the setpoint,
     * because the measurement comes from the recent-past Takt and the updated
     * setpoint will be aiming at the next one.
     */
    boolean atGoal();

    void stop();

    void close();

    /** for logging */
    void periodic();
}
