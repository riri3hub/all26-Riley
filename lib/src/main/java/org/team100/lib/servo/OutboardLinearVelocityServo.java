package org.team100.lib.servo;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

/**
 * Profiled or direct velocity control using the feedback controller in the
 * motor controller hardware.
 * 
 * WARNING: the velocity control in REV motors is not very good. If you must use
 * it, you'll want to reduce the filtering on the sensor.
 */
public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;

    private final LinearMechanism m_mechanism;
    private final ReferenceR1 m_ref;
    private final double m_tolerance;

    private final DoubleLogger m_log_goal;
    private final ControlR1Logger m_log_control;
    private final DoubleLogger m_log_velocity;

    // For calculating acceleration
    private double m_prevGoal = 0;
    // For calculating acceleration
    private double m_prevT = 0;

    private ModelR1 m_goal;
    /** We use the "x" field for *VELOCITY* */
    private ControlR1 m_nextSetpoint;

    public OutboardLinearVelocityServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ReferenceR1 ref,
            double tolerance) {
        LoggerFactory log = parent.type(this);
        m_mechanism = mechanism;
        m_ref = ref;
        m_tolerance = tolerance;
        m_log_goal = log.doubleLogger(Level.COMP, "goal (m_s)");
        m_log_control = log.ControlR1Logger(Level.COMP, "control (m_s)");
        m_log_velocity = log.doubleLogger(Level.COMP, "velocity (m_s)");
    }

    @Override
    public void reset() {
        if (DEBUG)
            System.out.println("WARNING: make sure resetting encoder position doesn't break anything");
        // m_mechanism.resetEncoderPosition();
        ControlR1 measurement = new ControlR1(getVelocity(), 0);
        m_nextSetpoint = measurement;
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
    }

    /** Resets the profile if necessary */
    @Override
    public void setVelocityProfiled(double goalM_S) {
        m_log_goal.log(() -> goalM_S);
        ModelR1 goal = new ModelR1(goalM_S, 0);
        if (!goal.near(m_goal, m_tolerance, Double.MAX_VALUE)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            if (m_nextSetpoint == null) {
                m_nextSetpoint = new ControlR1(getVelocity(), 0);
            }
            m_ref.init(m_nextSetpoint.model());
        }
        actuate(m_ref.get());
    }

    /** Invalidates the current profile */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     */
    @Override
    public void setVelocityDirect(double setpointM_S) {
        setVelocityDirect(setpointM_S, accel(setpointM_S));
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     * Uses the same setpoint for "current" and "next".
     * TODO: expose both setpoints here.
     * TODO: change name of this method to "direct"
     */
    @Override
    public void setVelocityDirect(double setpointM_S, double setpointM_S2) {
        m_goal = null;
        ControlR1 setpoint = new ControlR1(setpointM_S, setpointM_S2);
        actuate(new SetpointsR1(setpoint, setpoint));
    }

    private void actuate(SetpointsR1 setpoints) {
        m_nextSetpoint = setpoints.next();
        double velocityM_S = m_nextSetpoint.x();
        double accelM_S2 = m_nextSetpoint.v();
        m_mechanism.setVelocity(velocityM_S, accelM_S2, 0);
        m_log_control.log(() -> m_nextSetpoint);
    }

    /**
     * Current velocity measurement. Note this can be noisy, maybe filter it.
     */
    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    /** invalidates the current goal and setpoint */
    @Override
    public void stop() {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.stop();
    }

    @Override
    public double getDistance() {
        return m_mechanism.getPositionM();
    }

    @Override
    public void play(double freq) {
        m_mechanism.play(freq);
    }

    @Override
    public boolean atSetpoint() {
        // Note x field for velocity.
        double vErr = m_nextSetpoint.x() - m_mechanism.getVelocityM_S();
        return Math.abs(vErr) < m_tolerance;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_velocity.log(() -> getVelocity());
    }

    ////////////////////////////////////////////////

    /**
     * Acceleration from trailing difference in velocity.
     * 
     * Note: in simulation, if you pull the setpoint directly from the simulated
     * joystick input, acceleration will be choppy: zero acceleration every other
     * cycle, because the simulated inputs seem to be polled at only 10 hz.
     * 
     * To avoid this problem, use the SwerveLimiter.
     */
    private double accel(double setpoint) {
        double t = Takt.get();
        double dt = t - m_prevT;
        m_prevT = t;
        double accel = (setpoint - m_prevGoal) / dt;
        m_prevGoal = setpoint;
        return accel;
    }
}
