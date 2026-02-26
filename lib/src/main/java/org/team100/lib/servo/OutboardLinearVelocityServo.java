package org.team100.lib.servo;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.LinearMechanism;

/** There is no profile here. */
public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;
    private final double m_tolerance;

    private final LoggerFactory m_log;
    private final LinearMechanism m_mechanism;
    private final DoubleLogger m_log_setpoint_v;
    private final DoubleLogger m_log_setpoint_a;

    // For calculating acceleration
    private double m_prevGoal = 0;
    // For calculating acceleration
    private double m_prevT = 0;

    private double m_goal;

    public OutboardLinearVelocityServo(LoggerFactory parent, LinearMechanism mechanism, double tolerance) {
        m_log = parent.type(this);
        m_mechanism = mechanism;
        m_tolerance = tolerance;
        m_log_setpoint_v = m_log.doubleLogger(Level.TRACE, "setpoint v (m_s)");
        m_log_setpoint_a = m_log.doubleLogger(Level.TRACE, "setpoint a (m_s2)");
    }

    @Override
    public void reset() {
        if (DEBUG)
            System.out.println("WARNING: make sure resetting encoder position doesn't break anything");
        m_mechanism.resetEncoderPosition();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /** Passthrough to the outboard control. */
    @Override
    public void setVelocity(double setpointM_S) {
        setVelocity(setpointM_S, accel(setpointM_S));
    }

    /** Passthrough to the outboard control. */
    @Override
    public void setVelocity(double setpointM_S, double setpointM_S2) {
        if (DEBUG) {
            System.out.printf("setpointM_S %6.3f\n", setpointM_S);
        }
        m_goal = setpointM_S;
        m_mechanism.setVelocity(setpointM_S, setpointM_S2, 0);
        m_log_setpoint_v.log(() -> setpointM_S);
        m_log_setpoint_a.log(() -> setpointM_S2);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atGoal() {
        return Math.abs(m_goal - m_mechanism.getVelocityM_S()) < m_tolerance;
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public double getDistance() {
        return m_mechanism.getPositionM();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
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
        if (DEBUG)
            System.out.printf("dt %5.3f setpoint %5.3f accel %5.3f %s\n", dt, setpoint, accel, m_log.getRoot());
        m_prevGoal = setpoint;
        return accel;
    }

    @Override
    public void play(double freq) {
        m_mechanism.play(freq);
    }
}
