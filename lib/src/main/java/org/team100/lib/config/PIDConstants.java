package org.team100.lib.config;

import java.util.ArrayList;
import java.util.List;

/**
 * PID Units here use SI units:
 * 
 * position
 * P = volt/rad -- start with 0.1.
 * I = volt/rad-sec
 * D = volt-sec/rad
 * 
 * velocity
 * P = volt-sec/rad -- start with 0.001
 * I = volt/rad
 * D = volt-sec^2/rad
 * 
 * These units are converted to motor-native units by the consumers.
 * 
 * WARNING! REV velocity control does not work well for light mechanisms (e.g.
 * flywheels), no matter what you do with PID, so don't do it.
 */
public class PIDConstants {
    private final List<Runnable> m_listeners = new ArrayList<>();

    /** volt/rad */
    private final double m_positionP;
    /** volt/rad-sec */
    private final double m_positionI;
    /** volt-sec/rad */
    private final double m_positionD;

    /** volt-sec/rad */
    private final double m_velocityP;
    /** volt/rad */
    private final double m_velocityI;
    /** volt-sec^2/rad */
    private final double m_velocityD;

    /**
     * @param p Volt/rad
     */
    public static PIDConstants makePositionPID(
            double p) {
        return new PIDConstants(p, 0, 0, 0, 0, 0);
    }

    public static PIDConstants makePositionPID(
            double p, double i, double d) {
        return new PIDConstants(p, i, d, 0, 0, 0);
    }

    /**
     * WARNING! REV velocity control does not work well for light mechanisms (e.g.
     * flywheels), so don't do it.
     * 
     * To guess a good starting point: say you're running at 300 rad/s,
     * you'll want 10% of full-range (1 volt) for a 10% error (30 rad/s),
     * so 0.03 might work; start lower.
     * 
     * @param p Volt-sec/rad
     */
    public static PIDConstants makeVelocityPID(
            double p) {
        return new PIDConstants(0, 0, 0, p, 0, 0);
    }

    public static PIDConstants makeVelocityPID(
            double p, double i, double d) {
        return new PIDConstants(0, 0, 0, p, i, d);
    }

    /** Zero is for when you're not using the motor's PID controller */
    public static PIDConstants zero() {
        return new PIDConstants(0, 0, 0, 0, 0, 0);
    }

    /** volt/rad */
    public double getPositionPV_Rad() {
        return m_positionP;
    }

    /** volt/rad-sec */
    public double getPositionIV_RadS() {
        return m_positionI;
    }

    /** volt-sec/rad */
    public double getPositionDVS_Rad() {
        return m_positionD;
    }

    /** volt-sec/rad */
    public double getVelocityPVS_Rad() {
        return m_velocityP;
    }

    /** volt/rad */
    public double getVelocityIVolt_Rad() {
        return m_velocityI;
    }

    /** volt-sec^2/rad */
    public double getVelocityDVS2_Rad() {
        return m_velocityD;
    }

    public void register(Runnable listener) {
        m_listeners.add(listener);
    }

    //////////////////////////////////////////////////////

    public PIDConstants(
            double positionP, double positionI, double positionD,
            double velocityP, double velocityI, double velocityD) {
        m_positionP = positionP;
        m_positionI = positionI;
        m_positionD = positionD;

        m_velocityP = velocityP;
        m_velocityI = velocityI;
        m_velocityD = velocityD;
    }
}
