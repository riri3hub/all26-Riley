package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Friction model for static, dynamic, and viscous friction.
 * 
 * Applicable for motor feedforward.
 * 
 * These values describe the entire mechanism; the motor friction itself is
 * usually negligible.
 * 
 * @see https://mogi.bme.hu/TAMOP/robot_applications/ch07.html
 * @see https://en.wikipedia.org/wiki/Friction
 * @see https://en.wikipedia.org/wiki/Stribeck_curve
 * @see https://engee.com/helpcenter/stable/en/fmod-mechanical-translational-elements/translational-friction.html
 */
public class Friction {
    /** Volts */
    private final Mutable kS;
    /** Volts */
    private final Mutable kD;
    /** Volt-sec/rad */
    private final Mutable kV;
    /** rad/sec */
    private final double vS;

    /**
     * @param log for mutables
     * @param kS  Static friction. Voltage to just barely get the mechanism moving
     *            from a stop.
     * @param kD  Dynamic friction. Voltage to just barely keep the mechanism
     *            moving, independent of speed.
     * @param kV  Viscous friction. Constant to compute voltage to keep moving at a
     *            constant velocity. Units are Volt-sec/rad.
     * @param vS  Velocity threshold for static friction, rad/s.
     */
    public Friction(
            LoggerFactory log,
            double kS,
            double kD,
            double kV,
            double vS) {
        if (kS < kD)
            throw new IllegalArgumentException("static friction is always at least as high as dynamic friction");
        LoggerFactory fLog = log.type(this);
        this.kS = new Mutable(fLog, "kS", kS);
        this.kD = new Mutable(fLog, "kD", kD);
        this.kV = new Mutable(fLog, "kV", kV);
        this.vS = vS;
    }

    /**
     * Voltage to balance friction (i.e. this has the same sign as the supplied
     * speed).
     * 
     * Includes viscous friction (proportional to speed), dynamic friction (constant
     * while moving), and static friction (constant while almost stopped).
     * 
     * @param motorRad_S setpoint speed rad/s
     */
    public double frictionFFVolts(double motorRad_S) {
        double viscous = kV.getAsDouble() * motorRad_S;
        double direction = Math.signum(motorRad_S);
        if (Math.abs(motorRad_S) < vS) {
            return viscous + kS.getAsDouble() * direction;
        }
        return viscous + kD.getAsDouble() * direction;
    }
}
