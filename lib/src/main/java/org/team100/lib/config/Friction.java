package org.team100.lib.config;

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
    private final double kS;
    /** Volts */
    private final double kD;
    /** Volt-sec/rad */
    private final double kV;
    /** rad/sec */
    private final double vS;

    /**
     * @param kS  Static friction. Voltage to just barely get the mechanism moving
     *            from a stop.
     * @param kD  Dynamic friction. Voltage to just barely keep the mechanism
     *            moving, independent of speed.
     * @param kV  Viscous friction. Constant to compute voltage to keep moving at a
     *            constant velocity. Units are Volt-sec/rad.
     * @param vS  Velocity threshold for static friction, rad/s.
     */
    public Friction(
            double kS,
            double kD,
            double kV,
            double vS) {
        if (kS < kD)
            throw new IllegalArgumentException("static friction is always at least as high as dynamic friction");
        this.kS = kS;
        this.kD = kD;
        this.kV = kV;
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
        double viscous = kV * motorRad_S;
        double direction = Math.signum(motorRad_S);
        if (Math.abs(motorRad_S) < vS) {
            return viscous + kS * direction;
        }
        return viscous + kD * direction;
    }
}
