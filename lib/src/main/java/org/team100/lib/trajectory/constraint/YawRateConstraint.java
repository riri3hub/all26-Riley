package org.team100.lib.trajectory.constraint;

import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.path.PathSE2Point;

/**
 * Linear velocity limit based on spatial yaw rate, drivetrain omega limit
 * (scaled), and drivetrain alpha limit (scaled).
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 * 
 * Does not affect maximum acceleration.
 */
public class YawRateConstraint implements TimingConstraint {
    private final double m_maxOmegaRad_S;
    private final double m_maxAlphaRad_S2;

    public YawRateConstraint(double maxOmega, double maxAlpha) {
        m_maxOmegaRad_S = maxOmega;
        m_maxAlphaRad_S2 = maxAlpha;
    }

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum angular speed to get the actual
     *               constraint. The absolute maximum yaw rate is *very* high, and
     *               never useful for trajectories. A good number to try here might
     *               be 0.2.
     */
    public YawRateConstraint(SwerveKinodynamics limits, double scale) {
        this(limits.getMaxAngleSpeedRad_S() * scale, limits.getMaxAngleAccelRad_S2() * scale);
    }

    @Override
    public double maxV(PathSE2Point point) {
        // Heading rate in rad/m
        double heading_rate = point.waypoint().course().headingRate();
        // rad/s / rad/m => m/s.
        return m_maxOmegaRad_S / Math.abs(heading_rate);
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocity) {
        // Heading rate in rad/m
        double heading_rate = point.waypoint().course().headingRate();
        // rad/s^2 / rad/m => m/s^2
        return m_maxAlphaRad_S2 / Math.abs(heading_rate);
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        // Heading rate in rad/m
        double heading_rate = point.waypoint().course().headingRate();
        // rad/s^2 / rad/m => m/s^2
        return -(m_maxAlphaRad_S2 / Math.abs(heading_rate));
    }

}