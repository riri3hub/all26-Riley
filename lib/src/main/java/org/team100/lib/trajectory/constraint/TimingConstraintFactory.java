package org.team100.lib.trajectory.constraint;

import java.util.List;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

public class TimingConstraintFactory {
    private final SwerveKinodynamics m_limits;

    public TimingConstraintFactory(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    /**
     * Return wheel speed, yaw rate, and centripetal acceleration constraints with
     * reasonable scale to prevent too-fast spinning and to slow down in sharp
     * curves. The velocity is set to half of maximum.
     * 
     * If you want to adjust these scales, make a new factory method while you
     * figure out what you need, don't just change them here.
     */
    public List<TimingConstraint> allGood(LoggerFactory log) {
        return scaled(log, 0.5, 0.5, 0.2, 0.2);
    }

    /** Absolute maximum. Probably too fast to actually use. */
    public List<TimingConstraint> fast(LoggerFactory log) {
        return scaled(log, 1.0, 1.0, 1.0, 0.25);
    }

    /** Very slow, 25% speed. */
    public List<TimingConstraint> slow(LoggerFactory log) {
        return scaled(log, 0.25, 0.25, 0.25, 0.25);
    }

    /** Maybe unrealistically fast? */
    public List<TimingConstraint> medium(LoggerFactory log) {
        return scaled(log, 0.75, 1, 0.75, 0.25);
    }

    /** Maybe unrealistically fast? */
    public List<TimingConstraint> auto(LoggerFactory log) {
        return scaled(log, 0.85, 0.8, 0.95, 0.75);
    }

    /** see TrajectoryVelocityProfileTest.testAuto() */
    public List<TimingConstraint> testAuto(LoggerFactory log) {
        return scaled(log, 1, 1, 1, 1);
    }

    /**
     * Use absolute max as the constraints. Shouldn't be used on a real robot.
     */
    public List<TimingConstraint> forTest(LoggerFactory log) {
        return scaled(log, 1.0, 1.0, 1.0, 1.0);
    }

    private List<TimingConstraint> scaled(
            LoggerFactory log,
            double vScale,
            double aScale,
            double centripetalScale,
            double yawRateScale) {
        return List.of(
                new ConstantConstraint(log, vScale, aScale, m_limits),
                new SwerveDriveDynamicsConstraint(log, m_limits, vScale, aScale),
                new YawRateConstraint(log, m_limits, yawRateScale),
                new CapsizeAccelerationConstraint(log, m_limits, centripetalScale));
    }

}
