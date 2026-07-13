package org.team100.lib.trajectory.constraint;

import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.path.PathSE2Point;

/** Trivial constraint for testing. */
public class ConstantConstraint implements TimingConstraint {
    private final double m_maxVelocity;
    private final double m_maxAccel;

    public ConstantConstraint(double maxV, double maxA) {
        m_maxVelocity = maxV;
        m_maxAccel = maxA;
    }

    /**
     * @param log
     * @param vScale cartesian velocity scale
     * @param aScale cartesian acceleration scale
     * @param limits absolute maxima
     */
    public ConstantConstraint(
            double vScale,
            double aScale,
            SwerveKinodynamics limits) {
        this(
                vScale * limits.getMaxDriveVelocityM_S(),
                aScale * limits.getMaxDriveAccelerationM_S2());
    }

    @Override
    public double maxV(PathSE2Point point) {
        return m_maxVelocity;
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocityM_S) {
        return m_maxAccel;
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return -m_maxAccel;
    }
}
