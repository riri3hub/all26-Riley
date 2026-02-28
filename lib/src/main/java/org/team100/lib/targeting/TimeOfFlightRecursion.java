package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Time-of-flight recursion, as described by @oblarg is an iterative approach
 * that works as follows:
 * 
 * 1. shoot at the target location
 * 2. evolve the target location based on the TOF of that shot
 * 3. repeat until the shooting solution doesn't change
 * 
 * see
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815/21?u=truher
 */
public class TimeOfFlightRecursion implements Solver {
    private static final boolean DEBUG = false;

    /** Look up solution parameters for range. */
    private final DoubleFunction<FiringParameters> m_inverseRange;
    /** Solution TOF tolerance, seconds. */
    private final double m_tolerance;

    /**
     * @param inverseRange FiringParameters as a function of desired range
     * @param tolerance    complete when the solution doesn't change more than this,
     *                     in seconds
     */
    public TimeOfFlightRecursion(
            DoubleFunction<FiringParameters> inverseRange,
            double tolerance) {
        m_inverseRange = inverseRange;
        m_tolerance = tolerance;
    }

    @Override
    public Optional<Solution> solve(
            ModelSE2 state,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        Translation2d robotPosition = state.translation();
        GlobalVelocityR2 robotVelocity = state.velocityR2();
        // Target relative to robot
        Translation2d T0 = targetPosition.minus(robotPosition);
        // Target velocity relative to robot
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);

        double range = T0.getNorm();
        double tof = m_inverseRange.apply(range).tof();
        double iter = 100;
        while (iter-- > 0) {
            Translation2d impact = vT.integrate(T0, tof);
            range = impact.getNorm();
            FiringParameters p2 = m_inverseRange.apply(range);
            if (DEBUG)
                System.out.printf("range %f elevation %f tof %f\n", range, p2.elevation(), p2.tof());
            if (Math.abs(tof - p2.tof()) < m_tolerance) {
                // Found a good solution!
                double targetMotion = TargetUtil.targetMotion(state, impact);
                return Optional.of(new Solution(
                        impact.getAngle(),
                        targetMotion,
                        new Rotation2d(p2.elevation())));
            }
            tof = p2.tof();
        }
        return Optional.empty();
    }

}
