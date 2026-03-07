package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.optimization.GradientDescent;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Solve the intercept problem via the "shooting method" using fixed spin and
 * three variables:
 * 
 * * azimuth
 * * muzzle velocity
 * * elevation
 * 
 * For each target problem, there is a continuum of solutions with different
 * velocities and elevations, so we need a reason to choose a particular one.
 * 
 * For example, if we want the ball to pass through a horizontal hoop (like in
 * basketball), the inbound path needs to be close enough to vertical so that
 * the ball goes into a hoop, avoiding a "clank" bounce-out.
 * 
 * Another option is to aim for the minimium muzzle velocity that can solve the
 * problem: that way the shooter will be ready faster, and take less battery
 * power.
 * 
 * We can combine those two criteria into a "loss function" that penalizes
 * extra velocity and insufficient elevation.
 * 
 * Since we're minimizing a scalar loss rather than finding the zero, the
 * natural solver is gradient descent.
 */
@SuppressWarnings("unused")
public class VariableVelocityShootingMethod {
    private static final boolean DEBUG = false;

    public record Solution(Rotation2d azimuth, double velocity, Rotation2d elevation) {
    }

    /** Minimum (azimuth, velocity, elevation) */
    private final Vector<N3> m_xMin;
    /** Maximum (azimuth, velocity, elevation) */
    private final Vector<N3> m_xMax;
    /**
     * In between iterations, the solver chooses a random starting point within the
     * bounds above.
     */
    private static final int ITERATIONS = 10;
    /**
     * Maximum step in x per iteration. This keeps very low gradient from pushing
     * the solution far away.
     */
    private static final double DX_LIMIT = 0.1;

    private final IVVRange m_range;
    private final double m_tolerance;

    public VariableVelocityShootingMethod(
            IVVRange range, double minElevation, double maxElvation, double tolerance) {
        m_range = range;
        m_xMin = VecBuilder.fill(-Math.PI, 3, minElevation);
        m_xMax = VecBuilder.fill(Math.PI, 20, maxElvation);
        m_tolerance = tolerance;
    }

    /**
     * Given inputs:
     * 
     * * absolute position and velocity for robot and target,
     * * target elevation
     * 
     * produce a solution for (azimuth, muzzle velocity, and elevation).
     */
    public Optional<Solution> solve(
            Translation2d robotPosition,
            GlobalVelocityR2 robotVelocity,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity,
            double targetElevation,
            double initialElevation) {
        Translation2d T0 = targetPosition.minus(robotPosition);
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);
        Vector<N3> initialX = VecBuilder.fill(
                T0.getAngle().getRadians(), 5, initialElevation);
        GradientDescent<N3> solver2 = new GradientDescent<>(
                Nat.N3(),
                fn2(T0, vT, targetElevation),
                m_tolerance,
                ITERATIONS);

        // NewtonsMethod<N3, N3> solver = new NewtonsMethod<>(
        // Nat.N3(),
        // Nat.N3(),
        // fn(T0, vT, targetElevation),
        // m_xMin,
        // m_xMax,
        // m_tolerance,
        // ITERATIONS,
        // DX_LIMIT);
        try {
            // Vector<N3> x = solver.solve2(initialX, 3, true);
            Vector<N3> x = solver2.solve(initialX);
            return Optional.of(
                    new Solution(
                            new Rotation2d(x.get(0)),
                            x.get(1),
                            new Rotation2d(x.get(2))));
        } catch (IllegalArgumentException ex) {
            if (DEBUG)
                ex.printStackTrace();
            return Optional.empty();
        }
    }

    Function<Vector<N3>, Double> fn2(Translation2d T0,
            GlobalVelocityR2 vT,
            double targetElevation) {
        return x -> this.f2(x, T0, vT, targetElevation);
    }

    /**
     * @return a function of (azimuth, velocity, elevation)
     *         that returns error (x, y, targetElevation)
     */
    // Function<Vector<N3>, Vector<N3>> fn(
    // Translation2d T0,
    // GlobalVelocityR2 vT,
    // double targetElevation) {
    // return x -> this.f(x, T0, vT, targetElevation);
    // }

    /**
     * @param x                  (azimuth, velocity, elevation)
     * @param T0                 target relative position
     * @param vT                 target relative velocity
     * @param minTargetElevation minimum arrival path elevation
     * @return error (x, y, angle)
     */
    // Vector<N3> f(
    // Vector<N3> x,
    // Translation2d T0,
    // GlobalVelocityR2 vT,
    // double minTargetElevation) {
    // if (DEBUG)
    // System.out.printf("x %s\n", StrUtil.vecStr(x));
    // Rotation2d azimuth = new Rotation2d(x.get(0));
    // double v = x.get(1);
    // double elevation = x.get(2);
    // Interception rangeSolution = m_range.get(v, elevation);
    // if (DEBUG)
    // System.out.printf("soln %s\n", rangeSolution);
    // Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
    // Translation2d T = vT.integrate(T0, rangeSolution.tof());
    // Translation2d err = b.minus(T);
    // double gamma = rangeSolution.targetElevation();
    // double elevationErr = gamma - minTargetElevation;
    // if (DEBUG)
    // System.out.printf("gamma %f err %f\n", gamma, elevationErr);
    // return VecBuilder.fill(err.getX(), err.getY(), elevationErr);
    // }

    double f2(Vector<N3> x,
            Translation2d T0,
            GlobalVelocityR2 vT,
            double minTargetElevation) {
        if (DEBUG)
            System.out.printf("x %s\n", StrUtil.vecStr(x));
        Rotation2d azimuth = new Rotation2d(x.get(0));
        double v = x.get(1);
        double elevation = x.get(2);
        Interception rangeSolution = m_range.get(v, elevation);
        if (DEBUG)
            System.out.printf("soln %s\n", rangeSolution);
        Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
        Translation2d T = vT.integrate(T0, rangeSolution.tof());
        Translation2d err = b.minus(T);
        double gamma = rangeSolution.targetElevation();
        double elevationErr = gamma - minTargetElevation;
        if (DEBUG)
            System.out.printf("gamma %f err %f\n", gamma, elevationErr);
        double loss = err.getNorm() + Math.abs(elevationErr);
        return loss;
    }

}
