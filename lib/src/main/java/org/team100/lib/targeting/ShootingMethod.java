package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.optimization.NewtonsMethod;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/**
 * Solve the intercept problem via the "shooting method" using fixed muzzle
 * velocity and spin, with two variables:
 * 
 * * azimuth
 * * elevation
 */
public class ShootingMethod implements Solver {
    private static final boolean DEBUG = false;

    /**
     * Minimum (azimuth, elevation)
     * 
     * Very low elevation breaks the solver because the ball is below the floor in
     * one time step. Accordingly, the minimum elevation and time step need to be
     * tuned together. See RangeSolverTest for details.
     */
    private static final Vector<N2> X_MIN = VecBuilder.fill(-Math.PI, 0.1);
    /**
     * Maximum (azimuth, elevation)
     * 
     * There's no reason for elevation near vertical, so just limit it.
     */
    private static final Vector<N2> X_MAX = VecBuilder.fill(Math.PI, 1.4);
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
    /** Shooting range solver. */
    private final IRange m_range;
    /** Solution tolerance, radial distance to target in meters. */
    private final double m_tolerance;
    private final double m_initialElevation;

    public ShootingMethod(IRange range, double tolerance, double initialElevation) {
        m_range = range;
        m_tolerance = tolerance;
        m_initialElevation = initialElevation;
    }

    /**
     * Given absolute position and velocity for robot and target, produce a
     * solution for azimuth and elevation.
     * 
     * If it is possible to do so, this solver will find a "nearby" solution to the
     * initial one. Since the problem often has two solutions ("direct" low
     * elevation and "indirect" high elevation), you should choose an initial
     * elevation close to the solution you want.
     */
    public Optional<Solution> solve(
            ModelSE2 state,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        if (DEBUG)
            System.out.println("ShootingMethod.solve()");
        Translation2d robotPosition = state.translation();
        GlobalVelocityR2 robotVelocity = state.velocityR2();
        // Target relative to robot
        Translation2d T0 = targetPosition.minus(robotPosition);
        // Target velocity relative to robot
        GlobalVelocityR2 vT = targetVelocity.minus(robotVelocity);
        // Initial azimuth guess is the current target bearing.
        Vector<N2> initialX = VecBuilder.fill(
                T0.getAngle().getRadians(), m_initialElevation);
        NewtonsMethod<N2, N2> solver = new NewtonsMethod<>(
                Nat.N2(),
                Nat.N2(),
                fn(T0, vT),
                X_MIN,
                X_MAX,
                m_tolerance,
                ITERATIONS,
                DX_LIMIT);
        try {
            Vector<N2> x = solver.solve2(initialX, 3, true);
            // use zero azimuth velocity for now.
            // TODO: solve for that
            return Optional.of(
                    new Solution(
                            new Rotation2d(x.get(0)),
                            0,
                            new Rotation2d(x.get(1))));
        } catch (IllegalArgumentException ex) {
            return Optional.empty();
        }
    }

    /**
     * @return a function of (azimuth, elevation)
     *         that returns translational error (x, y)
     */
    Function<Vector<N2>, Vector<N2>> fn(Translation2d T0, GlobalVelocityR2 vT) {
        return x -> this.f(x, T0, vT);
    }

    /**
     * @param x  (azimuth, elevation)
     * @param T0 target relative position
     * @param vT target relative velocity
     * @return translational error (x, y)
     */
    Vector<N2> f(Vector<N2> x, Translation2d T0, GlobalVelocityR2 vT) {
        if (DEBUG)
            System.out.println("ShootingMethod.f()");
        // Extract contents of the state variable
        Rotation2d azimuth = new Rotation2d(x.get(0));
        double elevation = x.get(1);
        if (DEBUG)
            System.out.printf("input: azimuth %s elevation %f\n",
                    StrUtil.rotStr(azimuth), elevation);
        // Lookup for this state.
        FiringSolution rangeSolution = m_range.get(elevation);
        if (DEBUG)
            System.out.printf("solution: %s\n", rangeSolution);
        // Ball location at impact, relative to initial robot position
        Translation2d b = new Translation2d(rangeSolution.range(), azimuth);
        // target location at impact, relative to initial robot position
        Translation2d T = vT.integrate(T0, rangeSolution.tof());
        Translation2d err = b.minus(T);
        if (DEBUG)
            System.out.printf("error: %s\n", StrUtil.transStr(err));
        // result error is (x, y)
        return GeometryUtil.toVec(err);
    }

}
