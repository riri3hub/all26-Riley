package org.team100.lib.kinematics.pr;

import org.team100.lib.geometry.pr.PRAcceleration;
import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/**
 * Kinematics for the arm/elevator combination.
 * 
 * This system is represented by two measurements: height of the pivot, and
 * angle of the arm. The angle is measured from vertical. There are physical
 * limits in the real system; these are not represented here.
 * 
 * The cartesian coordinate orientation is with X vertical.
 */
public class PRKinematics {
    /** Rotating arm length, meters */
    private final double l;

    /**
     * @param l Rotating arm length, meters
     */
    public PRKinematics(double l) {
        this.l = l;
    }

    /**
     * Forward position kinematics: cartesian position from joint configuration.
     * 
     * x = f(q)
     */
    public Translation2d forward(PRConfig q) {
        double x = q.q1() + l * Math.cos(q.q2());
        double y = l * Math.sin(q.q2());
        return new Translation2d(x, y);
    }

    /**
     * Forward velocity kinematics: cartesian velocity from joint configuration and
     * velocity.
     * 
     * \dot{x} = J(q) \dot{q}
     */
    public VelocityR2 forward(PRConfig q, PRVelocity qdot) {
        Matrix<N2, N2> J = J(q);
        return VelocityR2.fromVector2(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J}\dot{q} + J\ddot{q}
     */
    public AccelerationR2 forward(
            PRConfig q, PRVelocity qdot, PRAcceleration qddot) {
        Matrix<N2, N2> J = J(q);
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return AccelerationR2.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse position kinematics: joint configuration from cartesian position.
     * 
     * The inverse kinematics are not unique: there are two ways to get to almost
     * all points in the envelope: the "arm pointing up" orientation and the "arm
     * pointing down" orientation. The use of arcsin below prefers the "arm pointing
     * up" case.
     * 
     * There are also unreachable points outside the envelope; in that case we
     * return null.
     */
    public PRConfig inverse(Translation2d t) {
        double x = t.getX();
        double y = t.getY();
        double q2 = Math.asin(y / l);
        double q1 = x - Math.sqrt(l * l - y * y);
        if (Double.isNaN(q2) || Double.isNaN(q1))
            return null;
        return new PRConfig(q1, q2);
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1}(x) \dot{x}
     */
    public PRVelocity inverse(Translation2d x, VelocityR2 xdot) {
        PRConfig q = inverse(x);
        Matrix<N2, N2> Jinv = Jinv(q);
        return PRVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
     * 
     * See doc/README.md equation 9
     */
    public PRAcceleration inverse(Translation2d x, VelocityR2 xdot, AccelerationR2 xddot) {
        PRConfig q = inverse(x);
        Matrix<N2, N2> Jinv = Jinv(q);
        PRVelocity qdot = PRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return PRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    //////////////////////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     */
    private Matrix<N2, N2> J(PRConfig q) {
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
                1, -l * s2, //
                0, l * c2);
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     */
    private Matrix<N2, N2> Jdot(PRConfig q, PRVelocity qdot) {
        double c2 = Math.cos(q.q2());
        double s2 = Math.sin(q.q2());
        double q2dot = qdot.q2dot();
        return MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, -l * c2 * q2dot, //
                0, -l * s2 * q2dot);
    }

    /**
     * Inverse Jacobian, or zero if singular.
     */
    private Matrix<N2, N2> Jinv(PRConfig q) {
        Matrix<N2, N2> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            // not invertible
            System.out.printf("WARNING: zero jacobian for config %s\n", q.toString());
            return new Matrix<>(Nat.N2(), Nat.N2());
        }
        return J.inv();
    }

}
