package org.team100.lib.subsystems.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.function.Function;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.trajectory.TrajectorySE2Entry;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2Factory;
import org.team100.lib.trajectory.TrajectorySE2Planner;
import org.team100.lib.trajectory.constraint.ConstantConstraint;
import org.team100.lib.trajectory.constraint.TimingConstraint;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathSE2Factory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

public class JacobianTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Function<Vector<N3>, Vector<N3>> f = q -> Jacobian.pose(k.forward(Jacobian.config(q)));

        Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                Nat.N3(),
                Nat.N3(),
                f,
                Jacobian.config(new EAWConfig(1, 0, 0)));
        if (DEBUG)
            System.out.println(j);

        // dx/dh should be 1 in all positions
        assertEquals(1, j.get(0, 0), DELTA);
        // dx/dshoulder, zero in this position
        assertEquals(0, j.get(0, 1), 0.002);
        // dx/dwrist, zero in this position
        assertEquals(0, j.get(0, 2), DELTA);

        // dy/dh should be zero in all positions
        assertEquals(0, j.get(1, 0), DELTA);
        // dy/dshoulder depends on the radius (3 here)
        assertEquals(3, j.get(1, 1), DELTA);
        // dy/dwrist depends on the radius (1 here)
        assertEquals(1, j.get(1, 2), DELTA);

        // dr/dh should always be zero
        assertEquals(0, j.get(2, 0), DELTA);
        // dr/dshoulder = 1
        assertEquals(1, j.get(2, 1), DELTA);
        // dr/dwrist = 1
        assertEquals(1, j.get(2, 2), DELTA);

        // invertible
        assertEquals(2, j.det(), DELTA);

        Matrix<N3, N3> jinv = j.inv();
        // dh/dx
        assertEquals(1, jinv.get(0, 0), DELTA);
        // dh/dy
        assertEquals(0, jinv.get(0, 1), DELTA);
        // dh/dr
        assertEquals(0, jinv.get(0, 2), DELTA);

        // dshoulder/dx
        assertEquals(0, jinv.get(1, 0), DELTA);
        // dshoulder/dy, +y -> +shoulder.
        assertEquals(0.5, jinv.get(1, 1), DELTA);
        // dshoulder/dr, to rotate only, the shoulder compensates the other way
        assertEquals(-0.5, jinv.get(1, 2), DELTA);

        // dwrist/dx
        assertEquals(0, jinv.get(2, 0), DELTA);
        // dwrist/dy, +y -> shoulder moves, wrist compensates the other way
        assertEquals(-0.5, jinv.get(2, 1), DELTA);
        // dwrist/dr extra to counter the compensation
        assertEquals(1.5, jinv.get(2, 2), DELTA);

        // some example velocities
        // zero velocity
        Vector<N3> v = VecBuilder.fill(0, 0, 0);
        Vector<N3> m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), DELTA);
        assertEquals(0, m.get(1), DELTA);
        assertEquals(0, m.get(2), DELTA);

        // +x
        v = VecBuilder.fill(1, 0, 0);
        m = new Vector<>(jinv.times(v));
        assertEquals(1, m.get(0), DELTA);
        assertEquals(0, m.get(1), DELTA);
        assertEquals(0, m.get(2), DELTA);

        // +y
        v = VecBuilder.fill(0, 1, 0);
        m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), DELTA);
        assertEquals(0.5, m.get(1), DELTA);
        assertEquals(-0.5, m.get(2), DELTA);

        // +theta
        v = VecBuilder.fill(0, 0, 1);
        m = new Vector<>(jinv.times(v));
        assertEquals(0, m.get(0), DELTA);
        assertEquals(-0.5, m.get(1), DELTA);
        assertEquals(1.5, m.get(2), DELTA);
    }

    @Test
    void test05() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Jacobian j = new Jacobian(k);

        EAWConfig c = new EAWConfig(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        ModelSE2 v = new ModelSE2(p);

        JointVelocities jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +x
        v = new ModelSE2(p, new VelocitySE2(1, 0, 0));
        jv = j.inverse(v);
        assertEquals(1, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +y
        v = new ModelSE2(p, new VelocitySE2(0, 1, 0));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0.5, jv.shoulder(), DELTA);
        assertEquals(-0.5, jv.wrist(), DELTA);

        // +theta
        v = new ModelSE2(p, new VelocitySE2(0, 0, 1));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(-0.5, jv.shoulder(), DELTA);
        assertEquals(1.5, jv.wrist(), DELTA);
    }

    @Test
    void testDet() {
        // this prints a table of jacobian determinants ("scale") to show the
        // singularities at the edges.
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);

        // forward jacobian goes from config to pose
        Function<Vector<N3>, Vector<N3>> f = q -> Jacobian.pose(k.forward(Jacobian.config(q)));

        for (double x = 4; x >= 0; x -= 0.2) {
            for (double y = -2; y <= 2; y += 0.2) {
                // for now, end-effector rotation is always zero (i.e. facing up)
                Pose2d p = new Pose2d(x, y, Rotation2d.kZero);
                Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                        Nat.N3(),
                        Nat.N3(),
                        f,
                        Jacobian.config(k.inverse(p)));
                double det = j.det();
                if (DEBUG)
                    System.out.printf("%8.2f", det);
            }
            if (DEBUG)
                System.out.println();
        }
    }

    @Test
    void testPath() {
        // a very simple path showing pose, config, and velocities.
        //
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Function<Vector<N3>, Vector<N3>> f = q -> Jacobian.pose(k.forward(Jacobian.config(q)));

        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        double stepSize = 0.1;
        Pose2d prev = start;
        for (double s = 0; s <= 1.0; s += stepSize) {
            Pose2d p = GeometryUtil.interpolate(start, end, s);
            Vector<N3> tv = VecBuilder.fill(
                    p.getX() - prev.getX(),
                    p.getY() - prev.getY(),
                    p.getRotation().minus(prev.getRotation()).getRadians());
            EAWConfig c = k.inverse(p);
            Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                    Nat.N3(),
                    Nat.N3(),
                    f,
                    Jacobian.config(c));
            Matrix<N3, N3> jinv = j.inv();
            Vector<N3> cv = new Vector<>(jinv.times(tv));
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) cv(%5.2f %5.2f %5.2f)\n",
                        s,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.shoulderHeight(), c.shoulderAngle(), c.wristAngle(),
                        tv.get(0), tv.get(1), tv.get(2),
                        cv.get(0), cv.get(1), cv.get(2));
            prev = p;
        }
    }

    @Test
    void testTrajectory() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Jacobian j = new Jacobian(k);

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(logger, 1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TrajectorySE2Entry tp = t.sample(time);
            ModelSE2 sm = ModelSE2.fromMovingPathPointSE2(tp.point().point(), tp.point().velocity());
            Pose2d p = sm.pose();
            VelocitySE2 v = sm.velocity();
            EAWConfig c = k.inverse(p);
            JointVelocities jv = j.inverse(sm);
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                        time,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.shoulderHeight(), c.shoulderAngle(), c.wristAngle(),
                        v.x(), v.y(), v.theta(),
                        jv.elevator(), jv.shoulder(), jv.wrist());
        }
    }

}
