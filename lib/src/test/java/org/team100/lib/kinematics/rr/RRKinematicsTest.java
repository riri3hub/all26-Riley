package org.team100.lib.kinematics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;
import org.team100.lib.geometry.rr.RRVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class RRKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testf1() {
        // stretched along x
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(1, 0), new Translation2d(2, 0));
        RRConfig q = new RRConfig(0, 0);
        verify(k, p, q);
    }

    @Test
    void testf2() {
        // up and then out
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(0, 1), new Translation2d(1, 1));
        RRConfig q = new RRConfig(Math.PI / 2, -1 * Math.PI / 2);
        verify(k, p, q);
    }

    @Test
    void testf3() {
        // equilateral triangle, first link up
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(0, 1), new Translation2d(Math.sqrt(3) / 2, 0.5));
        RRConfig q = new RRConfig(Math.PI / 2, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test4() {
        // vertical equilateral triangle
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(-Math.sqrt(3) / 2, 0.5), new Translation2d(0, 1));
        RRConfig q = new RRConfig(5 * Math.PI / 6, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test5() {
        // behind
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(-1, 0), new Translation2d(-1, 1));
        RRConfig q = new RRConfig(Math.PI, -Math.PI / 2);
        verify(k, p, q);
    }

    @Test
    void testForwardV() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        verify(0, 2, xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        verify(0, 1, xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(1, 0));
        verify(-1, 1, xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 1));
        verify(0, 1, xdot);
    }

    @Test
    void testInverseV() {
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(1, 0));
        verify(0, 0, qdot);
        qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(1, 0));
        verify(-1, 0, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(-1, 1));
        verify(1, 0, qdot);
    }

    @Test
    void testInverseA() {
        RRKinematics k = new RRKinematics(1, 1);
        RRAcceleration qddot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 0), new AccelerationR2(0, 0));
        verify(0, 0, qddot);
        // steady +x does not require accel.
        qddot = k.inverse(new Translation2d(1, 1), new VelocityR2(1, 0), new AccelerationR2(0, 0));
        verify(0, 0, qddot);
        // steady +y requires shoulder accel
        qddot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 1), new AccelerationR2(0, 0));
        verify(-1, 0, qddot);
    }

    @Test
    void testForwardA() {
        RRKinematics k = new RRKinematics(1, 1);
        AccelerationR2 xddot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0), new RRAcceleration(0, 0));
        verify(0, 0, xddot);
        // move shoulder: centripetal towards shoulder
        xddot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(1, 0), new RRAcceleration(0, 0));
        verify(-1, -1, xddot);
        // move elbow: centripetal towards elbow
        xddot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 1), new RRAcceleration(0, 0));
        verify(-1, 0, xddot);
    }

    void verify(RRKinematics k, RRPosition p, RRConfig q) {
        verifyFwd(p, k.forward(q));
        verifyInv(q, k.inverse(p.p2()));
    }

    void verifyFwd(RRPosition expected, RRPosition actual) {
        assertEquals(expected.p1(), actual.p1(), "fwd p1");
        assertEquals(expected.p2(), actual.p2(), "fwd p2");
    }

    void verifyInv(RRConfig expected, RRConfig actual) {
        assertEquals(MathUtil.angleModulus(expected.q1()), MathUtil.angleModulus(actual.q1()), DELTA, "inv q1");
        assertEquals(MathUtil.angleModulus(expected.q2()), MathUtil.angleModulus(actual.q2()), DELTA, "inv q2");
    }

    void verify(double x, double y, VelocityR2 v) {
        assertEquals(x, v.x(), 1e-3);
        assertEquals(y, v.y(), 1e-3);
    }

    void verify(double x, double y, AccelerationR2 v) {
        assertEquals(x, v.x(), 1e-3);
        assertEquals(y, v.y(), 1e-3);
    }

    void verify(double q1dot, double q2dot, RRVelocity v) {
        assertEquals(q1dot, v.q1dot(), 1e-3);
        assertEquals(q2dot, v.q2dot(), 1e03);
    }

    void verify(double q1dot, double q2dot, RRAcceleration a) {
        assertEquals(q1dot, a.q1ddot(), 1e-3);
        assertEquals(q2dot, a.q2ddot(), 1e03);
    }
}