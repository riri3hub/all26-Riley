package org.team100.lib.kinematics.pr;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.pr.PRAcceleration;
import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;

import edu.wpi.first.math.geometry.Translation2d;

public class PRKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testForward() {
        PRKinematics k = new PRKinematics(1);
        // at the origin, arm points 1m up
        Translation2d f = k.forward(new PRConfig(0, 0));
        assertEquals(1, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
        // raise 1m, so arm is 2m up
        f = k.forward(new PRConfig(1, 0));
        assertEquals(2, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
        // straight out
        f = k.forward(new PRConfig(0, Math.PI / 2));
        assertEquals(0, f.getX(), DELTA);
        assertEquals(1, f.getY(), DELTA);
        // 45 degrees
        f = k.forward(new PRConfig(0, Math.PI / 4));
        assertEquals(0.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
        // 45 degrees
        f = k.forward(new PRConfig(1, Math.PI / 4));
        assertEquals(1.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
    }

    @Test
    void testInverse() {
        PRKinematics k = new PRKinematics(1);
        // at the origin, arm points 1m up
        PRConfig j = k.inverse(new Translation2d(1, 0));
        assertEquals(0, j.q1(), DELTA);
        assertEquals(0, j.q2(), DELTA);
        // raise 1m, arm points 2m up
        j = k.inverse(new Translation2d(2, 0));
        assertEquals(1, j.q1(), DELTA);
        assertEquals(0, j.q2(), DELTA);
        // straight out
        j = k.inverse(new Translation2d(0, 1));
        assertEquals(0, j.q1(), DELTA);
        assertEquals(Math.PI / 2, j.q2(), DELTA);
        // 45 degrees?
        j = k.inverse(new Translation2d(0.707107, 0.707107));
        assertEquals(0, j.q1(), DELTA);
        assertEquals(Math.PI / 4, j.q2(), DELTA);
        // 45 degrees?
        j = k.inverse(new Translation2d(1.707107, 0.707107));
        assertEquals(1, j.q1(), DELTA);
        assertEquals(Math.PI / 4, j.q2(), DELTA);
        // these are unreachable
        assertNull(k.inverse(new Translation2d(0, 2)));
        assertNull(k.inverse(new Translation2d(0, -2)));
    }

    @Test
    void testForwardV() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = k.forward(new PRConfig(0, 0), new PRVelocity(1, 0));
        verify(1, 0, xdot);
        xdot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 1));
        verify(0, 1, xdot);
        xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(1, 0));
        verify(1, 0, xdot);
        xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 1));
        verify(-1, 0, xdot);
    }

    @Test
    void testInverseV() {
        PRKinematics k = new PRKinematics(1);
        PRVelocity qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(1, 0));
        verify(1, 0, qdot);
        qdot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(1, 0));
        verify(0, -1, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = k.inverse(new Translation2d(1, 1), new VelocityR2(-1, 1));
        verify(0, 0, qdot);
    }

    @Test
    void testForwardA() {
        PRKinematics k = new PRKinematics(1);
        AccelerationR2 xddot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 0), new PRAcceleration(0, 0));
        verify(0, 0, xddot);
        // centripetal
        xddot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 1), new PRAcceleration(0, 0));
        verify(-1, 0, xddot);
    }

    @Test
    void testInverseA() {
        PRKinematics k = new PRKinematics(1);
        PRAcceleration qddot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 0), new AccelerationR2(0, 0));
        verify(0, 0, qddot);
        // P extends to support +y
        qddot = k.inverse(new Translation2d(2, 0), new VelocityR2(0, 1), new AccelerationR2(0, 0));
        verify(1, 0, qddot);
    }

    void verify(double x, double y, VelocityR2 v) {
        assertEquals(x, v.x(), 1e-3);
        assertEquals(y, v.y(), 1e-3);
    }

    void verify(double x, double y, AccelerationR2 a) {
        assertEquals(x, a.x(), 1e-3);
        assertEquals(y, a.y(), 1e-3);
    }

    void verify(double q1dot, double q2dot, PRVelocity v) {
        assertEquals(q1dot, v.q1dot(), 1e-3);
        assertEquals(q2dot, v.q2dot(), 1e03);
    }

    void verify(double q1dot, double q2dot, PRAcceleration a) {
        assertEquals(q1dot, a.q1ddot(), 1e-3);
        assertEquals(q2dot, a.q2ddot(), 1e03);
    }

}
