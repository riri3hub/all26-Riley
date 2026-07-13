package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

public class FieldRelativeCapsizeLimiterTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testScale() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), DELTA);
        // below the limit, scale = 1
        assertEquals(1, limiter.scale(1), DELTA);
        // at the limit, scale = 1
        assertEquals(1, limiter.scale(8.166), DELTA);
        // target is double the limit so scale is 0.5
        assertEquals(0.5, limiter.scale(16.332), DELTA);
    }

    /**
     * initial = target => zero delta v => no constraint
     */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        VelocitySE2 result = limiter.apply(
                new VelocitySE2(0, 0, 0),
                new VelocitySE2(0, 0, 0));
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testInline() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), DELTA);
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        VelocitySE2 result = limiter.apply(
                new VelocitySE2(0, 0, 0),
                new VelocitySE2(1, 0, 0));
        // 0.163 is 8.166 * 0.02
        assertEquals(0.163, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    /**
     * initial (1,0) target (0,1) => delta v is 1.414 m/s.
     * accel is ~70, way over the limit of around 8
     * allowed deltav in 0.02 is 0.163, so the resulting speed should
     * be quite close to the initial value.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), DELTA);
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        VelocitySE2 result = limiter.apply(
                new VelocitySE2(1, 0, 0),
                new VelocitySE2(0, 1, 0));
        assertEquals(0.884, result.x(), DELTA);
        assertEquals(0.115, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testLowCentripetal() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.lowCapsize();
        assertEquals(1.225, limits.getMaxCapsizeAccelM_S2(), DELTA);
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        VelocitySE2 result = limiter.apply(
                new VelocitySE2(1, 0, 0),
                new VelocitySE2(0, 1, 0));
        assertEquals(0.982, result.x(), DELTA);
        assertEquals(0.017, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testOverspeedCentripetal() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        assertEquals(8.166, limits.getMaxCapsizeAccelM_S2(), DELTA);
        FieldRelativeCapsizeLimiter limiter = new FieldRelativeCapsizeLimiter(logger, limits);
        VelocitySE2 result = limiter.apply(
                new VelocitySE2(5, 0, 0),
                new VelocitySE2(0, 5, 0));
        assertEquals(4.884, result.x(), DELTA);
        assertEquals(0.115, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

}
