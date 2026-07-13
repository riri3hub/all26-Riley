package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

public class FieldRelativeVelocityLimiterTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testDrive() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(5, k.getMaxDriveVelocityM_S(), DELTA);
        VelocitySE2 s = new VelocitySE2(10, 0, 0);
        VelocitySE2 i = limiter.proportional(s);
        assertEquals(5, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(0, i.theta(), DELTA);
    }

    @Test
    void testSpin() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), DELTA);
        VelocitySE2 target = new VelocitySE2(0, 0, -20);
        VelocitySE2 i = limiter.proportional(target);
        assertEquals(0, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(-14.142, i.theta(), DELTA);

    }

    @Test
    void testMotionless() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), DELTA);
        VelocitySE2 t = new VelocitySE2(0, 0, 0);
        VelocitySE2 i = l.preferRotation(t);
        assertEquals(0, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(0, i.theta(), DELTA);

    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        {
            // inside the envelope => no change
            VelocitySE2 target = new VelocitySE2(1, 0, 1);
            VelocitySE2 i = l.preferRotation(target);
            assertEquals(1, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(1, i.theta(), DELTA);
        }
        {
            // full v, half omega => half v
            VelocitySE2 target = new VelocitySE2(5, 0, 7.05);
            VelocitySE2 i = l.preferRotation(target);
            assertEquals(2.507, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(7.05, i.theta(), DELTA);
        }
        {
            // full v, full omega => zero v, sorry
            VelocitySE2 target = new VelocitySE2(5, 0, 14.142);
            VelocitySE2 i = l.preferRotation(target);
            assertEquals(0, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(14.142, i.theta(), DELTA);
        }
    }

    /** shouldn't allow any movement at 6v. */
    @Test
    void testBrownout() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 6);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        VelocitySE2 target = new VelocitySE2(1, 0, 0);
        VelocitySE2 limited = limiter.apply(target);
        assertEquals(0, limited.x(), DELTA);
        assertEquals(0, limited.y(), DELTA);
        assertEquals(0, limited.theta(), DELTA);
    }

    /** desaturation should keep the same instantaneous course. */
    @Test
    void testDesaturationCourseInvariant() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);

        { // both motionless
            VelocitySE2 target = new VelocitySE2(0, 0, 0);
            assertTrue(target.angle().isEmpty());
            VelocitySE2 desaturated = l.apply(target);
            assertTrue(desaturated.angle().isEmpty());
        }
        { // translating ahead
            VelocitySE2 target = new VelocitySE2(10, 0, 0);
            assertEquals(0, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            VelocitySE2 desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(5, desaturated.norm(), DELTA);
        }
        { // translating ahead and spinning
            VelocitySE2 target = new VelocitySE2(10, 0, 10);
            assertEquals(0, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            assertEquals(10, target.theta(), DELTA);
            VelocitySE2 desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(3.694, desaturated.norm(), DELTA);
            assertEquals(3.694, desaturated.theta(), DELTA);
        }
        { // translating 45 to the left and spinning
            VelocitySE2 target = new VelocitySE2(10 / Math.sqrt(2), 10 / Math.sqrt(2), 10);
            assertEquals(Math.PI / 4, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            assertEquals(10, target.theta(), DELTA);
            VelocitySE2 desaturated = l.apply(target);
            assertEquals(Math.PI / 4, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(2.612, desaturated.x(), DELTA);
            assertEquals(2.612, desaturated.y(), DELTA);
            // slightly different due to drivetrain geometry
            assertEquals(3.693, desaturated.norm(), DELTA);
            assertEquals(3.693, desaturated.theta(), DELTA);
        }
    }

    @Test
    void driveAndSpinLimited() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.limiting();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        VelocitySE2 target = new VelocitySE2(5, 0, 25);
        assertEquals(5, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(25, target.theta(), DELTA);
        // the spin is 5x the drive, as requested.
        // use the target as the previous setpoint
        VelocitySE2 setpoint = limiter.apply(target);
        assertEquals(1.806, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(9.032, setpoint.theta(), DELTA);
    }

}
