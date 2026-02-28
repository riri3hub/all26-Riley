package org.team100.lib.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Friction;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.reference.r1.NoReferenceR1;
import org.team100.lib.sensor.position.incremental.MockIncrementalBareEncoder;

class LinearVelocityServoTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        SimpleDynamics ff = new SimpleDynamics(logger, 0.100, 0.100);
        Friction friction = new Friction(logger, 0.100, 0.100, 0.0, 0.1);
        MockBareMotor driveMotor = new MockBareMotor(ff, friction);
        MockIncrementalBareEncoder driveEncoder = new MockIncrementalBareEncoder();
        LinearMechanism mech = new LinearMechanism(logger,
                driveMotor, driveEncoder, 1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger, mech, new NoReferenceR1(), 1);
        // 0.5 m/s
        servo.setVelocityDirect(0.5);
        // wheel radius is 0.5 m, so drive speed is 1 m/s
        assertEquals(1.0, driveMotor.velocity, 0.001);
    }
}
