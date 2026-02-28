package org.team100.lib.subsystems.swerve.module;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.servo.MockAngularPositionServo;
import org.team100.lib.servo.MockLinearVelocityServo;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule100Test {
    private static final double DELTA = 0.001;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        MockLinearVelocityServo s = new MockLinearVelocityServo();
        MockAngularPositionServo a = new MockAngularPositionServo();
        SwerveModule100 m = new SwerveModule100(log, s, a, 0.1, 5) {
        };
        SwerveModuleState100 s1 = new SwerveModuleState100(1, Optional.of(new Rotation2d()));
        SwerveModuleState100 s2 = new SwerveModuleState100(1, Optional.of(new Rotation2d(1)));
        m.setDesiredState(s1);
        assertEquals(1, s.getVelocity(), DELTA);
        assertEquals(0, a.getWrappedPositionRad(), DELTA);
        m.setDesiredState(s2);
        assertEquals(1, s.getVelocity(), DELTA);
        assertEquals(1, a.getWrappedPositionRad(), DELTA);
    }
}
