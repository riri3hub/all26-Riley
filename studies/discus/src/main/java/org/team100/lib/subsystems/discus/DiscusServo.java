package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Discus version that uses the "servo" abstraction, which
 * provides profiled motion control.
 */
public class DiscusServo extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double VELOCITY_TOLERANCE = 0.05;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;
    private static final double MAX_VELOCITY = 10; // rad/s
    private static final double MAX_ACCEL = 20; // rad/s/s

    private final ProxyRotaryPositionSensor m_sensorP1;
    private final AngularPositionServo m_servoP1;

    public DiscusServo(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);
        LoggerFactory loggerP1 = logger.name("p1");

        // zeros
        // PID = 1.0, 0.0, 0.05
        PIDConstants pid = PIDConstants.makePositionPID(logger, 1.0, 0, 0); // d = 0.12 was experimentally found
        SimpleDynamics ff = new SimpleDynamics(logger, 0, 0);
        Friction friction = new Friction(logger, 0.16, 0.15 , 0, 0);
        ProfileR1 profile = new TrapezoidProfileR1(
                logger, MAX_VELOCITY, MAX_ACCEL, POSITION_TOLERANCE);
        ReferenceR1 refP1 = new ProfileReferenceR1(
                loggerP1, () -> profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);

        BareMotor motorP1;
        switch (Identity.instance) {
            case TEAM100_2018 -> {
                motorP1 = new Falcon500Motor(
                        loggerP1,
                        currentLog,
                        new CanId(36),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        ff,
                        friction,
                        pid);
            }
            default -> {
                motorP1 = new SimulatedBareMotor(loggerP1, 600);
            }
        }
        m_sensorP1 = new ProxyRotaryPositionSensor(motorP1.encoder(), 1.0);

        RotaryMechanism mechP1 = new RotaryMechanism(
                loggerP1,
                motorP1,
                m_sensorP1,
                1.0,
                -100.0,
                100.0);

        m_servoP1 = new OutboardAngularPositionServo(
                loggerP1,
                mechP1,
                refP1);

    }

    public void setPosition(double p1) {
        // m_servoP1.setPositionProfiled(p1, 0);
        m_servoP1.actuateWithProfile(p1, 0);
    }

    public double getPosition() {
        return m_servoP1.getWrappedPositionRad();
    }

    @Override
    public void periodic() {
        m_servoP1.periodic();
    }

    //////////////////////

    private void setDutyCycle(double p1) {
        m_servoP1.setDutyCycle(p1);
    }

    private void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.05));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    public Command position(DoubleSupplier p1) {
        return run(() -> setPosition(p1.getAsDouble()));
    }
}
