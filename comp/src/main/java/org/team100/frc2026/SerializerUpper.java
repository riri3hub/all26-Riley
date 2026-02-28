package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SerializerUpper extends SubsystemBase {
    public static final CanId canID = new CanId(0);

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    private final Shooter m_Shooter;

    private final double m_speed = 30;

    public SerializerUpper(LoggerFactory parent, Shooter shooter) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("SerializerUpper1");
        LoggerFactory log2 = log.name("SerializerUpper2");

        m_Shooter = shooter;
        // first parameter is actually accel
        // second parameter is actually jerk
        ProfileR1 profile = new TrapezoidProfileR1(
                log, 10, 100, 1);
        ReferenceR1 ref = new ProfileReferenceR1(
                log, () -> profile, 1, Double.MAX_VALUE);

        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 20;

                SimpleDynamics dynamics = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                // TODO: set canIDs
                BareMotor m_motor1 = new KrakenX44Motor(
                        log1, canID, NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                BareMotor m_motor2 = new KrakenX44Motor(
                        log2, canID, NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                // verify these numbers
                LinearMechanism mechanism1 = new LinearMechanism(
                        log1, m_motor1, m_motor1.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                LinearMechanism mechanism2 = new LinearMechanism(
                        log2, m_motor2, m_motor2.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                double tolerance = 1;
                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism1, ref, tolerance);
                m_servo2 = new OutboardLinearVelocityServo(
                        log2, mechanism2, ref, tolerance);
            }
            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log.name("ShootmotorFeed"), 600);
                LinearMechanism mechanism = new LinearMechanism(
                        log, m_motor, m_motor.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism, ref, 1);
                m_servo2 = new OutboardLinearVelocityServo(
                        log1, mechanism, ref, 1);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    public Command serializerUpperFullspeed() {
        return run(this::fullSpeed).withName(" to Shooter full speed");
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop Shooter feed");
    }

    public void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void fullSpeed() {
        double Velocity = 0.5;
        double Zero = 0;
        if (m_Shooter.atSpeed()) {
            m_servo1.setVelocityProfiled(Velocity);
            m_servo2.setVelocityProfiled(Velocity);
        } else {
            m_servo1.setVelocityProfiled(Zero);
            m_servo2.setVelocityProfiled(Zero);
        }
    }

    public void setSpeed(double Velocity) {
        m_servo1.setVelocityProfiled(Velocity);
        m_servo2.setVelocityProfiled(Velocity);
    }

    public void setSerializerSpeed() {
        setSpeed(m_speed);
    }

}
