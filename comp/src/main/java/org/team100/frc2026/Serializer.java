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
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Serializer extends SubsystemBase {
    public static final CanId canID = new CanId(0);

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    private final double m_speed = 30;

    public Serializer(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Serializer1");
        LoggerFactory log2 = log.name("Serializer2");
    
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 20;

                SimpleDynamics dynamics = KrakenX60Motor.highFrictionFF(log);
                Friction friction = KrakenX60Motor.highFriction(log);
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
                m_servo1 = new OutboardLinearVelocityServo(log1, mechanism1, tolerance);
                m_servo2 = new OutboardLinearVelocityServo(log2, mechanism2, tolerance);

            }
            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log.name("Serializer"), 600);
                LinearMechanism mechanism = new LinearMechanism(
                        log, m_motor, m_motor.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo1 = new OutboardLinearVelocityServo(log1, mechanism, 1);
                m_servo2 = new OutboardLinearVelocityServo(log1, mechanism, 1);

            }

        }

    }

    @Override
    public void periodic() {
        m_servo1.periodic();
    }

    public Command serialize() {
        return run(this::fullSpeed).withName("Serialize");
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop serializing");
    }

    public void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
        
    }

    private void fullSpeed() {
        double Velocity = 450;
        m_servo1.setVelocity(Velocity, 0);
        m_servo2.setVelocity(Velocity, 0);
       
    }

    public void setSpeed(double Velocity) {
        m_servo1.setVelocity(Velocity, 0);
        m_servo2.setVelocity(Velocity, 0);
    }

    public void setSerializerSpeed() {
        setSpeed(m_speed);
    }

    public Boolean atSpeed() {
        return (m_servo1.atGoal() && m_servo2.atGoal());
    }
}
