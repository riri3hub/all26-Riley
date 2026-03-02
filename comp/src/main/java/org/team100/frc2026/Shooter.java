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
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.CurrentLimitedExponentialVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final CanId canID1 = new CanId(4);
    public static final CanId canID2 = new CanId(5);
    public static final CanId canID3 = new CanId(14);

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;
    private final OutboardLinearVelocityServo m_servo3;

    public Shooter(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Shooter1");
        LoggerFactory log2 = log.name("Shooter2");
        LoggerFactory log3 = log.name("Shooter3");

        VelocityProfileR1 profile = new CurrentLimitedExponentialVelocityProfileR1(
                10, 10, 20, 30);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 40;

                SimpleDynamics dynamics = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                // TODO: set canIDs
                BareMotor m_motor1 = new KrakenX60Motor(
                        log1, canID1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                BareMotor m_motor2 = new KrakenX60Motor(
                        log2, canID2, NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                BareMotor m_motor3 = new KrakenX60Motor(
                        log3, canID3, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, PID);

                // verify these numbers
                LinearMechanism mechanism1 = new LinearMechanism(
                        log1, m_motor1, m_motor1.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                LinearMechanism mechanism2 = new LinearMechanism(
                        log2, m_motor2, m_motor2.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                LinearMechanism mechanism3 = new LinearMechanism(
                        log3, m_motor3, m_motor3.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                double tolerance = 1;
                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism1, ref, tolerance);
                m_servo2 = new OutboardLinearVelocityServo(
                        log2, mechanism2, ref, tolerance);
                m_servo3 = new OutboardLinearVelocityServo(
                        log3, mechanism3, ref, tolerance);
            }
            default -> {
                SimulatedBareMotor m_motor1 = new SimulatedBareMotor(log1, 600);
                LinearMechanism mechanism1 = new LinearMechanism(
                        log1, m_motor1, m_motor1.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                SimulatedBareMotor m_motor2 = new SimulatedBareMotor(log2, 600);
                LinearMechanism mechanism2 = new LinearMechanism(
                        log2, m_motor2, m_motor2.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                SimulatedBareMotor m_motor3 = new SimulatedBareMotor(log3, 600);
                LinearMechanism mechanism3 = new LinearMechanism(
                        log3, m_motor3, m_motor3.encoder(), 1, 0.1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo1 = new OutboardLinearVelocityServo(
                        log1, mechanism1, ref, 1);
                m_servo2 = new OutboardLinearVelocityServo(
                        log2, mechanism2, ref, 1);
                m_servo3 = new OutboardLinearVelocityServo(
                        log3, mechanism3, ref, 1);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
        m_servo3.periodic();
    }

    public Command shooterFullspeed() {
        return run(this::fullSpeed).withName("Shoot full speed");
    }

    public Command testMotor1Command() {
        return run(this::testMotor1).withName("Motor 1 Spin");
    }

    public Command testMotor2Command() {
        return run(this::testMotor2).withName("Motor 2 Spin");
    }

    public Command testMotor3Command() {
        return run(this::testMotor3).withName("Motor 3 Spin");
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop Shooter");
    }

    public void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
        m_servo3.stop();
    }

    private void fullSpeed() {
        double Velocity = 10;
        m_servo1.setVelocityProfiled(Velocity);
        m_servo2.setVelocityProfiled(Velocity);
        m_servo3.setVelocityProfiled(Velocity);
    }
    
    private void testMotor1() {
        m_servo1.setDutyCycle(1);
    }

    private void testMotor2() {
        m_servo2.setDutyCycle(1);
    }

    private void testMotor3() {
        m_servo3.setDutyCycle(1);
    }

    public void setSpeed(double Velocity) {
        m_servo1.setVelocityProfiled(Velocity);
        m_servo2.setVelocityProfiled(Velocity);
        m_servo3.setVelocityProfiled(Velocity);
    }

    public Boolean atSpeed() {
        return (m_servo1.atGoal() && m_servo2.atGoal()
                && m_servo3.atGoal());
    }

}
