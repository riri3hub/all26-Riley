package org.team100.frc2026;

import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final CanId canID = new CanId(0);
    private final BareMotor m_motor;
    private final BareMotor m_motor2;
    private final BareMotor m_motor3;

    private final double m_speed = 30;

    public Shooter(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 20;
                m_motor = new KrakenX60Motor(
                        log.name("Shooter1"), // LoggerFactory parent,
                        canID, // CanId canId,
                        NeutralMode100.COAST, // NeutralMode neutral,
                        MotorPhase.REVERSE, // MotorPhase motorPhase,
                        supplyLimit, // supplyLimit,
                        statorLimit, // statorLimit,
                        KrakenX60Motor.highFrictionFF(log), // Feedforward100 ff
                        KrakenX60Motor.highFriction(log),
                        PID// PIDConstants pid,
                );

                m_motor2 = new KrakenX60Motor(
                        log.name("Shooter2"), // LoggerFactory parent,
                        canID, // CanId canId,
                        NeutralMode100.COAST, // NeutralMode neutral,
                        MotorPhase.REVERSE, // MotorPhase motorPhase,
                        supplyLimit, // supplyLimit,
                        statorLimit, // statorLimit,
                        KrakenX60Motor.highFrictionFF(log), // Feedforward100 ff
                        KrakenX60Motor.highFriction(log),
                        PID// PIDConstants pid,
                );
                m_motor3 = new KrakenX60Motor(
                        log.name("Shooter3"), // LoggerFactory parent,
                        canID, // CanId canId,
                        NeutralMode100.COAST, // NeutralMode neutral,
                        MotorPhase.REVERSE, // MotorPhase motorPhase,
                        supplyLimit, // supplyLimit,
                        statorLimit, // statorLimit,
                        KrakenX60Motor.highFrictionFF(log), // Feedforward100 ff
                        KrakenX60Motor.highFriction(log),
                        PID// PIDConstants pid,
                );
            }
            default -> {
                m_motor = new SimulatedBareMotor(log.name("Shootmotor1"), 600);
                m_motor2 = new SimulatedBareMotor(log.name("Shootmotor2"), 600);
                m_motor3 = new SimulatedBareMotor(log.name("Shootmotor3"), 600);

            }

        }

    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public Command shoot() {
        return run(this::fullSpeed).withName("Shoot full speed");
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop Shooter");
    }

    public void stopMotor() {
        m_motor.stop();
        m_motor2.stop();
        m_motor3.stop();
    }

    private void fullSpeed() {
        double Velocity = 450;
        m_motor.setVelocity(Velocity, 0, 0);
        m_motor2.setVelocity(Velocity, 0, 0);
        m_motor3.setVelocity(Velocity, 0, 0);
    }

    public void setSpeed(double Velocity) {
        m_motor.setVelocity(Velocity, 0, 0);
        m_motor2.setVelocity(Velocity, 0, 0);
        m_motor3.setVelocity(Velocity, 0, 0);
    }

    public void setSerializerSpeed() {

        setSpeed(m_speed);
    }

    public Boolean atSpeed() {
        return (m_motor.getVelocityRad_S() == m_speed && m_motor2.getVelocityRad_S() == m_speed
                && m_motor3.getVelocityRad_S() == m_speed);
    }

}
