package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberExtension extends SubsystemBase {
    private final LinearPositionServo m_servo;

    private final double m_maxExtension = 20;
    private final double m_minextension = 0.01;

    public ClimberExtension(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        IncrementalProfile profile = new TrapezoidIncrementalProfile(log, 1, 2, 0.05);
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        int wheelDiameterM = 2;
        int gearRatio = 1;

        switch (Identity.instance) {
            case COMP_BOT -> {
                int supplyLimit = 60;
                int statorLimit = 1;
                KrakenX60Motor m_motor = new KrakenX60Motor(
                        log,
                        new CanId(0),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        new SimpleDynamics(log, 0, 0),
                        new Friction(log, 0, 0, 0, 0),
                        new PIDConstants(log, 0, 0, 0, 0, 0, 0));
                IncrementalBareEncoder encoder = m_motor.encoder();
                LinearMechanism climberMech = new LinearMechanism(
                        log, m_motor, encoder, gearRatio, wheelDiameterM,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardLinearPositionServo(
                        log, climberMech, ref, 0.01, 0.01);
            }

            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log, 600);
                IncrementalBareEncoder encoder = m_motor.encoder();
                LinearMechanism climberMech = new LinearMechanism(
                        log, m_motor, encoder, gearRatio, wheelDiameterM,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardLinearPositionServo(
                        log, climberMech, ref, 0.01, 0.01);
            }
        }
    }

    public Command setPosition() {
        return run(this::setOutPosition);
    }

    public Command setHomePosition() {
        return run(this::setInPosition);
    }

    public void setOutPosition() {
        m_servo.setPositionProfiled(m_maxExtension, 0);

    }

    public void setInPosition() {
        m_servo.setPositionProfiled(m_minextension, 0.01);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }
}
