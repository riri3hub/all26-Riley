package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final BareMotor m_motor;
    private final AngularPositionServo m_servo;
    private static final double m_level0 = 0.1;
    private static final double m_level1 = 90;
    private static final double m_level3 = 180;

    public Climber(LoggerFactory parent, CanId CanId) {
        LoggerFactory log = parent.type(this);
        ProfileR1 profile = new TrapezoidProfileR1(log, 1, 2, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double gearRatio = 2;
        double initialPosition = 0;

        switch (Identity.instance) {
            case COMP_BOT -> {
                int supplyLimit = 60;
                int statorLimit = 1;
                m_motor = new KrakenX60Motor(
                        log,
                        new CanId(0),
                        NeutralMode100.BRAKE,
                        MotorPhase.FORWARD,
                        supplyLimit, statorLimit,
                        new SimpleDynamics(log, 0, 0),
                        new Friction(log, 0, 0, 0, 0),
                        new PIDConstants(log, 0, 0, 0, 0, 0, 0));
                IncrementalBareEncoder encoder = m_motor.encoder();
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log, climberMech, ref);
            }

            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
                IncrementalBareEncoder encoder = m_motor.encoder();
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log, climberMech, ref);
            }
        }
    }

    public Command setClimb0() {
        return run(this::setL0);
    }

    public Command setClimb1() {
        return run(this::setL1);
    }


    public Command setClimb3() {
        return run(this::setL3);
    }

    private void setL0() {
        m_servo.setPositionProfiled(m_level0, 0);
    }

    private void setL1() {
        m_servo.setPositionProfiled(m_level1, 0);
    }

    private void setL3() {
        m_servo.setPositionProfiled(m_level3, 0);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }
}
