package org.team100.frc2026.subsystems;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtend extends SubsystemBase {
    private static final CanId CAN_ID = new CanId(16);
    private static final double gearRatio = 15.3;

    private final AngularPositionServo m_servo;

    public IntakeExtend(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(log, 8, 8, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double initialPosition = 0;
        final BareMotor motor;
        switch (Identity.instance) {
            case COMP_BOT -> {
                double supplyLimit = 4;
                double statorLimit = 80;
                SimpleDynamics ff = new SimpleDynamics(log, 0.0, 0.0);
                Friction friction = new Friction(log, 1.26, 1.26, 0.006, 0.5);
                PIDConstants pid = PIDConstants.makePositionPID(log, 1);
                motor = new KrakenX44Motor(
                        log, CAN_ID,
                        NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit,
                        ff, friction, pid);
            }
            default -> {
                motor = new SimulatedBareMotor(log, 600);
            }
        }
        m_servo = OutboardAngularPositionServo.make(
                log, motor, ref, gearRatio, initialPosition);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    // ends when complete
    public Command goToExtendedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(3))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToExtendedPosition");
    }

    // never ends
    public Command goToRetractedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(0))
                .withName("Intake Extend GoToRetractedPosition");
    }

    public Command stop() {
        return run(this::stopServo).withName("Stop Intake Extend");
    }

    public Command goToWobbleSlightlyInExtendedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(0.75))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToWobbleExtendedPosition");
    }
    //ballscoredalreA


    public Command goToWobbleSlightlyOutRetractedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(0.25))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToWobbleRetractedPosition");
    }
    public Command goToWobbleOutExtendedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(2.25))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToWobbleExtendedPosition");
    }

    public Command goToWobbleInRetractedPosition() {
        return startRun(this::reset, () -> actuateWithProfile(1.75))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToWobbleRetractedPosition");
    }
    

    /////////////////////////////////////////

    private void stopServo() {
        m_servo.stop();
    }

    private void reset() {
        m_servo.reset();
    }

    private void actuateWithProfile(double value) {
        m_servo.actuateWithProfile(value, 0);
    }
    public boolean atExtendedPosition(){
        return m_servo.getUnwrappedPositionRad() > 2.5;
    }
}
