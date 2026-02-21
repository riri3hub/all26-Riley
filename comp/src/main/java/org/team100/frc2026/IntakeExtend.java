package org.team100.frc2026;

import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.sensor.position.incremental.ctre.Talon6Encoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtend extends SubsystemBase {
    private final AngularPositionServo m_servo;

    public IntakeExtend(LoggerFactory parent, CanId canID) {
        LoggerFactory log = parent.type(this);

        switch (Identity.instance) {

            case TEST_BOARD_B0 -> {
                double gearRatio = 15.3;
                PIDConstants PID = PIDConstants.makePositionPID(log, 1);
                double supplyLimit = 4;
                double statorLimit = 80;
                KrakenX44Motor m_motor = new KrakenX44Motor(
                        log, // LoggerFactor y parent,
                        canID, // CanId canId,
                        NeutralMode100.COAST, // NeutralMode neutral,
                        MotorPhase.REVERSE, // MotorPhase motorPhase,
                        supplyLimit, // og 50 //double supplyLimit,
                        statorLimit, // og 2 //double statorLimit,
                        new SimpleDynamics(log, 0.0, 0.0), // Feedforward100 ff
                        new Friction(log, 1.26, 1.26, 0.006, 0.5),
                        PID // PIDConstants pid,
                );
                Talon6Encoder encoder = m_motor.encoder();

                TrapezoidIncrementalProfile profile = new TrapezoidIncrementalProfile(log, 8, 8, 0.05);
                ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
                double initialPosition = 0;
                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, encoder, initialPosition, gearRatio,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_servo = new OutboardAngularPositionServo(log, climberMech, ref);

            }

            default -> {
                SimulatedBareMotor m_motor = new SimulatedBareMotor(log, 600);

                IncrementalProfile profile = new TrapezoidIncrementalProfile(log, 100, 100, 0.05);
                ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
                PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

                IncrementalBareEncoder encoder = m_motor.encoder();
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

                RotaryMechanism climberMech = new RotaryMechanism(
                        log, m_motor, sensor, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    public Command goToExtendedPosition() {
        return new FunctionalCommand(
                () -> reset(), // onInit
                () -> setAngle(3), // onExecute
                interrupted -> { // onEnd
                },
                () -> m_servo.atGoal(), // isFinished
                this).withName("Intake Extend GoToExtendedPosition");
    }

    public Command goToRetractedPosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(0)).withName("Intake Extend GoToRetractedPosition");
    }

    public Command stop() {
        return run(this::stopServo).withName("Intake Extend Stop");
    }

    public void stopServo() {
        m_servo.stop();
    }

    private void reset() {
        m_servo.reset();
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.actuateWithProfile(value, 0);
    }

}
