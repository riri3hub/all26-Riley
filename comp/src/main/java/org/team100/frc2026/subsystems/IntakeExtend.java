package org.team100.frc2026.subsystems;

import org.team100.frc2026.robot.CurrentLimits;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
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
import org.team100.lib.servo.Gravity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Intake must be retracted at startup. */
public class IntakeExtend extends SubsystemBase {
    private static final CanId CAN_ID = new CanId(19);
    private static final CanId CAN_ID2 = new CanId(17);
    private static final double gearRatio = 74.667;
    private static final double RETRACTED_POSITION = 0;
    // seems fine, 3/12/26
    private static final double EXTENDED_POSITION = 2.140017;

    private final Gravity m_gravity;
    private final AngularPositionServo m_servo;
    private final AngularPositionServo m_Servo2;

    public IntakeExtend(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Extend1");
        LoggerFactory log2 = log.name("Extend2");
        m_gravity = new Gravity(log,
                0, // Max gravity torque, Nm
                0); // Gravity torque position offset, rad
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(log, 4, 8, 0.1);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.1, 0.05);
        final BareMotor motor;
        final BareMotor motor2;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {

                SimpleDynamics ff = new SimpleDynamics(log, 0.0, 0.0);
                // friction test 3/12/26
                Friction friction = new Friction(log, 0.32, 0.32, 0.0, 0.5);
                // tuned 3/12/26
                // TODO: get correct pid value
                PIDConstants pid = PIDConstants.makePositionPID(log, 1);
                motor = new KrakenX44Motor(
                        log1, currentLog, CAN_ID,
                        NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.INTAKE_EXTEND,
                        ff, friction, pid);
                motor2 = new KrakenX44Motor(
                        log2, currentLog, CAN_ID2,
                        NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.INTAKE_EXTEND,
                        ff, friction, pid);
            }
            default -> {
                motor = new SimulatedBareMotor(log1, 600);
                motor2 = new SimulatedBareMotor(log2, 600);
            }
        }
        m_servo = OutboardAngularPositionServo.make(
                log1, motor, ref, gearRatio,
                RETRACTED_POSITION, RETRACTED_POSITION, EXTENDED_POSITION);
        m_Servo2 = OutboardAngularPositionServo.make(
                log2, motor2, ref, gearRatio,
                RETRACTED_POSITION, RETRACTED_POSITION, EXTENDED_POSITION);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
        m_Servo2.periodic();
    }

    /** Current position is out, or nearly so */
    public boolean isOut() {
        return MathUtil.isNear(m_servo.getUnwrappedPositionRad(), EXTENDED_POSITION, 1)
                &&
                MathUtil.isNear(m_Servo2.getUnwrappedPositionRad(), EXTENDED_POSITION, 1);
    }

    /**
     * Use a profile to go to the extended position.
     * Ends when complete.
     */
    public Command goToExtendedPosition() {
        return startRun(
                this::reset,
                () -> actuateWithGravity(EXTENDED_POSITION))
                .until(m_servo::atGoal)
                .withName("Intake Extend GoToExtendedPosition");
    }

    /**
     * Use a profile to go to the extended position.
     * Never ends, but stops the motor when interrupted.
     */
    public Command goToExtendedPositionEndlessly() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(EXTENDED_POSITION))
                .finallyDo(this::stopServo)
                .withName("Intake Extend GoToExtendedPositionEndlessly");
    }

    /** Servo is at goal. False immediately after reset. */
    public boolean atGoal() {
        return m_servo.atGoal() && m_Servo2.atGoal();
    }

    /**
     * Use a profile to go to the retracted position.
     * Never ends.
     */
    public Command goToRetractedPosition() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(RETRACTED_POSITION))
                .withName("Intake Extend GoToRetractedPosition");
    }

    /** Stop and then end -- this is for compositions where doing nothing is OK */
    public Command stopOnce() {
        return runOnce(this::stopServo)
                .withName("Intake Extend Stop Once");
    }

    /** Stop forever */
    public Command stop() {
        return run(this::stopServo)
                .withName("Stop Intake Extend");
    }

    /** For testing friction only */
    public Command setVelocity(double rad_S) {
        return startRun(
                this::reset,
                () -> {
                    m_servo.setVelocity(rad_S);
                    m_Servo2.setVelocity(rad_S);
                })
                .withName("set velocity");
    }

    public Command setPosition(double rad) {
        return startRun(
                this::reset,
                () -> {
                    m_servo.actuateWithProfile(rad, 0);
                    m_Servo2.actuateWithProfile(rad, 0);
                })
                .withName("set position");
    }

    /////////////////////////////////////////

    private void stopServo() {
        m_servo.stop();
        m_Servo2.stop();
    }

    private void reset() {
        m_servo.reset();
        m_Servo2.reset();
    }

    private void actuateWithProfile(double value) {
        m_servo.actuateWithProfile(value, 0);
        m_Servo2.actuateWithProfile(value, 0);
    }

    private void actuateWithGravity(double value) {
        m_servo.actuateWithProfile(value, gravityTorque());
        m_Servo2.actuateWithProfile(value, gravityTorque());
    }

    public boolean atExtendedPosition() {
        return MathUtil.isNear(m_servo.getUnwrappedPositionRad(), EXTENDED_POSITION, 0.1)
                && MathUtil.isNear(m_Servo2.getUnwrappedPositionRad(), EXTENDED_POSITION, 0.1);
    }

    private double gravityTorque() {
        return (m_gravity.applyAsDouble(m_servo.getWrappedPositionRad()) +
                m_gravity.applyAsDouble(m_Servo2.getUnwrappedPositionRad())) / 2;

    }
}
