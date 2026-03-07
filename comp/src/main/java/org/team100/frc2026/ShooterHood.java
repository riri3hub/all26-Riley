package org.team100.frc2026;

import java.util.OptionalDouble;
import java.util.function.Supplier;

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
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
    // TODO GET THIS CAN ID
    private static final CanId CAN_ID = new CanId(0);
    private static final double GEAR_RATIO = 10;

    private final Supplier<OptionalDouble> m_angle;
    private final AngularPositionServo m_servo;

    /**
     * @param parent log
     * @param angle  angle for auto mode
     */
    public ShooterHood(LoggerFactory parent, Supplier<OptionalDouble> angle) {
        LoggerFactory log = parent.type(this);
        m_angle = angle;
        double initialPosition = 0;
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(log, 1, 2, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);

        final BareMotor motor;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                double supplyLimit = 50;
                double statorLimit = 20;
                SimpleDynamics ff = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                PIDConstants pid = PIDConstants.makePositionPID(log, 1);

                motor = new KrakenX44Motor(
                        log, CAN_ID, NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit, ff, friction, pid);

            }
            default -> {
                motor = new SimulatedBareMotor(log, 600);
            }
        }
        m_servo = OutboardAngularPositionServo.make(
                log, motor, ref, GEAR_RATIO, initialPosition);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    /** Uses a profile. Maybe don't use a profile? */
    public Command position() {
        return startRun(this::reset, this::autoWork);
    }

    public Command stop() {
        return run(this::stopServo).withName("Shooter Hood Stop");
    }

    public boolean onTarget() {
        return m_servo.atGoal();
    }

    /////////////////////////////////////////

    /** For testing. */
    double getUnwrappedPositionRad() {
        return m_servo.getUnwrappedPositionRad();
    }

    /** For testing. */
    ModelR1 getUnwrappedGoal() {
        return m_servo.getUnwrappedGoal();
    }

    private void reset() {
        m_servo.reset();
    }

    private void stopServo() {
        m_servo.stop();
    }

    /** Use a profile to set the position. */
    private void actuateWithProfile(double value) {
        m_servo.actuateWithProfile(value, 0);
    }

    /** Do not use a profile. */
    private void actuateDirect(double value) {
        m_servo.actuateDirect(value, 0);
    }

    private void autoWork() {
        m_angle.get().ifPresentOrElse(
                this::actuateWithProfile, this::stopServo);
    }

}
