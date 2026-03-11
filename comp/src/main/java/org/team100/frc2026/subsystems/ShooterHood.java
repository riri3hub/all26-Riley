package org.team100.frc2026.subsystems;

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
import org.team100.lib.tuning.Mutable;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
    // TODO GET THIS CAN ID
    private static final CanId CAN_ID = new CanId(13);
    private static final double GEAR_RATIO = 10;

    // MECHANISM POSITIONS

    private static final double INITIAL_POSITION_RAD = 0;
    private static final double MIN_POSITION_RAD = 0;
    // TODO: this is definitely wrong
    private static final double MAX_POSITION_RAD = 1;

    private final Supplier<OptionalDouble> m_angle;
    private final AngularPositionServo m_servo;

    private final Mutable m_tuningSetting;

    /**
     * @param parent log
     * @param angle  angle for auto mode
     */
    public ShooterHood(LoggerFactory parent, Supplier<OptionalDouble> angle) {
        LoggerFactory log = parent.type(this);
        m_angle = angle;
        m_tuningSetting = new Mutable(log, "for tuning", 0);
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
                log, motor, ref, GEAR_RATIO, INITIAL_POSITION_RAD, MIN_POSITION_RAD, MAX_POSITION_RAD);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    /** Use a profile to set the position according to the angle supplier. */
    public Command autoPosition() {
        return startRun(
                this::reset,
                this::autoPositionWork)
                .withName("Hood Auto Position");
    }

    /**
     * Use a profile to set the position to minimum.
     * Never ends.
     */
    public Command in() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(MIN_POSITION_RAD))
                .withName("Hood In");
    }

    /**
     * Use a profile to set the position to maximum.
     * Never ends.
     */
    public Command out() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(MAX_POSITION_RAD))
                .withName("Hood Out");
    }

    /**
     * Set the position to the tuning value in glass, without a profile.
     * Never ends.
     */
    public Command tune() {
        return startRun(
                this::reset,
                () -> actuateDirect(
                        m_tuningSetting.getAsDouble()))
                .withName("Tune Hood");
    }

    public Command stop() {
        return run(this::stopServo)
                .withName("Stop Hood");
    }

    public Command stopOnce() {
        return runOnce(this::stopServo)
                .withName("Stop Hood Once");
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

    private void autoPositionWork() {
        m_angle.get().ifPresentOrElse(
                this::actuateWithProfile, this::stopServo);
    }

}
