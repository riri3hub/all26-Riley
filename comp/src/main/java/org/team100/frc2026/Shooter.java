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
    private static final CanId CAN_ID_1 = new CanId(4);
    private static final CanId CAN_ID_2 = new CanId(5);
    private static final CanId CAN_ID_3 = new CanId(14);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_M = 0.075;

    private final Supplier<OptionalDouble> m_speed;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;
    private final OutboardLinearVelocityServo m_servo3;

    /**
     * @param parent log
     * @param speed  speed (m/s) for auto mode
     */
    public Shooter(LoggerFactory parent, Supplier<OptionalDouble> speed) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Shooter1");
        LoggerFactory log2 = log.name("Shooter2");
        LoggerFactory log3 = log.name("Shooter3");
        m_speed = speed;

        VelocityProfileR1 profile = new CurrentLimitedExponentialVelocityProfileR1(
                10, 10, 20, 30);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        final BareMotor m3;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                double supplyLimit = 120;
                double statorLimit = 60;
                SimpleDynamics ff = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.01);

                m1 = new KrakenX60Motor(
                        log1, CAN_ID_1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, ff, friction, pid);
                m2 = new KrakenX60Motor(
                        log2, CAN_ID_2, NeutralMode100.COAST, MotorPhase.REVERSE,
                        supplyLimit, statorLimit, ff, friction, pid);
                m3 = new KrakenX60Motor(
                        log3, CAN_ID_3, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, ff, friction, pid);
            }
            default -> {
                m1 = new SimulatedBareMotor(log1, 600);
                m2 = new SimulatedBareMotor(log2, 600);
                m3 = new SimulatedBareMotor(log3, 600);
            }
        }
        m_servo1 = OutboardLinearVelocityServo.make(
                log1, m1, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo2 = OutboardLinearVelocityServo.make(
                log2, m2, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo3 = OutboardLinearVelocityServo.make(
                log3, m3, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
        m_servo3.periodic();
    }

    public Command shooterFullspeed() {
        return startRun(this::reset, () -> setVelocityProfiled(10))
                .withName("Shoot full speed");
    }

    public Command testShooterFullspeed() {
        return run(this::dutyCycleAll).withName("Test shoot full speed");
    }

    public Command testMotor1Command() {
        return run(this::dutyCycle1).withName("Motor 1 Spin");
    }

    public Command testMotor2Command() {
        return run(this::dutyCycle2).withName("Motor 2 Spin");
    }

    public Command testMotor3Command() {
        return run(this::dutyCycle3).withName("Motor 3 Spin");
    }

    public Command auto() {
        return startRun(this::reset, this::autoWork);
    }

    public Command stop() {
        return run(this::stopMotor).withName("stop Shooter");
    }

    public Boolean atSpeed() {
        return (m_servo1.atGoal() && m_servo2.atGoal()
                && m_servo3.atGoal());
    }

    /////////////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
        m_servo3.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
        m_servo3.stop();
    }

    private void setVelocityDirect(double setpointM_S) {
        m_servo1.setVelocityDirect(setpointM_S);
        m_servo2.setVelocityDirect(setpointM_S);
        m_servo3.setVelocityDirect(setpointM_S);
    }

    private void setVelocityProfiled(double goalM_S) {
        m_servo1.setVelocityProfiled(goalM_S);
        m_servo2.setVelocityProfiled(goalM_S);
        m_servo3.setVelocityProfiled(goalM_S);
    }

    private void dutyCycleAll() {
        m_servo1.setDutyCycle(1);
        m_servo2.setDutyCycle(1);
        m_servo3.setDutyCycle(1);
    }

    private void dutyCycle1() {
        m_servo1.setDutyCycle(1);
    }

    private void dutyCycle2() {
        m_servo2.setDutyCycle(1);
    }

    private void dutyCycle3() {
        m_servo3.setDutyCycle(1);
    }

    private void autoWork() {
        m_speed.get().ifPresentOrElse(
                this::setVelocityProfiled, this::stopMotor);
    }

}
