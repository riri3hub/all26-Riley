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
import org.team100.lib.profile.r1.CurrentLimitedExponentialVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static final CanId CAN_ID_1 = new CanId(15);
    private static final CanId CAN_ID_2 = new CanId(17);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_M = 0.025;

    private static final double NORMAL_SPEED = 50;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    public Intake(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("motor1");
        LoggerFactory log2 = log.name("motor2");
        VelocityProfileR1 profile = new CurrentLimitedExponentialVelocityProfileR1(
                10, 10, 20, 30);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                double supplyLimit = 50;
                double statorLimit = 50;
                SimpleDynamics ff = new SimpleDynamics(log, 0.004, 0.002);
                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.01);
                m1 = new KrakenX44Motor(
                        log1, CAN_ID_1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, ff, friction, pid);
                m2 = new KrakenX44Motor(
                        log2, CAN_ID_2, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, ff, friction, pid);
            }
            default -> {
                m1 = new SimulatedBareMotor(log1, 600);
                m2 = new SimulatedBareMotor(log2, 600);
            }
        }
        m_servo1 = OutboardLinearVelocityServo.make(
                log1, m1, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo2 = OutboardLinearVelocityServo.make(
                log2, m2, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
    }

    /**
     * Use a profile to spin up the roller to the normal speed.
     * Never ends.
     */
    public Command intake() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(NORMAL_SPEED))
                .withName("Intake Normal Speed");
    }

    /** Stop forever */
    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Intake");
    }

    /** Stop and then end */
    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Intake Once");
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    ////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void setVelocityProfiled(double velocityM_S) {
        m_servo1.setVelocityProfiled(velocityM_S);
        m_servo2.setVelocityProfiled(velocityM_S);
    }
}
