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
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public static final CanId canID1 = new CanId(8);
    public static final CanId canID2 = new CanId(9);
    private static final double TOLERANCE_M_S = 1.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double WHEEL_DIAMETER_M = 0.05;
    // TODO: TUNE
    private static final double NORMAL_SPEED = 5.0;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    private final Shooter m_Shooter;

    public Feeder(LoggerFactory parent, Shooter shooter) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Feeder1");
        LoggerFactory log2 = log.name("Feeder2");
        m_Shooter = shooter;
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                double supplyLimit = 120;
                // TODO: TUNE
                double statorLimit = 120;
                //SimpleDynamics dynamics = new SimpleDynamics(log, 0.004, 0.002);
                                SimpleDynamics dynamics = new SimpleDynamics(log, 0.00, 0.00);

                Friction friction = new Friction(log, 0.26, 0.26, 0.006, 0.5);
                // TODO: TUNE
                //PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.01);
                                PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0);

                m1 = new KrakenX44Motor(
                        log1, canID1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, pid);
                m2 = new KrakenX44Motor(
                        log2, canID2, NeutralMode100.COAST, MotorPhase.FORWARD,
                        supplyLimit, statorLimit, dynamics, friction, pid);
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

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    public Command fullspeed() {
        return startRun(
                this::reset,
                this::feedWhenReady)
                .withName("Feed");
    }

    /**
     * Use a profile to spin up the feeder to the normal speed.
     * Never ends.
     */
    public Command normal() {
        return startRun(
                this::reset,
                this::feedNormally)
                .withName("Feed Normally");
    }

    public Command testFeed() {
        return run(this::dutyCycleAll)
                .withName("Test Feed");
    }

    public Command testFeedBack() {
        return run(this::dutyCycleAllBack)
                .withName("Test Feed Back");
    }

    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Feeder");
    }

    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Feeder Once");
    }

    ///////////////////////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void feedWhenReady() {
        if (m_Shooter.atSpeed()) {
            feedNormally();
        } else {
            stopFeeding();
        }
    }

    private void feedNormally() {
        m_servo1.setVelocityProfiled(NORMAL_SPEED);
        m_servo2.setVelocityProfiled(NORMAL_SPEED);
    }

    private void stopFeeding() {
        m_servo1.setVelocityProfiled(0);
        m_servo2.setVelocityProfiled(0);
    }

    private void dutyCycleAll() {
        m_servo1.setDutyCycle(1);
        m_servo2.setDutyCycle(1);
    }

    private void dutyCycleAllBack() {
        m_servo1.setDutyCycle(-1);
        m_servo2.setDutyCycle(-1);
    }
}
