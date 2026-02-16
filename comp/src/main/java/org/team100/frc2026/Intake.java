package org.team100.frc2026;

import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final BareMotor m_motor;

    public Intake(LoggerFactory parent, CanId canID) {
        LoggerFactory log = parent.type(this);

        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                //
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 0.1);
                // two is too low, even for unloaded case
                double supplyLimit = 50;
                double statorLimit = 20;
                m_motor = new KrakenX44Motor(
                        log, // LoggerFactory parent,
                        canID, // CanId canId,
                        NeutralMode100.COAST, // NeutralMode neutral,
                        MotorPhase.REVERSE, // MotorPhase motorPhase,
                        supplyLimit, // supplyLimit,
                        statorLimit, // statorLimit,
                        KrakenX60Motor.highFrictionFF(log), // Feedforward100 ff
                        KrakenX60Motor.highFriction(log),
                        PID// PIDConstants pid,
                );
            }

            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
            }
        }
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public Command intake() {
        return run(this::fullSpeed).withName("Intake Full Speed");
    }   

    public Command stop() {
        return run(this::stopMotor).withName("Intake Stop");
    }

    public void stopMotor() {
        m_motor.stop();
    }

    private void fullSpeed() {
        // motor max velocity is 6000 RPM or 100 rev/s or 600 rad/s
        // we want to choose about 75% of that, so 450 rad/s
        double velocityRad_S = 450;
        m_motor.setVelocity(velocityRad_S, 0, 0);
        // m_motor.setDutyCycle(1);
        System.out.println(velocityRad_S + "wow");
    }

}
