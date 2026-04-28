package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter with left and right drums.
 */
public class DualDrumDutyCycleShooter extends SubsystemBase implements DualDrumShooter {
    /** full output duty cycle */
    private final double m_full;
    private final BareMotor m_left;
    private final BareMotor m_right;

    public DualDrumDutyCycleShooter(
            LoggerFactory log,
            double full,
            BareMotor left,
            BareMotor right) {
        m_full = full;
        m_left = left;
        m_right = right;
    }

    @Override
    public Command spin() {
        return run(this::full);
    }

    @Override
    public Command stop() {
        return run(this::zero);
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
    }

    ///////////////////////////////////////////////////////

    private void full() {
        set(m_full);
    }

    private void zero() {
        m_left.stop();
        m_right.stop();
    }

    private void set(double dutyCycle) {
        m_left.setDutyCycle(dutyCycle);
        m_right.setDutyCycle(dutyCycle);
    }

}