package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive shooter with left and right drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DualDrumShooter extends SubsystemBase {

    private final LinearVelocityServo m_left;
    private final LinearVelocityServo m_right;
    private final BooleanLogger m_log_atGoal;

    public DualDrumShooter(
            LoggerFactory log,
            LinearVelocityServo left,
            LinearVelocityServo right) {
        m_left = left;
        m_right = right;
        m_log_atGoal = log.booleanLogger(Level.TRACE, "At goal");
    }

    public void set(double velocityM_S) {
        m_left.setVelocityProfiled(velocityM_S);
        m_right.setVelocityProfiled(velocityM_S);
    }

    public double get() {
        return (m_left.getVelocity() + m_right.getVelocity()) / 2;
    }

    public void stop() {
        set(0);
    }

    public void spinUp(double speed) {
        set(speed);
    }

    public boolean atGoal() {
        return m_right.atGoal() && m_left.atGoal();
    }

    public Command spin(double x) {
        return run(() -> spinUp(x));
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
        m_log_atGoal.log(this::atGoal);
    }
}