package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Bare-motor version of the discus mechanism. Just takes
 * duty cycle input.
 */
public class DiscusBare extends SubsystemBase {
    private static final double SCALE = 0.05;
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;
    private final BareMotor m_motor;
    private final RotaryPositionSensor m_sensor;

    public DiscusBare(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);
        switch (Identity.instance) {
            case TEAM100_2018 -> {
                SimpleDynamics ff = new SimpleDynamics(logger, 0, 0);
                Friction friction = new Friction(logger, 0.1, 0.1, 0, 0);
                PIDConstants pid = PIDConstants.makePositionPID(logger, 0.0);
                m_motor = new Falcon500Motor(
                        logger,
                        currentLog,
                        new CanId(36),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        ff,
                        friction,
                        pid);

            }
            default -> {
                m_motor = new SimulatedBareMotor(logger, 600);

            }
        }
        m_sensor = new ProxyRotaryPositionSensor(m_motor.encoder(), 1.0, 0.0);
    }

    public double getPosition() {
        return m_sensor.getWrappedPositionRad();
    }

    private void setDutyCycle(double p) {
        m_motor.setDutyCycle(p);
    }

    public Command dutyCycle(DoubleSupplier p) {
        return run(() -> setDutyCycle(
                SCALE * p.getAsDouble()));
    }
}
