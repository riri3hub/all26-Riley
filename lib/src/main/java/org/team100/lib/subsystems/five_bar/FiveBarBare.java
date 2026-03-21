package org.team100.lib.subsystems.five_bar;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplest possible control of the five-bar: duty cycle only, no position
 * measurement.
 */
public class FiveBarBare extends SubsystemBase {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;

    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_motorP1;
    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_motorP5;

    public FiveBarBare(LoggerFactory logger, TotalCurrentLog currentLog) {
        LoggerFactory loggerP1 = logger.name("p1");
        LoggerFactory loggerP5 = logger.name("p5");

        switch (Identity.instance) {
            case COMP_BOT -> {
                m_motorP1 = makeMotor(loggerP1, currentLog, new CanId(1));
                m_motorP5 = makeMotor(loggerP5, currentLog, new CanId(2));
            }
            default -> {
                m_motorP1 = new SimulatedBareMotor(loggerP1, 600);
                m_motorP5 = new SimulatedBareMotor(loggerP5, 600);
            }
        }
    }

    /////////////////////

    private BareMotor makeMotor(LoggerFactory logger, TotalCurrentLog currentLog, CanId canId) {
        SimpleDynamics ff = new SimpleDynamics(logger, 0, 0);
        Friction friction = new Friction(logger, 0, 0, 0, 0);
        PIDConstants pid = PIDConstants.zero(logger);
        return new Falcon500Motor(
                logger,
                currentLog,
                canId,
                NeutralMode100.COAST,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                ff,
                friction,
                pid);
    }

    private void setDutyCycle(double p1, double p5) {
        m_motorP1.setDutyCycle(p1);
        m_motorP5.setDutyCycle(p5);
    }

    /////////////////////
    //
    // Commands

    public Command dutyCycle(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setDutyCycle(p1.getAsDouble(), p5.getAsDouble()));
    }
}
