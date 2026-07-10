package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.absolute.HomingRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Discus version that uses the "mechanism" abstraction,
 * which provides positional control.
 */
public class DiscusMech extends SubsystemBase {

    /** Low current limits */
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;

    private final RotaryMechanism m_mechP1;

    private final BareMotor m_motorP1;

    private final HomingRotaryPositionSensor m_sensorP1;

    public DiscusMech(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);
        /** Units of positional PID are volts per revolution. */
        PIDConstants pid = PIDConstants.makePositionPID(
                logger, 0.0); // 2.0
        Friction friction = new Friction(logger, 0.16, 0.15, 0, 0);

        switch (Identity.instance) {
            case TEAM100_2018 -> {
                Falcon500Motor motorP1 = new Falcon500Motor(
                        logger,
                        currentLog,
                        new CanId(36),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        friction,
                        pid);

                m_motorP1 = motorP1;

                m_sensorP1 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(motorP1.encoder(), 1.0));

                m_mechP1 = new RotaryMechanism(
                        logger,
                        motorP1,
                        m_sensorP1,
                        1.0,
                        -100.0,
                        100.0);

            }
            default -> {
                SimulatedBareMotor motorP1 = new SimulatedBareMotor(logger, 600);
                m_motorP1 = motorP1;

                m_sensorP1 = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(
                                motorP1.encoder(), 1.0));

                m_mechP1 = new RotaryMechanism(
                        logger,
                        motorP1,
                        m_sensorP1,
                        1.0,
                        -100.0,
                        100.0);

            }
        }
    }

    /** Update position by adding. */
    public void add(double p1) {
        double q1 = m_mechP1.getUnwrappedPositionRad() + p1;
        setPosition(q1);
    }

    /** Set position goal, motionless. */
    public void setPosition(double p1) {
        m_mechP1.setUnwrappedPosition(p1, 0, 0);
    }

    public void setVelocity(double v) {
        m_mechP1.setVelocity(v, 0);
    }

    @Override
    public void periodic() {
        m_mechP1.periodic();
    }

    public double getPosition() {
        return m_mechP1.getWrappedPositionRad();
    }

    //////////////////////

    /** For homing; ignores feasibility and limits. */
    private void setDutyCycle(double p1) {
        m_mechP1.setDutyCycleUnlimited(p1);
    }

    /**
     * The "home" position is the max value; the idea is that you've run the duty
     * cycle (gently) to the end of travel before pushing the "home" button.
     */
    private void setHomePosition() {
        m_motorP1.setUnwrappedEncoderPositionRad(0);
    }

    ///////////////////////
    //
    // Commands

    /** Move in the direction of home: q1 to max, q5 to min */
    public Command home() {
        return run(() -> setDutyCycle(0.01));
    }

    /** Set the position sensor to the home position. */
    public Command zero() {
        return runOnce(this::setHomePosition);
    }

    /** Update position by adding. */
    public Command position(DoubleSupplier p1) {
        return run(() -> add(p1.getAsDouble()));
    }

    public Command velocity(DoubleSupplier v) {
        return run(() -> setVelocity(v.getAsDouble()));
    }
}
