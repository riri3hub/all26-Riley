package org.team100.lib.motor.rev;

import java.util.function.Supplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.tuning.Mutable;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Configuration for one SparkBase motor */
public class RevConfigurator {
    /**
     * The CAN report period for the built-in encoder.
     * This is longer than the equivalent CTRE number, maybe
     * it should be 10?
     */
    private static final int ENCODER_REPORT_PERIOD_MS = 20;

    private final SparkBase m_motor;
    private final NeutralMode100 m_neutral;
    private final MotorPhase m_phase;
    private final Mutable m_statorCurrentLimit;
    private final PIDConstants m_pid;

    /**
     * Stator current limit is mutable.
     * Does not support supply limit.
     * pid has mutable parts.
     */
    public RevConfigurator(
            LoggerFactory log,
            SparkBase motor,
            NeutralMode100 neutral,
            MotorPhase phase,
            CurrentLimit limit,
            PIDConstants pid) {
        m_motor = motor;
        m_neutral = neutral;
        m_phase = phase;
        m_statorCurrentLimit = new Mutable(log, "stator current limit (a)", limit.stator(), (x) -> currentConfig());
        m_pid = pid;
        // reapply the pid parameters if any change.
        m_pid.register(this::pidConfig);
    }

    /**
     * Makes config synchronous so we can see the errors
     */
    public void longCANTimeout() {
        crash(() -> m_motor.setCANTimeout(500));
    }

    /**
     * Makes everything asynchronous.
     * NOTE: this makes error-checking not work at all.
     */
    public void zeroCANTimeout() {
        crash(() -> m_motor.setCANTimeout(0));
    }

    public void baseConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        conf.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        conf.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        conf.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        crash(() -> m_motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void motorConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.idleMode(switch (m_neutral) {
            case COAST -> IdleMode.kCoast;
            case BRAKE -> IdleMode.kBrake;
        });
        conf.inverted(switch (m_phase) {
            case FORWARD -> false;
            case REVERSE -> true;
        });
        conf.signals.primaryEncoderVelocityPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderVelocityAlwaysOn(true);
        conf.signals.primaryEncoderPositionPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderPositionAlwaysOn(true);
        // slower than default of 10; also affects things like motor temperature and
        // applied output.
        conf.signals.limitsPeriodMs(20);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void currentConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit((int) m_statorCurrentLimit.getAsDouble());
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    /**
     * Changes the stator limit. This is useful for "holding torque" which might be
     * less than "grabbing torque".
     */
    public void overrideStatorLimit(int limit) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit(limit);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    /**
     * Returns the current limit to the initial setup.
     */
    public void endCurrentLimitOverride() {
        currentConfig();
    }

    /**
     * The REV PID units are as follows:
     * 
     * position
     * P = duty cycle per rev, start with 1.
     * I = duty cycle per rev * ms
     * D = duty cycle per rev/ms
     * 
     * velocity
     * P = duty cycle per RPM, start with 0.0002; rev example is 0.00005
     * I = duty cycle per RPM*ms
     * D = duty cycle per RPM/ms
     * 
     * @see https://docs.revrobotics.com/revlib/spark/closed-loop/units
     * @see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
     */
    public void pidConfig() {
        double supplyVoltage = 12.0;
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.closedLoop.positionWrappingEnabled(false); // don't use position control
        conf.closedLoop.p(2 * Math.PI * m_pid.getPositionPV_Rad() / supplyVoltage, ClosedLoopSlot.kSlot0);
        conf.closedLoop.i(2 * Math.PI * m_pid.getPositionIV_RadS() / (1000 * supplyVoltage), ClosedLoopSlot.kSlot0);
        conf.closedLoop.d(2 * Math.PI * 1000 * m_pid.getPositionDVS_Rad() / supplyVoltage, ClosedLoopSlot.kSlot0);

        conf.closedLoop.p(2 * Math.PI * m_pid.getVelocityPVS_Rad() / (60 * supplyVoltage), ClosedLoopSlot.kSlot1);
        conf.closedLoop.i(2 * Math.PI * m_pid.getVelocityIVolt_Rad() / (60 * 1000 * supplyVoltage),
                ClosedLoopSlot.kSlot1);
        conf.closedLoop.d(2 * Math.PI * 1000 * m_pid.getVelocityDVS2_Rad() / (60 * supplyVoltage),
                ClosedLoopSlot.kSlot1);
        // We don't use wind-up control.
        conf.closedLoop.iZone(0, ClosedLoopSlot.kSlot0);
        conf.closedLoop.iZone(0, ClosedLoopSlot.kSlot1);
        // We don't use this type of feedforward at all, we use "arbitrary" feedforward.
        conf.closedLoop.apply(new FeedForwardConfig().kV(0, ClosedLoopSlot.kSlot0));
        conf.closedLoop.apply(new FeedForwardConfig().kV(0, ClosedLoopSlot.kSlot1));
        // Maximum output range.
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    private static void crash(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            throw new IllegalStateException(errorCode.name());
        }
    }
}
