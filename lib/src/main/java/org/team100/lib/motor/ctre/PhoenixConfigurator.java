package org.team100.lib.motor.ctre;

import java.util.function.Supplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Utilities for CTRE Phoenix motors: Falcon, Kraken. */
public class PhoenixConfigurator {
    private static final boolean DEBUG = false;
    private static final boolean ACTUALLY_CRASH = false;
    /**
     * The default is 0.05. This is much longer, to eliminate unnecessary config
     * failures.
     */
    private static final double TIMEOUT_SEC = 0.3;
    // speeding up the updates is a tradeoff between latency and CAN utilization.
    // 254 seems to think that 100 is a good compromise?
    // see
    // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveDrivetrain.java#L382
    private static final int SIGNAL_UPDATE_FREQ_HZ = 100;

    private final TalonFX m_motor;
    private final NeutralMode100 m_neutral;
    private final MotorPhase m_phase;
    private final CurrentLimit m_limit;
    private final PIDConstants m_pid;

    public PhoenixConfigurator(
            TalonFX motor,
            NeutralMode100 neutral,
            MotorPhase phase,
            CurrentLimit limit,
            PIDConstants pid) {
        m_motor = motor;
        m_neutral = neutral;
        m_phase = phase;
        m_limit = limit;
        m_pid = pid;
        // reapply the pid parameters if any change.
        m_pid.register(this::pidConfig);
    }

    public void logCrashStatus() {
        if (ACTUALLY_CRASH) {
            System.out.println("WARNING: ***** Config fail will CRASH the robot, NOT FOR COMP!");
        } else {
            System.out.println("***** Config fail will not be caught, NOT FOR DEV!");
        }

    }

    public void crash(Supplier<StatusCode> s) {
        StatusCode statusCode = s.get();
        if (statusCode.isError()) {
            if (ACTUALLY_CRASH)
                throw new IllegalStateException(statusCode.toString());
            System.out.println("WARNING: ******************************************************");
            System.out.printf("WARNING: ****** Motor ID %d\n", m_motor.getDeviceID());
            System.out.println("WARNING: ****** MOTOR CONFIG HAS FAILED MOTOR IS NOT SET CORRECTLY ******");
            System.out.println("WARNING: " + statusCode.toString());
        }
    }

    public void baseConfig() {
        TalonFXConfiguration base = new TalonFXConfiguration();
        crash(() -> m_motor.getConfigurator().apply(base, TIMEOUT_SEC));
    }

    public void motorConfig() {
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = switch (m_neutral) {
            case COAST -> NeutralModeValue.Coast;
            case BRAKE -> NeutralModeValue.Brake;
        };
        motorConfigs.Inverted = switch (m_phase) {
            case FORWARD -> InvertedValue.CounterClockwise_Positive;
            case REVERSE -> InvertedValue.Clockwise_Positive;
        };
        crash(() -> m_motor.getConfigurator().apply(motorConfigs, TIMEOUT_SEC));
        crash(() -> m_motor.getPosition().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getVelocity().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getAcceleration().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getTorqueCurrent().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
    }

    /**
     * @see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * @see https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
     */
    public void currentConfig() {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.SupplyCurrentLimit = m_limit.supply();
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.StatorCurrentLimit = m_limit.stator();
        currentConfigs.StatorCurrentLimitEnable = true;
        crash(() -> m_motor.getConfigurator().apply(currentConfigs, TIMEOUT_SEC));
    }

    /**
     * Changes the stator limit. This is useful for "holding torque" which might be
     * less than "grabbing torque".
     */
    public void overrideStatorLimit(double limit) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.SupplyCurrentLimit = m_limit.supply();
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.StatorCurrentLimit = limit;
        currentConfigs.StatorCurrentLimitEnable = true;
        crash(() -> m_motor.getConfigurator().apply(currentConfigs, TIMEOUT_SEC));
    }

    /**
     * Returns the current limit to the initial setup.
     */
    public void endCurrentLimitOverride() {
        currentConfig();
    }

    /**
     * CTRE PID units depend on the output type. Because we use "voltage" control
     * types ("PositionVoltage" and "VelocityVoltage"), our output type is volts.
     * 
     * position
     * P = volts per rev, start with 1.
     * I = volts per rev * sec
     * D = volts per rev/sec (volt-sec/rev)
     * 
     * velocity
     * P = volts per rev/sec (volt-sec/rev), start with 0.01
     * I = volts per rev
     * D = volts per rev/s^2 (volt-sec^2/rev)
     * 
     * @see https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
     */
    public void pidConfig() {
        Slot0Configs slot0Configs = new Slot0Configs();
        Slot1Configs slot1Configs = new Slot1Configs();
        slot0Configs.kV = 0.0; // we use "arbitrary feedforward", not this.
        slot1Configs.kV = 0.0;
        // Our control modes use volts, so we can use the PID volt constants.
        slot0Configs.kP = 2 * Math.PI * m_pid.getPositionPV_Rad();
        slot0Configs.kI = 2 * Math.PI * m_pid.getPositionIV_RadS();
        slot0Configs.kD = 2 * Math.PI * m_pid.getPositionDVS_Rad();
        slot1Configs.kP = 2 * Math.PI * m_pid.getVelocityPVS_Rad();
        slot1Configs.kI = 2 * Math.PI * m_pid.getVelocityIVolt_Rad();
        slot1Configs.kD = 2 * Math.PI * m_pid.getVelocityDVS2_Rad();

        if (DEBUG) {
            System.out.printf("POSITION P VALUE %f\n", slot0Configs.kP);
            System.out.printf("VELOCITY P VALUE %f\n", slot1Configs.kP);
        }
        crash(() -> m_motor.getConfigurator().apply(slot0Configs, TIMEOUT_SEC));
        crash(() -> m_motor.getConfigurator().apply(slot1Configs, TIMEOUT_SEC));
    }

    /** Allow music during disable: this is for warning sounds. */
    public void audioConfig() {
        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.AllowMusicDurDisable = true;
        crash(() -> m_motor.getConfigurator().apply(audioConfigs, TIMEOUT_SEC));
    }
}
