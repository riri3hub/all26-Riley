package org.team100.lib.subsystems.tank;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.reference.r1.NoVelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

public class TankDriveFactory {

    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canL,
            CanId canR,
            double trackWidthM,
            double maxSpeedM_S,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        SimpleDynamics ff = new SimpleDynamics(log, 0.00, 0.00);
        Friction friction = new Friction(log, 0.2, 0.2, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.005);

        // ensure the simulated motor can go fast enough.
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);

        BareMotor motorL = getMotor(
                logL, currentLog, freeSpeedRad_S, canL,
                MotorPhase.FORWARD, limit, ff, friction, pid);
        BareMotor motorR = getMotor(
                logR, currentLog, freeSpeedRad_S, canR,
                MotorPhase.REVERSE, limit, ff, friction, pid);

        LinearMechanism mechL = new LinearMechanism(
                logL, motorL, motorL.encoder(),
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        LinearMechanism mechR = new LinearMechanism(
                logR, motorR, motorR.encoder(),
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);

        return new TankDrive(
                log,
                fieldLogger,
                trackWidthM,
                maxSpeedM_S,
                new OutboardLinearVelocityServo(logL, mechL, new NoVelocityReferenceR1(), 1),
                new OutboardLinearVelocityServo(logR, mechR, new NoVelocityReferenceR1(), 1));
    }

    /**
     * Real or simulated depending on identity.
     * 
     * TODO: verify the velocity averaging parameters
     */
    private static BareMotor getMotor(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double freeSpeedRad_S,
            CanId can,
            MotorPhase phase,
            CurrentLimit limit,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK -> new SimulatedBareMotor(log, freeSpeedRad_S);
            case DEMO_BOT -> new Neo550CANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, ff, friction, pid, 2, 4);
            default -> new NeoCANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, ff, friction, pid, 2, 4);
        };
    }
}
