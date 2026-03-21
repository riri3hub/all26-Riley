package org.team100.lib.subsystems.tank;

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
            int statorLimit,
            CanId canL,
            CanId canR,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        SimpleDynamics ff = new SimpleDynamics(log, 0.01, 0.01);
        Friction friction = new Friction(log, 0.5, 0.5, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.005);

        BareMotor motorL = getMotor(
                logL, currentLog, canL, MotorPhase.REVERSE, statorLimit, ff, friction, pid);
        BareMotor motorR = getMotor(
                logR, currentLog, canR, MotorPhase.FORWARD, statorLimit, ff, friction, pid);

        LinearMechanism mechL = new LinearMechanism(
                logL, motorL, motorL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechR = new LinearMechanism(
                logR, motorR, motorR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        return new TankDrive(fieldLogger, trackWidthM,
                new OutboardLinearVelocityServo(logL, mechL, new NoVelocityReferenceR1(), 1),
                new OutboardLinearVelocityServo(logR, mechR, new NoVelocityReferenceR1(), 1));
    }

    /** Real or simulated depending on identity */
    public static BareMotor getMotor(
            LoggerFactory log, TotalCurrentLog currentLog, CanId can, MotorPhase phase,
            int statorLimit, SimpleDynamics ff, Friction friction, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK -> new SimulatedBareMotor(log, 600);
            default -> new NeoCANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    statorLimit, ff, friction, pid);
        };
    }
}
