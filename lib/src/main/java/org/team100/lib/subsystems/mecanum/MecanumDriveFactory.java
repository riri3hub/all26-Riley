package org.team100.lib.subsystems.mecanum;

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
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.reference.r1.NoVelocityReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.ReduxGyro;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.mecanum.kinematics.MecanumKinematics100.Slip;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId gyroId,
            CanId canFL,
            CanId canFR,
            CanId canRL,
            CanId canRR,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        LoggerFactory logFL = log.name("frontLeft");
        LoggerFactory logFR = log.name("frontRight");
        LoggerFactory logRL = log.name("rearLeft");
        LoggerFactory logRR = log.name("rearRight");

        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.01);
        SimpleDynamics ff = new SimpleDynamics(log, 0.01, 0.01);
        Friction friction = new Friction(log, 0.5, 0.5, 0.0, 0.5);

        Gyro gyro = gyro(log, gyroId);
        slip = slip(slip);

        BareMotor motorFL = getMotor(
                logFL, currentLog, canFL, MotorPhase.REVERSE,
                limit, ff, friction, pid);
        BareMotor motorFR = getMotor(
                logFR, currentLog, canFR, MotorPhase.FORWARD,
                limit, ff, friction, pid);
        BareMotor motorRL = getMotor(
                logRL, currentLog, canRL, MotorPhase.REVERSE,
                limit, ff, friction, pid);
        BareMotor motorRR = getMotor(
                logRR, currentLog, canRR, MotorPhase.FORWARD,
                limit, ff, friction, pid);

        LinearMechanism mechFL = new LinearMechanism(
                logFL, motorFL, motorFL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechFR = new LinearMechanism(
                logFR, motorFR, motorFR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechRL = new LinearMechanism(
                logRL, motorRL, motorRL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechRR = new LinearMechanism(
                logRR, motorRR, motorRR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        VelocityReferenceR1 ref = new NoVelocityReferenceR1();

        return new MecanumDrive100(
                log, fieldLogger, gyro, trackWidthM, wheelbaseM, slip,
                new OutboardLinearVelocityServo(logFL, mechFL, ref, 1),
                new OutboardLinearVelocityServo(logFR, mechFR, ref, 1),
                new OutboardLinearVelocityServo(logRL, mechRL, ref, 1),
                new OutboardLinearVelocityServo(logRR, mechRR, ref, 1));
    }

    /** Real or simulated depending on identity */
    public static BareMotor getMotor(
            LoggerFactory log, TotalCurrentLog currentLog, CanId can, MotorPhase phase,
            CurrentLimit limit, SimpleDynamics ff, Friction friction, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK -> new SimulatedBareMotor(log, 600);
            default -> new NeoCANSparkMotor(
                    log, currentLog, can, NeutralMode100.BRAKE, phase,
                    limit, ff, friction, pid);
        };
    }

    static Gyro gyro(LoggerFactory log, CanId gyroId) {
        if (gyroId == null)
            return null;
        return switch (Identity.instance) {
            case BLANK -> null;
            default -> new ReduxGyro(log, gyroId);
        };
    }

    static Slip slip(Slip slip) {
        return switch (Identity.instance) {
            case BLANK -> new Slip(1, 1, 1);// sim does not slip;
            default -> slip;
        };
    }

}
