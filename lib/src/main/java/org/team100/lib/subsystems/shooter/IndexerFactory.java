package org.team100.lib.subsystems.shooter;

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
import org.team100.lib.motor.rev.MinionSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

public class IndexerFactory {
    private static final double VELOCITY = 10;
    private static final double ACCEL = 20;

    public static PWMIndexer makePWMIndexer(
            LoggerFactory parent, int channel) {
        return new PWMIndexer(parent, new RoboRioChannel(channel));
    }

    public static DutyCycleIndexer makeDutyCycleIndexer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            double full, // full output duty cycle
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("indexer");
        SimpleDynamics ff = new SimpleDynamics(log, 0, 0);
        Friction friction = new Friction(log, 0.07, 0.07, 0.01, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.02);
        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);
        BareMotor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                MotorPhase.REVERSE, ff, friction, pid);
        return new DutyCycleIndexer(parent, full, motor);
    }

    public static PositionIndexer makePositionIndexer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM,
            boolean profiled) {
        LoggerFactory log = parent.name("indexer");
        LinearMechanism mech = getMech(
                log, currentLog, limit, canId, gearRatio, wheelDiaM);
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(
                log, VELOCITY, ACCEL, 0.02);
        ProfileReferenceR1 ref = new ProfileReferenceR1(
                log, () -> profile, 0.02, 0.02);
        OutboardLinearPositionServo servo = new OutboardLinearPositionServo(
                log, mech, ref, 0.02, 0.02);
        return new PositionIndexer(parent, servo, profiled);
    }

    public static VelocityIndexer makeVelocityIndexer(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM,
            boolean profiled) {
        LoggerFactory log = parent.name("indexer");
        LinearMechanism mech = getMech(
                log, currentLog, limit,
                canId, gearRatio, wheelDiaM);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                log, mech, ref, 1);
        return new VelocityIndexer(parent, servo, profiled);
    }

    private static LinearMechanism getMech(
            LoggerFactory log,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canId,
            double gearRatio,
            double wheelDiaM) {
        SimpleDynamics ff = new SimpleDynamics(log, 0, 0);
        Friction friction = new Friction(log, 0.07, 0.07, 0.01, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.02);
        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);
        BareMotor motor = getMotor(
                limit, log, currentLog, freeSpeedRad_S, canId,
                MotorPhase.REVERSE, ff, friction, pid);
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return mech;
    }

    /**
     * TODO: verify the velocity averaging parameters
     */
    private static BareMotor getMotor(
            CurrentLimit limit,
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double freeSpeedRad_S,
            CanId canId,
            MotorPhase phase,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, freeSpeedRad_S);
            default -> new MinionSparkMotor(
                    log, currentLog, canId, NeutralMode100.BRAKE, phase,
                    limit, ff, friction, pid, 2, 4);
        };
    }

}
