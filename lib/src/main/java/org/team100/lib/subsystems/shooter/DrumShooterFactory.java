package org.team100.lib.subsystems.shooter;

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
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 5.2307692308;
    private static final double WHEEL_DIA_M = .33;

    public static DualDrumShooter make(
            LoggerFactory parent, TotalCurrentLog currentLog, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        CanId canL = new CanId(39);
        CanId canR = new CanId(19);

        SimpleDynamics ff = new SimpleDynamics(log, 0, 0);
        Friction friction = new Friction(log, 0, 0.07, 0.01, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.02);

        BareMotor motorL = getMotor(currentLimit, logL, currentLog, canL, ff, friction, pid);
        BareMotor motorR = getMotor(currentLimit, logR, currentLog, canR, ff, friction, pid);

        LinearMechanism mechL = new LinearMechanism(
                logL, motorL, motorL.encoder(), GEAR_RATIO, WHEEL_DIA_M,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechR = new LinearMechanism(
                logR, motorR, motorR.encoder(), GEAR_RATIO, WHEEL_DIA_M,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        return new DualDrumShooter(parent,
                new OutboardLinearVelocityServo(logL, mechL, ref, 1),
                new OutboardLinearVelocityServo(logR, mechR, ref, 1));
    }

    private static BareMotor getMotor(int currentLimit,
            LoggerFactory log, TotalCurrentLog currentLog,
            CanId canId, SimpleDynamics ff,
            Friction friction, PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, 600);
            default -> new Neo550CANSparkMotor(
                    log, currentLog, canId, NeutralMode100.BRAKE, MotorPhase.REVERSE,
                    currentLimit, ff, friction, pid);
        };
    }

}
