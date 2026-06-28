package org.team100.lib.subsystems.swerve.module;

import java.util.List;
import java.util.Optional;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.music.Player;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Control of a single module.
 * 
 * Everything here is package-private because SwerveModuleCollection is the only
 * client.
 * 
 * Implements drive/steer coupling. The path from the drive motor to the wheel
 * involves a shaft on the steering axis; driving the wheel requires *relative*
 * motion of this shaft and the steering axis itself. So when the steering axis
 * moves, it affects the relative position of the drive motor and the drive
 * wheel.
 * 
 * There is some discussion of this topic here:
 * https://www.chiefdelphi.com/t/kcoupleratio-in-ctre-swerve/483380
 * 
 * In the past, we tried to reduce "cross track error" by moderating
 * module speed when the wheels were pointing in the wrong direction,
 * as some other teams do. This was found to be unhelpful, because it
 * happened on a per-module basis, disturbing the rotation of the whole
 * robot.
 */
public abstract class SwerveModule100 implements Player {
    private static final boolean DEBUG = false;

    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;
    /** For steer/drive coupling. */
    private final double m_wheelRadiusM;
    /**
     * This is actually the final drive times the intermediate. The WCP modules
     * final is 3, and the intermediate is like 0.9 or 1.1 depending on which ratio
     * we're using.
     */
    private final double m_finalDriveRatio;

    /** For playing sounds */
    private final List<Player> m_players;

    private final DoubleLogger m_log_dt;
    private final DoubleLogger m_log_speed;
    private final DoubleLogger m_log_omega;

    /**
     * The previous desired angle, used if the current desired angle is empty (i.e.
     * the module is motionless) and to calculate steering velocity.
     * This is set to the "next" value and read, one step later, as the "current"
     * value
     */
    private Rotation2d m_previousDesiredWrappedAngle;
    private double m_previousTime;

    protected SwerveModule100(
            LoggerFactory log,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo,
            double wheelDiameterM,
            double finalDriveRatio) {
        m_log_dt = log.doubleLogger(Level.TRACE, "dt");
        m_log_speed = log.doubleLogger(Level.TRACE, "speed");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        m_wheelRadiusM = wheelDiameterM / 2;
        // The initial previous angle is the measurement.
        m_previousDesiredWrappedAngle = new Rotation2d(m_turningServo.getWrappedPositionRad());
        m_previousTime = Takt.get();
        m_finalDriveRatio = finalDriveRatio;
        m_players = List.of(m_driveServo, m_turningServo);
    }

    @Override
    public void play(double freq) {
        m_driveServo.play(freq);
        m_turningServo.play(freq);
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    /**
     * Optimizes.
     * 
     * Given an empty angle, it uses the previous one.
     * 
     * Note: this uses "wrapped" states that come from inverse kinematics, since the
     * kinematics doesn't care about the total turns of the modules;
     * 
     * @param desiredWrapped for now+dt
     */
    void setDesiredState(SwerveModuleState100 desiredWrapped) {
        desiredWrapped = usePreviousAngleIfEmpty(desiredWrapped);
        desiredWrapped = optimize(desiredWrapped);
        actuate(desiredWrapped);
    }

    /**
     * Does not optimize.
     * 
     * Given an empty angle, it uses the previous one.
     */
    void setRawDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        actuate(desired);
    }

    /** Set turning setpoint to measurement, zero drive encoder. */
    void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    void close() {
        m_turningServo.close();
    }

    /** FOR TEST ONLY */
    SwerveModuleState100 getState() {
        double driveVelocity = m_driveServo.getVelocity();
        double turningPosition = m_turningServo.getWrappedPositionRad();

        return new SwerveModuleState100(
                driveVelocity,
                Optional.of(new Rotation2d(turningPosition)));
    }

    /** Uses Cache so the position is fresh and coherent. */
    SwerveModulePosition100 getPosition() {
        double driveM = m_driveServo.getDistance();
        double unwrappedAngleRad = m_turningServo.getUnwrappedPositionRad();
        switch (Identity.instance) {
            case SWERVE_ONE:
            case SWERVE_TWO:
            case COMP_BOT:
                driveM = correctPositionForSteering(driveM, unwrappedAngleRad);
                break;
            case BLANK:
            default:
                break;
        }
        return new SwerveModulePosition100(
                driveM,
                Optional.of(new Rotation2d(unwrappedAngleRad)));
    }

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
    }

    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }

    /** Update logs. */
    void periodic() {
        m_driveServo.periodic();
        m_turningServo.periodic();
    }

    /**
     * Turning servo commands compute the velocity based on the previous desired
     * angle.
     * 
     * @param nextWrapped for now+dt, i.e. "next"
     */
    void actuate(SwerveModuleState100 nextWrapped) {
        if (nextWrapped.angle().isEmpty())
            throw new IllegalArgumentException("actuation needs a real angle");
        Rotation2d nextWrappedAngle = nextWrapped.angle().get();
        // TODO: Experiment with fixed DT here.
        double dt = dt();
        m_log_dt.log(() -> dt);

        // Deduce the desired omega using backward finite difference.
        double nextOmega = omega(nextWrappedAngle, dt);
        m_log_omega.log(() -> nextOmega);

        // Adjust the drive speed to compensate for steering movement.
        double nextSpeed = correctSpeedForSteering(
                nextWrapped.speedMetersPerSecond(),
                nextOmega,
                dt);
        m_log_speed.log(() -> nextSpeed);

        // Acceleration may be a source of noise, so optionally ignore it.
        if (Experiments.instance.enabled(Experiment.DriveWithoutAccel)) {
            // Zero acceleration.
            m_driveServo.setVelocityDirect(nextSpeed, 0);
        } else {
            // Deduces acceleration.
            m_driveServo.setVelocityDirect(nextSpeed);
        }

        // Steering omega may be a source of noise, so optionally ignore it.
        double omega = nextOmega;
        if (Experiments.instance.enabled(Experiment.SteerWithoutVelocity)) {
            omega = 0;
        }

        // Direct actuation uses more current than a profile, but only briefly,
        // and it's much faster, and avoids the oscillation that the profile can produce
        // around the goal.
        //
        // Note that, if the steering controller "P" value and current limits are
        // set quite high, then direct mode can consume a whole lot of current, which
        // is generally not worth the small benefit in steering quickness.
        //
        // So be careful! Use a reasonable "P" value. Use supply limits to
        // regulate the impact on the battery. This will induce some delay,
        // and controller error, but it's not bad: it's using the actual
        // constraint (current) instead of a proxy (profile acceleration).
        m_turningServo.setPositionDirect(nextWrappedAngle.getRadians(), omega, 0);

        // This is the alternative profiled steering used in 2025.
        // m_turningServo.setPositionProfiled(nextWrappedAngle.getRadians(), omega);

        m_previousDesiredWrappedAngle = nextWrappedAngle;
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Correct the desired speed for steering coupling.
     */
    private double correctSpeedForSteering(double speed, double omega, double dt) {
        double correction = speedCorrection(speed, omega, dt);
        if (DEBUG) {
            System.out.printf("correction %6.3f\n", correction);
        }
        return speed + correction;
    }

    private double speedCorrection(double desiredSpeed, double omega, double dt) {
        if (dt > 0.04) {
            // clock is unreliable, don't do anything.
            return 0;
        }
        if (dt < 0.01) {
            // avoid short intervals
            return 0;
        }
        return m_wheelRadiusM * omega / m_finalDriveRatio;
    }

    /**
     * use the previous desired angle to compute steering angular velocity in
     * radians per sec
     * 
     * @param desiredWrappedAngle angle for the next timestep
     * @param dt                  time until then (s)
     * @returns rad/s
     */
    private double omega(Rotation2d desiredWrappedAngle, double dt) {
        if (dt < 1e-6)
            return 0;
        // dtheta is definitely a lot less than 2pi so wrapped is fine.
        Rotation2d dthetaWrapped = desiredWrappedAngle.minus(m_previousDesiredWrappedAngle);
        return dthetaWrapped.getRadians() / dt;
    }

    /**
     * Correct position measurement for steering coupling.
     */
    private double correctPositionForSteering(double drive_M, double unwrappedAngleRad) {
        // steering opposes driving.
        return drive_M - m_wheelRadiusM * unwrappedAngleRad / m_finalDriveRatio;
    }

    /**
     * Use the current turning servo position to optimize the desired state.
     */
    private SwerveModuleState100 optimize(SwerveModuleState100 desiredWrapped) {
        return SwerveModuleState100.optimize(
                desiredWrapped,
                new Rotation2d(m_turningServo.getWrappedPositionRad()));
    }

    /**
     * If the desired angle is empty, replace it with the previous desired angle.
     */
    private SwerveModuleState100 usePreviousAngleIfEmpty(SwerveModuleState100 desired) {
        if (desired.angle().isEmpty()) {
            return new SwerveModuleState100(
                    desired.speedMetersPerSecond(),
                    Optional.of(m_previousDesiredWrappedAngle));
        }
        return desired;
    }

    private double dt() {
        double now = Takt.get();
        double dt = now - m_previousTime;
        m_previousTime = now;
        return dt;
    }
}
