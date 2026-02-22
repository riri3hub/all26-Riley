package org.team100.lib.subsystems.swerve.module;

import java.util.List;
import java.util.Optional;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.music.Player;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;

import edu.wpi.first.math.MathUtil;
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
 * TODO: Verify the coupling
 * 
 * There is some discussion of this topic here:
 * https://www.chiefdelphi.com/t/kcoupleratio-in-ctre-swerve/483380
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

    /**
     * The previous desired angle, used if the current desired angle is empty (i.e.
     * the module is motionless) and to calculate steering velocity.
     * This is set to the "next" value and read, one step later, as the "current"
     * value
     */
    private Rotation2d m_previousDesiredWrappedAngle;
    private double m_previousTime;

    private final List<Player> m_players;

    protected SwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo,
            double wheelDiameterM,
            double finalDriveRatio) {
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

    double turningPosition() {
        return m_turningServo.getWrappedPositionRad();
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

    static double reduceCrossTrackError(
            double measuredWrappedAngleRad, double desiredSpeed, Rotation2d desiredWrappedAngle) {
        double error = MathUtil.angleModulus(desiredWrappedAngle.getRadians() - measuredWrappedAngleRad);
        // cosine is pretty forgiving of misalignment
        // double scale = Math.abs(Math.cos(error));
        // gaussian is much less forgiving. note the adjustable factor. The value of
        // 4 means there is almost no motion past about 60 degrees of error.
        final double width = 4.0;
        double scale = Math.exp(-width * error * error);
        return scale * desiredSpeed;
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Turning servo commands compute the velocity based on the previous desired
     * angle.
     * 
     * @param nextWrapped for now+dt, i.e. "next"
     */
    private void actuate(SwerveModuleState100 nextWrapped) {
        double nextSpeed = nextWrapped.speedMetersPerSecond();
        if (nextWrapped.angle().isEmpty())
            throw new IllegalArgumentException("actuation needs a real angle");

        Rotation2d nextWrappedAngle = nextWrapped.angle().get();
        double dt = dt();
        double nextOmega = omega(nextWrappedAngle, dt);

        // help drive motors overcome steering.
        nextSpeed = correctSpeedForSteering(nextSpeed, nextOmega, dt);

        m_driveServo.setVelocity(nextSpeed);

        // Don't use a profile. This uses more current, but only briefly,
        // and it's crisper.
        m_turningServo.setPositionDirect(nextWrappedAngle.getRadians(), nextOmega, 0);

        m_previousDesiredWrappedAngle = nextWrappedAngle;
    }

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
