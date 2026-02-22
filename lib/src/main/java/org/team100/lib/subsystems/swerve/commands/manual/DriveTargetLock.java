
package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 */
public class DriveTargetLock extends Command {
    /**
     * While driving manually, pay attention to tags even if they are somewhat far
     * away.
     */
    private static final double HEED_RADIUS_M = 6.0;

    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveLimiter m_limiter;

    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double ROTATION_SPEED = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_target;
    private final FeedbackR1 m_thetaController;
    private final IncrementalProfile m_profile;

    private final DoubleLogger m_log_apparent_motion;
    public final DoubleArrayLogger m_log_target;

    private ControlR1 m_thetaSetpoint;

    public DriveTargetLock(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Translation2d> target,
            FeedbackR1 thetaController,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_profile = new TrapezoidIncrementalProfile(
                log,
                swerveKinodynamics.getMaxAngleSpeedRad_S() * ROTATION_SPEED,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * ROTATION_SPEED,
                0.01);
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(m_drive.getVelocity());
        ModelSE2 p = m_drive.getState();
        m_thetaSetpoint = p.theta().control();
        m_thetaController.reset();
    }

    @Override
    public void execute() {
        Velocity t = m_twistSupplier.get();
        ModelSE2 s = m_drive.getState();
        VelocitySE2 v = apply(s, t);
        // scale for driver skill.
        VelocitySE2 scaled = GeometryUtil.scale(v, DriverSkill.level().scale());
        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * This uses the current-instant setpoint to calculate feedback error.
     * 
     * It uses the next-time-step setpoint for feedforward.
     * 
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    public VelocitySE2 apply(ModelSE2 state, Velocity input) {

        //
        // feedback is based on the previous setpoint.
        //

        final double thetaFB = m_thetaController.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // update the setpoint for the next time step
        //

        // the goal omega should match the target's apparent motion
        final Translation2d target = m_target.get();
        final double targetMotion = TargetUtil.targetMotion(state, target);

        final Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d absoluteBearing = TargetUtil.absoluteBearing(currentTranslation, target);

        final double yaw = state.theta().x();
        absoluteBearing = new Rotation2d(
                Math100.getMinDistance(yaw, absoluteBearing.getRadians()));

        final ModelR1 goal = new ModelR1(absoluteBearing.getRadians(), targetMotion);

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new ControlR1(
                Math100.getMinDistance(yaw, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goal);

        // feedforward is for the next time step
        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        final VelocitySE2 scaledInput = getScaledInput(input);

        final VelocitySE2 twistWithLockM_S = new VelocitySE2(
                scaledInput.x(), scaledInput.y(), omega);

        m_log_apparent_motion.log(() -> targetMotion);
        m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });

        return twistWithLockM_S;
    }

    private VelocitySE2 getScaledInput(Velocity input) {
        // clip the input to the unit circle
        Velocity clipped = input.clip(1.0);
        // this is user input scaled to m/s and rad/s
        VelocitySE2 scaledInput = VelocitySE2.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        return scaledInput;
    }

}
