
package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.targeting.TargetUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses velocity feedforward and feedback, but no profile, because I
 * think the profile might be a source of noise.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 */
public class DriveTargetLockDirect extends Command {
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

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Optional<Translation2d>> m_target;
    private final FeedbackR1 m_thetaController;

    private final DoubleLogger m_log_apparent_motion;
    private final DoubleArrayLogger m_log_target;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;
    private final DoubleLogger m_log_omega;

    public DriveTargetLockDirect(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Optional<Translation2d>> target,
            FeedbackR1 thetaController,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(m_drive.getVelocity());
        m_thetaController.reset();
    }

    @Override
    public void execute() {
        ModelSE2 state = m_drive.getState();

        Optional<Translation2d> oTarget = m_target.get();
        if (oTarget.isEmpty()) {
            // no target, just use driver input
            actuate(null);
            return;
        }
        // the goal omega should match the target's apparent motion
        Translation2d target = oTarget.get();

        m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });
        double targetMotion = TargetUtil.targetMotion(state, target);
        m_log_apparent_motion.log(() -> targetMotion);

        double unwrappedBearing = TargetUtil.unwrappedAbsoluteBearing(state.pose(), target);
        // eliminate target motion to reduce noise
        // TODO: put back target motion
        // ModelR1 goal = new ModelR1(unwrappedBearing, 0);
        final ModelR1 goal = new ModelR1(unwrappedBearing, targetMotion);
        m_log_goal.log(() -> goal);

        // Feedforward is the goal velocity
        double thetaFF = goal.v();
        m_log_thetaFF.log(() -> thetaFF);

        // Feedback uses the goal, there's no profile.
        double thetaFB = m_thetaController.calculate(state.theta(), goal);
        m_log_thetaFB.log(() -> thetaFB);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        m_log_omega.log(() -> omega);

        actuate(omega);
    }

    private void actuate(Double omega) {
        // Clip and scale user input.
        VelocitySE2 v = VelocitySE2.scale(
                m_twistSupplier.get().clip(1.0),
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // Scale for driver skill.
        v = GeometryUtil.scale(v, DriverSkill.level().scale());

        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            v = m_limiter.apply(v);
        }

        // Override omega.
        if (omega != null) {
            v = new VelocitySE2(v.x(), v.y(), omega);
        }

        // Actuate the drivetrain.
        m_drive.setVelocity(v);
    }

}
