
package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.controller.r1.SimpleAim;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

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
     * Pay attention to tags even if they are far away.
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

    private final SimpleAim m_aim;
    private final BooleanLogger m_log_aiming;

    public DriveTargetLockDirect(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Optional<Translation2d>> target,
            SimpleAim aim,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_log_aiming = log.booleanLogger(Level.TRACE, "aiming");
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_aim = aim;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(m_drive.getVelocity());
        m_aim.reset();
    }

    @Override
    public void execute() {
        actuate(getOmega());
    }

    private Double getOmega() {
        Optional<Translation2d> oTarget = m_target.get();
        if (oTarget.isEmpty()) {
            // No target, so use the driver input.
            return null;
        }
        Translation2d target = oTarget.get();
        return m_aim.getOmega(m_drive.getState(), target);
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
        m_log_aiming.log(() -> omega != null);
        if (omega != null) {
            v = new VelocitySE2(v.x(), v.y(), omega);
        }

        // Actuate the drivetrain.
        m_drive.setVelocity(v);
    }

}
