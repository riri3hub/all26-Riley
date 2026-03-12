package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveFieldRelative extends Command {
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
    // LOGGERS
    private final VelocitySE2Logger m_log_scaled;

    public DriveFieldRelative(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        LoggerFactory log = parent.type(this);
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_log_scaled = log.VelocitySE2Logger(Level.TRACE, "scaled");
        m_swerveKinodynamics = swerveKinodynamics;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(m_drive.getVelocity());
    }

    @Override
    public void execute() {
        Velocity clipped = m_twistSupplier.get().clip(1.0);
        VelocitySE2 scaled1 = VelocitySE2.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        m_log_scaled.log(() -> scaled1);

        // scale for driver skill.
        VelocitySE2 scaled = GeometryUtil.scale(scaled1, DriverSkill.level().scale());
        // Apply field-relative limits.

        if (Experiments.instance.enabled(Experiment.UseSwerveLimiter)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

}
