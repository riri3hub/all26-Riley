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
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.targeting.Solution;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A version of target lock that allows moving robot, with fixed target.
 * 
 * NOTE this does not yet work :-)
 */
public class DriveMovingTargetLock extends Command {
    /**
     * I'm not sure why feedforward seems too low; this just makes it bigger.
     * TODO: get rid of this.
     */
    private static final int FEEDFORWARD_SCALE = 2;

    private static final double HEED_RADIUS_M = 6.0;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveLimiter m_limiter;
    private final Supplier<Optional<Solution>> m_solver;
    private final FeedbackR1 m_thetaController;
    private final SwerveDriveSubsystem m_drive;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;
    private final DoubleLogger m_log_omega;

    // feedback operates on the previous goal;
    // feedforward should suffice for the next goal.
    private ModelR1 m_goal;

    public DriveMovingTargetLock(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveLimiter limiter,
            Supplier<Optional<Solution>> solver,
            FeedbackR1 thetaController,
            SwerveDriveSubsystem drive) {
        LoggerFactory log = parent.type(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_limiter = limiter;
        m_solver = solver;
        m_thetaController = thetaController;
        m_drive = drive;
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        log.doubleLogger(Level.TRACE, "max omega").log(swerveKinodynamics::getMaxAngleSpeedRad_S);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        m_limiter.updateSetpoint(m_drive.getVelocity());
        m_goal = m_drive.getState().theta();
        m_thetaController.reset();
    }

    @Override
    public void execute() {

        Optional<Solution> oSolution = m_solver.get();
        if (oSolution.isEmpty()) {
            // there's no target, so use the driver input.
            actuate(null);
            return;
        }
        // Setpoints for the next time step
        Solution solution = oSolution.get();

        ModelSE2 state = m_drive.getState();

        // Feedback uses the previous goal
        double thetaFB = m_thetaController.calculate(state.theta(), m_goal);
        m_log_thetaFB.log(() -> thetaFB);

        double yaw = state.pose().getRotation().getRadians();
        double goalYaw = Math100.getMinDistance(yaw, solution.azimuth().getRadians());
        m_goal = new ModelR1(goalYaw, 0);
        // m_goal = new ModelR1(goalYaw, solution.azimuthVelocity());
        m_log_goal.log(() -> m_goal);

        double thetaFF = m_goal.v() * FEEDFORWARD_SCALE;
        m_log_thetaFF.log(() -> thetaFF);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        m_log_omega.log(() -> omega);

        actuate(omega);
    }

    /** Null to skip override */
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
