package org.team100.frc2026.robot;

import static org.team100.frc2026.util.TriggerUtil.whileTrue;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;
import org.team100.lib.targeting.CachedSolution;
import org.team100.lib.targeting.Drag;
import org.team100.lib.targeting.InverseRange;
import org.team100.lib.targeting.LaserSolver;
import org.team100.lib.targeting.Solver;
import org.team100.lib.targeting.TimeOfFlightRecursion;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Stuff I want to come back to sometime.
 * 
 * At the moment (Feb 26) neither TOFR nor Laser seem to work.
 */
public class SolverBinder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private static final boolean TOFR = false;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;

    public SolverBinder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");
    }

    public void bind() {

        DriverXboxControl driver = new DriverXboxControl(0);

        // aim at the hub, button 5 and also in the alliance zone
        // this does not yet work.
        Solver solver = getSolver();
        Supplier<Optional<Translation2d>> target = () -> {
            if (FieldConstants2026.ALLIANCE_ZONE.contains(m_machinery.m_drive.getPose().getTranslation())) {
                return Optional.of(FieldConstants2026.HUB.toTranslation2d());
            }
            if (FieldConstants2026.NEUTRAL_ZONE.contains(m_machinery.m_drive.getPose().getTranslation())) {
                return Optional.of(new Translation2d(0,
                        m_machinery.m_drive.getPose().getTranslation().getY()));
            }
            return Optional.empty();
        };
        CachedSolution tofSolution = new CachedSolution(
                fieldLogger, m_machinery.m_drive::getState, target, solver);
        // here we rely only on PID so make it stronger
        FeedbackR1 aggressiveFeedback = new FullStateFeedback(
                m_log, 1, 0.1, true, 0.025, 0.25);
        whileTrue(() -> driver.leftBumper(),
                new DriveMovingTargetLock(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_limiter,
                        tofSolution,
                        aggressiveFeedback,
                        m_machinery.m_drive)
                        .withName("Moving target lock"));
    }

    private Solver getSolver() {
        if (TOFR) {
            // TOF solver is for a real 3d trajectory
            // this makes a parabolic path for testing
            Drag d = new Drag(0, 0, 0, 1, 0);
            double v = 7;
            InverseRange ir = new InverseRange(d, 0, v, 0);
            return new TimeOfFlightRecursion(ir, 0.01);
        }
        // Laser solver is for aiming with the flashlight
        return new LaserSolver();
    }

}
