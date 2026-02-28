package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.frc2026.util.TriggerUtil.onTrue;
import static org.team100.frc2026.util.TriggerUtil.whileTrue;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.hid.InterLinkDX;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveTargetLockDirect;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Control bindings for the Interlink DX. Also default commands.
 * 
 * See https://my.spektrumrc.com/ProdInfo/Files/SPMRFTX1-Manual-EN.pdf
 */
public class InterlinkBinder {

    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private final Machinery m_machinery;
    final LoggerFactory m_log;

    public InterlinkBinder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");
    }

    public void bind() {

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///
        InterLinkDX driver = new InterLinkDX(0);

        ////////////////////////////////////////////////////
        ///
        /// DEFAULT COMMANDS
        ///
        m_machinery.m_drive.setDefaultCommand(
                new DriveFieldRelative(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter));
        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        m_machinery.m_intake.setDefaultCommand(
                m_machinery.m_intake.stop());
        m_machinery.m_extender.setDefaultCommand(
                m_machinery.m_extender.stop());
        m_machinery.m_shooterHood.setDefaultCommand(
                m_machinery.m_shooterHood.stop());

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        onTrue(driver::reset, m_machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///
        whileTrue(driver::c2,
                m_machinery.m_extender.goToRetractedPosition());
        whileTrue(driver::c0,
                m_machinery.m_extender.goToExtendedPosition()
                        .andThen(m_machinery.m_intake.intake()));

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        FeedbackR1 thetaFeedback = new PIDFeedback(
                m_log, 3.2, 0, 0, true, 0.05, 1);
        // aim at the hub while in the alliance zone
        whileTrue(() -> driver.a1()
                && FieldConstants2026.ALLIANCE_ZONE.contains(m_machinery.m_drive.getPose().getTranslation()),
                new DriveTargetLockDirect(
                        fieldLogger,
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        () -> FieldConstants2026.HUB.toTranslation2d(),
                        thetaFeedback,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter)
                        .withName("Aim to shoot"));
        // aim at our zone while in the neutral zone
        whileTrue(() -> driver.a1()
                && FieldConstants2026.NEUTRAL_ZONE.contains(m_machinery.m_drive.getPose().getTranslation()),
                new DriveTargetLockDirect(
                        fieldLogger,
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        () -> {
                            Translation2d t = m_machinery.m_drive.getPose().getTranslation();
                            return new Translation2d(0, t.getY());
                        },
                        thetaFeedback,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter)
                        .withName("Aim to lob"));

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///
        Command runshooter = m_machinery.m_shooter.shooterFullspeed();
        Command runhood = m_machinery.m_shooterHood.position();
        Command runserial = m_machinery.m_serializer.serialize();
        whileTrue(driver::i,
                parallel(
                        runshooter,
                        runhood,
                        Commands.repeatingSequence(
                                waitUntil(m_machinery.m_shooter::atSpeed),
                                runserial.onlyWhile(m_machinery.m_shooter::atSpeed))));

        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///
        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.reset() && driver.cancel()),
                tester.prematch());

    }

}
