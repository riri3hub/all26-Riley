package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.frc2026.util.TriggerUtil.onTrue;
import static org.team100.frc2026.util.TriggerUtil.whileTrue;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveTargetLockDirect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Binds buttons to commands. Also creates default commands.
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");
    }

    public void bind() {

        ////////////////////////////////////////////////////
        ///
        /// CONTROLLER
        ///

        DriverXboxControl driver = new DriverXboxControl(0);

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
        m_machinery.m_serializer.setDefaultCommand(
            m_machinery.m_serializer.stop());
        m_machinery.m_serializerUpper.setDefaultCommand(
            m_machinery.m_serializerUpper.stop());

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        onTrue(driver::back, m_machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// TEST/DEV
        ///

        // This is for testing pose estimation accuracy and drivetrain positioning
        // accuracy.
        HolonomicProfile profile = HolonomicProfileFactory.get(
                m_log, m_machinery.m_swerveKinodynamics, 1, 0.5, 1, 0.2);
        onTrue(driver::b,
                new DriveToPoseWithProfile(
                        m_log, m_machinery.m_drive, m_machinery.m_holonomicController,
                        profile, () -> new Pose2d(15.387, 3.501, new Rotation2d(0))));



        // whileTrue(driver::leftBumper, m_machinery.m_extender.goToExtendedPosition());
        // whileTrue(driver::rightBumper,
        // m_machinery.m_extender.goToRetractedPosition());
        //
        // whileTrue(driver::rightTrigger,
        // m_machinery.m_ClimberExtension.setPosition());

        //CLIMBER
        whileTrue(driver::x,
        m_machinery.m_ClimberExtension.setPosition()
        .andThen(m_machinery.m_Climber.setClimb1()));
        whileTrue(driver::a,
        m_machinery.m_ClimberExtension.setPosition()
        .andThen(m_machinery.m_Climber.setClimb3()));
        whileTrue(driver::y,
        m_machinery.m_Climber.setClimb0()
        .andThen(m_machinery.m_ClimberExtension.setHomePosition()));

        // whileTrue(driver::y, m_machinery.m_shooter.testMotor1Command());
        // whileTrue(driver::a, m_machinery.m_shooter.testMotor2Command());
        // whileTrue(driver::x, m_machinery.m_shooter.testMotor3Command());
        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        whileTrue(driver::leftBumper,
                m_machinery.m_extender.goToRetractedPosition());
        whileTrue(driver::leftTrigger,
                m_machinery.m_extender.goToExtendedPosition()
                        .andThen(m_machinery.m_intake.intake()));

        ////////////////////////////////////////////////////
        ///
        /// AIM
        FeedbackR1 thetaFeedback = new PIDFeedback(
                m_log, 3.2, 0, 0, true, 0.05, 1);
        // aim at the hub, button 6 and also in the alliance zone
        whileTrue(() -> driver.rightBumper()
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
        // aim at our zone, button 6 and in the neutral zone
        whileTrue(() -> driver.rightBumper()
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
        Command runShooter = m_machinery.m_shooter.shooterFullspeed();
        Command runHood = m_machinery.m_shooterHood.position();
        Command runSerial = m_machinery.m_serializer.serialize();
        Command runSerialUpper = m_machinery.m_serializerUpper.serializerUpperFullspeed();

        whileTrue(driver::rightTrigger,
                parallel(
                        runHood,
                        runShooter,
                        runSerialUpper,
                        Commands.repeatingSequence(
                                waitUntil(m_machinery.m_shooter::atSpeed),
                                runSerial.onlyWhile(m_machinery.m_shooter::atSpeed))));

        ////////////////////////////////////////////////////
        //
        // TEST
        //
        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());

    }
}
