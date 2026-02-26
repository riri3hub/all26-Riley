package org.team100.frc2026.robot;


import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveTargetLockDirect;
import org.team100.lib.targeting.LaserSolver;
import org.team100.lib.targeting.Solver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Binds buttons to commands. Also creates default commands.
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
    private final Machinery m_machinery;
    public final ControllerSE2 m_holonomicController;
    final LoggerFactory m_log;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");
        m_holonomicController = ControllerFactorySE2.byIdentity(m_log);

    }

    public void bind() {

        /////////////////////////////////////////////////
        ///
        /// CONTROLS
        ///
        final DriverXboxControl driver = new DriverXboxControl(0);

        /////////////////////////////////////////////////
        //
        // DEFAULT COMMANDS
        //

        // SwerveLimiter limiter = new SwerveLimiter(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // RobotController::getBatteryVoltage);
        // limiter.updateSetpoint(m_machinery.m_drive.getVelocity());

        m_machinery.m_drive.setDefaultCommand(
                new DriveFieldRelative(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter)
                        .withName("drive default"));

        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        m_machinery.m_intake.setDefaultCommand(
                m_machinery.m_intake.stop());
        m_machinery.m_extender.setDefaultCommand(
                m_machinery.m_extender.stop());
        m_machinery.m_shooterHood.setDefaultCommand(
                m_machinery.m_shooterHood.stop());

        ///////////////////////////
        //
        // DRIVETRAIN
        //

        // Become very unsure of location, so that camera authority is high for awhile.
        onTrue(driver::back, m_machinery.disorient());

        // This is for testing pose estimation accuracy and drivetrain positioning
        // accuracy.
        HolonomicProfile profile = HolonomicProfileFactory.get(
                m_log, m_machinery.m_swerveKinodynamics, 1, 0.5, 1, 0.2);
        onTrue(driver::a,
                new DriveToPoseWithProfile(
                        m_log, m_machinery.m_drive, m_holonomicController, profile,
                        () -> new Pose2d(15.387, 3.501, new Rotation2d(0))));
        /////////////////////////////////////////////////////////
        ///
        /// SUBSYSTEMS
        ///

        whileTrue(driver::b, m_machinery.m_shooter.shooterFullspeed());

        whileTrue(driver::x, m_machinery.m_intake.intake());

        whileTrue(driver::y, m_machinery.m_serializer.serialize());
        // Test bindings
        // whileTrue(driver::leftBumper, m_machinery.m_extender.goToExtendedPosition());
        // whileTrue(driver::rightBumper,
        // m_machinery.m_extender.goToRetractedPosition());
        // whileTrue(driver::rightTrigger,
        // m_machinery.m_ClimberExtension.setPosition());
        // whileTrue(driver::x,
        // m_machinery.m_ClimberExtension.setPosition()
        // .andThen(m_machinery.m_Climber.setClimb1()));
        // whileTrue(driver::b,
        // m_machinery.m_ClimberExtension.setPosition()
        // .andThen(m_machinery.m_Climber.setClimb3()));
        // whileTrue(driver::a,
        // m_machinery.m_Climber.setClimb0()
        // .andThen(m_machinery.m_ClimberExtension.setHomePosition()));

        // The real bindings
        whileTrue(driver::leftBumper,
                m_machinery.m_extender.goToRetractedPosition());
        whileTrue(driver::leftTrigger,
                m_machinery.m_extender.goToExtendedPosition()
                    .andThen(m_machinery.m_intake.intake()));


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

        // aim at the hub, button 5 and also in the alliance zone
        // this does not yet work.
        // Solver solver = getSolver();
        // Supplier<Optional<Translation2d>> target = () -> {
        // if
        // (FieldConstants2026.ALLIANCE_ZONE.contains(m_machinery.m_drive.getPose().getTranslation()))
        // {
        // return Optional.of(FieldConstants2026.HUB.toTranslation2d());
        // }
        // if
        // (FieldConstants2026.NEUTRAL_ZONE.contains(m_machinery.m_drive.getPose().getTranslation()))
        // {
        // return Optional.of(new Translation2d(0,
        // m_machinery.m_drive.getPose().getTranslation().getY()));
        // }
        // return Optional.empty();
        // };
        // CachedSolution tofSolution = new CachedSolution(
        // fieldLogger, m_machinery.m_drive::getState, target, solver);
        // here we rely only on PID so make it stronger
        // FeedbackR1 aggressiveFeedback = new FullStateFeedback(
        // m_log, 1, 0.1, true, 0.025, 0.25);
        // whileTrue(() -> driver.leftBumper(),
        // new DriveMovingTargetLock(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // driver::velocity,
        // m_machinery.m_localizer::setHeedRadiusM,
        // m_machinery.m_limiter,
        // tofSolution,
        // aggressiveFeedback,
        // m_machinery.m_drive)
        // .withName("Moving target lock"));

        ///////////////////////////
        /// SHOOT WHEN AT SPEED
        Command runshooter = m_machinery.m_shooter.shooterFullspeed();
        Command runhood = m_machinery.m_shooterHood.position();
        Command runserial = m_machinery.m_serializer.serialize();
        // TODO: maybe add a delay between runhood and runshooter?
        whileTrue(driver::rightTrigger,
                parallel(
                        runshooter,
                        runhood,
                        Commands.repeatingSequence(
                                waitUntil(m_machinery.m_shooter::atSpeed),
                                runserial.onlyWhile(m_machinery.m_shooter::atSpeed))));

        ///////////////////////////////////////////////////////////
        //
        // TEST
        //
        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());

    }

    private Solver getSolver() {
        // TOF solver is for a real 3d trajectory
        // this makes a parabolic path for testing
        // Drag d = new Drag(0, 0, 0, 1, 0);
        // double v = 7;
        // InverseRange ir = new InverseRange(d, 0, v, 0);
        // return new TimeOfFlightRecursion(ir, 0.01);

        // Laser solver is for aiming with the flashlight
        return new LaserSolver();
    }

    private static Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    private static Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }
}
