package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static org.team100.frc2026.util.TriggerUtil.onTrue;
import static org.team100.frc2026.util.TriggerUtil.whileTrue;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;
import org.team100.lib.subsystems.swerve.commands.manual.DriveTargetLockDirect;
import org.team100.lib.targeting.CachedSolution;
import org.team100.lib.targeting.InverseRange;
import org.team100.lib.targeting.ProxySolver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Binds buttons to commands. Also creates default commands.
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;
    private final ProxySolver solver;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        m_log = rootLogger.name("Commands");

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
        m_machinery.m_intake.setDefaultCommand(
                m_machinery.m_intake.stop());
        m_machinery.m_intakeExtend.setDefaultCommand(
                m_machinery.m_intakeExtend.stop());
        m_machinery.m_conveyor.setDefaultCommand(
                m_machinery.m_conveyor.stop());
        m_machinery.m_shooter.setDefaultCommand(
                m_machinery.m_shooter.stop());
        m_machinery.m_feeder.setDefaultCommand(
                m_machinery.m_feeder.stop());
        m_machinery.m_shooterHood.setDefaultCommand(
                m_machinery.m_shooterHood.stop());
        m_machinery.m_Climber.setDefaultCommand(
                m_machinery.m_Climber.stop());
        m_machinery.m_ClimberExtension.setDefaultCommand(
                m_machinery.m_ClimberExtension.stop());

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///

        // Forget the current pose, listen to camera input.
        onTrue(driver::back, m_machinery.disorient());
        // Nudge the rotation towards zero.
        onTrue(driver::start, m_machinery.zeroRotation());

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

        ////////////////////////////////////////////////////
        ///
        /// CLIMBER
        ///

        // whileTrue(driver::x,
        // m_machinery.m_ClimberExtension.setPosition()
        // .andThen(m_machinery.m_Climber.setClimb1()));
        // whileTrue(driver::a,
        // sequence(
        // m_machinery.m_ClimberExtension.setPosition().withTimeout(1),
        // m_machinery.m_Climber.setClimb3().withTimeout(1)));

        // whileTrue(driver::y,
        // m_machinery.m_Climber.setClimb0()
        // .andThen(m_machinery.m_ClimberExtension.setHomePosition()));

        // These are from ClimberExtendTEST
        // whileTrue(driver::x, m_machinery.m_ClimberExtension.setPosition());
        // whileTrue(driver::y, m_machinery.m_ClimberExtension.setHomePosition());
        // whileTrue(driver::rightTrigger,
        // m_machinery.m_ClimberExtension.setPosition());
        // whileTrue(driver::a, m_machinery.m_Climber.setClimb3());
        // whileTrue(driver::b, m_machinery.m_Climber.setClimb0());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///

        whileTrue(driver::leftBumper,
                m_machinery.m_intakeExtend.goToRetractedPosition());
        whileTrue(driver::leftTrigger,
                m_machinery.m_intakeExtend.goToExtendedPosition()
                        .andThen(m_machinery.m_intake.intake()));

        // For testing
        // whileTrue(driver::leftBumper,
        // m_machinery.m_intakeExtender.goToExtendedPosition());
        // whileTrue(driver::rightBumper,
        // m_machinery.m_intakeExtender.goToRetractedPosition());

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///

        FeedbackR1 thetaFeedback = new PIDFeedback(
                m_log, 3.2, 0, 0, true, 0.05, 1);

        Supplier<Optional<Translation2d>> target = () -> {
            return FieldConstants2026.TARGET(
                    m_machinery.m_drive.getState().translation());
        };

        // aim at the hub or our zone, button 6
        whileTrue(() -> driver.rightBumper(),
                new DriveTargetLockDirect(
                        fieldLogger,
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        target,
                        thetaFeedback,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_drive,
                        m_machinery.m_limiter)
                        .withName("Direct target lock"));

        InverseRange ir = new InverseRange(
                FieldConstants2026.FUEL_DRAG,
                0.75,
                2.5,
                FieldConstants2026.HUB.getZ(),
                FieldConstants2026.HUB_ELEVATION,
                7,
                1);
        solver = new ProxySolver(ir);
        CachedSolution tofSolution = new CachedSolution(
                fieldLogger, m_machinery.m_drive::getState, target, solver);
        // here we rely only on PID so make it stronger
        FeedbackR1 aggressiveFeedback = new FullStateFeedback(
                m_log, 3, 0.1, true, 0.025, 0.25);

        // button 5
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

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///

        Command runShooter = m_machinery.m_shooter.testShooterFullspeed();
        Command runHood = m_machinery.m_shooterHood.position();
        Command runConveyor = m_machinery.m_conveyor.testConveyor();
        Command runConveyorBack = m_machinery.m_conveyor.testConveyorBack();
        Command runFeeder = m_machinery.m_feeder.testFeed();
        Command runShooter3 = m_machinery.m_shooter.testMotor3Command();
        Command runFeederBack = m_machinery.m_feeder.testFeedBack();
        // whileTrue(driver::rightTrigger,
        // parallel(
        // runHood,
        // runShooter,
        // runSerialUpper,
        // Commands.repeatingSequence(
        // waitUntil(m_machinery.m_shooter::atSpeed),
        // runSerial.onlyWhile(m_machinery.m_shooter::atSpeed))));

        // For testing
        // whileTrue(driver::x, m_machinery.m_shooter.shooterFullspeed());
        // whileTrue(driver::x, m_machinery.m_shooter.testMotor1Command());
        // whileTrue(driver::y, m_machinery.m_shooter.testMotor2Command());
        // whileTrue(driver::a, m_machinery.m_shooter.testMotor3Command());
        // whileTrue(driver::b, parallel(runShooter, runSerial, runSerialUpper));
        whileTrue(driver::a, parallel(runConveyor, runFeeder));
        whileTrue(driver::b, parallel(runConveyorBack, runFeederBack));
        whileTrue(driver::x, runShooter);

        // whileTrue(driver::rightTrigger, parallel(runSerial, runSerialUpper,
        // runShooter));
        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///

        Tester tester = new Tester(m_machinery);
        whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
                tester.prematch());

    }

    /** Keeps tests from conflicting. */
    public void close() {
        solver.close();
    }
}
