package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.frc2026.util.TriggerUtil.onTrue;
import static org.team100.frc2026.util.TriggerUtil.whileTrue;

import org.team100.lib.controller.r1.AzimuthController;
import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.swerve.commands.manual.DriveFieldRelative;
import org.team100.lib.subsystems.swerve.commands.manual.DriveMovingTargetLock;

/**
 * Binds buttons to commands. Also creates default commands.
 * 
 * See
 * https://docs.google.com/document/d/15HcburjCvwOEBL8ZtQdk-7iotF5qATGK3fO7c5HyWCk
 */
public class Binder {
    private static final LoggerFactory rootLogger = Logging.instance().rootLogger;
    @SuppressWarnings("unused")
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Machinery m_machinery;
    private final LoggerFactory m_log;

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

        ////////////////////////////////////////////////////
        ///
        /// DISORIENT
        ///
        /// Back: nudge the rotation towards zero.
        /// Start: forget the current pose, listen to camera input.

        onTrue(driver::back, m_machinery.zeroRotation());
        onTrue(driver::start, m_machinery.disorient());

        ////////////////////////////////////////////////////
        ///
        /// INTAKE
        ///
        /// Right trigger: extend, then hold extended and intake
        /// Right bumper: retract
        /// Both: roll backwards to clear jams (only when out)
        /// "Y" wobble intake to help clear jams

        whileTrue(() -> driver.rightBumper()
                && !driver.rightTrigger(),
                m_machinery.m_intakeExtend.goToRetractedPosition());

        whileTrue(() -> driver.rightTrigger()
                && !driver.rightBumper(),
                parallel(
                        m_machinery.m_intakeExtend.goToExtendedPositionEndlessly(),
                        sequence(
                                waitUntil(m_machinery.m_intakeExtend::atGoal),
                                m_machinery.m_intake.intake())));

        whileTrue(() -> driver.rightTrigger()
                && driver.rightBumper()
                && m_machinery.m_intakeExtend.isOut(),
                m_machinery.m_intake.back());

        whileTrue(driver::y,
                repeatingSequence(
                        m_machinery.m_intakeExtend.goToWobbleSlightlyInExtendedPosition().withTimeout(0.5),
                        m_machinery.m_intakeExtend.goToWobbleSlightlyOutRetractedPosition().withTimeout(0.5)));

        ////////////////////////////////////////////////////
        ///
        /// AIM
        ///
        /// Left bumper: rotate the robot to hit the target

        FeedbackR1 thetaFeedback = new FullStateFeedback(
                m_log, 6, 0.1, true, 0.025, 0.25);

        // button 6
        AzimuthController aim = new AzimuthController(
                m_log,
                m_machinery.m_swerveKinodynamics::getMaxAngleSpeedRad_S,
                thetaFeedback);
        whileTrue(() -> driver.leftBumper(),
                new DriveMovingTargetLock(
                        m_log,
                        m_machinery.m_swerveKinodynamics,
                        aim,
                        driver::velocity,
                        m_machinery.m_localizer::setHeedRadiusM,
                        m_machinery.m_limiter,
                        m_machinery.m_cachedSolution,
                        m_machinery.m_drive)
                        .withName("Target lock"));

        ////////////////////////////////////////////////////
        ///
        /// SHOOT
        ///
        /// Left trigger:
        /// * run the drums
        /// * set the hood
        /// * run the conveyor
        /// * feed when at speed.
        /// "A" failsafe, run everything at a speed for 2.5m
        /// "B" run the conveyor and feeder backwards

        whileTrue(driver::leftTrigger,
                parallel(
                        m_machinery.m_shooterHood.autoPosition(),
                        m_machinery.m_shooter.auto(),
                        m_machinery.m_conveyor.convey(),
                        repeatingSequence(
                                waitUntil(m_machinery.m_shooter::atSpeed),
                                m_machinery.m_feeder.normal()
                                        .onlyWhile(m_machinery.m_shooter::atSpeed))));

        whileTrue(driver::a,
                parallel(
                        m_machinery.m_shooterHood.failsafe(),
                        m_machinery.m_shooter.failsafe(),
                        m_machinery.m_conveyor.convey(),
                        m_machinery.m_feeder.normal()));

        whileTrue(driver::b,
                parallel(
                        m_machinery.m_conveyor.back(),
                        m_machinery.m_feeder.back()));

        ////////////////////////////////////////////////////
        ///
        /// AUTON TESTING
        ///
        /// Auton test mode is with POV down.

        // whileTrue(() -> driver.povDown() && driver.a(),
        // new CenterFullSweepAuton(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // m_machinery.m_holonomicController,
        // m_machinery).command());

        // whileTrue(() -> driver.povDown() && driver.b(),
        // new CenterHalfSweepAuton(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // m_machinery.m_holonomicController,
        // m_machinery).command());
        // whileTrue(() -> driver.povDown() && driver.x(),
        // new RightBumpFullSweepAuton(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // m_machinery.m_holonomicController,
        // m_machinery).command());
        // whileTrue(() -> driver.povDown() && driver.y(),
        // new RightBumpHalfSweepAuton(
        // m_log,
        // m_machinery.m_swerveKinodynamics,
        // m_machinery.m_holonomicController,
        // m_machinery).command());

        ////////////////////////////////////////////////////
        ///
        /// TEST
        ///
        /// In test mode, "a" and "b" together runs prematch test.

        // Tester tester = new Tester(m_machinery);
        // onTrue(() -> RobotState.isTest(), tester.prompt());
        // whileTrue(() -> (RobotState.isTest() && driver.a() && driver.b()),
        // tester.prematch());
    }

    /** Keeps tests from conflicting. */
    public void close() {
        //
    }
}
