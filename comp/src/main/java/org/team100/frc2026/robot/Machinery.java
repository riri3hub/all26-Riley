package org.team100.frc2026.robot;

import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.UnaryOperator;

import org.team100.frc2026.IntakeExtend;
import org.team100.lib.coherence.Takt;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.GroundTruthCache;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryNoise;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.SwerveDriveFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;

    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Runnable m_robotViz;
    private final Runnable m_groundTruthViz;
    private final SwerveModuleCollection m_modules;
    private final Runnable m_simulatedTagDetector;
    private final Consumer<Pose2d> m_groundTruthResetter;

    public final TrajectoryVisualization m_trajectoryViz;
    public final SwerveKinodynamics m_swerveKinodynamics;
    final AprilTagRobotLocalizer m_localizer;
    public final SwerveDriveSubsystem m_drive;
    final Beeper m_beeper;
    //public final Shooter m_shooter;
    //public final Intake m_intake;
    public final IntakeExtend m_extender;
    //final Serializer m_serializer;

    //public final ClimberExtension m_ClimberExtension;
    //public final Climber m_Climber;  
   // final ShooterHood m_shooterHood;
  
    public Machinery() {

        final LoggerFactory driveLog = logger.name("Drive");
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get(driveLog);

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //

        // Subsystem initializers go here.
        //m_shooter = new Shooter(driveLog);
        //m_intake = new Intake(driveLog, new CanId(14));
        m_extender = new IntakeExtend(driveLog, new CanId(20));
        //m_serializer = new Serializer(driveLog);
        //m_ClimberExtension = new ClimberExtension(driveLog);
        //m_shooterHood = new ShooterHood(driveLog, null);
        //m_Climber = new Climber(driveLog, new CanId(32));

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //

        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);

        ////////////////////////////////////////////////////////////
        //
        // POSE ESTIMATION
        //
        m_modules = SwerveModuleCollection.get(
                driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        final SwerveHistory history = new SwerveHistory(
                driveLog,
                m_swerveKinodynamics,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                m_modules.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        // New! Odometry noise is applied in simulation.
        UnaryOperator<Twist2d> odometryNoise = RobotBase.isReal() ? UnaryOperator.identity() : new OdometryNoise();
        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                driveLog,
                m_swerveKinodynamics,
                gyro,
                history,
                m_modules::positions,
                odometryNoise);
        // odometryUpdater.m_debug = true;
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high());
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                driveLog, history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        final AprilTagFieldLayoutWithCorrectOrientation layout = getLayout();

        m_localizer = new AprilTagRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                visionUpdater,
                100);

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_drive = SwerveDriveFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_localizer,
                odometryUpdater,
                history,
                m_modules);
        // NOTE: Initial rotation is 180 degrees, because that was common in, like,
        // 2024?
        // TODO: maybe don't do that?
        // Pose2d initialPose = new Pose2d(m_drive.getPose().getTranslation(), new
        // Rotation2d(Math.PI));
        // Initialize both ground truth and observations.
        // m_drive.resetPose(initialPose);

        // Visualization of the robot pose estimate
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState().pose(), "robot");

        ////////////////////////////////////////////////////////////
        ///
        /// GROUND TRUTH DRIVETRAIN FOR SIMULATION
        ///
        /// To correctly simulate the influence of vision and gyro drift on robot
        /// rotation, we need to track the "ground truth" of the robot separately from
        /// the simulated measurements (which include drift).
        ///

        if (RobotBase.isReal()) {
            // Real robots get an empty simulated tag detector.
            m_groundTruthViz = () -> {
            };
            m_simulatedTagDetector = () -> {
            };
            m_groundTruthResetter = (p) -> {
            };
        } else {
            // This is all for simulation only.
            final LoggerFactory simLog = logger.name("Simulation");

            // Ground-truth simulated gyro does not drift at all.
            SimulatedGyro groundTruthGyro = new SimulatedGyro(simLog,
                    m_swerveKinodynamics, m_modules, 0);

            // History of ground-truth poses is based only on odometry.
            SwerveHistory groundTruthHistory = new SwerveHistory(
                    simLog,
                    m_swerveKinodynamics,
                    0.2,
                    groundTruthGyro.getYawNWU(),
                    VariableR1.fromStdDev(0, 1),
                    m_modules.positions(),
                    Pose2d.kZero,
                    IsotropicNoiseSE2.high(),
                    Takt.get());

            // Read positions and ground truth gyro (which are perfectly consistent) and
            // maintain the ground truth history.
            OdometryUpdater groundTruthUpdater = new OdometryUpdater(
                    simLog, m_swerveKinodynamics, groundTruthGyro,
                    groundTruthHistory, m_modules::positions,
                    UnaryOperator.identity());
            m_groundTruthResetter = (p) -> groundTruthUpdater.reset(p, IsotropicNoiseSE2.high());

            GroundTruthCache groundTruthCache = new GroundTruthCache(
                    groundTruthUpdater, groundTruthHistory);

            // Visualization of the simulated "ground truth" of the robot pose.
            m_groundTruthViz = new RobotPoseVisualization(
                    fieldLogger, () -> groundTruthCache.apply(Takt.get()).pose(), "ground truth");

            // Simulated camera uses the ground truth because the real cameras are not aware
            // of the pose estimate.
            m_simulatedTagDetector = SimulatedTagDetector.get(
                    layout, groundTruthHistory);
        }

        ////////////////////////////////////////////////////////////
        //
        // INDICATOR
        //

        // There's no LED this year, unless we need it for testing or setup.

        // This makes beeps to warn about testing.
        m_beeper = new Beeper(m_drive);
    }

    /**
     * Purge the history and assert the given pose as the current estimate.
     */
    public void resetPose(Pose2d p) {
        m_drive.resetPose(p, IsotropicNoiseSE2.high());
        m_groundTruthResetter.accept(p);
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish pose estimate
        m_robotViz.run();
        // publish ground truth pose
        if (m_groundTruthViz != null)
            m_groundTruthViz.run();
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
    }

    /** Trap the IO exception. */
    private static AprilTagFieldLayoutWithCorrectOrientation getLayout() {
        try {
            return new AprilTagFieldLayoutWithCorrectOrientation();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }
}
