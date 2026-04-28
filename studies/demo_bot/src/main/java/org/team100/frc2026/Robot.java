package org.team100.frc2026;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.subsystems.shooter.DualDrumShooterFactory;
import org.team100.lib.subsystems.shooter.IndexerFactory;
import org.team100.lib.subsystems.shooter.PivotDefault;
import org.team100.lib.subsystems.shooter.PivotSubsystem;
import org.team100.lib.subsystems.shooter.ShooterIndexer;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.TankDriveFactory;
import org.team100.lib.subsystems.tank.commands.TankManual;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double DRIVE_GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double DRIVE_WHEEL_DIAM = 0.098425;
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_OMEGA_RAD_S = 3.0;

    private static final double SHOOTER_GEAR_RATIO = 1.00;
    private static final double SHOOTER_WHEEL_DIA_M = 0.16;

    private final RobotLog m_robotLog;
    private final TankDrive m_drive;
    private final Command m_auton;
    private final DualDrumShooter m_shooter;
    private final ShooterIndexer m_indexer;
    private final PivotSubsystem m_pivot;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());

        m_robotLog = new RobotLog();
        TotalCurrentLog m_currentLog = m_robotLog.totalCurrentLog();

        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        LoggerFactory logger = logging.rootLogger;

        DriverXboxControl driverControl = new DriverXboxControl(0);

        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                m_currentLog,
                new CurrentLimit(20, 20),
                new CanId(3),
                new CanId(2),
                0.4,
                MAX_SPEED_M_S,
                DRIVE_GEAR_RATIO,
                DRIVE_WHEEL_DIAM);
        m_drive.setDefaultCommand(new TankManual(
                logger, driverControl::rightY, driverControl::rightX,
                MAX_SPEED_M_S, MAX_OMEGA_RAD_S, m_drive));

        //////////////////////////////////////////////////////
        ///
        /// There are multiple shooter implementations
        ///
        // m_shooter = DualDrumShooterFactory.makeDutyCycleShooter(
        // logger,
        // m_currentLog,
        // 0.1, // full output duty cycle
        // new CurrentLimit(20, 20),
        // new CanId(39),
        // new CanId(8));
        m_shooter = DualDrumShooterFactory.makeVelocityShooter(
                logger,
                m_currentLog,
                10,
                true,
                new CurrentLimit(20, 20),
                new CanId(39),
                new CanId(8),
                SHOOTER_GEAR_RATIO,
                SHOOTER_WHEEL_DIA_M);
        m_shooter.setDefaultCommand(m_shooter.stop());

        /////////////////////////////////////////////////
        ///
        /// There are several indexer implementations.
        ///
        // m_indexer = new PWMIndexerServo(logger, 0);
        // m_indexer = IndexerFactory.makeVelocityIndexer(
        // logger, m_currentLog,
        // new CurrentLimit(20, 20),
        // new CanId(100), // TODO: CANID
        // 1.0, 0.1);
        m_indexer = IndexerFactory.makePositionIndexer(
                logger, m_currentLog,
                new CurrentLimit(20, 20),
                new CanId(100), // TODO: CANID
                1.0, 0.1, false);
        m_indexer.setDefaultCommand(m_indexer.stop());

        m_pivot = new PivotSubsystem(
                logger,
                m_currentLog,
                new CurrentLimit(15, 15),
                new CanId(5));
        m_pivot.setDefaultCommand(
                new PivotDefault(driverControl::leftY, m_pivot));

        /////////////////////////////////////////////////////////////////////////////////////
        ///
        /// SHOOTER CONTROLS
        ///
        new Trigger(driverControl::leftTrigger)
                .whileTrue(m_shooter.spin()
                        .withName("Shooter spin"));
        new Trigger(driverControl::leftBumper)
                .onTrue(m_indexer.single()
                        .withName("Indexer single"));
        new Trigger(driverControl::rightBumper)
                .whileTrue(m_indexer.continuous()
                        .withName("Indexer continuous"));

        m_auton = null;
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     * 
     * This is what should do all the work.
     */
    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotLog.periodic();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void autonomousInit() {
        if (m_auton == null)
            return;
        CommandScheduler.getInstance().schedule(m_auton);
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_pivot.setEncoderPosition(Math.PI / 2);
    }

}