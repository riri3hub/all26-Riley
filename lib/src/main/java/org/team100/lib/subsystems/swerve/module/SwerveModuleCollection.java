package org.team100.lib.subsystems.swerve.module;

import java.util.List;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.music.Player;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

/**
 * Represents the modules in the drivetrain.
 * Do not put logic here; this is just for bundling the modules together.
 */
public class SwerveModuleCollection implements Player {
    private static final boolean DEBUG = false;
    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    private final List<Player> m_players;

    SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_players = List.of(
                m_frontLeft.players(),
                m_frontRight.players(),
                m_rearLeft.players(),
                m_rearRight.players())
                .stream().flatMap(List::stream).toList();
    }

    @Override
    public List<Player> players() {
        return m_players;
    }

    /**
     * Creates collections according to Identity.
     */
    public static SwerveModuleCollection get(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit driveLimit,
            CurrentLimit steerLimit,
            SwerveKinodynamics kinodynamics) {
        LoggerFactory collectionLogger = parent.name("Swerve Modules");
        LoggerFactory frontLeftLogger = collectionLogger.name("Front Left");
        LoggerFactory frontRightLogger = collectionLogger.name("Front Right");
        LoggerFactory rearLeftLogger = collectionLogger.name("Rear Left");
        LoggerFactory rearRightLogger = collectionLogger.name("Rear Right");

        switch (Identity.instance) {
            // TODO: turned off while testing
            case COMP_BOT:
                // case SWERVE_TWO:
                System.out.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getKrakenDrive(
                                frontLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(1), // drive
                                DriveRatio.MEDIUM,
                                new CanId(3), // steer
                                new RoboRioChannel(8),
                                0.109162,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                frontRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(22), // drive
                                DriveRatio.MEDIUM,
                                new CanId(18), // steer
                                  new RoboRioChannel(6),
                                0.361342,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                rearLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(8), // drive
                                DriveRatio.MEDIUM,
                                new CanId(7), // steer
                                new RoboRioChannel(7),
                                0.611814,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                rearRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(23), // drive
                                DriveRatio.MEDIUM,
                                new CanId(21), // steer
                                new RoboRioChannel(9),
                                0.279052,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
            case SWERVE_ONE:
                System.out.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getFalconDrive(
                                frontLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(12), // drive
                                DriveRatio.FAST,
                                new CanId(32), // steer
                                new RoboRioChannel(6),
                                0.936189,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(
                                frontRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(11), // drive
                                DriveRatio.FAST,
                                new CanId(30), // steer
                                new RoboRioChannel(8),
                                0.875511,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(
                                rearLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(21), // drive
                                DriveRatio.FAST,
                                new CanId(31), // steer
                                new RoboRioChannel(7),
                                0.409354,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(
                                rearRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(22), // drive
                                DriveRatio.FAST,
                                new CanId(33), // steer
                                new RoboRioChannel(9),
                                0.029534,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
            case BETA_BOT:
                // case SWERVE_TWO:
                System.out.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getKrakenDrive(
                                frontLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(3), // drive
                                DriveRatio.MEDIUM,
                                new CanId(44), // steer
                                new RoboRioChannel(3),
                                0.228237,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                frontRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(8), // drive
                                DriveRatio.MEDIUM,
                                new CanId(7), // steer
                                  new RoboRioChannel(2),
                                0.817243,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                rearLeftLogger, currentLog, driveLimit, steerLimit,
                                new CanId(2), // drive
                                DriveRatio.MEDIUM,
                                new CanId(50), // steer
                                new RoboRioChannel(1),
                                0.147507,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(
                                rearRightLogger, currentLog, driveLimit, steerLimit,
                                new CanId(4), // drive
                                DriveRatio.MEDIUM,
                                new CanId(62), // steer
                                new RoboRioChannel(0),
                                0.853782,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode100.COAST, MotorPhase.REVERSE));
            


            case BLANK:
            default:
                if (DEBUG)
                    System.out.println("************** SIMULATED MODULES **************");
                /*
                 * Uses simulated position sensors, must be used with clock control (e.g.
                 * {@link Timeless}).
                 */
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(frontLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(frontRightLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearRightLogger, kinodynamics));
        }
    }

    public static SwerveModuleCollection forTest(LoggerFactory log, SwerveKinodynamics kinodynamics) {
        return new SwerveModuleCollection(
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics));
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     * 
     * @param nextStates for now+dt
     */
    public void setDesiredStates(SwerveModuleStates nextStates) {
        if (DEBUG) {
            System.out.printf("setDesiredStates() %s\n", nextStates);
        }
        m_frontLeft.setDesiredState(nextStates.frontLeft());
        m_frontRight.setDesiredState(nextStates.frontRight());
        m_rearLeft.setDesiredState(nextStates.rearLeft());
        m_rearRight.setDesiredState(nextStates.rearRight());
    }

    /**
     * Does not optimize.
     * 
     * This "raw" mode is just for testing.
     * 
     * Works fine with empty angles.
     */
    public void setRawDesiredStates(SwerveModuleStates swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setRawDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setRawDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setRawDesiredState(swerveModuleStates.rearRight());
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    /** Set turning setpoint to measurement, zero drive encoders. */
    public void reset() {
        m_frontLeft.reset();
        m_frontRight.reset();
        m_rearLeft.reset();
        m_rearRight.reset();
    }

    //////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Uses Cache so the positions are fresh and coherent. */
    public SwerveModulePositions positions() {
        return new SwerveModulePositions(
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition());
    }

    /** FOR TEST ONLY */
    public SwerveModuleStates states() {
        return new SwerveModuleStates(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public boolean[] atSetpoint() {
        return new boolean[] {
                m_frontLeft.atSetpoint(),
                m_frontRight.atSetpoint(),
                m_rearLeft.atSetpoint(),
                m_rearRight.atSetpoint()
        };
    }

    ////////////////////////////////////////////

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public SwerveModule100[] modules() {
        return new SwerveModule100[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight };
    }

    /** Updates visualization. */
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    @Override
    public void play(double freq) {
        m_frontLeft.play(freq);
        m_frontRight.play(freq);
        m_rearLeft.play(freq);
        m_rearRight.play(freq);
    }
}
