package org.team100.frc2026;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.network.NetworkUtil;
import org.team100.lib.network.RawTags;
import org.team100.lib.network.Sync;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.SimulatedAS5048;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.Rotation2dInterpolator;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot100 {
    private static final boolean DEBUG = false;
    /**
     * The sensor itself has a propagation delay of 100 us and a sampling rate of 10
     * kHz, with a PWM output rate of 1 kHz.
     * 
     * https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4-00-1100510.pdf
     * 
     * The RoboRIO is counting the duty cycle duration, and needs to get to the end
     * of the pulse (1ms) in order to make a measurement.
     * 
     * So the measurement represents at least 1ms in the past, plus the 100 us
     * delay.
     */
    private static final double SENSOR_DELAY_S = 0.0011;
    // Note with positional control very high speeds don't work
    // because of the discretization of the goal
    private static final double MOTOR_SPEED_RAD_S = 50.0;

    private final BareMotor m_motor;
    private final AS5048RotaryPositionSensor m_sensor;
    private final RawTags m_rawTags;

    // where the motor says it is
    private final TimeInterpolatableBuffer100<Rotation2d> m_motorBuffer;
    // where we *want* the motor to be
    private final TimeInterpolatableBuffer100<Rotation2d> m_motorCmdBuffer;

    private final TimeInterpolatableBuffer100<Rotation2d> m_sensorBuffer;
    private final TimeInterpolatableBuffer100<Rotation2d> m_cameraBuffer;

    private final Sync sync;

    private final Rotation2dLogger m_logMotor;
    private final Rotation2dLogger m_logSensor;
    private final Rotation2dLogger m_logCamera;
    private final Rotation2dLogger m_logSensorMinusCamera;
    private final DoubleLogger m_logDiff;
    private final Rotation2dLogger m_logMotorMinusCamera;
    private final Rotation2dLogger m_logMotorMinusSensor;
    private final Rotation2dLogger m_logMotorCmdMinusMotor;

    private final Rotation2dLogger m_logMotorCmd;
    private final Rotation2dLogger m_logMotorMeasurement;

    private final DoubleLogger m_logActualSpeed;

    private final SimulatedAS5048 m_simSensor;
    private final SimulatedCamera m_simCamera;

    private final XboxController m_controller;

    private double m_positionRad;

    public Robot() {
        Logging logging = Logging.instance();
        LoggerFactory log = logging.rootLogger;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        sync = new Sync(inst);

        m_controller = new XboxController(0);

        if (RobotBase.isReal()) {
            m_motor = NeoVortexCANSparkMotor.get(
                    log,
                    new CanId(1),
                    MotorPhase.FORWARD,
                    20, // current limit
                    NeoVortexCANSparkMotor.ff(log),
                    // NeoVortexCANSparkMotor.friction(log),
                    // motor is free spinning, low friction, set by experiment
                    // to get the RPM correct using back EMF only
                    new Friction(log, 0.15, 0.06, 0.0, 0.5),
                    // not much P required
                    PIDConstants.makePositionPID(log, 0.5));
            // PIDConstants.makePositionPID(log, 0.5));

        } else {
            m_motor = new SimulatedBareMotor(log, 600);
        }

        RoboRioChannel sensorChannel = new RoboRioChannel(1);
        // The offset here is from the testboard, experimenting until the "sensor minus
        // camera" is zero at rest.
        m_sensor = new AS5048RotaryPositionSensor(
                log,
                sensorChannel,
                0.515, // offset in turns
                EncoderDrive.INVERSE);

        m_motorBuffer = new TimeInterpolatableBuffer100<>(
                new Rotation2dInterpolator(), 2, 0, Rotation2d.kZero);
        m_motorCmdBuffer = new TimeInterpolatableBuffer100<Rotation2d>(
                new Rotation2dInterpolator(), 2, 0, Rotation2d.kZero);
        m_sensorBuffer = new TimeInterpolatableBuffer100<>(
                new Rotation2dInterpolator(), 2, 0, Rotation2d.kZero);
        m_cameraBuffer = new TimeInterpolatableBuffer100<>(
                new Rotation2dInterpolator(), 2, 0, Rotation2d.kZero);

        // Update the buffer with the roll component, and accept the
        // supplied timestamp as true.
        Roll sink = new Roll(log, (r, t) -> m_cameraBuffer.put(t, r));
        m_rawTags = new RawTags(log, sink);

        m_logMotor = log.rotation2dLogger(Level.TRACE, "lagged motor");
        m_logSensor = log.rotation2dLogger(Level.TRACE, "lagged sensor");
        m_logCamera = log.rotation2dLogger(Level.TRACE, "lagged camera");
        m_logSensorMinusCamera = log.rotation2dLogger(Level.TRACE, "sensor minus camera");
        m_logDiff = log.doubleLogger(Level.TRACE, "diff (ms)");
        m_logMotorMinusCamera = log.rotation2dLogger(Level.TRACE, "motor minus camera");
        m_logMotorMinusSensor = log.rotation2dLogger(Level.TRACE, "motor minus sensor");
        m_logMotorCmdMinusMotor = log.rotation2dLogger(Level.TRACE, "motor cmd minus motor");
        m_logMotorCmd = log.rotation2dLogger(Level.TRACE, "motor cmd now (rad)");
        m_logMotorMeasurement = log.rotation2dLogger(Level.TRACE, "motor measurement now (rad)");
        m_logActualSpeed = log.doubleLogger(Level.TRACE, "actual speed (rad_s)");


        if (RobotBase.isSimulation()) {
            // these extra additions are to wrap the result.
            m_simSensor = new SimulatedAS5048(
                    (x) -> m_motorBuffer.get(x).plus(Rotation2d.kZero),
                    m_sensor);
            m_simCamera = new SimulatedCamera(
                    (x) -> m_motorBuffer.get(x).plus(Rotation2d.kZero));
        } else {
            m_simSensor = null;
            m_simCamera = null;
        }

        waitForDutyCycleBug();
    }

    static void waitForDutyCycleBug() {
        try {
            if (RobotBase.isReal())
                Thread.sleep(3000);
        } catch (InterruptedException e) {
        }
    }

    Rotation2d prevValue = Rotation2d.kZero;
    double prevTime = 0;

    @Override
    public void robotPeriodic() {
        // Refreshing the cache twice here. :-(

        Takt.update();
        // reply to sync requests.
        sync.run();

        if (DEBUG)
            System.out.printf("takt %f\n", Takt.actual());

        // This updates the motor output to match the input.
        Cache.refresh();

        // the motor state is updated in the cache refresh
        Rotation2d newValue = new Rotation2d(MathUtil.angleModulus(m_motor.getUnwrappedPositionRad()));
        m_logMotorMeasurement.log(() -> newValue);

        m_motorBuffer.put(Takt.actual(), newValue);
        if (DEBUG)
            System.out.printf("motor value %s\n", newValue);

        if (m_simSensor != null) {
            m_simSensor.run();
        }
        if (m_simCamera != null) {
            m_simCamera.run();
        }

        // This updates the real AS5048 to read the value written by the simulated one.
        Cache.refresh();

        // Read the sensor and update the sensor buffer.
        // The value here is the same one that was just written by the sim
        Rotation2d sensorValue = new Rotation2d(m_sensor.getWrappedPositionRad());
        m_sensorBuffer.put(Takt.actual() - SENSOR_DELAY_S, sensorValue);
        if (DEBUG)
            System.out.printf("sensor value %s\n", sensorValue);

        // Read the camera and update the camera buffer.
        m_rawTags.update();

        // Sample the buffers from 1 sec ago and log.
        double past = Takt.actual() - 1.0;

        // the extra addition wraps the result
        Rotation2d laggedMotor = m_motorBuffer.get(past).plus(Rotation2d.kZero);
        if (DEBUG)
            System.out.printf("lagged motor %s\n", laggedMotor);
        m_logMotor.log(() -> laggedMotor);

        Rotation2d laggedMotorCmd = m_motorCmdBuffer.get(past).plus(Rotation2d.kZero);

        // the extra addition wraps the result
        Rotation2d laggedSensor = m_sensorBuffer.get(past).plus(Rotation2d.kZero);
        if (DEBUG)
            System.out.printf("lagged sensor %s\n", laggedSensor);
        m_logSensor.log(() -> laggedSensor);

        // the extra addition wraps the result
        Rotation2d laggedCamera = m_cameraBuffer.get(past).plus(Rotation2d.kZero);
        m_logCamera.log(() -> laggedCamera);
        // difference in radians
        Rotation2d sensorCameraDiff = laggedSensor.minus(laggedCamera);

        // use sensor as ground truth for rotation speed
        Rotation2d dRad = laggedSensor.minus(prevValue);
        double dt = past - prevTime;
        prevValue = laggedSensor;
        prevTime = past;
        if (Math.abs(dRad.getRadians()) > 1e-2) {
            double actualSpeed = dRad.getRadians() / dt;
            m_logActualSpeed.log(() -> actualSpeed);
            double diffMs = 1000 * sensorCameraDiff.getRadians() / actualSpeed;
            m_logDiff.log(() -> diffMs);
        }
        m_logSensorMinusCamera.log(() -> sensorCameraDiff);
        m_logMotorMinusCamera.log(() -> laggedMotor.minus(laggedCamera));
        m_logMotorMinusSensor.log(() -> laggedMotor.minus(laggedSensor));
        m_logMotorCmdMinusMotor.log(() -> laggedMotorCmd.minus(laggedMotor));

        m_motor.periodic();
        m_sensor.periodic();
        NetworkTableInstance.getDefault().flush();
    }

    double prevTime2 = 0;

    // value of nowpi upon init
    // long nowpiinit = 0;
    // value of fpgatime upon init
    // long nowRioInit = 0;

    @Override
    public void teleopInit() {
        // nowpiinit = nowpi.get();
        // nowRioInit = RobotController.getFPGATime();
        m_positionRad = 0;
        prevTime2 = Takt.actual();
        m_motor.setUnwrappedEncoderPositionRad(m_positionRad);
        // maybe we're using the wrong version of the network tables library?
        for (ConnectionInfo ci : NetworkTableInstance.getDefault().getConnections()) {
            System.out.printf("*** CONNECTION %s\n", NetworkUtil.ciString(ci));
        }
    }

    @Override
    public void teleopPeriodic() {
        // this comes before robotPeriodic.
        double t = Takt.actual();
        double dt = t - prevTime2;
        prevTime2 = t;

        double motorSpeedRadS = MOTOR_SPEED_RAD_S * (m_controller.getLeftTriggerAxis());
        m_positionRad += motorSpeedRadS * dt;

        m_motor.setUnwrappedPosition(m_positionRad, motorSpeedRadS, 0, 0);

        Rotation2d motorRot = new Rotation2d(MathUtil.angleModulus(m_positionRad));
        m_logMotorCmd.log(() -> motorRot);
        m_motorCmdBuffer.put(t, motorRot);

        // Full speed, to look at blur.
        // m_motor.setDutyCycle(1.0);

    }

    @Override
    public void teleopExit() {
        m_motor.stop();
    }

}
