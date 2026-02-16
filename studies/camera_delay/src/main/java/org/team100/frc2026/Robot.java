package org.team100.frc2026;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.network.RawTags;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.SimulatedAS5048;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.Rotation2dInterpolator;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot100 {
    private static final boolean DEBUG = false;
    /**
     * The position sensor is assumed to have a fixed delay of 600 us.
     */
    private static final double SENSOR_DELAY_S = 0.0006;
    private static final double MOTOR_SPEED_RAD_S = 50.0;

    private final BareMotor m_motor;
    private final AS5048RotaryPositionSensor m_sensor;
    private final RawTags m_rawTags;

    private final TimeInterpolatableBuffer100<Rotation2d> m_motorBuffer;
    private final TimeInterpolatableBuffer100<Rotation2d> m_sensorBuffer;
    private final TimeInterpolatableBuffer100<Rotation2d> m_cameraBuffer;

    private final Rotation2dLogger m_logMotor;
    private final Rotation2dLogger m_logSensor;
    private final Rotation2dLogger m_logCamera;
    private final Rotation2dLogger m_logSensorMinusCamera;
    private final Rotation2dLogger m_logMotorMinusCamera;
    private final Rotation2dLogger m_logMotorMinusSensor;

    private final SimulatedAS5048 m_simSensor;
    private final SimulatedCamera m_simCamera;

    private double m_positionRad;
    private double t0;
    private double t;

    public Robot() {
        Logging logging = Logging.instance();
        LoggerFactory log = logging.rootLogger;

        if (RobotBase.isReal()) {
            m_motor = NeoVortexCANSparkMotor.get(
                    log,
                    new CanId(1),
                    MotorPhase.FORWARD,
                    20, // current limit
                    NeoVortexCANSparkMotor.ff(log),
                    NeoVortexCANSparkMotor.friction(log),
                    PIDConstants.makePositionPID(log, 0.25));
        } else {
            m_motor = new SimulatedBareMotor(log, 600);
        }

        RoboRioChannel sensorChannel = new RoboRioChannel(1);
        // The offset here is from the testboard, experimenting until the "sensor minus
        // camera" is zero at rest.
        m_sensor = new AS5048RotaryPositionSensor(
                log,
                sensorChannel,
                0.402, // offset in turns
                EncoderDrive.INVERSE);

        m_motorBuffer = new TimeInterpolatableBuffer100<>(
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
        m_logMotorMinusCamera = log.rotation2dLogger(Level.TRACE, "motor minus camera");
        m_logMotorMinusSensor = log.rotation2dLogger(Level.TRACE, "motor minus sensor");
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

    @Override
    public void robotPeriodic() {
        // Refreshing the cache twice here. :-(

        Takt.update();
        if (DEBUG)
            System.out.printf("takt %f\n", Takt.actual());

        // This updates the motor output to match the input.
        Cache.refresh();

        // the motor state is updated in the cache refresh
        Rotation2d newValue = new Rotation2d(MathUtil.angleModulus(m_motor.getUnwrappedPositionRad()));
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

        // the extra addition wraps the result
        Rotation2d laggedSensor = m_sensorBuffer.get(past).plus(Rotation2d.kZero);
        if (DEBUG)
            System.out.printf("lagged sensor %s\n", laggedSensor);
        m_logSensor.log(() -> laggedSensor);

        // the extra addition wraps the result
        Rotation2d laggedCamera = m_cameraBuffer.get(past).plus(Rotation2d.kZero);
        m_logCamera.log(() -> laggedCamera);

        m_logSensorMinusCamera.log(() -> laggedSensor.minus(laggedCamera));
        m_logMotorMinusCamera.log(() -> laggedMotor.minus(laggedCamera));
        m_logMotorMinusSensor.log(() -> laggedMotor.minus(laggedSensor));

        m_motor.periodic();
        m_sensor.periodic();
    }

    @Override
    public void teleopInit() {
        m_positionRad = 0;
        t0 = Takt.get();
        m_motor.setUnwrappedEncoderPositionRad(m_positionRad);
    }

    @Override
    public void teleopPeriodic() {
        // this comes before robotPeriodic.
        t = Takt.get() - t0;
        double motorPositionIncrementRad = MOTOR_SPEED_RAD_S * TimedRobot100.LOOP_PERIOD_S;
        m_positionRad += motorPositionIncrementRad;
        m_motor.setUnwrappedPosition(m_positionRad, 0, 0, 0);
    }

    @Override
    public void teleopExit() {
        m_motor.stop();
    }

}
