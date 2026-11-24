package frc.robot;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Nosepicker extends SubsystemBase {
    private final NeoVortexCANSparkMotor m_motor1;
    private final NeoVortexCANSparkMotor m_motor2;
    private final RotaryMechanism rotary1;
    private final RotaryMechanism rotary2;
    private final double TORQUE_NM = 3.6;
    private double OPEN = 0;
    private double CLOSED = 1.55;
    public Nosepicker() {
        LoggerFactory log = Logging.root();
        RobotLog robotLog = new RobotLog(log);
        TotalCurrentLog currentLog = robotLog.totalCurrentLog();
        System.out.println("Initializing motors");
        m_motor1 = new NeoVortexCANSparkMotor(
                log.name("vortex motor L"),
                currentLog,
                new CanId(2), 
                NeutralMode100.BRAKE,
                MotorPhase.REVERSE, // f for closing
                new CurrentLimit(40, 40),
                new Friction(0, 0, 0, 0),
                PIDConstants.makePositionPID(20),
                0,
                0);
        m_motor2 = new NeoVortexCANSparkMotor(
                log.name("vortex motor R"),
                currentLog,
                new CanId(5), 
                NeutralMode100.BRAKE,
                MotorPhase.FORWARD,
                new CurrentLimit(40, 40),
                new Friction(0, 0, 0, 0),
                PIDConstants.makePositionPID(20),
                0,
                0);
        rotary1 = new RotaryMechanism(
                log.name("rotary 2 L"),
                m_motor1,
                m_motor1.encoder(),
                0,
                1,
                OPEN,
                CLOSED);
        rotary2 = new RotaryMechanism(
                log.name("rotary 5 R"),
                m_motor2,
                m_motor2.encoder(),
                0,
                1,
                OPEN,
                CLOSED);

    }

    public void closeGrabber() {
        rotary1.setUnwrappedPosition(CLOSED, 0, TORQUE_NM);
        rotary2.setUnwrappedPosition(CLOSED, 0, TORQUE_NM);
    }

    public void openGrabber() {
        rotary1.setUnwrappedPosition(OPEN, 0, TORQUE_NM);
        rotary2.setUnwrappedPosition(OPEN, 0, TORQUE_NM);
    }

    public void stopGrabber() {
        rotary1.stop();
        rotary2.stop();
    }

    public void resetGrabber() {
        rotary1.setUnwrappedEncoderPositionRad(OPEN);
        rotary2.setUnwrappedEncoderPositionRad(OPEN);
    }

    public void testMotor1(int velocityRad_S) {
        rotary1.setVelocity(velocityRad_S, TORQUE_NM);
        rotary2.stop();
    }

    public void testMotor2(int velocityRad_S) {
        rotary1.stop();
        rotary2.setVelocity(velocityRad_S, TORQUE_NM);
    }

}