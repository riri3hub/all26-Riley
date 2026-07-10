package frc.robot;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 20;
    private static final double STATOR_LIMIT = 20;
    final Logging logging = Logging.instance();
    final LoggerFactory logger = logging.rootLogger;
    TotalCurrentLog currentLog = new TotalCurrentLog(logger);
    PIDConstants pid = PIDConstants.makePositionPID(logger, 2.0);
    SimpleDynamics ff = new SimpleDynamics(logger, 0, 0);
    Friction friction = new Friction(logger, 0, 0, 0, 0);
    KrakenX60Motor motor1;
    KrakenX60Motor motor2;

    public Robot() {
        motor1 = new KrakenX60Motor(
                logger.name("one"),
                currentLog,
                new CanId(18),
                NeutralMode100.COAST,
                MotorPhase.FORWARD,
                new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                ff,
                friction,
                pid);
        motor2 = new KrakenX60Motor(
                logger.name("two"),
                currentLog,
                new CanId(19),
                NeutralMode100.COAST,
                MotorPhase.REVERSE,
                new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                ff,
                friction,
                pid);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        motor1.setDutyCycle(1.0);
        motor2.setDutyCycle(1.0);
    }

    @Override
    public void teleopExit() {
    }

}
