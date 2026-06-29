package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.subsystems.five_bar.kinematics.Scenario;
import org.team100.lib.subsystems.five_bar.setups.SetupCartesian;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final Scenario SCENARIO;
    static {
        // origin is P1
        SCENARIO = new Scenario();
        // These are fake link lengths.
        SCENARIO.a1 = 0.2;
        SCENARIO.a2 = 0.2;
        SCENARIO.a3 = 0.2;
        SCENARIO.a4 = 0.2;
        SCENARIO.a5 = 0.1;
        SCENARIO.xcenter = -0.05;
        SCENARIO.ycenter = 0.24;
    }

    private final Runnable m_setup;

    public Robot() {
        /////////////////////////////
        //
        // Choose one of the setups.
        //

        // manual control of each axis
        // m_setup = new SetupBare(SCENARIO);

        // PID positional control of axes independently
        // m_setup = new SetupMech(SCENARIO);

        // profiled control of axes independently
        // m_setup = new SetupServo(SCENARIO);

        // cartesian coordinated control
        m_setup = new SetupCartesian(SCENARIO);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_setup.run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }
}
